#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/filters/passthrough.h>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

#include <Eigen/Core>
#include <pcl/common/transforms.h>

#include <pcl/filters/voxel_grid.h>

#include <visualization_msgs/Marker.h>

// #include <pcl/common/angles.h>

#include <pcl/sample_consensus/sac_model_normal_plane.h>

#include <math.h>
#include <vector>

#include <iostream>

using namespace std;

float x_max = 638.0;
float x_min = 0.0;
float y_max = 478.0;
float y_min = 0.0;

ros::Publisher pub;
ros::Publisher marker_pub;

float vector_length(vector<float> vec_a);
float vector_dot(vector<float> vec_a, vector<float> vec_b);

void bbox_cb (const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
    // cout << "msg_len: " << sizeof(msg->bounding_boxes) << endl;
    // cout << "msg_size: " << msg->bounding_boxes.size() << endl; //because it's a vector (not array actually)
    // cout << "msg[0]_len: " << sizeof(msg->bounding_boxes[0]) << endl;
    for(int i=0 ; i < msg->bounding_boxes.size(); i++){     //problem: how to get msg (array) length, avoiding i out of range 
        if(msg->bounding_boxes[i].Class == "blue_cube"){
            cout << "get blue" << endl;
            x_max = msg->bounding_boxes[i].xmax>=640 ? 639 : msg->bounding_boxes[i].xmax;   //avoiding cloud.at std::out_of_range
            x_min = msg->bounding_boxes[i].xmin<=  0 ?   0 : msg->bounding_boxes[i].xmin;
            y_max = msg->bounding_boxes[i].ymax>=480 ? 479 : msg->bounding_boxes[i].ymax;   //y_max=480 does cause an error before 
            y_min = msg->bounding_boxes[i].ymin<=  0 ?   0 : msg->bounding_boxes[i].ymin;
            cout << "x_max: " << x_max << endl;
            cout << "x_min: " << x_min << endl;
            cout << "y_max: " << y_max << endl;
            cout << "y_min: " << y_min << endl;
            break;
        }
        else{
            cout << "Not that color cube" << endl;       
        }
    }
    cout << "end of bbox_cb" << endl;
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    cout << "original width: " << input->width << endl;
    cout << "original height: " << input->height << endl;

    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*input, cloud);


    // get bbox Euclidean coordinates
    // pt1: left top
    pcl::PointXYZ pt1 = cloud.at(x_min, y_min);     //unit: pixel (column, row)   //must be organized point cloud
    cout << "pt1: " << pt1 << endl;                 //unit: m (p1.x, p1.y, p1.z)  
    // pt2: right bottom
    pcl::PointXYZ pt2 = cloud.at(x_max, y_max);     //unit: pixel (column, row)
    cout << "pt2: " << pt2 << endl;                 //unit: m (p1.x, p1.y, p1.z)


    // voxel grid (downsample)
    pcl::PointCloud<pcl::PointXYZ> cloud_voxel;
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud.makeShared());
    voxel.setLeafSize(0.005f, 0.005f, 0.005f);
    voxel.filter(cloud_voxel);
    cout << "cloud_voxel width: " << cloud_voxel.width << endl;
    cout << "cloud_voxel height: " << cloud_voxel.height << endl;

    // first segmentation to seg ground and get ground coeff.
    pcl::ModelCoefficients::Ptr coefficients_ground(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_ground(new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg1;
    // Optional
    seg1.setOptimizeCoefficients (true);
    // Mandatory
    seg1.setModelType(pcl::SACMODEL_PLANE);
    seg1.setMethodType(pcl::SAC_RANSAC);
    seg1.setDistanceThreshold (0.01);  //unit: m

    seg1.setInputCloud(cloud_voxel.makeShared());
    seg1.segment (*inliers_ground, *coefficients_ground);
    cout << "coefficients_ground: " << *coefficients_ground << endl;

    // first extraction to extract ground
    pcl::ExtractIndices<pcl::PointXYZ> extract1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_ground(new pcl::PointCloud<pcl::PointXYZ>);
    extract1.setInputCloud(cloud_voxel.makeShared());// 载入点云
    extract1.setIndices(inliers_ground);// 设置分割后的内点为需要提取的点集
    extract1.setNegative(true);// true: 外點   //false: 內點
    extract1.filter(*cloud_without_ground);// 开始分割

    vector<float> vec1 { coefficients_ground->values[0], coefficients_ground->values[1], coefficients_ground->values[2] };

    // pass through filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr pass_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pass2_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_without_ground);  
    pass.setFilterFieldName("x");
    pass.setFilterLimits(pt1.x, pt2.x);     //unit: m?
    // pass.setFilterLimits(-0.02, 0.057);
    pass.filter(*pass_cloud);

    pass.setInputCloud(pass_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(pt1.y, pt2.y);
    // pass.setFilterLimits(-0.1, 0.042);
    pass.filter(*pass2_cloud);

    cout << "pt1.x: " << pt1.x << endl;
    cout << "pt2.x: " << pt2.x << endl;
    cout << "pt1.y: " << pt1.y << endl;
    cout << "pt2.y: " << pt2.y << endl;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // select the top plane of the cube
    while(1){
        // second (original) segmentation
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        // pcl::PointIndices inliers;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.003);  //unit: m

        seg.setInputCloud(pass2_cloud);
        seg.segment (*inliers, *coefficients);
        cout << "coefficients: " << *coefficients << endl;

        // extract the final plane
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remain(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setInputCloud(pass2_cloud);// 载入点云
        extract.setIndices(inliers);// 设置分割后的内点为需要提取的点集
        extract.setNegative(false);// 设置提取内点
        extract.filter(*cloud_filtered);// 开始分割
        extract.setNegative(true);
        extract.filter(*cloud_remain);// 开始分割
        cout << "cloud_filtered width: " << cloud_filtered->width << endl;
        if(cloud_filtered->width < 50){
            cout << "No plane can be segment anymore, bombbbbbbbbbbbbbbbbbbbbbbbbbb, break the while loop."<< endl;     //if not break, it does cause an error (coefficients->values[0] will not have value)
            break;
        }

        vector<float> vec2 { coefficients->values[0], coefficients->values[1], coefficients->values[2] };
        float vec_dot = vector_dot(vec1, vec2);
        cout << "vec_dot: " << vec_dot << endl;
        float vec1_len = vector_length(vec1);
        cout << "vec1_len: " << vec1_len << endl;
        float vec2_len = vector_length(vec2);
        cout << "vec2_len: " << vec2_len << endl;
        double vec_theta = acos((double) vec_dot/(vec1_len*vec2_len));  //radian
        cout << "vec_theta: " << vec_theta*180/3.14159 << endl;         //degree

        if(vec_theta < 5 *3.14159/180){         // 5 is degree
            cout << "It's right plane !"<< endl;
            break;
        }
        else{
            pass2_cloud.swap(cloud_remain);
            cout << "Not that plane AAAAAAAAAAAAAAAAAAAAAAAAAAAAAA."<< endl;
        }
    }


    // // second (original) segmentation with normal
    // pcl::ModelCoefficients coefficients;
    // // pcl::PointIndices inliers;
    // pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // // Create the segmentation object
    // // pcl::SACSegmentation<pcl::PointXYZ> seg;
    // pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    // // Optional
    // seg.setOptimizeCoefficients (true);
    // // Mandatory
    // seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setDistanceThreshold (0.003);  //unit: m

    // // create a pointer of normal vector
    // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    // cloud_normals->points[0].normal_x = coefficients_ground->values[0];
    // cloud_normals->points[0].normal_y = coefficients_ground->values[1];
    // cloud_normals->points[0].normal_z = coefficients_ground->values[2];
    // cloud_normals->points[0].curvature = coefficients_ground->values[3];
    // cout << "cloud_normals: " << *cloud_normals << endl;

    // seg.setInputNormals(cloud_normals);     //arg: pointer of normal vector
    // seg.setNormalDistanceWeight(0.1);
    
    // seg.setAxis(Eigen::Vector3f (coefficients_ground.values[0], coefficients_ground.values[1], coefficients_ground.values[2]));
    // seg.setEpsAngle (5 *3.14159/180);

    // seg.setInputCloud(pass2_cloud);
    // seg.segment (*inliers, coefficients);
    // cout << "coefficients: " << coefficients << endl;

    // // Publish the model coefficients
    // // pcl_msgs::ModelCoefficients ros_coefficients;
    // // pcl_conversions::fromPCL(coefficients, ros_coefficients);
    // // pub.publish (ros_coefficients);


    sensor_msgs::PointCloud2 output;
    // pcl_conversions::fromPCL(*inliers, output);
    // pcl::toROSMsg(*pass2_cloud, output);
    pcl::toROSMsg(*cloud_filtered, output);
    pub.publish(output);
    cout << "output.width: " << output.width << endl;
    cout << "output.height: " << output.height << endl;
    cout << "GGGG" << endl; 

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_filtered, centroid);   // 齐次坐标，（c0,c1,c2,1）
    cout << "centroid: " << centroid << endl;
    cout << "centroid_x: " << centroid(0) << endl;
    cout << "centroid_y: " << centroid(1) << endl;

    //visualize centroid in rviz
    visualization_msgs::Marker points;
    points.header.frame_id = "camera_depth_optical_frame";
    points.header.stamp = ros::Time::now();
    points.ns = "segmentation";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::ARROW;
    // points.type = visualization_msgs::Marker::SPHERE;
    // points.pose.position.x = centroid(0);
    // points.pose.position.y = centroid(1);
    // points.pose.position.z = centroid(2);
    // points.scale.x = 0.02;   //size
    // points.scale.y = 0.02;
    // points.scale.z = 0.02;
    points.scale.x = 0.008;     //shaft diameter
    points.scale.y = 0.013;     //head diameter
    points.scale.z = 0.03;      //head length
    points.color.r = 1.0f;      //color: red
    points.color.a = 1.0;
    geometry_msgs::Point p;
    p.x = centroid(0);
    p.y = centroid(1);
    p.z = centroid(2);
    points.points.push_back(p);
    p.x = centroid(0) + coefficients_ground->values[0]/10;
    p.y = centroid(1) + coefficients_ground->values[1]/10;
    p.z = centroid(2) + coefficients_ground->values[2]/10;
    points.points.push_back(p);
    marker_pub.publish(points);

    cout << "end of cloud_cb" << endl;
}

float vector_dot(vector<float> vec_a, vector<float> vec_b){
    float vec_dot = 0;
    for(int i=0; i< vec_a.size(); i++){
        vec_dot += vec_a[i] * vec_b[i];
    }
    return vec_dot;
}

float vector_length(vector<float> vec_a){
    float length_pow = 0;
    for(int i=0; i< vec_a.size(); i++){
        length_pow += pow(vec_a[i], 2);
    }
    return sqrt(length_pow);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    // ros::init (argc, argv, "my_pcl_tutorial");
    ros::init (argc, argv, "segmentation");
    ros::NodeHandle nh;
    cout << "GGGGGG" << endl;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
    ros::Subscriber bbox_sub = nh.subscribe ("darknet_ros/bounding_boxes", 100, bbox_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    // Spin
    ros::spin ();
}