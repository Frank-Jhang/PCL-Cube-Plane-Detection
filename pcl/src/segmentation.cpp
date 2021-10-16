#include "pcl/segmentation.h"


void Segmentation::bbox_cb (const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
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


void Segmentation::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    cout << "original width: " << input->width << endl;
    cout << "original height: " << input->height << endl;

//-------------------- Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud ---------------------
    //pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*input, *cloud_input);


//-------------------- get bbox Euclidean coordinates ---------------------
    // pt1: left top
    pcl::PointXYZRGB pt1 = cloud_input->at(x_min, y_min);     //unit: pixel (column, row)   //must be organized point cloud !!!
    cout << "pt1: " << pt1 << endl;                 //unit: m (p1.x, p1.y, p1.z)  
    // pt2: right bottom
    pcl::PointXYZRGB pt2 = cloud_input->at(x_max, y_max);     //unit: pixel (column, row)
    cout << "pt2: " << pt2 << endl;                 //unit: m (p1.x, p1.y, p1.z)


//-------------------- voxel grid (downsample) ---------------------
    //pcl::PointCloud<pcl::PointXYZ> cloud_voxel;
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(cloud_input);
    voxel.setLeafSize(0.005f, 0.005f, 0.005f);
    voxel.filter(*cloud_voxel);
    cout << "cloud_voxel width: " << cloud_voxel->width << endl;
    cout << "cloud_voxel height: " << cloud_voxel->height << endl;

//-------------------- first segmentation to seg ground and get ground coeff. ---------------------
    //pcl::ModelCoefficients::Ptr coefficients_ground(new pcl::ModelCoefficients);
    //pcl::PointIndices::Ptr inliers_ground(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZRGB> seg1;
    seg1.setOptimizeCoefficients (true);
    seg1.setModelType(pcl::SACMODEL_PLANE);
    seg1.setMethodType(pcl::SAC_RANSAC);
    seg1.setDistanceThreshold (0.01);  //unit: m
    seg1.setInputCloud(cloud_voxel);
    seg1.segment (*inliers_ground, *coefficients_ground);
    //cout << "coefficients_ground: " << *coefficients_ground << endl;
    vector<float> vec1 { coefficients_ground->values[0], coefficients_ground->values[1], coefficients_ground->values[2] };
    
//-------------------- first extraction to extract ground ---------------------
    pcl::ExtractIndices<pcl::PointXYZRGB> extract1;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_ground(new pcl::PointCloud<pcl::PointXYZ>);
    extract1.setInputCloud(cloud_voxel);
    extract1.setIndices(inliers_ground);
    extract1.setNegative(true);     // true: 外點   //false: 內點
    extract1.filter(*cloud_without_ground);

//-------------------- pass through filter ---------------------
    //pcl::PointCloud<pcl::PointXYZ>::Ptr pass_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr pass2_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
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
    
//-------------------- select the top plane of the cube ---------------------
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_top(new pcl::PointCloud<pcl::PointXYZ>);   
    while(1){
//-------------------- second (original) segmentation ---------------------
        //pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        //pcl::PointIndices inliers;
        //pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients (true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.003);  //unit: m
        seg.setInputCloud(pass2_cloud);
        seg.segment (*inliers, *coefficients);
        //cout << "coefficients: " << *coefficients << endl;
        vector<float> vec2 { coefficients->values[0], coefficients->values[1], coefficients->values[2] };

//-------------------- extract the final plane ---------------------
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remain(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setInputCloud(pass2_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_top);
        extract.setNegative(true);
        extract.filter(*cloud_remain);
        cout << "cloud_top width: " << cloud_top->width << endl;
        if(cloud_top->width < 50){
            cout << "No plane can be segment anymore, bombbbbbbbbbbbbbbbbbbbbbbbbbb, break the while loop."<< endl;     //if not break, it does cause an error (coefficients->values[0] will not have value)
            break;
        }

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
            pass2_cloud.swap(cloud_remain);     //why not pass2_cloud->swap()
            cout << "Not that plane AAAAAAAAAAAAAAAAAAAAAAAAAAAAAA."<< endl;
        }
    }

//-------------------- choose the right color point cloud ---------------------
    int num = cloud_top->width;
    cout << "num: " << num << endl;
    for(int i=0; i<num; i++){
        cout << "color: " << cloud_top->points[i].b << " ";
        if(cloud_top->points[i].b > 200){
            cloud_output->push_back(cloud_top->points[i]);
        }
    }
    cout << endl;

//-------------------- calculate centroid ---------------------
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_output, centroid);
    cout << "centroid: " << centroid << endl;
    cout << "centroid_x: " << centroid(0) << endl;
    cout << "centroid_y: " << centroid(1) << endl;

//-------------------- visualize centroid in rviz ---------------------
    visualize(centroid(0), centroid(1), centroid(2));

//-------------------- output ---------------------
    sensor_msgs::PointCloud2 output;
    // pcl_conversions::fromPCL(*inliers, output);
    pcl::toROSMsg(*cloud_output, output);
    pub.publish(output);
    cout << "output.width: " << output.width << endl;
    cout << "output.height: " << output.height << endl;
    cout << "GGGG" << endl; 

    cout << "end of cloud_cb" << endl;
    cloud_output->clear();      //clear all the point in clout_output
}


float Segmentation::vector_dot(vector<float> vec_a, vector<float> vec_b){
    float vec_dot = 0;
    for(int i=0; i< vec_a.size(); i++){
        vec_dot += vec_a[i] * vec_b[i];
    }
    return vec_dot;
}


float Segmentation::vector_length(vector<float> vec_a){
    float length_pow = 0;
    for(int i=0; i< vec_a.size(); i++){
        length_pow += pow(vec_a[i], 2);
    }
    return sqrt(length_pow);
}


void Segmentation::visualize(int center_x, int center_y, int center_z){
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
    p.x = center_x;
    p.y = center_y;
    p.z = center_z;
    points.points.push_back(p);
    p.x = center_x + coefficients_ground->values[0]/10;
    p.y = center_y + coefficients_ground->values[1]/10;
    p.z = center_z + coefficients_ground->values[2]/10;
    points.points.push_back(p);
    marker_pub.publish(points);
}


int main (int argc, char** argv)
{
    ros::init (argc, argv, "segmentation");
    Segmentation seg_obj;
    ros::spin ();
}