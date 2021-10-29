#ifndef _SEGMENTATION_
#define _SEGMENTATION_

#include <ros/ros.h>

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

#include <pcl/sample_consensus/sac_model_normal_plane.h>

#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include <math.h>
#include <vector>

#include <iostream>

using namespace std;

class Segmentation{
public:
	Segmentation(){
		pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
		temp_pub = nh.advertise<sensor_msgs::PointCloud2> ("temp_output", 1);
		marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
		//sub = nh.subscribe ("input", 1, cloud_cb);	//error: invalid use of non-static member function (why?)
		sub = nh.subscribe ("input", 1, &Segmentation::cloud_cb, this);		//success (c.f. the upper error)
		//bbox_sub = nh.subscribe ("darknet_ros/bounding_boxes", 100, bbox_cb);
		bbox_sub = nh.subscribe ("darknet_ros/bounding_boxes", 100, &Segmentation::bbox_cb, this);
		accel_sub = nh.subscribe("camera/accel/sample", 10, &Segmentation::accel_cb, this);
		point_pub = nh.advertise<geometry_msgs::PointStamped> ("point_output", 1);
	}
	void bbox_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);     //bounding box callback
	void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);
	void visualize(float center_x, float center_y, float center_z);
	float vector_dot(vector<float> vec_a, vector<float> vec_b);
	float vector_length(vector<float> vec_a);
	void RGBtoHSV(float& fR, float& fG, float& fB, float& fH, float& fS, float& fV);
	void accel_cb(const sensor_msgs::Imu& msg);

private:
	float x_max = 638.0;
	float x_min = 0.0;
	float y_max = 478.0;
	float y_min = 0.0;
	float fR = 0, fG = 0, fB = 0, fH = 0, fS = 0, fV = 0;

	enum Color{RED, GREEN, BLUE};
	int selected_color = 0;		//default: red cube
	int count = 0;
	
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Publisher temp_pub;
	ros::Publisher marker_pub;
	ros::Subscriber sub;
	ros::Subscriber bbox_sub;
	ros::Subscriber accel_sub;
	ros::Publisher point_pub;

	//static tf::TransformBroadcaster br;	//will cause cmake error (why?)
	tf::TransformBroadcaster br;
    tf::Transform transform;
	tf::Quaternion q;
    double roll = 0, pitch = 0;		//each roll, pitch. Below, calculated from the value of each topic

	tf::TransformListener listener;
	geometry_msgs::PointStamped camera_point;
	geometry_msgs::PointStamped base_point;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input{new pcl::PointCloud<pcl::PointXYZRGB>};	//why use {}, tried () and = but both fail 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxel{new pcl::PointCloud<pcl::PointXYZRGB>};
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_without_ground{new pcl::PointCloud<pcl::PointXYZRGB>};
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pass_cloud{new pcl::PointCloud<pcl::PointXYZRGB>};
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pass2_cloud{new pcl::PointCloud<pcl::PointXYZRGB>};
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_top{new pcl::PointCloud<pcl::PointXYZRGB>};
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_remain{new pcl::PointCloud<pcl::PointXYZRGB>};
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output{new pcl::PointCloud<pcl::PointXYZRGB>};
	pcl::ModelCoefficients::Ptr coefficients_ground{new pcl::ModelCoefficients};
	pcl::PointIndices::Ptr inliers_ground{new pcl::PointIndices()};
	pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients};
	pcl::PointIndices::Ptr inliers{new pcl::PointIndices()};
};

#endif