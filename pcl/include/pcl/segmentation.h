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

#include <math.h>
#include <vector>

#include <iostream>

using namespace std;

class Segmentation{
public:
	Segmentation(){
		pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
		marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
		//sub = nh.subscribe ("input", 1, cloud_cb);	//error: invalid use of non-static member function (why?)
		sub = nh.subscribe ("input", 1, &Segmentation::cloud_cb, this);		//success (c.f. the upper error)
		//bbox_sub = nh.subscribe ("darknet_ros/bounding_boxes", 100, bbox_cb);
		bbox_sub = nh.subscribe ("darknet_ros/bounding_boxes", 100, &Segmentation::bbox_cb, this);			
	}
	void bbox_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);     //bounding box callback
	void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);
	void visualize(int center_x, int center_y, int center_z);
	float vector_dot(vector<float> vec_a, vector<float> vec_b);
	float vector_length(vector<float> vec_a);

private:
	float x_max = 638.0;
	float x_min = 0.0;
	float y_max = 478.0;
	float y_min = 0.0;
	
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Publisher marker_pub;
	ros::Subscriber sub;
	ros::Subscriber bbox_sub;

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