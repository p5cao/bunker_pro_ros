#include <ros/ros.h>
#include <iostream>
#include <vector>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
 
ros::Publisher pub;
ros::Duration currentDuration(0), accumDuration(0);
ros::Time begin;
std::string inputTopic;
double radius, leafSize = 0.1;
std_msgs::Float64 averageDuration, averageRate;
int minNeighbours, noCloudsProcessed = 0;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	//Container for original & filtered data
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;
	
	// Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);
	
	// Perform the actual filtering
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloudPtr);
	sor.setLeafSize (leafSize, leafSize, leafSize);
	sor.filter (cloud_filtered);
	
	// Convert to ROS data type
	sensor_msgs::PointCloud2 output;
	pcl_conversions::fromPCL(cloud_filtered, output);
	
	// Publish the data
	pub.publish (output);
}

int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "voxelGridFilter");
	ros::NodeHandle nh;
  	std::cout << std::endl;
  	ROS_INFO("My PCL tutorial: voxelGridFilter Node Initialize");
  	
  	
  	  // Get parameters from ROS parameter server
  	
  	ros::param::get("voxel/inputTopic", inputTopic);
  	ros::param::get("voxel/leafSize", leafSize);
  	//ros::param::get("ouster/points", inputTopic);
  	//ros::param::get("voxel/radius_search", radius);
  	//ros::param::get("voxel/minNeighbours", minNeighbours);
  	ROS_INFO("The input topic is %s" , inputTopic.c_str());
  	
  	// Create a ROS subscriber for the input point cloud
  	ros::Subscriber sub = nh.subscribe (inputTopic, 1, cloud_cb);
  	
  	// Create a ROS publisher for the output point cloud
  	pub = nh.advertise<sensor_msgs::PointCloud2> ("voxel/output", 1);
  	
  	// Spin
  	ros::spin();
}





