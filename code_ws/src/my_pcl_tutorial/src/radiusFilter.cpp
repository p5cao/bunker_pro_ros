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
 
#include <pcl/filters/radius_outlier_removal.h>
 
ros::Publisher pub; // pubAvgDuration, pubAvgRate;
ros::Duration currentDuration(0), accumDuration(0);
ros::Time begin;
std::string inputTopic;
double radius, leafSize = 0.1;
std_msgs::Float64 averageDuration, averageRate;
int minNeighbours, noCloudsProcessed = 0;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

	// Count number of point clouds processed
	noCloudsProcessed++;
	//Container for original & filtered data
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered_1;
	pcl::PCLPointCloud2ConstPtr cloudPtr_1(cloud_filtered_1);
	pcl::PCLPointCloud2 cloud_filtered_2;
	
		// Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);
	
		 // Perform the voxel grid filtering
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloudPtr);
	sor.setLeafSize (leafSize, leafSize, leafSize);
	sor.filter (cloud_filtered_1);
	
	// Perform the radius outlier filtering
	pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> outrem;
	outrem.setInputCloud(cloudPtr_1);
	outrem.setRadiusSearch(radius);
	outrem.setMinNeighborsInRadius(minNeighbours);
	 // apply filter
        outrem.filter (cloud_filtered_2);

	

	
	// Convert to ROS data type
	sensor_msgs::PointCloud2 output;
	pcl_conversions::fromPCL(cloud_filtered_2, output);
	
	// Publish the data
	pub.publish (output);
}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "radiusFilter");
  ros::NodeHandle nh;
  std::cout << std::endl;
  ROS_INFO("Radius Outlier Removal Node Initialize");

  // Get parameters from ROS parameter server
  ros::param::get("radius/inputTopic", inputTopic);
  ros::param::get("radius/radius_search", radius);
  ros::param::get("radius/minNeighbours", minNeighbours);
  ROS_INFO("The input topic is %s" , inputTopic.c_str());
  ROS_INFO("Radius search dimension is set to: %.2f", radius);
  ROS_INFO("Minimum neighbours required in each search radius is set to: %d", minNeighbours);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe (inputTopic, 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("radius/output", 1);
  //pubAvgDuration = nh.advertise<std_msgs::Float64> ("/radius/AverageProcessTime", 1);
  //pubAvgRate = nh.advertise<std_msgs::Float64> ("/radius/AverageProcessRate", 1);

  // Spin
  ros::spin ();
}

