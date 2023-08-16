#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/extract_indices.h>
 
ros::Publisher pub; // pubAvgDuration, pubAvgRate;
ros::Duration currentDuration(0), accumDuration(0);
ros::Time begin;
std::string inputTopic;
double radius, leafSize = 0.1;
std_msgs::Float64 averageDuration, averageRate;
int minNeighbours, noCloudsProcessed = 0;
sensor_msgs::PointCloud2 output;
using namespace std;
//passthrough filter

pcl::PCLPointCloud2::Ptr 
passthrough_filter (const pcl::PCLPointCloud2ConstPtr &input) {
	//pcl::PCLPointCloud2::Ptr  center(new pcl:pcl::PCLPointCloud2::Ptr );
	static pcl::PCLPointCloud2::Ptr  nearGround(new pcl::PCLPointCloud2);
	static pcl::PCLPointCloud2::Ptr  center(new pcl::PCLPointCloud2);
	static pcl::PCLPointCloud2::Ptr  outskirt(new pcl::PCLPointCloud2);

	pcl::PassThrough<pcl::PCLPointCloud2> zfilter;
	zfilter.setInputCloud(input);
	zfilter.setFilterFieldName("z");
	zfilter.setFilterLimits (-0.8, +1.5);
  	zfilter.filter (*nearGround);
	

	// pcl::PassThrough<pcl::PCLPointCloud2> xyfilter;
	// xyfilter.setInputCloud(nearGround);
	// xyfilter.setFilterFieldName("x");
	// xyfilter.setFilterLimits (-0.52, +0.52);
	// xyfilter.setFilterFieldName("y");
	// xyfilter.setFilterLimits (-0.51, +0.51);
	// //xyfilter.setNegative(true);
	// xyfilter.filter (*outskirt);
	pcl::CropBox<pcl::PCLPointCloud2> crop(false);
	crop.setInputCloud(nearGround);

	static double x_filter_min = -0.52;
	static double x_filter_max =  0.52;
	static double y_filter_min = -0.60;
	static double y_filter_max =  0.60;
	static double z_filter_min = -0.6;
	static double z_filter_max =  0.2;

	Eigen::Vector4f min_point = Eigen::Vector4f(x_filter_min, y_filter_min, z_filter_min, 0);
	Eigen::Vector4f max_point = Eigen::Vector4f(x_filter_max, y_filter_max, z_filter_max, 0);
	crop.setMin(min_point);
	crop.setMax(max_point);
	crop.setNegative(true);

	//crop.filter(*center);
	static vector<int> indices;
    crop.filter (*outskirt);

	// pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	// for (int i = 0; i < (*indices).size(); i++){
	// 	pcl::PCLPointCloud2 pt(nearGround->points[i])
	// 	inliers->indices.push_back(i);
	// }

	// pcl::ExtractIndices<pcl::PCLPointCloud2> extract;
	// extract.setInputCloud (nearGround);
	// extract.setIndices (inliers);
	// extract.setNegative (true);
    // extract.filter (*outskirt);


	// pcl::PassThrough<pcl::PCLPointCloud2> yfilter;
	// yfilter.setInputCloud(center);
	// yfilter.setFilterFieldName("y");
	// yfilter.setFilterLimits (-0.41, +0.41);
	// yfilter.setNegative(true);
	// yfilter.filter (*outskirt);
	return outskirt;

}

void colorize(const pcl::PointCloud<pcl::PointXYZ> &pc,
              pcl::PointCloud<pcl::PointXYZRGB> &pc_colored,
              const std::vector<int> &color) {

    int N = pc.points.size();

    pc_colored.clear();
    pcl::PointXYZRGB pt_tmp;
    for (int i = 0; i < N; ++i) {
        const auto &pt = pc.points[i];
        pt_tmp.x = pt.x;
        pt_tmp.y = pt.y;
        pt_tmp.z = pt.z;
        pt_tmp.r = color[0];
        pt_tmp.g = color[1];
        pt_tmp.b = color[2];
        pc_colored.points.emplace_back(pt_tmp);
    }
}


void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

	// Count number of point clouds processed
	noCloudsProcessed++;
	//Container for original & filtered data
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2* cloud_rfed;

	

	//pcl::PCLPointCloud2* cloud_vfed = new pcl::PCLPointCloud2; 
	//pcl::PCLPointCloud2::Ptr cloudPtr_vfed(cloud_vf);
	//copyPointCloud(cloud_vf, *cloudPtr_vfed);	
	pcl::PCLPointCloud2ConstPtr cloudPtr_vfed(new pcl::PCLPointCloud2);
	//pcl::PCLPointCloud2 cloud_rfed;
	
	// 	// Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);

	cloudPtr_vfed = passthrough_filter(cloudPtr);
	
	// 	 // Perform the voxel grid filtering
	// pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	// sor.setInputCloud (cloudPtr);
	// sor.setLeafSize (leafSize, leafSize, leafSize);
	// sor.filter (*cloud_vf);
	
	// Perform the radius outlier filtering
	// pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> outrem;
	// outrem.setInputCloud(cloudPtr_vfed);
	// outrem.setRadiusSearch(radius);
	// outrem.setMinNeighborsInRadius(minNeighbours);
	//  // apply filter
    // outrem.filter (cloud_rfed);

	
	// Convert to ROS data type
	
	pcl_conversions::fromPCL(*cloudPtr_vfed, output);
	
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

