#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
//passthrough 滤波
#include <pcl/filters/passthrough.h>
//voxel_grid 体素滤波
#include <pcl/filters/voxel_grid.h>
//半径滤波，去除噪点
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
	//pcl::PointCloud<pcl::PointXYZ> cloud;
  
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2* cloud_filtered_pass = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2* cloud_filtered_voxel = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2 cloud_filtered_radius;
  // Convert to PCL data type
	//pcl::fromROSMsg (*input, cloud);
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Create the filtering object
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud (cloudPtr);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 2.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered_pass);
  
	*cloud = *cloud_filtered_pass;

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.02, 0.02, 0.02);
  sor.filter (*cloud_filtered_voxel);

	*cloud = *cloud_filtered_voxel;

  // Perform the actual filtering
  pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> outrem;
    // build the filter
  outrem.setInputCloud(cloudPtr);
  outrem.setRadiusSearch(0.08);
  outrem.setMinNeighborsInRadius (50);
    // apply filter
  outrem.filter (cloud_filtered_radius);
  // Convert to ROS data type
  sensor_msgs::PointCloud2 cloud_radius;
  pcl_conversions::moveFromPCL(cloud_filtered_radius, cloud_radius);
  // Publish the data
  pub.publish (cloud_radius);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "points_processed");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect2/sd/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/points_processed", 1);

  // Spin
  ros::spin ();
}
