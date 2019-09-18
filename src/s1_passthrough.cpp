#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Create the filtering object
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud (cloudPtr);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 2.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (cloud_filtered);
  
  //pcl::PCDWriter writer;
  //writer.write ("table_scene_lms400.pcd", *cloud, 
   //      Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
 // writer.write ("table_scene_lms400_downsampled.pcd", cloud_filtered, 
 //        Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 cloud_pt;
  pcl_conversions::moveFromPCL(cloud_filtered, cloud_pt);

  // Publish the data
  pub.publish (cloud_pt);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "PassThrough");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/cloud_input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_PassThrough", 1);

  // Spin
  ros::spin ();
}
