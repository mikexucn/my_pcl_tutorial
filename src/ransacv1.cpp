#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  /*// Container for original & filtered data
  pcl::PCLPointCloud2 cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
*/
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
  //pcl::PointCloud<PCLPointCloud2>::Ptr cloudPtr(cloud);

  //pcl::PointCloud<PCLPointCloud2>::Ptr cloud_filtered (new pcl::PointCloud<PCLPointCloud2>);

  // Convert to PCL data type
  pcl::fromROSMsg(*cloud_msg, cloud);

  std::vector<int> inliers;
  // created RandomSampleConsensus object and compute the appropriated model
  //pcl::SampleConsensusModelPlane<pcl::PointXYZ> model_p;
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (cloud);
  //ransac.setDistanceThreshold (0.05);
  //ransac.computeModel();
  //ransac.getInliers(inliers);
  // copies all inliers of the model computed to another PointCloud
  //pcl::copyPointCloud<pcl::PointXYZ>(cloud, inliers, cloud_filtered);

  // Convert to ROS data type
  //sensor_msgs::PointCloud2 cloud_ran;
  //pcl_conversions::moveFromPCL(cloud_filtered, cloud_ran);

  // Publish the data
  //pub.publish (cloud_ran);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "ransac1");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/cloud_rads", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_ran", 1);

  // Spin
  ros::spin ();
}
