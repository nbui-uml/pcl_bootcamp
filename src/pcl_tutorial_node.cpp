#include "ros/node_handle.h"
#include "ros/param.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/subscriber.h"
#include "sensor_msgs/PointCloud2.h"
#include <algorithm>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// Get point cloud
// Convert to PCL
// Publish it

std::string camera_topic;
std::string output_topic;
ros::Publisher pub;
float voxel_leaf_size;
float segmenter_distance_threshold;
int segmenter_min_cluster_size;
float passthrough_filter_x_min;
float passthrough_filter_x_max;
float passthrough_filter_z_min;
float passthrough_filter_z_max;

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rawcloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedcloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*msg, *rawcloud);

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(rawcloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(passthrough_filter_z_min, passthrough_filter_z_max);
  pass.filter(*transformedcloud);

  pass.setInputCloud(transformedcloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(passthrough_filter_x_min, passthrough_filter_x_max);
  pass.filter(*transformedcloud);

  pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::IndicesPtr indices(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*transformedcloud, *indices);

  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud(transformedcloud);
  reg.setIndices(indices);
  reg.setSearchMethod(tree);
  reg.setDistanceThreshold(segmenter_distance_threshold);
  reg.setPointColorThreshold(6);
  reg.setRegionColorThreshold(5);
  reg.setMinClusterSize(segmenter_min_cluster_size);

  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();

  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud(colored_cloud);
  sor.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  sor.filter(*transformedcloud);

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*transformedcloud, output);
  output.header.frame_id = msg->header.frame_id;

  pub.publish(output);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "PCL_tutorial");
  ros::NodeHandle nh;
  ros::param::param<std::string>("camera_topic", camera_topic,
                                 "camera/depth/points");
  ros::param::param<std::string>("pcl_output_topic", output_topic,
                                 "pcl_tutorial_output_topic");
  ros::Subscriber sub =
      nh.subscribe<sensor_msgs::PointCloud2>(camera_topic, 1, cloudCallback);
  pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 1);
  ros::Rate rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
    ros::param::param<float>("voxel_leaf_size_param", voxel_leaf_size, 0.05);
    ros::param::param<float>("segmenter_distance_threshold_param",
                           segmenter_distance_threshold, 10);
    ros::param::param<int>("segmenter_min_clustersize_param",
                           segmenter_min_cluster_size, 600);
    ros::param::param<float>("passthrough_filter_z_min_param",
                             passthrough_filter_z_min, 0.0);
    ros::param::param<float>("passthrough_filter_z_max_param",
                             passthrough_filter_z_max, 1.0);
    ros::param::param<float>("passthrough_filter_x_min_param",
                             passthrough_filter_x_min, 0.0);
    ros::param::param<float>("passthrough_filter_x_max_param",
                             passthrough_filter_x_max, 1.0);
  }

  return 0;
}