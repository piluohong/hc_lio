/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/
/*
    @brief  发布和保存稀疏的全局点云地图
    @addindex 增加发布增量八叉树地图
*/
#include "dlio/map.h"
#include <omp.h>

dlio::MapNode::MapNode(ros::NodeHandle node_handle) : nh(node_handle) {

  this->getParams();

  this->publish_timer = this->nh.createTimer(ros::Duration(this->publish_freq_), &dlio::MapNode::publishTimer, this);
  this->map_server = this->nh.advertiseService("/map_save", &dlio::MapNode::serviceCallback, this);
  this->keyframe_sub = this->nh.subscribe("keyframes", 100,
      &dlio::MapNode::callbackKeyframe, this, ros::TransportHints().tcpNoDelay());
  this->map_pub = this->nh.advertise<sensor_msgs::PointCloud2>("map", 100);
  this->octomap_pub = this->nh.advertise<octomap_msgs::Octomap>("octomap", 100);
  this->save_pcd_srv = this->nh.advertiseService("save_pcd", &dlio::MapNode::savePcd, this);

  this->dlio_map = pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

}

dlio::MapNode::~MapNode() {}

void dlio::MapNode::getParams() {

  ros::param::param<std::string>("~dlio/odom/odom_frame", this->odom_frame, "map");
  ros::param::param<double>("~dlio/map/sparse/frequency", this->publish_freq_, 1.0);
  ros::param::param<double>("~dlio/map/sparse/leafSize", this->leaf_size_, 0.25);

  // Get Node NS and Remove Leading Character
  std::string ns = ros::this_node::getNamespace();
  ns.erase(0,1);

  // Concatenate Frame Name Strings
  this->odom_frame = ns + "/" + this->odom_frame;

}

void dlio::MapNode::start() {
}

bool dlio::MapNode::serviceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  pcl::VoxelGrid<PointType> downSizeFilterMap;
  downSizeFilterMap.setLeafSize(0.3, 0.3, 0.3);
  downSizeFilterMap.setInputCloud(this->dlio_map);
  downSizeFilterMap.filter(*this->dlio_map);

  std::string file_name = std::string("scans.pcd");
  std::string all_points_dir("/home/hong/DLIO_PCD/" + file_name);
  pcl::PCDWriter pcd_writer;
  ROS_INFO("current scan saved to /PCD/%c", file_name.c_str());
  pcd_writer.writeBinary(all_points_dir, *this->dlio_map);
  ROS_INFO("successfully saved");
  return true;

}

void dlio::MapNode::publishTimer(const ros::TimerEvent& e) {

  if (this->dlio_map->points.size() == this->dlio_map->width * this->dlio_map->height) {
    sensor_msgs::PointCloud2 map_ros;
    pcl::toROSMsg(*this->dlio_map, map_ros);
    map_ros.header.stamp = ros::Time::now();
    map_ros.header.frame_id = this->odom_frame;
    this->map_pub.publish(map_ros);
  }

}

void dlio::MapNode::callbackKeyframe(const sensor_msgs::PointCloud2ConstPtr& keyframe) {

  // convert scan to pcl format
  pcl::PointCloud<PointType>::Ptr keyframe_pcl =
    pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());
  pcl::fromROSMsg(*keyframe, *keyframe_pcl);
  this->visualizeOctomap(keyframe_pcl);
  // voxel filter
  // this->voxelgrid.setLeafSize(this->leaf_size_, this->leaf_size_, this->leaf_size_);
  // this->voxelgrid.setInputCloud(keyframe_pcl);
  // this->voxelgrid.filter(*keyframe_pcl);

  // // save filtered keyframe to map for rviz
  // *this->dlio_map += *keyframe_pcl;

}
void dlio::MapNode::visualizeOctomap(pcl::PointCloud<PointType>::Ptr& global_cloud)
{
  auto temcloud = global_cloud;
  static octomap::OcTree* octree = new octomap::OcTree(0.4);
  for(const auto& point : temcloud->points)
  {
    octree->updateNode(octomap::point3d(point.x,point.y,point.z),true);
  }
  octree->updateInnerOccupancy();

  octomap_msgs::Octomap octomap_msg;
  
  octomap_msg.header.frame_id = this->odom_frame;
  octomap_msg.header.stamp = ros::Time::now();
  octomap_msgs::fullMapToMsg(*octree,octomap_msg);

  this->octomap_pub.publish(octomap_msg);
}

bool dlio::MapNode::savePcd(direct_lidar_inertial_odometry::save_pcd::Request& req,
                            direct_lidar_inertial_odometry::save_pcd::Response& res) {

  pcl::PointCloud<PointType>::Ptr m =
    pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>(*this->dlio_map));

  float leaf_size = req.leaf_size;
  std::string p = req.save_path;

  std::cout << std::setprecision(2) << "Saving map to " << p + "/dlio_map.pcd"
    << " with leaf size " << to_string_with_precision(leaf_size, 2) << "... "; std::cout.flush();

  // voxelize map
  pcl::VoxelGrid<PointType> vg;
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.setInputCloud(m);
  vg.filter(*m);

  // save map
  int ret = pcl::io::savePCDFileBinary(p + "/dlio_map.pcd", *m);
  res.success = ret == 0;

  if (res.success) {
    std::cout << "done" << std::endl;
  } else {
    std::cout << "failed" << std::endl;
  }

  return res.success;

}

