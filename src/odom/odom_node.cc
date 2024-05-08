

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

#include "dlio/odom.h"

int main(int argc, char** argv) {
  setlocale(LC_ALL, "");

  ros::init(argc, argv, "dlio_odom_node");
  ros::NodeHandle nh("~");

  dlio::OdomNode node(nh);

  //loop thread
  std::thread loop_thread(&dlio::OdomNode::performLoopThread, &node);
  loop_thread.detach();
  // 多线程回调，参数0表示使用尽可能多的线程来处理消息。如果参数为0，则ROS会根据CPU核心数量自动选择线程数，以实现最佳性能
  node.start();
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
  if(node.savefile)
  {
    //test to save time file
    // node.saveTimefile(node.time_odom,"/home/hhh/project_hhh/temp/slam/WORK/build/t_odom.txt");
    // node.saveTimefile(node.time_built_submap,"/home/hhh/project_hhh/temp/slam/WORK/build/t_submap.txt");
    // node.saveTimefile(node.time_total_time,"/home/hhh/project_hhh/temp/slam/WORK/build/t_total.txt");
  }
  return 0;

}
