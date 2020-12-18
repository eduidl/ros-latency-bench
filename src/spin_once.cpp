#include <ros/ros.h>

#include "sub_node_common.hpp"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "spin_once");
  ros::NodeHandle pnh("~");

  latency_bench::SubNodeCommon node(pnh, false, false);

  double frequency = 10.;
  pnh.getParam("frequency", frequency);
  ROS_ASSERT(1. <= frequency && frequency <= 1000.);

  ros::Rate rate(frequency);
  while (ros::ok()) {
    ros::spinOnce();
    node.publish_msg();
    // ros::spinOnce();
    rate.sleep();
  }
}
