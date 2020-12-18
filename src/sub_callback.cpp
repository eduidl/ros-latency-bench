#include <ros/ros.h>

#include "sub_node_common.hpp"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "sub_callback");
  ros::NodeHandle pnh("~");

  latency_bench::SubNodeCommon node(pnh, false, true);
  ros::spin();
}
