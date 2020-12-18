#include <ros/ros.h>

#include "latency_bench/Stamp.h"

namespace latency_bench
{
class SourcePub
{
public:
  explicit SourcePub(ros::NodeHandle & pnh);
  ~SourcePub() = default;

private:
  void callback(const ros::TimerEvent &);

  ros::Timer timer_;
  ros::Publisher pub_;
};

SourcePub::SourcePub(ros::NodeHandle & pnh) : pub_(pnh.advertise<Stamp>("output", 10, true))
{
  double frequency = 10.;
  pnh.getParam("frequency", frequency);
  ROS_ASSERT(1. <= frequency && frequency <= 1000.);

  timer_ = pnh.createTimer(ros::Duration(1. / frequency), &SourcePub::callback, this);
}

void SourcePub::callback(const ros::TimerEvent &)
{
  const auto now = ros::Time::now();
  Stamp out;
  out.stamp = now;
  pub_.publish(out);
}

}  // namespace latency_bench

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "source_pub");
  ros::NodeHandle pnh("~");

  latency_bench::SourcePub node(pnh);
  ros::spin();
}
