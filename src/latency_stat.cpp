#include <ros/ros.h>

#include <algorithm>
#include <limits>
#include <string>

#include "latency_bench/TwoStamps.h"

namespace latency_bench
{
class Stat
{
public:
  Stat(const std::string & label)
  : label_(label),
    count_(0),
    max_(0.),
    min_(std::numeric_limits<double>::max()),
    sum_(0.),
    square_sum_(0.)
  {
  }
  ~Stat() = default;

  void push(double val);

private:
  std::string label_;
  uint32_t count_;
  double max_;
  double min_;
  double sum_;
  double square_sum_;
};

void Stat::push(double val)
{
  count_++;
  max_ = std::max(max_, val);
  min_ = std::min(min_, val);
  sum_ += val;
  square_sum_ += val * val;

  if (count_ % 100 == 0) {
    ROS_INFO_STREAM(
      "[" << label_ << "] n_samples: " << count_ << ", max: " << max_ << ", min: " << min_
          << ", mean: " << sum_ / count_ << ", var: " << square_sum_ / count_);
  }
}

class LatencyStatCalc
{
public:
  explicit LatencyStatCalc(ros::NodeHandle & pnh);
  ~LatencyStatCalc() = default;

private:
  void callback(const TwoStampsConstPtr & msg);

  Stat pub_delay_stat_;
  Stat total_delay_stat_;
  ros::Subscriber sub_;
  int skip_n_frames_;
};

LatencyStatCalc::LatencyStatCalc(ros::NodeHandle & pnh)
: pub_delay_stat_("pub_delay"), total_delay_stat_("total_delay"), skip_n_frames_(10)
{
  pnh.getParam("skip_n_frames", skip_n_frames_);
  sub_ = pnh.subscribe("input", 10, &LatencyStatCalc::callback, this);
}

void LatencyStatCalc::callback(const TwoStampsConstPtr & msg)
{
  const auto now = ros::Time::now();
  if (skip_n_frames_ > 0) {
    skip_n_frames_--;
    return;
  }

  const auto delay = (msg->pub_stamp - msg->sub_stamp).toSec();
  pub_delay_stat_.push(delay * 1000);
  const auto total_delay = (now - msg->sub_stamp).toSec();
  total_delay_stat_.push(total_delay * 1000);
}

}  // namespace latency_bench

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "latency_stat");
  ros::NodeHandle pnh("~");

  latency_bench::LatencyStatCalc node(pnh);
  ros::spin();
}
