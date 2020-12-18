#pragma once

#include <ros/ros.h>

#include <memory>

#include "dummy_process.hpp"
#include "latency_bench/Stamp.h"
#include "latency_bench/TwoStamps.h"

namespace latency_bench
{
class SubNodeCommon
{
public:
  SubNodeCommon(ros::NodeHandle & pnh, bool use_timer, bool pub_in_sub_callback);
  ~SubNodeCommon() = default;

  void publish_msg();

private:
  void sub_callback(const StampConstPtr & msg);
  void timer_callback(const ros::TimerEvent &);

  std::unique_ptr<DummyProcess> process_;
  bool use_timer_;
  bool pub_in_sub_callback_;
  ros::Timer timer_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  StampConstPtr latest_;
};

SubNodeCommon::SubNodeCommon(ros::NodeHandle & pnh, bool use_timer, bool pub_in_sub_callback)
: process_(nullptr),
  use_timer_(use_timer),
  pub_in_sub_callback_(pub_in_sub_callback),
  pub_(pnh.advertise<TwoStamps>("output", 10)),
  latest_(nullptr)
{
  sub_ = pnh.subscribe("input", 10, &SubNodeCommon::sub_callback, this);

  double sleep_ms = 0.;
  pnh.getParam("sleep_ms", sleep_ms);
  process_.reset(new DummyProcess(static_cast<int>(sleep_ms * 1000)));

  if (use_timer_) {
    double frequency = 10.;
    pnh.getParam("frequency", frequency);
    ROS_ASSERT(1. <= frequency && frequency <= 1000.);
    timer_ = pnh.createTimer(ros::Duration(1. / frequency), &SubNodeCommon::timer_callback, this);
  }
}

void SubNodeCommon::publish_msg()
{
  if (!latest_) {
    ROS_DEBUG("latest_ is empty");
    return;
  }
  ROS_DEBUG_STREAM("publish_msg() is called: " << latest_->stamp);
  process_->execute();

  const auto now = ros::Time::now();
  TwoStamps out;
  out.sub_stamp = latest_->stamp;
  out.pub_stamp = now;
  pub_.publish(out);
}

void SubNodeCommon::sub_callback(const StampConstPtr & msg)
{
  ROS_DEBUG_STREAM("sub_callback() is called: " << msg->stamp);
  latest_ = msg;
  if (pub_in_sub_callback_) {
    publish_msg();
  }
}

void SubNodeCommon::timer_callback(const ros::TimerEvent &) { publish_msg(); }

}  // namespace latency_bench
