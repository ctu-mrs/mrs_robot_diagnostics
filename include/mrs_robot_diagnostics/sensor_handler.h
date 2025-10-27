#pragma once
#include <ros/ros.h>
#include <mrs_robot_diagnostics/SensorStatus.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <nlohmann/json.hpp>
// #include <iroc_fleet_manager/utils/json_var_parser.h>

namespace mrs_robot_diagnostics
{

namespace sensor_handlers
{

using json = nlohmann::json;

class SensorHandler {
public:
  virtual bool initialize(ros::NodeHandle &nh, const std::string &name, const std::string &name_space, const std::string &topic) = 0;

  virtual void updateStatus(mrs_robot_diagnostics::SensorStatus &ss_msg) = 0;

  virtual ~SensorHandler() = default;

protected:
  ros::Time last_msg_time_;
  double current_rate_;

  double calculateRate(const ros::Time &current_msg_time) {
    if (last_msg_time_.isZero()) {
      last_msg_time_ = current_msg_time;
      return -1.0;
    }

    double dt      = (current_msg_time - last_msg_time_).toSec();
    last_msg_time_ = current_msg_time;

    return (dt > 0.0) ? (1.0 / dt) : -1.0;
  }

  // Helper function to create a subscriber with rate calculation of the main topic
  template <typename MessageType>
  mrs_lib::SubscribeHandler<MessageType> createSubscriber(ros::NodeHandle &nh, const std::string &topic_name,
                                                          const ros::Duration &timeout = mrs_lib::no_timeout) {

    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh                 = nh;
    shopts.node_name          = "StateMonitor";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 10;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    auto callback = [this](const typename MessageType::ConstPtr& msg) {
      current_rate_ = calculateRate(msg->header.stamp);
    };

    return mrs_lib::SubscribeHandler<MessageType>(shopts, topic_name, callback);
  }
};

} // namespace sensor_handlers
} // namespace mrs_robot_diagnostics
