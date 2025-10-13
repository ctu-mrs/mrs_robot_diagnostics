#pragma once
#include <ros/ros.h>
#include <mrs_robot_diagnostics/SensorStatus.h>


namespace mrs_robot_diagnostics
{

namespace sensor_handlers
{

class SensorHandler {
public:
  virtual bool initialize(const ros::NodeHandle &nh, const std::string &name, const std::string &name_space) = 0;

  virtual void updateStatus(mrs_robot_diagnostics::SensorStatus &ss_msg) = 0;

  virtual ~SensorHandler() = default;

};

} // namespace sensor_handlers
} // namespace mrs_robot_diagnostics
