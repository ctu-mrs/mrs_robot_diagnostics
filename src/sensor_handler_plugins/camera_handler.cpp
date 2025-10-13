
#include <mrs_robot_diagnostics/sensor_plugins/camera_handler.h>

namespace mrs_robot_diagnostics
{

namespace sensor_handlers
{

namespace camera_handler
{

bool CameraHandler::initialize(const ros::NodeHandle &nh, const std::string &name, const std::string &name_space) {
  ros::NodeHandle nh_(nh, name_space);
  _name_ = name;
  ros::Time::waitForValid();
  // Initialize camera handler
  ROS_INFO("Camera handler '%s' initialized in namespace '%s'", name.c_str(), name_space.c_str());
  is_initialized_ = true;
  return true;
}

void CameraHandler::updateStatus(mrs_robot_diagnostics::SensorStatus &ss_msg) {
  ss_msg.name = _name_;

  if (!is_initialized_) {
    ss_msg.ready  = false;
    ss_msg.rate   = -1;
    ss_msg.status = "NOT_INITIALIZED";
    return;
  }
}


} // namespace camera_handler
} // namespace sensor_handlers
} // namespace mrs_robot_diagnostics
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_robot_diagnostics::sensor_handlers::camera_handler::CameraHandler, mrs_robot_diagnostics::sensor_handlers::SensorHandler)
