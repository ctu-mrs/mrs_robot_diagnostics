
#include <mrs_robot_diagnostics/sensor_plugins/camera_handler.h>

namespace mrs_robot_diagnostics
{

namespace sensor_handlers
{

namespace camera_handler
{

bool CameraHandler::initialize(const ros::NodeHandle &nh, const std::string &name, const std::string &name_space, const std::string &topic) {
  ros::NodeHandle nh_(nh, name_space);
  _name_  = name;
  _topic_ = topic;
  ros::Time::waitForValid();


  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "StateMonitor";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  // TODO to remap of take a new parameter
  const std::string image_topic_name = "/" + name_space + topic + "image_raw";
  sh_image_ = createSubscriber<sensor_msgs::Image>(nh_, image_topic_name);

  // Don't use rate calculation helper for camera info
  const std::string camera_info_topic_name = "/" + name_space + topic + "camera_info";
  sh_camera_info_ = mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo>(shopts, camera_info_topic_name);

  
  ROS_INFO("Camera handler '%s' initialized in namespace '%s'", name.c_str(), name_space.c_str());
  is_initialized_ = true;
  return true;
}

void CameraHandler::updateStatus(mrs_robot_diagnostics::SensorStatus &ss_msg) {
  ss_msg.name = _name_;
  ss_msg.type = mrs_robot_diagnostics::SensorStatus::TYPE_CAMERA;

  if (!is_initialized_) {
    ss_msg.ready  = false;
    ss_msg.rate   = -1;
    ss_msg.status = "NOT_INITIALIZED";
    return;
  }

  if (sh_camera_info_.hasMsg()) {

    ss_msg.rate = current_rate_; 

    bool has_image = sh_image_.hasMsg();
    ss_msg.ready   = has_image;

    if (has_image) {
      ss_msg.status = "OK";
    } else {
      ss_msg.status = "NO_IMAGE_DATA";
    }
  } else {
    ss_msg.ready  = false;
    ss_msg.rate   = -1;
    ss_msg.status = "NO_CAMERA_INFO";
  }
}


} // namespace camera_handler
} // namespace sensor_handlers
} // namespace mrs_robot_diagnostics
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_robot_diagnostics::sensor_handlers::camera_handler::CameraHandler, mrs_robot_diagnostics::sensor_handlers::SensorHandler)
