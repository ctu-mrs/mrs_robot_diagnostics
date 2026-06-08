#include <mrs_robot_diagnostics/sensor_plugins/gnss_handler.h>

namespace mrs_robot_diagnostics
{
namespace gnss_handler
{

bool GNSSSensorHandler::onInitialize(rclcpp::Node::SharedPtr &node, [[maybe_unused]] const std::string &config_key,
                                     [[maybe_unused]] const std::string &name_space, [[maybe_unused]] rclcpp::CallbackGroup::SharedPtr cbkgrp_subs) {

  mrs_lib::ParamLoader param_loader(node, "GNSSSensorHandler");

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path, std::string(""));
  if (!custom_config_path.empty()) {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("config");
  param_loader.setPrefix("robot_diagnostics/sensor_handlers/");

  // Read GNSSSensorHandler-specific params
  std::string status_topic;
  param_loader.loadParam(config_key + "/status_topic", status_topic);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node->get_logger(), "[GNSSSensorHandler] Failed to load config for '%s', not initializing", config_key.c_str());
    return false;
  }

  RCLCPP_INFO(node->get_logger(), "[GNSSSensorHandler] Initializing '%s', topic: '%s'", name_.c_str(), topic_.c_str());
  RCLCPP_INFO(node->get_logger(), "[GNSSSensorHandler] Initializing '%s', topic: '%s'", name_.c_str(), status_topic.c_str());

  // Create subscriber handlers for GNSS data and status
  sh_gnns_        = create_main_subscriber<sensor_msgs::msg::NavSatFix>(node, topic_, cbkgrp_subs);
  sh_gnss_status_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::GpsInfo>(shopts_, status_topic);
  return true;
}

std::vector<diagnostic_msgs::msg::KeyValue> GNSSSensorHandler::fill_details() {

  std::vector<diagnostic_msgs::msg::KeyValue> details;

  auto gnss_msg        = sh_gnns_.getMsg();
  auto gnss_status_msg = sh_gnss_status_.getMsg();

  if (!gnss_msg) {
    // Initialize with default values if no GNSS data has been received yet
    diagnostic_msgs::msg::KeyValue info;
    info.key   = "uncertainty";
    info.value = "nan";
    details.push_back(info);
    info.key   = "quality";
    info.value = "nan";
    details.push_back(info);
  } else {
    diagnostic_msgs::msg::KeyValue info;
    info.key                  = "uncertainty";
    const Eigen::Matrix3d cov = cov2eigen(gnss_msg->position_covariance);
    info.value                = std::to_string(std::cbrt(cov.determinant()));
    details.push_back(info);
    info.key         = "quality";
    double gnss_qual = (gnss_msg->position_covariance[0] + gnss_msg->position_covariance[4] + gnss_msg->position_covariance[8]) / 3;
    info.value       = std::to_string(gnss_qual);
    details.push_back(info);
  }

  if (!gnss_status_msg) {
    diagnostic_msgs::msg::KeyValue info;
    info.key   = "fix_type";
    info.value = "nan";
    details.push_back(info);
    info.key   = "num_satellites";
    info.value = "nan";
    details.push_back(info);
    info.key   = "position_accuracy";
    info.value = "nan";
    details.push_back(info);
  } else {
    diagnostic_msgs::msg::KeyValue info;
    info.key   = "fix_type";
    info.value = std::to_string(gnss_status_msg->fix_type);
    details.push_back(info);
    info.key   = "num_satellites";
    info.value = std::to_string(gnss_status_msg->satellites_visible);
    details.push_back(info);
    // Position accuracy
    info.key                 = "position_accuracy";
    double position_accuracy = (gnss_status_msg->h_acc + gnss_status_msg->v_acc) / 2.0;
    info.value               = std::to_string(position_accuracy);
    details.push_back(info);
  }

  return details;
}

} // namespace gnss_handler
} // namespace mrs_robot_diagnostics

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_robot_diagnostics::gnss_handler::GNSSSensorHandler, mrs_robot_diagnostics::SensorHandler)
