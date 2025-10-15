#include <mrs_robot_diagnostics/sensor_handler.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <mrs_robot_diagnostics/SensorInfo.h>
#include <std_msgs/Float32MultiArray.h>


namespace mrs_robot_diagnostics
{

namespace sensor_handlers
{

namespace camera_handler
{

class CameraHandler : public mrs_robot_diagnostics::sensor_handlers::SensorHandler {
public:
  bool initialize(ros::NodeHandle &nh, const std::string &name, const std::string &name_space, const std::string &topic) override;
  void updateStatus(mrs_robot_diagnostics::SensorStatus &ss_msg) override;


private:
  std::string _name_;
  std::string _topic_;
  bool is_initialized_ = false;
  bool is_active_      = false;


  mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo> sh_camera_info_;
  mrs_lib::SubscribeHandler<sensor_msgs::Image> sh_image_;
  mrs_lib::SubscribeHandler<std_msgs::Float32MultiArray> sh_camera_orientation_;
  mrs_lib::PublisherHandler<mrs_robot_diagnostics::SensorInfo> ph_camera_details_;

};

} // namespace camera_handler
} // namespace sensor_handlers
} // namespace mrs_robot_diagnostics
