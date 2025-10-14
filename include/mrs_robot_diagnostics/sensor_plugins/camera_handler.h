#include <mrs_robot_diagnostics/sensor_handler.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace mrs_robot_diagnostics
{

namespace sensor_handlers
{

namespace camera_handler
{

class CameraHandler : public mrs_robot_diagnostics::sensor_handlers::SensorHandler {
public:
  bool initialize(const ros::NodeHandle &nh, const std::string &name, const std::string &name_space, const std::string &topic) override;
  void updateStatus(mrs_robot_diagnostics::SensorStatus &ss_msg) override;


private:
  std::string _name_;
  std::string _topic_;
  bool is_initialized_ = false;
  bool is_active_      = false;


  mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo> sh_camera_info_;
  mrs_lib::SubscribeHandler<sensor_msgs::Image> sh_image_;
};

} // namespace camera_handler
} // namespace sensor_handlers
} // namespace mrs_robot_diagnostics
