#include <mrs_robot_diagnostics/sensor_handler.h>

namespace mrs_robot_diagnostics
{

namespace sensor_handlers
{

namespace camera_handler
{

class CameraHandler : public mrs_robot_diagnostics::sensor_handlers::SensorHandler {
public:
  bool initialize(const ros::NodeHandle &nh, const std::string &name, const std::string &name_space) override;
  void updateStatus(mrs_robot_diagnostics::SensorStatus &ss_msg) override;


private:
  std::string _name_;
  bool is_initialized_ = false;
  bool is_active_      = false;
};

} // namespace camera_handler
} // namespace sensor_handlers
} // namespace mrs_robot_diagnostics
