/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>

#include <mrs_msgs/HwApiStatus.h>
#include "mrs_msgs/ControlManagerDiagnostics.h"


//}

namespace mrs_robot_diagnostics
{

  /* enums //{ */
  typedef enum {
    UNKNOWN,
    DISARMED,
    ARMED,
    TAKEOFF,
    LANDING,
    MANUAL,
    AUTONOMOUS
  } UavState_t;

  typedef enum {
    NULL_TRACKER,
    LANDOFF_TRACKER,
    AUTO_TRACKER
  } TrackerState_t;

  namespace states
  {
    // clang-format off
    const std::vector<std::string> uav_state_names = {
      "UNKNOWN",
      "DISARMED",
      "ARMED",
      "TAKEOFF",
      "LANDING",
      "MANUAL",
      "AUTONOMOUS"
    };

    const std::vector<std::string> tracker_state_names = {
      "NULL_TRACKER",
      "LANDOFF_TRACKER",
      "AUTO_TRACKER"
    };
    // clang-format on
  } // namespace states

  /*//}*/

/* class StateMonitor //{ */

class StateMonitor : public nodelet::Nodelet {
public:
  virtual void onInit();

private:
  ros::NodeHandle   nh_;
  std::atomic<bool> is_initialized_ = false;

  ros::Time last_update_time_;

  UavState_t uav_state_ = UavState_t::UNKNOWN;
  TrackerState_t tracker_state_ = TrackerState_t::NULL_TRACKER;

  // | ---------------------- ROS subscribers --------------------- |
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus> sh_hw_api_status_;
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diagnostics_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent &event);
  double     _main_timer_rate_;

  // | ------------------ Additional functions ------------------ |

  void parseHwApiStatus(const mrs_msgs::HwApiStatus::ConstPtr &hw_api_status);
  void parseControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnostics::ConstPtr &control_manager_diagnostics);
  void updateUavState(const UavState_t &new_state);
  void updateTrackerState(const TrackerState_t &new_state);
};
//}

/* onInit() //{ */

void StateMonitor::onInit() {

  /* obtain node handle */
  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();
  
  /* load parameters */
  mrs_lib::ParamLoader param_loader(nh_, "StateMonitor");

  std::string custom_config_path;

  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("config");

  param_loader.loadParam("main_timer_rate", _main_timer_rate_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[StateMonitor]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "StateMonitor";
  shopts.no_message_timeout = ros::Duration(1.0);
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_hw_api_status_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus>(shopts, "hw_api_status_in");
  sh_control_manager_diagnostics_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diagnostics_in");

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(_main_timer_rate_), &StateMonitor::timerMain, this);

  // | --------------------- finish the init -------------------- |

  ROS_INFO("[StateMonitor]: initialized");
  ROS_INFO("[StateMonitor]: --------------------");
  is_initialized_ = true;
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void StateMonitor::timerMain([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  bool got_hw_api_status = sh_hw_api_status_.newMsg();
  bool got_control_manager_diagnostics = sh_control_manager_diagnostics_.newMsg();

  if (got_hw_api_status){
    auto hw_api_status = sh_hw_api_status_.getMsg();
    parseHwApiStatus(hw_api_status);
  }

  if (got_control_manager_diagnostics){
    auto control_manager_diagnostics = sh_control_manager_diagnostics_.getMsg();
    parseControlManagerDiagnostics(control_manager_diagnostics);
  }

  /* TODO: add more messages types */
}

//}

// | -------------------- support functions ------------------- |

/* parseHwApiStatus() //{ */

void StateMonitor::parseHwApiStatus(const mrs_msgs::HwApiStatus::ConstPtr &hw_api_status) {
  if (!hw_api_status->armed){
    updateUavState(DISARMED);
  } else if (uav_state_ == DISARMED || hw_api_status->mode == "AUTO.LOITER" || tracker_state_ == NULL_TRACKER){
    updateUavState(ARMED);
  }
}

//}

/* parseControlManagerDiagnostics() //{ */

void StateMonitor::parseControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnostics::ConstPtr &control_manager_diagnostics) {
  if (control_manager_diagnostics->active_tracker == "LandoffTracker"){
    updateTrackerState(LANDOFF_TRACKER);

    if (uav_state_ == ARMED){
      updateUavState(TAKEOFF);
    } else if (uav_state_ != TAKEOFF){
      updateUavState(LANDING);
    }
  } else if (control_manager_diagnostics->active_tracker == "NullTracker" && uav_state_ == LANDING){
    updateTrackerState(NULL_TRACKER);
  } else if (control_manager_diagnostics->joystick_active){
    updateUavState(MANUAL);
  } else if (control_manager_diagnostics->flying_normally){
    updateTrackerState(AUTO_TRACKER);

    updateUavState(AUTONOMOUS);
  }
}

//}

/* updateUavState() //{ */

void StateMonitor::updateUavState(const UavState_t &new_state) {

  if (uav_state_ == new_state){
    return;
  }

  ROS_INFO("[StateMonitor]: SWITCHING UAV STATE: \"%s\" => \"%s\"", states::uav_state_names[uav_state_].c_str(), states::uav_state_names[new_state].c_str());
  uav_state_ = new_state;
}

//}

/* updateTrackerState() //{ */

void StateMonitor::updateTrackerState(const TrackerState_t &new_state) {

  if (tracker_state_ == new_state){
    return;
  }

  ROS_INFO("[StateMonitor]: SWITCHING TRACKER STATE: \"%s\" => \"%s\"", states::tracker_state_names[tracker_state_].c_str(), states::tracker_state_names[new_state].c_str());
  tracker_state_ = new_state;
}

//}

//}


}  // namespace mrs_robot_diagnostics

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_robot_diagnostics::StateMonitor, nodelet::Nodelet);
