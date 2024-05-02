/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_msgs/Bool.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>

#include <mrs_msgs/UavStatus.h>
#include <mrs_msgs/HwApiStatus.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>

#include "mrs_robot_diagnostics/enums/uav_state.h"
#include "mrs_robot_diagnostics/enums/tracker_state.h"


//}

namespace mrs_robot_diagnostics
{

/* class StateMonitor //{ */

class StateMonitor : public nodelet::Nodelet {
public:
  virtual void onInit();

private:
  ros::NodeHandle   nh_;
  std::atomic<bool> is_initialized_ = false;

  uav_state_t uav_state_ = uav_state_t::UNKNOWN;
  tracker_state_t tracker_state_ = tracker_state_t::NULL_TRACKER;

  // | ---------------------- ROS subscribers --------------------- |
  mrs_lib::SubscribeHandler<std_msgs::Bool> sh_automatic_start_can_takeoff_;
  mrs_lib::SubscribeHandler<mrs_msgs::UavStatus> sh_uav_status_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus> sh_hw_api_status_;
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diagnostics_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent &event);
  double     _main_timer_rate_;

  // | ------------------ Additional functions ------------------ |

  void parseUavStatus(const mrs_msgs::UavStatus::ConstPtr &uav_status);
  void parseHwApiStatus(const mrs_msgs::HwApiStatus::ConstPtr &hw_api_status);
  void parseControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnostics::ConstPtr &control_manager_diagnostics);
  void updateUavState(const uav_state_t &new_state);
  void updateTrackerState(const tracker_state_t &new_state);
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
  shopts.no_message_timeout = ros::Duration(5.0);
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_uav_status_ = mrs_lib::SubscribeHandler<mrs_msgs::UavStatus>(shopts, "uav_status_in");
  sh_hw_api_status_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus>(shopts, "hw_api_status_in");
  sh_control_manager_diagnostics_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diagnostics_in");

  shopts.no_message_timeout = mrs_lib::no_timeout;
  sh_automatic_start_can_takeoff_ = mrs_lib::SubscribeHandler<std_msgs::Bool>(shopts, "automatic_start_can_takeoff_in");

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
  bool got_automatic_start_can_takeoff = sh_automatic_start_can_takeoff_.newMsg();
  std_msgs::Bool::ConstPtr automatic_start_can_takeoff; 

  bool got_uav_status = sh_uav_status_.newMsg();
  mrs_msgs::UavStatus::ConstPtr uav_status; 

  bool got_hw_api_status = sh_hw_api_status_.newMsg();
  mrs_msgs::HwApiStatus::ConstPtr hw_api_status; 
  
  bool got_control_manager_diagnostics = sh_control_manager_diagnostics_.newMsg();
  mrs_msgs::ControlManagerDiagnostics::ConstPtr control_manager_diagnostics; 

  if (got_automatic_start_can_takeoff){
    automatic_start_can_takeoff = sh_automatic_start_can_takeoff_.getMsg();
  }

  if (got_uav_status){
    uav_status = sh_uav_status_.getMsg();
  }

  if (got_hw_api_status){
    hw_api_status = sh_hw_api_status_.getMsg();
    parseHwApiStatus(hw_api_status);
  }

  if (got_control_manager_diagnostics){
    control_manager_diagnostics = sh_control_manager_diagnostics_.getMsg();
    parseControlManagerDiagnostics(control_manager_diagnostics);
  }

}

//}

// | -------------------- support functions ------------------- |

/* parseUavStatus() //{ */

void StateMonitor::parseUavStatus(const mrs_msgs::UavStatus::ConstPtr &uav_status) {
}

//}

/* parseHwApiStatus() //{ */

void StateMonitor::parseHwApiStatus(const mrs_msgs::HwApiStatus::ConstPtr &hw_api_status) {
  if (!hw_api_status->armed){
    updateUavState(uav_state_t::DISARMED);
  } else if (uav_state_ == uav_state_t::DISARMED || hw_api_status->mode == "AUTO.LOITER" || tracker_state_ == tracker_state_t::NULL_TRACKER){
    updateUavState(uav_state_t::ARMED);
  }
}

//}

/* parseControlManagerDiagnostics() //{ */

void StateMonitor::parseControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnostics::ConstPtr &control_manager_diagnostics) {
  if (control_manager_diagnostics->active_tracker == "LandoffTracker"){
    updateTrackerState(tracker_state_t::LANDOFF_TRACKER);

    if (uav_state_ == uav_state_t::ARMED){
      updateUavState(uav_state_t::TAKEOFF);
    } else if (uav_state_ != uav_state_t::TAKEOFF){
      updateUavState(uav_state_t::LANDING);
    }

  } else if (control_manager_diagnostics->active_tracker == "NullTracker" && uav_state_ == uav_state_t::LANDING){
    updateTrackerState(tracker_state_t::NULL_TRACKER);
  } else if (control_manager_diagnostics->joystick_active){
    updateUavState(uav_state_t::MANUAL);
  } else if (control_manager_diagnostics->flying_normally){
    updateTrackerState(tracker_state_t::AUTO_TRACKER);
    updateUavState(uav_state_t::AUTONOMOUS);
  }
}

//}

/* updateUavState() //{ */

void StateMonitor::updateUavState(const uav_state_t &new_state) {

  if (uav_state_ == new_state){
    return;
  }

  ROS_INFO("[StateMonitor]: SWITCHING UAV STATE: \"%s\" => \"%s\"", to_string(uav_state_), to_string(new_state));
  uav_state_ = new_state;
}

//}

/* updateTrackerState() //{ */

void StateMonitor::updateTrackerState(const tracker_state_t &new_state) {

  if (tracker_state_ == new_state){
    return;
  }

  ROS_INFO("[StateMonitor]: SWITCHING TRACKER STATE: \"%s\" => \"%s\"", to_string(tracker_state_), to_string(new_state));
  tracker_state_ = new_state;
}

//}

//}


}  // namespace mrs_robot_diagnostics

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_robot_diagnostics::StateMonitor, nodelet::Nodelet);
