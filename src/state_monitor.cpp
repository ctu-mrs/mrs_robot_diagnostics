/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>

/* custom msgs of MRS group */
#include <mrs_msgs/UavStatus.h>

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

  ros::Time last_update_time_;

  // | ---------------------- ROS subscribers --------------------- |
  mrs_lib::SubscribeHandler<mrs_msgs::UavStatus> sh_uav_status_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent &event);
  double     _main_timer_rate_;

  // | ------------------ Additional functions ------------------ |

  void parseLocalPosition(const mrs_msgs::UavStatusConstPtr &uav_status);
};
//}

/* onInit() //{ */

void StateMonitor::onInit() {

  /* obtain node handle */
  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();
  
  last_update_time_ = ros::Time(0);

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
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_uav_status_ = mrs_lib::SubscribeHandler<mrs_msgs::UavStatus>(shopts, "uav_status_in");

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

  bool got_uav_status = sh_uav_status_.newMsg();
  ros::Time time_now = ros::Time::now();

  if (!got_uav_status) {
    ros::Duration last_message_diff = time_now - last_update_time_;
    if(last_message_diff > ros::Duration(5.0)){
      ROS_WARN_THROTTLE(5.0, "[StateMonitor]: waiting for ROS data");
    }
    return;
  }


  if (got_uav_status){
    auto uav_status = sh_uav_status_.getMsg();
    parseLocalPosition(uav_status);
  }

  /* TODO: add more messages types */
  last_update_time_ = time_now;
}

//}

// | -------------------- support functions ------------------- |
//
/* parseLocalPosition() //{ */

void StateMonitor::parseLocalPosition(const mrs_msgs::UavStatusConstPtr &uav_status) {
  ROS_INFO_THROTTLE(1,"[IROCBridge]: LocalPosition: x: %.2f, y: %.2f, z: %.2f, heading: %.2f", 
      uav_status->odom_x, uav_status->odom_y, uav_status->odom_z, uav_status->odom_hdg);
}

//}

//}


}  // namespace mrs_robot_diagnostics

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_robot_diagnostics::StateMonitor, nodelet::Nodelet);
