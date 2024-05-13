/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/BatteryState.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>

#include <mrs_msgs/UavStatus.h>
#include <mrs_msgs/HwApiStatus.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>

#include <mrs_msgs/UavDiagnostics.h>

#include <mrs_robot_diagnostics/GeneralRobotInfo.h>

#include "mrs_robot_diagnostics/enums/uav_state.h"
#include "mrs_robot_diagnostics/enums/tracker_state.h"


//}

namespace mrs_robot_diagnostics
{

  /* class StateMonitor //{ */

  class StateMonitor : public nodelet::Nodelet
  {
  private:
    using out_diags_msg_t = mrs_msgs::UavDiagnostics;

  public:
    virtual void onInit();

  private:
    ros::NodeHandle nh_;

    enum_helpers::enum_updater<uav_state_t> uav_state_ = {"UAV STATE", uav_state_t::UNKNOWN};
    enum_helpers::enum_updater<tracker_state_t> tracker_state_ = {"TRACKER STATE", tracker_state_t::NULL_TRACKER};

    std::string _robot_name_;
    std::string _robot_type_;

    // | ---------------------- ROS subscribers --------------------- |
    mrs_lib::SubscribeHandler<std_msgs::Bool> sh_automatic_start_can_takeoff_;
    mrs_lib::SubscribeHandler<mrs_msgs::UavStatus> sh_uav_status_;
    mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus> sh_hw_api_status_;
    mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diagnostics_;

    mrs_lib::SubscribeHandler<sensor_msgs::BatteryState> sh_battery_state_;

    ros::Publisher pub_out_diags_;
    ros::Publisher pub_general_robot_info_;

    // | ----------------------- main timer ----------------------- |

    ros::Timer timer_main_;
    void timerMain(const ros::TimerEvent& event);

    // | ------------------ Additional functions ------------------ |

    tracker_state_t parse_tracker_state(mrs_msgs::ControlManagerDiagnostics::ConstPtr control_manager_diagnostics);
    uav_state_t parse_uav_state(mrs_msgs::HwApiStatus::ConstPtr hw_api_status, mrs_msgs::ControlManagerDiagnostics::ConstPtr control_manager_diagnostics);
    mrs_robot_diagnostics::GeneralRobotInfo parse_general_robot_info(sensor_msgs::BatteryState::ConstPtr battery_state);
  };
  //}

  /* onInit() //{ */

  void StateMonitor::onInit()
  {

    /* obtain node handle */
    nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    /* waits for the ROS to publish clock */
    ros::Time::waitForValid();

    /* load parameters */
    mrs_lib::ParamLoader param_loader(nh_, "StateMonitor");

    std::string custom_config_path;

    param_loader.loadParam("custom_config", custom_config_path);

    if (custom_config_path != "")
    {
      param_loader.addYamlFile(custom_config_path);
    }

    param_loader.addYamlFileFromParam("config");

    param_loader.loadParam("robot_name", _robot_name_);
    param_loader.loadParam("robot_type", _robot_type_);
    const auto main_timer_rate = param_loader.loadParam2<double>("main_timer_rate");

    if (!param_loader.loadedSuccessfully())
    {
      ROS_ERROR("[StateMonitor]: Could not load all parameters!");
      ros::shutdown();
    }

    pub_out_diags_ = nh_.advertise<out_diags_msg_t>("diagnostics", 10);
    pub_general_robot_info_ = nh_.advertise<mrs_robot_diagnostics::GeneralRobotInfo>("general_robot_info", 10);

    // | ----------------------- subscribers ---------------------- |

    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh = nh_;
    shopts.node_name = "StateMonitor";
    shopts.no_message_timeout = ros::Duration(5.0);
    shopts.threadsafe = true;
    shopts.autostart = true;
    shopts.queue_size = 10;
    shopts.transport_hints = ros::TransportHints().tcpNoDelay();

    sh_uav_status_ = mrs_lib::SubscribeHandler<mrs_msgs::UavStatus>(shopts, "uav_status_in");
    sh_hw_api_status_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus>(shopts, "hw_api_status_in");
    sh_control_manager_diagnostics_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diagnostics_in");

    // | ------------------------ RobotInfo ----------------------- |
    sh_battery_state_ = mrs_lib::SubscribeHandler<sensor_msgs::BatteryState>(shopts, "battery_state_in");

    shopts.no_message_timeout = mrs_lib::no_timeout;
    sh_automatic_start_can_takeoff_ = mrs_lib::SubscribeHandler<std_msgs::Bool>(shopts, "automatic_start_can_takeoff_in");

    // | ------------------------- timers ------------------------- |

    timer_main_ = nh_.createTimer(ros::Rate(main_timer_rate), &StateMonitor::timerMain, this);

    // | --------------------- finish the init -------------------- |

    ROS_INFO("[StateMonitor]: initialized");
    ROS_INFO("[StateMonitor]: --------------------");
  }

  //}

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  /* timerMain() //{ */

  void StateMonitor::timerMain([[maybe_unused]] const ros::TimerEvent& event)
  {

    if (sh_battery_state_.newMsg())
      pub_general_robot_info_.publish(parse_general_robot_info(sh_battery_state_.getMsg()));

    // | -------------------- UAV state parsing ------------------- |
    const bool got_all_messages = sh_hw_api_status_.hasMsg() && sh_control_manager_diagnostics_.hasMsg();
    if (got_all_messages)
    {
      const auto hw_api_status = sh_hw_api_status_.getMsg();
      const auto control_manager_diagnostics = sh_control_manager_diagnostics_.getMsg();
      
      const auto new_state = parse_uav_state(hw_api_status, control_manager_diagnostics);
      uav_state_.set(new_state);
      
      out_diags_msg_t out_diags_msg;
      out_diags_msg.stamp = ros::Time::now();
      out_diags_msg.state = to_string(uav_state_.value());
      pub_out_diags_.publish(out_diags_msg);
    }
  }

  //}

  // | -------------------- support functions ------------------- |

  /* parse_tracker_state() method //{ */
  tracker_state_t StateMonitor::parse_tracker_state(mrs_msgs::ControlManagerDiagnostics::ConstPtr control_manager_diagnostics)
  {
    if (control_manager_diagnostics->active_tracker == "NullTracker")
      return tracker_state_t::NULL_TRACKER;
  
    switch (control_manager_diagnostics->tracker_status.state)
    {
      case mrs_msgs::TrackerStatus::STATE_IDLE:       return tracker_state_t::NULL_TRACKER;
      case mrs_msgs::TrackerStatus::STATE_TAKEOFF:    return tracker_state_t::TAKEOFF;
      case mrs_msgs::TrackerStatus::STATE_HOVER:      return tracker_state_t::HOVER;
      case mrs_msgs::TrackerStatus::STATE_REFERENCE:  return tracker_state_t::REFERENCE;
      case mrs_msgs::TrackerStatus::STATE_TRAJECTORY: return tracker_state_t::TRAJECTORY;
      case mrs_msgs::TrackerStatus::STATE_LAND:       return tracker_state_t::LAND;
      default:                                        return tracker_state_t::UNKNOWN;
    }
  }
  //}

  /* parse_uav_state() method //{ */
  uav_state_t StateMonitor::parse_uav_state(mrs_msgs::HwApiStatus::ConstPtr hw_api_status, mrs_msgs::ControlManagerDiagnostics::ConstPtr control_manager_diagnostics)
  {
    const bool hw_armed = hw_api_status->armed;
    // not armed
    if (!hw_armed)
      return uav_state_t::DISARMED;
  
    // armed, flying in manual mode
    const bool manual_mode = hw_api_status->mode == "MANUAL";
    if (control_manager_diagnostics->joystick_active)
      return uav_state_t::MANUAL;
  
    // armed, not flying
    const bool hw_loitering = hw_api_status->mode == "AUTO.LOITER";
    const auto tracker_state = parse_tracker_state(control_manager_diagnostics);
    const bool null_tracker = tracker_state == tracker_state_t::NULL_TRACKER;
    if (hw_loitering || null_tracker)
      return uav_state_t::ARMED;
  
    // flying using the MRS system in RC joystick mode
    if (control_manager_diagnostics->joystick_active)
      return uav_state_t::RC_MODE;
  
    // unless the RC mode is active, just parse the tracker state
    switch (tracker_state)
    {
      case tracker_state_t::TAKEOFF:      return uav_state_t::TAKEOFF;
      case tracker_state_t::HOVER:        return uav_state_t::HOVER;
      case tracker_state_t::REFERENCE:    return uav_state_t::GOTO;
      case tracker_state_t::TRAJECTORY:   return uav_state_t::TRAJECTORY;
      case tracker_state_t::LAND:         return uav_state_t::LAND;
      default:                            return uav_state_t::UNKNOWN;
    }
  }
  //}

  /* parse_general_robot_info() method //{ */
  mrs_robot_diagnostics::GeneralRobotInfo StateMonitor::parse_general_robot_info(sensor_msgs::BatteryState::ConstPtr battery_state)
  {
    mrs_robot_diagnostics::GeneralRobotInfo msg;
    msg.robot_name = _robot_name_;
    msg.robot_type = _robot_type_;
  
    msg.battery_state.voltage = battery_state->voltage;
    msg.battery_state.percentage = battery_state->percentage;
    msg.battery_state.wh_drained = -1.0;
  
    const bool autostart_running = sh_automatic_start_can_takeoff_.getNumPublishers();
    const bool autostart_ready = sh_automatic_start_can_takeoff_.hasMsg() && sh_automatic_start_can_takeoff_.getMsg()->data;
    const bool state_ok = uav_state_.value() == uav_state_t::DISARMED;
    msg.ready_to_start = state_ok && autostart_running & autostart_ready;
    if (is_flying(uav_state_.value()))
      msg.problem_preventing_start = "UAV is in flight";
    else if (!state_ok)
      msg.problem_preventing_start = "UAV is not in DISARMED state";
    else if (!autostart_running)
      msg.problem_preventing_start = "Automatic start node is not running";
    else if (!autostart_ready)
      msg.problem_preventing_start = "Automatic start reports UAV not ready";
    return msg;
  }
  //}

}  // namespace mrs_robot_diagnostics

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_robot_diagnostics::StateMonitor, nodelet::Nodelet);
