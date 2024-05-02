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

#include <mrs_msgs/UavDiagnostics.h>

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
    std::atomic<bool> is_initialized_ = false;

    enum_helpers::enum_updater<uav_state_t> uav_state_ = {"UAV STATE", uav_state_t::UNKNOWN};
    enum_helpers::enum_updater<tracker_state_t> tracker_state_ = {"TRACKER STATE", tracker_state_t::NULL_TRACKER};

    // | ---------------------- ROS subscribers --------------------- |
    mrs_lib::SubscribeHandler<std_msgs::Bool> sh_automatic_start_can_takeoff_;
    mrs_lib::SubscribeHandler<mrs_msgs::UavStatus> sh_uav_status_;
    mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus> sh_hw_api_status_;
    mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diagnostics_;

    ros::Publisher pub_out_diags_;

    // | ----------------------- main timer ----------------------- |

    ros::Timer timer_main_;
    void timerMain(const ros::TimerEvent& event);

    // | ------------------ Additional functions ------------------ |

    tracker_state_t parse_tracker_state(mrs_msgs::ControlManagerDiagnostics::ConstPtr control_manager_diagnostics);
    uav_state_t parse_uav_state(mrs_msgs::HwApiStatus::ConstPtr hw_api_status, mrs_msgs::ControlManagerDiagnostics::ConstPtr control_manager_diagnostics, std_msgs::Bool::ConstPtr automatic_start_can_takeoff);
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

    const auto main_timer_rate = param_loader.loadParam2<double>("main_timer_rate");

    if (!param_loader.loadedSuccessfully())
    {
      ROS_ERROR("[StateMonitor]: Could not load all parameters!");
      ros::shutdown();
    }

    pub_out_diags_ = nh_.advertise<out_diags_msg_t>("diagnostics", 10);

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

    shopts.no_message_timeout = mrs_lib::no_timeout;
    sh_automatic_start_can_takeoff_ = mrs_lib::SubscribeHandler<std_msgs::Bool>(shopts, "automatic_start_can_takeoff_in");

    // | ------------------------- timers ------------------------- |

    timer_main_ = nh_.createTimer(ros::Rate(main_timer_rate), &StateMonitor::timerMain, this);

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

  void StateMonitor::timerMain([[maybe_unused]] const ros::TimerEvent& event)
  {

    if (!is_initialized_)
      return;

    const bool got_all_messages = sh_hw_api_status_.hasMsg() && sh_control_manager_diagnostics_.hasMsg() && sh_automatic_start_can_takeoff_.hasMsg();
    if (!got_all_messages)
      return;

    const auto hw_api_status = sh_hw_api_status_.getMsg();
    const auto control_manager_diagnostics = sh_control_manager_diagnostics_.getMsg();
    const auto automatic_start_can_takeoff = sh_automatic_start_can_takeoff_.getMsg();

    const auto new_state = parse_uav_state(hw_api_status, control_manager_diagnostics, automatic_start_can_takeoff);
    uav_state_.set(new_state);

    out_diags_msg_t out_diags_msg;
    out_diags_msg.stamp = ros::Time::now();
    out_diags_msg.state = to_string(uav_state_.value());
    pub_out_diags_.publish(out_diags_msg);
  }

  //}

  // | -------------------- support functions ------------------- |

  tracker_state_t parse_tracker_state(mrs_msgs::ControlManagerDiagnostics::ConstPtr control_manager_diagnostics)
  {
    if (control_manager_diagnostics->active_tracker == "NullTracker")
      return tracker_state_t::NULL_TRACKER;

    if (control_manager_diagnostics->active_tracker == "LandoffTracker")
      return tracker_state_t::LANDOFF_TRACKER;

    if (control_manager_diagnostics->flying_normally)
      return tracker_state_t::AUTO_TRACKER;

    return tracker_state_t::UNKNOWN;
  }

  uav_state_t parse_uav_state(mrs_msgs::HwApiStatus::ConstPtr hw_api_status, mrs_msgs::ControlManagerDiagnostics::ConstPtr control_manager_diagnostics, std_msgs::Bool::ConstPtr automatic_start_can_takeoff)
  {
    const bool hw_armed = hw_api_status->armed;
    // not armed
    if (!hw_armed)
      return uav_state_t::DISARMED;

    // armed, not flying
    const bool hw_loitering = hw_api_status->mode == "AUTO.LOITER";
    const auto tracker_state = parse_tracker_state(control_manager_diagnostics);
    const bool tracker_none = tracker_state == tracker_state_t::NULL_TRACKER;
    if (hw_loitering || tracker_none)
      return uav_state_t::ARMED;

    // armed, flying, taking off
    const bool tracker_landoff = tracker_state == tracker_state_t::LANDOFF_TRACKER;
    if (tracker_landoff)
      return uav_state_t::TAKEOFF;

    // catch an invalid state
    if (tracker_state != tracker_state_t::AUTO_TRACKER)
      return uav_state_t::UNKNOWN;

    // armed, flying autonomously, trajectory tracking
    if (control_manager_diagnostics->tracker_status.tracking_trajectory)
      return uav_state_t::TRAJECTORY;

    // armed, flying autonomously, not tracking trajectory, but have goal
    if (control_manager_diagnostics->tracker_status.have_goal)
      return uav_state_t::GOTO;

    // armed, flying autonomously, not tracking trajectory, no goal
    return uav_state_t::HOVERING;
  }

  //}


}  // namespace mrs_robot_diagnostics

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_robot_diagnostics::StateMonitor, nodelet::Nodelet);
