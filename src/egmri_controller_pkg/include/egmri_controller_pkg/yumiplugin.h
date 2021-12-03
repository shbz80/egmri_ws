/*
This is the PR2-specific version of the robot plugin.
*/
#pragma once
// Headers.
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
// #include "egmri_controller_pkg/kdlchain.h"
#include <Eigen/Dense>
#include <pluginlib/class_list_macros.h>

// Superclass.
#include "egmri_controller_pkg/robotplugin.h"
#include "egmri_controller_pkg/controller.h"
#include "egmri_controller_pkg/positioncontroller.h"
#include "egmri_controller_pkg/encodersensor.h"
#include "egmri/proto/egmri.pb.h"

namespace egmri_control
{

class EGMRIYumiPlugin: public RobotPlugin, public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
private:
    // Right arm joint state handlers.
    std::vector< hardware_interface::JointHandle > right_arm_joint_states_;
    // Left arm joint state handlers.
    std::vector< hardware_interface::JointHandle > left_arm_joint_states_;
    // Right arm joint names.
    std::vector<std::string> right_arm_joint_names_;
    // Left arm joint names.
    std::vector<std::string> left_arm_joint_names_;
    // Left arm static friction %.
    std::vector<double> left_arm_stat_fric_percent;
    // Left arm static friction.
    std::vector<double> left_arm_stat_fric_;
    // Left arm dynamic friction.
    std::vector<double> left_arm_dyn_fric_;
    // Left arm friction torque.
    std::vector<double> left_arm_fric_trq_;
    // Right arm static friction %.
    std::vector<double> right_arm_stat_fric_percent;
    // Right arm static friction.
    std::vector<double> right_arm_stat_fric_;
    // Right arm dynamic friction.
    std::vector<double> right_arm_dyn_fric_;
    // Right arm friction torque.
    std::vector<double> right_arm_fric_trq_;
    // Left arm pos.
    Eigen::VectorXd left_arm_pos_;
    // Right arm pos.
    Eigen::VectorXd right_arm_pos_;
    // Time of last state update.
    ros::Time last_update_time_;
    // Counter for keeping track of controller steps.
    int controller_counter_;
    // Controller frequency in Hz.
    int controller_period_ms_;
    // Controller period counter ms.
    int current_controller_period_ms_;
    int NUM_JOINTS;
public:
    // Constructor (this should do nothing).
    EGMRIYumiPlugin();
    // Destructor.
    virtual ~EGMRIYumiPlugin();
    // Functions inherited from superclass.
    // This called by the superclass to allow us to initialize all the PR2-specific stuff.
    /* IMPORTANT: note that some sensors require a KDL chain to do FK, which we need the RobotState to get... */
    //virtual bool init(pr2_mechanism_model::RobotState* robot, ros::NodeHandle& n);
    virtual bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
    // This is called by the controller manager before starting the controller.
    //virtual void starting();
    virtual void starting(const ros::Time& time);
    // This is called by the controller manager before stopping the controller.
    //virtual void stopping();
    virtual void stopping(const ros::Time& time);
    // This is the main update function called by the realtime thread when the controller is running.
    //virtual void update();
    virtual void update(const ros::Time& time, const ros::Duration& period);
    /* the pr2-specific update function should do the following:
       - perform whatever housekeeping is needed to note the current time.
       - update all sensors (this might be a no-op for vision, but for
         joint angle "sensors," they need to know the current robot state).
       - update the appropriate controller (position or trial) depending on
         what we're currently doing
       - if the controller wants to send something via a publisher, publish
         that at the end -- it will typically be a completion message that
         includes the recorded robot state for the controller's execution.
     */
    // Accessors.
    // Get current time.
    virtual ros::Time get_current_time() const;
    // Get current encoder readings (robot-dependent).
    virtual void get_joint_encoder_readings(Eigen::VectorXd &angles, egmri::ActuatorType arm) const;
    virtual void get_joint_state_readings(Eigen::VectorXd &angles, Eigen::VectorXd &angle_velocities, egmri::ActuatorType arm) const;
};

}
