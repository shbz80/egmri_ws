#include "egmri_controller_pkg/positioncontroller.h"
#include "egmri_controller_pkg/robotplugin.h"
#include "egmri_controller_pkg/util.h"

using namespace egmri_control;

// Constructor.

// Constructor.
PositionController::PositionController(ros::NodeHandle& n, egmri::ActuatorType arm, int size)
    : Controller(n, arm, size)
{
    // Initialize PD gains.
    pd_gains_p_.resize(size);
    pd_gains_d_.resize(size);
    pd_gains_i_.resize(size);

    // Initialize velocity bounds.
    max_velocities_.resize(size);

    // Initialize integral terms to zero.
    pd_integral_.resize(size);
    i_clamp_.resize(size);

    // Initialize current angle and position.
    current_angles_.resize(size);
    current_angle_velocities_.resize(size);
    current_pose_.resize(size);

    // Initialize target angle and position.
    target_angles_.resize(size);
    dest_angles_.resize(size);

    target_pose_.resize(size);

    // Initialize joints temporary storage.
    temp_angles_.resize(size);

    // Initialize Jacobian temporary storage.
    temp_jacobian_.resize(6,size);

    // Set initial mode.
    mode_ = egmri::NO_CONTROL;

    // Set initial time.
    last_update_time_ = ros::Time(0.0);

    // Set arm.
    arm_ = arm;

    //
    report_waiting = false;
}

// Destructor.
PositionController::~PositionController()
{
}

// Update the controller (take an action).
void PositionController::update(RobotPlugin *plugin, ros::Time current_time, boost::scoped_ptr<Sample>& sample, Eigen::VectorXd &torques)
{
    // Get current joint angles.
    plugin->get_joint_encoder_readings(temp_angles_, arm_);

    // Check dimensionality.
    assert(temp_angles_.rows() == torques.rows());
    assert(temp_angles_.rows() == current_angles_.rows());

    // Estimate joint angle velocities.
    double update_time = current_time.toSec() - last_update_time_.toSec();
    if (!last_update_time_.isZero())
    { // Only compute velocities if we have a previous sample.
        current_angle_velocities_ = (temp_angles_ - current_angles_)/update_time;
    }

    // Store new angles.
    current_angles_ = temp_angles_;

    // Update last update time.
    last_update_time_ = current_time;

    // If we're doing any kind of control at all, compute torques now.
    if (mode_ != egmri::NO_CONTROL)
    {
        // computer current angle to dest. This is added to slow down the speed
        temp_angles_ = dest_angles_ - current_angles_;
        for(int i=0;i<7;i++){
          target_angles_(i) = std::min(std::abs(temp_angles_(i)),ANGLE)*((temp_angles_(i) >= 0.0) ? 1.0 : -1.0) + current_angles_(i);
        }

        // Compute error.
        temp_angles_ = current_angles_ - target_angles_;
        // ROS_INFO_STREAM_THROTTLE(0.5, "error angles:"<<temp_angles_);

        // Add to integral term.
        pd_integral_ += temp_angles_ * update_time;

        // Clamp integral term
        for (int i = 0; i < temp_angles_.rows(); i++){
            if (pd_integral_(i) * pd_gains_i_(i) > i_clamp_(i)) {
                pd_integral_(i) = i_clamp_(i) / pd_gains_i_(i);
            }
            else if (pd_integral_(i) * pd_gains_i_(i) < -i_clamp_(i)) {
                pd_integral_(i) = -i_clamp_(i) / pd_gains_i_(i);
            }
        }

          // Compute torques.
          torques = -((pd_gains_p_.array() * temp_angles_.array()) +
                      (pd_gains_d_.array() * current_angle_velocities_.array()) +
                      (pd_gains_i_.array() * pd_integral_.array())).matrix();
          // ROS_INFO_STREAM_THROTTLE(0.5, "pid torques:"<<torques);


    }
    else
    {
        torques = Eigen::VectorXd::Zero(torques.rows());
    }

}

// Configure the controller.
void PositionController::configure_controller(OptionsMap &options)
{
    // This sets the target position.
    // This sets the mode
    ROS_INFO_STREAM("Received controller configuration");
    // needs to report when finished
    report_waiting = true;
    mode_ = (egmri::PositionControlMode) boost::get<int>(options["mode"]);
    //ROS_DEBUG_STREAM("mode: "<<mode_);
    if (mode_ != egmri::NO_CONTROL){
        Eigen::VectorXd data = boost::get<Eigen::VectorXd>(options["data"]);
        ROS_INFO_STREAM("target angles: "<<data);
        Eigen::MatrixXd pd_gains = boost::get<Eigen::MatrixXd>(options["pd_gains"]);
        ROS_INFO_STREAM("pid gains:"<<pd_gains);
        for(int i=0; i<pd_gains.rows(); i++){
            // ROS_INFO_STREAM("pid P: "<<pd_gains(i, 0));
            pd_gains_p_(i) = pd_gains(i, 0);
            // ROS_INFO_STREAM("pid I: "<<pd_gains(i, 1));
            pd_gains_i_(i) = pd_gains(i, 1);
            // ROS_INFO_STREAM("pid D: "<<pd_gains(i, 2));
            pd_gains_d_(i) = pd_gains(i, 2);
            // ROS_INFO_STREAM("pid C: "<<pd_gains(i, 3));
            i_clamp_(i) = pd_gains(i, 3);
        }
        if(mode_ == egmri::JOINT_SPACE){
            // target_angles_ = data;
            dest_angles_ = data;
        }else{
            ROS_ERROR("Unimplemented position control mode!");
        }
    }
}

// Check if controller is finished with its current task.
bool PositionController::is_finished() const
{
    // Check whether we are close enough to the current target.
    if (mode_ == egmri::JOINT_SPACE){
        // double epspos = 0.015;
        // double epspos = 0.0075;
        double epspos = 0.01;
        double epsvel = 0.075; // TODO: restore this
        // double epsvel = 0.1;
        // double error = (current_angles_ - target_angles_).norm();
        // ROS_INFO_STREAM_THROTTLE(1,"dest_angles:"<<dest_angles_<<"current_angles:"<<current_angles_);
        double error = (current_angles_ - dest_angles_).norm();
        double vel = current_angle_velocities_.norm();
        ROS_INFO_STREAM_THROTTLE(1,"pos error:"<<error<<"vel:"<<vel);
        return (error < epspos && vel < epsvel);
    }
    else if (mode_ == egmri::NO_CONTROL){
        return true;
    }
}

// Reset the controller -- this is typically called when the controller is turned on.
void PositionController::reset(ros::Time time)
{
    // Clear the integral term.
    pd_integral_.fill(0.0);

    // Clear update time.
    last_update_time_ = ros::Time(0.0);
}
