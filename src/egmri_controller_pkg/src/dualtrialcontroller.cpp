#include "egmri_controller_pkg/robotplugin.h"
#include "egmri_controller_pkg/dualtrialcontroller.h"

using namespace egmri_control;

// Constructor.
DualTrialController::DualTrialController() : Controller()
{
    // Set initial time.
    last_update_time_ = ros::Time(0.0);
    step_counter_ = 0;
    trial_end_step_ = 1;
}

// Destructor.
DualTrialController::~DualTrialController() {
}

// Update the controller (take an action).
void DualTrialController::update(RobotPlugin *plugin,
                             ros::Time current_time,
                             boost::scoped_ptr<Sample>& sample_left,
                             boost::scoped_ptr<Sample>& sample_right,
                             Eigen::VectorXd &torques_left,
                             Eigen::VectorXd &torques_right)
{
    if (is_finished()){
        ROS_ERROR("Updating when controller is finished. May seg fault.");
    }
    Eigen::VectorXd X_left, X_right, obs_left, obs_right;
    // state and observations are the same.
    // get values for X_left, X_right, obs_left, obs_right
    sample_left->get_data(step_counter_, X_left, state_datatypes_);
    sample_right->get_data(step_counter_, X_right, state_datatypes_);
    sample_left->get_data(step_counter_, obs_left, obs_datatypes_);
    sample_right->get_data(step_counter_, obs_right, obs_datatypes_);

    // plugin->get_joint_encoder_readings(obs_left, egmri::LEFT_ARM);
    // plugin->get_joint_encoder_readings(obs_right, egmri::RIGHT_ARM);
    //publish the observation for consumption. Can be implemented in subclass if you want
    //the observations published to a ros node. Used for async controllers like the tf_controller.
    // publish_obs(obs, plugin);
    // Ask subclass to fill in torques
    // get_action(step_counter_, X, obs, torques);
    ROS_INFO_STREAM("Dual Impedance Controller Command begin ");
    update_control_command(step_counter_, current_time, obs_left, obs_right, torques_left, torques_right);
    ROS_INFO_STREAM("Dual Impedance Controller Command end ");
    // update_control_command(step_counter_, X_left, X_right, torques_left, torques_right)
    // Set the torques for the sample
    sample_left->set_data(step_counter_,egmri::ACTION,torques_left,torques_left.size(),SampleDataFormatDouble);
    sample_right->set_data(step_counter_,egmri::ACTION,torques_right,torques_right.size(),SampleDataFormatDouble);

    // Update last update time.
    last_update_time_ = current_time;
    step_counter_++;
    // ROS_INFO("Step counter: %d", step_counter_);
}

void DualTrialController::configure_controller(OptionsMap &options)
{
    ROS_INFO_STREAM(">DualTrialController::configure_controller");
    if(!is_finished()){
        // TODO(chelsea/sergey/zoe) This error happens every time...
        ROS_ERROR("Cannot configure controller while a trial is in progress");
    }
    std::vector<int> datatypes;

    int T = boost::get<int>(options["T"]);
    step_counter_ = 0;
    trial_end_step_ = T;

    datatypes = boost::get<std::vector<int> >(options["state_datatypes"]);
    state_datatypes_.resize(datatypes.size());
    for(int i=0; i<datatypes.size(); i++){
        state_datatypes_[i] = (egmri::SampleType) datatypes[i];
    }

    datatypes = boost::get<std::vector<int> >(options["obs_datatypes"]);
    obs_datatypes_.resize(datatypes.size());
    for(int i=0; i<datatypes.size(); i++){
        obs_datatypes_[i] = (egmri::SampleType) datatypes[i];
    }

    arm_ = (egmri::ActuatorType) boost::get<int>(options["arm_datatype"]);

}

// void DualTrialController::update_control_command(const int &step_count,
//                                     ros::Time current_time,
//                                     const Eigen::VectorXd &X_left,
//                                     const Eigen::VectorXd &X_right,
//                                     Eigen::VectorXd &torques_left,
//                                     Eigen::VectorXd &torques_right)
// {
//
// }

// Check if controller is finished with its current task.
bool DualTrialController::is_finished() const
{
    return step_counter_ >= trial_end_step_;
}

int DualTrialController::get_step_counter(){
    return step_counter_;
}

int DualTrialController::get_trial_length(){
    return trial_end_step_;
}

// Reset the controller -- this is typically called when the controller is turned on.
void DualTrialController::reset(ros::Time time)
{
    last_update_time_ = time;
    step_counter_ = 0;
    trial_end_step_ = 1;
}
