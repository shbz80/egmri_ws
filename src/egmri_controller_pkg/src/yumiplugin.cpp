#include "egmri_controller_pkg/yumiplugin.h"
#include "egmri_controller_pkg/positioncontroller.h"
#include "egmri_controller_pkg/trialcontroller.h"
#include "egmri_controller_pkg/encodersensor.h"
#include "egmri_controller_pkg/util.h"

namespace egmri_control {

// // Plugin constructor.
EGMRIYumiPlugin::EGMRIYumiPlugin()
{
    NUM_JOINTS = 7;
    // Some basic variable initialization.
    controller_counter_ = 0;
    // control frequency is decided here, set period in micoseconds
    // cannot be lesser than 4
    controller_period_ms_ = 10;
    // a counter, should be initialized to zero
    current_controller_period_ms_ = 0;

    left_arm_pos_.resize(NUM_JOINTS);
    right_arm_pos_.resize(NUM_JOINTS);

    left_arm_stat_fric_.resize(NUM_JOINTS);
    left_arm_stat_fric_.clear();
    left_arm_stat_fric_percent.resize(NUM_JOINTS);
    left_arm_stat_fric_percent.clear();
    left_arm_dyn_fric_.resize(NUM_JOINTS);
    left_arm_dyn_fric_.clear();

    right_arm_stat_fric_.resize(NUM_JOINTS);
    right_arm_stat_fric_.clear();
    right_arm_stat_fric_percent.resize(NUM_JOINTS);
    right_arm_stat_fric_percent.clear();
    right_arm_dyn_fric_.resize(NUM_JOINTS);
    right_arm_dyn_fric_.clear();

    // effective friction compensation as a percentage of full frcition
    // for each joints
    left_arm_stat_fric_percent.push_back(0.8);
    left_arm_stat_fric_percent.push_back(0.8);
    left_arm_stat_fric_percent.push_back(0.6);
    left_arm_stat_fric_percent.push_back(0.6);
    left_arm_stat_fric_percent.push_back(0.5);
    left_arm_stat_fric_percent.push_back(0.5);
    left_arm_stat_fric_percent.push_back(0.5);

    right_arm_stat_fric_percent.push_back(0.8);
    right_arm_stat_fric_percent.push_back(0.8);
    right_arm_stat_fric_percent.push_back(0.6);
    right_arm_stat_fric_percent.push_back(0.6);
    right_arm_stat_fric_percent.push_back(0.5);
    right_arm_stat_fric_percent.push_back(0.5);
    right_arm_stat_fric_percent.push_back(0.5);

    // static friction
    left_arm_stat_fric_.push_back(2.43);
    left_arm_stat_fric_.push_back(2.76);
    left_arm_stat_fric_.push_back(1.11);
    left_arm_stat_fric_.push_back(0.52);
    left_arm_stat_fric_.push_back(0.4);
    left_arm_stat_fric_.push_back(0.2);
    left_arm_stat_fric_.push_back(0.4);

    right_arm_stat_fric_.push_back(2.43);
    right_arm_stat_fric_.push_back(2.76);
    right_arm_stat_fric_.push_back(1.11);
    right_arm_stat_fric_.push_back(0.52);
    right_arm_stat_fric_.push_back(0.4);
    right_arm_stat_fric_.push_back(0.2);
    right_arm_stat_fric_.push_back(0.4);

    // dynamic friction
    left_arm_dyn_fric_.push_back(1.06);
    left_arm_dyn_fric_.push_back(1.09);
    left_arm_dyn_fric_.push_back(0.61);
    left_arm_dyn_fric_.push_back(0.08);
    left_arm_dyn_fric_.push_back(0.08);
    left_arm_dyn_fric_.push_back(0.08);
    left_arm_dyn_fric_.push_back(0.08);

    right_arm_dyn_fric_.push_back(1.06);
    right_arm_dyn_fric_.push_back(1.09);
    right_arm_dyn_fric_.push_back(0.61);
    right_arm_dyn_fric_.push_back(0.08);
    right_arm_dyn_fric_.push_back(0.08);
    right_arm_dyn_fric_.push_back(0.08);
    right_arm_dyn_fric_.push_back(0.08);

    // variable for final friction torque
    left_arm_fric_trq_.resize(NUM_JOINTS);
    left_arm_fric_trq_.clear();

    right_arm_fric_trq_.resize(NUM_JOINTS);
    right_arm_fric_trq_.clear();

}
//
// // Destructor.
EGMRIYumiPlugin::~EGMRIYumiPlugin()
{
//     // Nothing to do here, since all instance variables are destructed automatically.
}

// Initialize the object and store the robot state.
bool EGMRIYumiPlugin::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
// bool EGMRIYumiPlugin::init(pr2_mechanism_model::RobotState* robot, ros::NodeHandle& n)
{
    // Variables.
    // std::string root_name, left_tip_name, right_tip_name, robot_desc_string;

    ROS_INFO_STREAM("EGMRIYumiPlugin init begin");

    // // Create FK solvers.
    // // Get the name of the root.
    // if(!n.getParam("root_name", root_name)) {
    //     ROS_ERROR("Property root_name not found in namespace: '%s'", n.getNamespace().c_str());
    //     return false;
    // }
    //
    // // Get left and right arm end-effector names.
    // if(!n.getParam("left_tip_name", left_tip_name)) {
    //     ROS_ERROR("Property left_tip_name not found in namespace: '%s'", n.getNamespace().c_str());
    //     return false;
    // }
    // if(!n.getParam("right_tip_name", right_tip_name)) {
    //     ROS_ERROR("Property right_tip_name not found in namespace: '%s'", n.getNamespace().c_str());
    //     return false;
    // }
    //
    // // Create the robot model from param server.
    // // TODO:the robot model may be available from the hardware interface given by the
    // // controller manager. Change this later
    // if(!n.getParam("robot_description", robot_desc_string)) {
    //     ROS_ERROR("Property robot_description not found in namespace: '%s'", n.getNamespace().c_str());
    //     return false;
    // }
    //
    //
    // // Create left arm chain.
    // if(!left_arm_chain_.init(hw, robot_desc_string, root_name, left_tip_name)) {
    //     ROS_ERROR("Controller could not use the chain from '%s' to '%s'", root_name.c_str(), left_tip_name.c_str());
    //     return false;
    // }

    // // Create right arm chain.
    // if(!right_arm_chain_.init(hw, robot_desc_string, root_name, right_tip_name)) {
    //     ROS_ERROR("Controller could not use the chain from '%s' to '%s'", root_name.c_str(), right_tip_name.c_str());
    //     return false;
    // }

    // Pull out joint states.
    int joint_index;
    left_arm_joint_states_.clear();
    // Put together joint states for the left arm.
    joint_index = 1;
    while (true)
    {
        // Check if the parameter for this left joint exists.
        std::string joint_name;
        std::string param_name = std::string("/yumi/left_arm_joint_name_" + to_string(joint_index));
        //if(!n.getParam(param_name.c_str(), joint_name)){
        if(!n.getParam(param_name, joint_name)){
            ROS_INFO_STREAM("param name: "+param_name);
            ROS_INFO_STREAM("joint name: "+joint_name);
            break;
        }
        // Push back the joint state and name.
        hardware_interface::JointHandle jointHandle = hw->getHandle(joint_name);
        left_arm_joint_states_.push_back(jointHandle);
        //TODO: any other check possible?
        //if (left_arm_joint_states_[joint_index] == NULL) // TODO this may not be correct
        //    ROS_INFO_STREAM("jointState: " + joint_name + " is null");

        left_arm_joint_names_.push_back(joint_name);
        ROS_INFO_STREAM("left arm joint names"+joint_name);

        // Increment joint index.
        joint_index++;
    }
    // Validate that the number of joints in the chain equals the length of the left arm joint state.

    // if (left_arm_fk_chain_.getNrOfJoints() != left_arm_joint_states_.size())
    if (NUM_JOINTS != left_arm_joint_states_.size())
    {
        ROS_INFO_STREAM("num_fk_chain: " + to_string(NUM_JOINTS));
        ROS_INFO_STREAM("num_joint_state: " + to_string(left_arm_joint_states_.size()));
        ROS_ERROR("Number of joints in the left arm FK chain does not match the number of joints in the left arm joint state!");
        return false;
    }

    // Put together joint states for the right arm.
    joint_index = 1;
    right_arm_joint_states_.clear();
    while (true)
    {
        // Check if the parameter for this right joint exists.
        std::string joint_name;
        std::string param_name = std::string("/yumi/right_arm_joint_name_" + to_string(joint_index));
        if(!n.getParam(param_name, joint_name))
            break;

        // Push back the joint state and name.
        hardware_interface::JointHandle jointHandle = hw->getHandle(joint_name);
        right_arm_joint_states_.push_back(jointHandle);
        //if (right_arm_joint_states_[joint_index] == NULL) // TODO this may not be correct
        //    ROS_INFO_STREAM("jointState: " + joint_name + " is null");
        right_arm_joint_names_.push_back(joint_name);
        //ROS_DEBUG_STREAM("right arm joint names: "<<joint_index<<right_arm_joint_names_[joint_index]);
        // Increment joint index.
        joint_index++;
    }
    // Validate that the number of joints in the chain equals the length of the left arm joint state.
    // if (right_arm_fk_chain_.getNrOfJoints() != right_arm_joint_states_.size())
    if (NUM_JOINTS != right_arm_joint_states_.size())
    {
        ROS_INFO_STREAM("num_fk_chain: " + to_string(NUM_JOINTS));
        ROS_INFO_STREAM("num_joint_state: " + to_string(right_arm_joint_states_.size()));
        ROS_ERROR("Number of joints in the right arm FK chain does not match the number of joints in the right arm joint state!");
        return false;
    }

    // Allocate torques array.
    left_arm_torques_.resize(NUM_JOINTS);
    right_arm_torques_.resize(NUM_JOINTS);
    //
    // Initialize ROS subscribers/publishers, sensors, and position controllers.
    // Note that this must be done after the FK solvers are created, because the sensors
    // will ask to use these FK solvers!
    initialize(n);
    ROS_INFO_STREAM("EGMRIYumiPlugin init end");
    // Tell the PR2 controller manager that we initialized everything successfully.
    return true;
}

// This is called by the controller manager before starting the controller.
void EGMRIYumiPlugin::starting(const ros::Time& time)
// void EGMRIYumiPlugin::starting()
{
    ROS_INFO_STREAM("EGMRIYumiPlugin starting begin");
    // Get current time.
    //last_update_time_ = robot_->getTime();
    last_update_time_ = time;
    ROS_INFO_STREAM("Plugin starting time: "+to_string(last_update_time_));
    controller_counter_ = 0;
    current_controller_period_ms_ = 0;

    // Reset all the sensors. This is important for sensors that try to keep
    // track of the previous state somehow.
    //for (int sensor = 0; sensor < TotalSensorTypes; sensor++)
    for (int sensor = 0; sensor < 1; sensor++)
    {
        left_sensors_[sensor]->reset(this,last_update_time_);
    }

    // Reset position controllers.
    right_pos_controller_->reset(last_update_time_);
    left_pos_controller_->reset(last_update_time_);

    // Reset trial controller, if any.
    if (trial_controller_ != NULL) trial_controller_->reset(last_update_time_);
    for (unsigned i = 0; i < left_arm_joint_states_.size(); i++) {
        left_arm_pos_[i] =  left_arm_joint_states_[i].getPosition();
      }
    ROS_INFO_STREAM("left arm joint pos: "<<left_arm_pos_);
    for (unsigned i = 0; i < right_arm_joint_states_.size(); i++) {
        right_arm_pos_[i] =  right_arm_joint_states_[i].getPosition();
      }
    ROS_INFO_STREAM("right arm joint pos: "<<right_arm_pos_);
    ROS_INFO_STREAM("EGMRIYumiPlugin starting end");
}

// This is called by the controller manager before stopping the controller.
void EGMRIYumiPlugin::stopping(const ros::Time& time)
// void EGMRIYumiPlugin::stopping()
{
    // Nothing to do here.
}

// This is the main update function called by the realtime thread when the controller is running.
void EGMRIYumiPlugin::update(const ros::Time& time, const ros::Duration& period)
// void EGMRIYumiPlugin::update()
{
    // ROS_INFO_STREAM("EGMRIYumiPlugin update begin");
    // Get current time.
    //last_update_time_ = robot_->getTime();
    ros::Duration dur = time - last_update_time_;
    last_update_time_ = time;
    current_controller_period_ms_ += (int)(dur.toNSec()*1e-6);
    //ROS_INFO_STREAM("Plugin update time: "+to_string(last_update_time_));

    // Check if this is a controller step based on the current controller frequency.
    controller_counter_++;
    //bool is_controller_step = (controller_counter_ == 0);
    if (current_controller_period_ms_ >= controller_period_ms_){
      controller_counter_ = 0;
      //ROS_INFO_STREAM("controller update tick"<<current_controller_period_ms_);
      current_controller_period_ms_ = 0;
    }
    bool is_controller_step = (current_controller_period_ms_ == 0);
    // bool is_controller_step = true;

    // Update the sensors and fill in the current step sample.
    update_sensors(last_update_time_,is_controller_step);

    // Update the controllers.
    update_controllers(last_update_time_,is_controller_step);

    // Store the torques.
    // compute friction torque here

    for (unsigned i = 0; i < left_arm_joint_states_.size(); i++) {
        // double speed;
        // double sign = 0;
        left_arm_pos_[i] =  left_arm_joint_states_[i].getPosition();
        // speed = left_arm_joint_states_[i].getVelocity();
        // if (speed > 0.0) sign = 1.0;
        // else if (speed < 0.0) sign = -1.0;
        // else sign = 0.0;
        // left_arm_fric_trq_[i] = sign*left_arm_stat_fric_[i]*0.5 + speed*left_arm_dyn_fric_[i]*0;
        // if (left_arm_torques_[i]==0) left_arm_fric_trq_[i] = 0;
        left_arm_fric_trq_[i] = (controller_counter_%2)?(left_arm_stat_fric_[i]*left_arm_stat_fric_percent[i]):(left_arm_stat_fric_[i]*-left_arm_stat_fric_percent[i]);
        left_arm_joint_states_[i].setCommand(left_arm_torques_[i]);
        //left_arm_joint_states_[i].setCommand(left_arm_torques_[i] + left_arm_fric_trq_[i]);
        // left_arm_joint_states_[i].setCommand(0);
      }
    // ROS_INFO_STREAM_THROTTLE(1,"left arm joint pos: "<<left_arm_pos_);
    // ROS_INFO_STREAM_THROTTLE(1,"left arm joint torques: "<<left_arm_torques_);


    for (unsigned i = 0; i < right_arm_joint_states_.size(); i++) {
        //right_arm_joint_states_[i].setCommand(right_arm_torques_[i]);
        right_arm_pos_[i] =  right_arm_joint_states_[i].getPosition();
        right_arm_fric_trq_[i] = (controller_counter_%2)?(right_arm_stat_fric_[i]*right_arm_stat_fric_percent[i]):(right_arm_stat_fric_[i]*-right_arm_stat_fric_percent[i]);
        right_arm_joint_states_[i].setCommand(right_arm_torques_[i]);
        //right_arm_joint_states_[i].setCommand(right_arm_torques_[i] + right_arm_fric_trq_[i]);
        // right_arm_joint_states_[i].setCommand(0);
      }
    // ROS_INFO_STREAM_THROTTLE(1,"right arm joint pos: "<<right_arm_pos_);
    // ROS_INFO_STREAM_THROTTLE(1,"right arm joint torques: "<<right_arm_torques_);
    //ROS_INFO_STREAM("EGMRIYumiPlugin update end");
}

// Get current time.
ros::Time EGMRIYumiPlugin::get_current_time() const
{
    // return last_update_time_;
}

// Get current encoder readings (robot-dependent).
void EGMRIYumiPlugin::get_joint_encoder_readings(Eigen::VectorXd &angles, egmri::ActuatorType arm) const
{
    if (arm == egmri::RIGHT_ARM)
    {
        if (angles.rows() != right_arm_joint_states_.size())
            angles.resize(right_arm_joint_states_.size());
        for (unsigned i = 0; i < angles.size(); i++)
            angles(i) = right_arm_joint_states_[i].getPosition();
        //ROS_INFO_STREAM_THROTTLE(1,"right arm joint angles: "<<angles);
    }
    else if (arm == egmri::LEFT_ARM)
    {
        if (angles.rows() != left_arm_joint_states_.size())
            angles.resize(left_arm_joint_states_.size());
        for (unsigned i = 0; i < angles.size(); i++){
            angles(i) = left_arm_joint_states_[i].getPosition();
            //ROS_INFO_STREAM("trial arm joint angle "+to_string(i)+": "+to_string(angle(i)));}
            //ROS_DEBUG_STREAM_THROTTLE(10,"trial arm joint angle %d: %f",i,angles(i));
          }
          // ROS_INFO_STREAM_THROTTLE(1,"trial arm joint angles: "<<angles);
    }
    else
    {
        ROS_ERROR("Unknown ArmType %i requested for joint encoder readings!",arm);
    }
}

// Get current encoder state readings (robot-dependent).
void EGMRIYumiPlugin::get_joint_state_readings(Eigen::VectorXd &angles, Eigen::VectorXd &angle_velocities, egmri::ActuatorType arm) const
{
    if (arm == egmri::RIGHT_ARM)
    {
        if (angles.rows() != right_arm_joint_states_.size())
            angles.resize(right_arm_joint_states_.size());
        for (unsigned i = 0; i < angles.size(); i++)
            angles(i) = right_arm_joint_states_[i].getPosition();

        if (angle_velocities.rows() != right_arm_joint_states_.size())
            angle_velocities.resize(right_arm_joint_states_.size());
        for (unsigned i = 0; i < angle_velocities.size(); i++)
            angle_velocities(i) = right_arm_joint_states_[i].getVelocity();
        //ROS_INFO_STREAM_THROTTLE(1,"right arm joint angles: "<<angles);
    }
    else if (arm == egmri::LEFT_ARM)
    {
        if (angles.rows() != left_arm_joint_states_.size())
            angles.resize(left_arm_joint_states_.size());
        for (unsigned i = 0; i < angles.size(); i++){
            angles(i) = left_arm_joint_states_[i].getPosition();
          }

        if (angle_velocities.rows() != left_arm_joint_states_.size())
            angle_velocities.resize(left_arm_joint_states_.size());
        for (unsigned i = 0; i < angle_velocities.size(); i++)
            angle_velocities(i) = left_arm_joint_states_[i].getVelocity();
        //   // ROS_INFO_STREAM_THROTTLE(1,"trial arm joint angles: "<<angles);
    }
    else
    {
        ROS_ERROR("Unknown ArmType %i requested for joint encoder readings!",arm);
    }
}

}

// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(egmri_control::EGMRIYumiPlugin, controller_interface::ControllerBase)
