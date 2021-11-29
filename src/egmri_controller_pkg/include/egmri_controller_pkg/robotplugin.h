/*
This is the base class for the robot plugin, which takes care of interfacing
with the robot.
*/
#pragma once

// Headers.
#include <vector>
#include <Eigen/Dense>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
// #include <kdl/chain.hpp>
// #include <kdl/chainjnttojacsolver.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>
#include <realtime_tools/realtime_publisher.h>

#include "egmri_controller_pkg/PositionCommand.h"
#include "egmri_controller_pkg/TrialCommand.h"
#include "egmri_controller_pkg/SampleResult.h"
#include "egmri_controller_pkg/DataRequest.h"
#include "egmri_controller_pkg/TfActionCommand.h"
#include "egmri_controller_pkg/TfObsData.h"
#include "egmri_controller_pkg/TfParams.h"
#include "egmri_controller_pkg/sensor.h"
#include "egmri_controller_pkg/controller.h"
#include "egmri_controller_pkg/positioncontroller.h"
#include "egmri/proto/egmri.pb.h"

// Convenience defines.
#define ros_publisher_ptr(X) boost::scoped_ptr<realtime_tools::RealtimePublisher<X> >
#define MAX_TRIAL_LENGTH 2000

namespace egmri_control
{

// Forward declarations.
// Controllers.
class PositionController;
class TrialController;
// Sensors.
class Sensor;
// Sample.
class Sample;
// Custom ROS messages.
class SampleResult;
class PositionCommand;
class TrialCommand;
// class RelaxCommand;


class RobotPlugin
{
protected:
    ros::Time last_update_time_;
    // Temporary storage for left arm torques to be applied at each step.
    Eigen::VectorXd left_arm_torques_;
    // Temporary storage for right arm torques to be applied at each step.
    Eigen::VectorXd  right_arm_torques_;
    // Position controller for right arm.
    boost::scoped_ptr<PositionController> right_pos_controller_;
    // // Position controller for left arm.
    boost::scoped_ptr<PositionController> left_pos_controller_;
    // // Current trial controller (if any).
    boost::scoped_ptr<TrialController> trial_controller_;
    // // Sensor data for the current time step.
    boost::scoped_ptr<Sample> current_time_step_sample_;
    // // Auxiliary Sensor data for the current time step.
    boost::scoped_ptr<Sample> aux_current_time_step_sample_;
    // // Sensors.
    std::vector<boost::shared_ptr<Sensor> > sensors_;
    // // Auxiliary Sensors.
    std::vector<boost::shared_ptr<Sensor> > aux_sensors_;
    // // KDL chains for the end-effectors.
    // KDL::Chain right_arm_fk_chain_, left_arm_fk_chain_;
    // // KDL solvers for the end-effectors.
    // boost::shared_ptr<KDL::ChainFkSolverPos> right_arm_fk_solver_, left_arm_fk_solver_;
    // // KDL solvers for end-effector Jacobians.
    // boost::shared_ptr<KDL::ChainJntToJacSolver> right_arm_jac_solver_, left_arm_jac_solver_;
    // // Subscribers.
    // // Subscriber for position control commands.
    ros::Subscriber position_subscriber_;
    // // Subscriber trial commands.
    ros::Subscriber trial_subscriber_;
    ros::Subscriber test_sub_;
    // // Subscriber for relax commands.
    // ros::Subscriber relax_subscriber_;
    // // Subscriber for current state report request.
    ros::Subscriber data_request_subscriber_;
    // // Publishers.
    // // Publish result of a trial, completion of position command, or just a report.
    ros_publisher_ptr(egmri_controller_pkg::SampleResult) report_publisher_;
    // Is a trial arm data request pending?
    bool trial_data_request_waiting_;
    // Is a auxiliary data request pending?
    bool aux_data_request_waiting_;
    // Are the sensors initialized?
    bool sensors_initialized_;
    // Is everything initialized for the trial controller?
    bool controller_initialized_;
    // //tf publisher
    ros_publisher_ptr(egmri_controller_pkg::TfObsData) tf_publisher_;
    // //tf action subscriber
    ros::Subscriber action_subscriber_tf_;
public:
    // Constructor (this should do nothing).
    RobotPlugin();
    // Destructor.
    virtual ~RobotPlugin();
    // Initialize everything.
    virtual void initialize(ros::NodeHandle& n);
    // Initialize all of the ROS subscribers and publishers.
    virtual void initialize_ros(ros::NodeHandle& n);
    // Initialize all of the position controllers.
    virtual void initialize_position_controllers(ros::NodeHandle& n);
    // Initialize all of the sensors (this also includes FK computation objects).
    virtual void initialize_sensors(ros::NodeHandle& n);
    // TODO: Comment
    virtual void initialize_sample(boost::scoped_ptr<Sample>& sample, egmri::ActuatorType actuator_type);

    //Helper method to configure all sensors
    virtual void configure_sensors();

    // Report publishers
    // Publish a sample with data from up to T timesteps
    virtual void publish_sample_report(boost::scoped_ptr<Sample>& sample, int T=1);
    //
    // // Subscriber callbacks.
    // // Position command callback.
    virtual void position_subscriber_callback(const egmri_controller_pkg::PositionCommand::ConstPtr& msg);
    // // Trial command callback.
    virtual void trial_subscriber_callback(const egmri_controller_pkg::TrialCommand::ConstPtr& msg);
    virtual void test_callback(const std_msgs::Empty::ConstPtr& msg);
    // // Relax command callback.
    // virtual void relax_subscriber_callback(const egmri_controller_pkg::RelaxCommand::ConstPtr& msg);
    // // Data request callback.
    virtual void data_request_subscriber_callback(const egmri_controller_pkg::DataRequest::ConstPtr& msg);
    // //tf callback
    virtual void tf_robot_action_command_callback(const egmri_controller_pkg::TfActionCommand::ConstPtr& msg);

    // Update functions.
    // Update the sensors at each time step.
    virtual void update_sensors(ros::Time current_time, bool is_controller_step);
    // Update the controllers at each time step.
    virtual void update_controllers(ros::Time current_time, bool is_controller_step);
    // Accessors.
    // Get current time.
    virtual ros::Time get_current_time() const = 0;
    // Get sensor
    virtual Sensor *get_sensor(SensorType sensor, egmri::ActuatorType actuator_type);
    // Get current encoder readings (robot-dependent).
    virtual void get_joint_encoder_readings(Eigen::VectorXd &angles, egmri::ActuatorType arm) const = 0;
    // Get forward kinematics solver.
    // virtual void get_fk_solver(boost::shared_ptr<KDL::ChainFkSolverPos> &fk_solver, boost::shared_ptr<KDL::ChainJntToJacSolver> &jac_solver, egmri::ActuatorType arm);

    //tf controller commands.
    //tf publish observation command.
    virtual void tf_publish_obs(Eigen::VectorXd obs);

};

}
