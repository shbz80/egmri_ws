#include "egmri_controller_pkg/robotplugin.h"
#include "egmri_controller_pkg/sensor.h"
#include "egmri_controller_pkg/controller.h"
#include "egmri_controller_pkg/positioncontroller.h"
#include "egmri_controller_pkg/trialcontroller.h"
#include "egmri_controller_pkg/tfcontroller.h"
#include "egmri_controller_pkg/TfParams.h"
#include "egmri_controller_pkg/ControllerParams.h"
#include "egmri_controller_pkg/util.h"
#include "egmri/proto/egmri.pb.h"
#include <vector>

using namespace egmri_control;

// Plugin constructor.
RobotPlugin::RobotPlugin()
{
    // Nothing to do here, since all variables are initialized in initialize(...)
}

// Destructor.
RobotPlugin::~RobotPlugin()
{
    // Nothing to do here, since all instance variables are destructed automatically.
}

// Initialize everything.
void RobotPlugin::initialize(ros::NodeHandle& n)
{
    ROS_INFO_STREAM("Initializing RobotPlugin");
    trial_data_request_waiting_ = false;
    aux_data_request_waiting_ = false;
    sensors_initialized_ = false;
    controller_initialized_ = false;

    // Initialize all ROS communication infrastructure.
    initialize_ros(n);

    // Initialize all sensors.
    initialize_sensors(n);

    // Initialize the position controllers.
    // Note that the trial controllers are created from scratch for each trial.
    // However, the position controllers persist, since there is only one type.
    initialize_position_controllers(n);

    // After this, we still need to create the kinematics solvers. How these are
    // created depends on the particular robot, and should be implemented in a
    // subclass.
}

// Initialize ROS communication infrastructure.
void RobotPlugin::initialize_ros(ros::NodeHandle& n)
{
    ROS_INFO_STREAM("Initializing ROS subs/pubs");
    // Create subscribers.
    position_subscriber_ = n.subscribe("/egmri_controller_position_command", 1, &RobotPlugin::position_subscriber_callback, this);
    trial_subscriber_ = n.subscribe("/egmri_controller_trial_command", 1, &RobotPlugin::trial_subscriber_callback, this);
    test_sub_ = n.subscribe("/test_sub", 1, &RobotPlugin::test_callback, this);
    relax_subscriber_ = n.subscribe("/egmri_controller_relax_command", 1, &RobotPlugin::relax_subscriber_callback, this);
    data_request_subscriber_ = n.subscribe("/egmri_controller_data_request", 1, &RobotPlugin::data_request_subscriber_callback, this);

    // Create publishers.
    report_publisher_.reset(new realtime_tools::RealtimePublisher<egmri_controller_pkg::SampleResult>(n, "/egmri_controller_report", 1));

    //for async tf controller.
    action_subscriber_tf_ = n.subscribe("/egmri_controller_sent_robot_action_tf", 1, &RobotPlugin::tf_robot_action_command_callback, this);
    tf_publisher_.reset(new realtime_tools::RealtimePublisher<egmri_controller_pkg::TfObsData>(n, "/egmri_obs_tf", 1));
}

// Initialize all sensors.
void RobotPlugin::initialize_sensors(ros::NodeHandle& n)
{
    ROS_INFO_STREAM("Init sensors");
    // Clear out the old sensors.
    sensors_.clear();

    // Create all sensors.
    for (int i = 0; i < 1; i++)
    // TODO: ZDM: read this when more sensors work
    //for (int i = 0; i < TotalSensorTypes; i++)
    {
        ROS_INFO_STREAM("creating sensor: " + to_string(i));
        boost::shared_ptr<Sensor> sensor(Sensor::create_sensor((SensorType)i,n,this, egmri::LEFT_ARM));
        sensors_.push_back(sensor);
    }

    // Create current state sample and populate it using the sensors.
    current_time_step_sample_.reset(new Sample(MAX_TRIAL_LENGTH));
    initialize_sample(current_time_step_sample_, egmri::LEFT_ARM);

    aux_sensors_.clear();
    // Create all auxiliary sensors.  Currently only an encodersensor
    for (int i = 0; i < 1; i++)
    {
        ROS_INFO_STREAM("creating auxiliary sensor: " + to_string(i));
        boost::shared_ptr<Sensor> sensor(Sensor::create_sensor((SensorType)i,n,this, egmri::RIGHT_ARM));
        aux_sensors_.push_back(sensor);
    }

    // Create current state sample and populate it using the sensors.
    aux_current_time_step_sample_.reset(new Sample(1));
    initialize_sample(aux_current_time_step_sample_, egmri::RIGHT_ARM);

    sensors_initialized_ = true;
}


// Helper method to configure all sensors
void RobotPlugin::configure_sensors()
{
    ROS_INFO("configure sensors");
    sensors_initialized_ = false;
    for (int i = 0; i < sensors_.size(); i++)
    {
        sensors_[i]->set_sample_data_format(current_time_step_sample_);
    }
    // Set sample data format on the actions, which are not handled by any sensor.
    OptionsMap sample_metadata;
    current_time_step_sample_->set_meta_data(
        egmri::ACTION,left_arm_torques_.size(),SampleDataFormatEigenVector,sample_metadata);

    // configure auxiliary sensors
    for (int i = 0; i < aux_sensors_.size(); i++)
    {
        aux_sensors_[i]->set_sample_data_format(aux_current_time_step_sample_);
    }
    sensors_initialized_ = true;
}

// Initialize position controllers.
void RobotPlugin::initialize_position_controllers(ros::NodeHandle& n)
{
    // Create right arm position controller.
    // TODO: fix this to be something that comes out of the robot itself
    right_pos_controller_.reset(new PositionController(n, egmri::RIGHT_ARM, 7));

    // Create left arm position controller.
    left_pos_controller_.reset(new PositionController(n, egmri::LEFT_ARM, 7));
}

// // Helper function to initialize a sample from the current sensors.
void RobotPlugin::initialize_sample(boost::scoped_ptr<Sample>& sample, egmri::ActuatorType actuator_type)
{
    // Go through all of the sensors and initialize metadata.
    if (actuator_type == egmri::LEFT_ARM)
    {
        for (int i = 0; i < sensors_.size(); i++)
        {
            sensors_[i]->set_sample_data_format(sample);
        }
        // Set sample data format on the actions, which are not handled by any sensor.
        OptionsMap sample_metadata;
        sample->set_meta_data(egmri::ACTION,left_arm_torques_.size(),SampleDataFormatEigenVector,sample_metadata);
    }
    else if (actuator_type == egmri::RIGHT_ARM)
    {
        for (int i = 0; i < aux_sensors_.size(); i++)
        {
            aux_sensors_[i]->set_sample_data_format(sample);
        }
    }
    ROS_INFO("set sample data format");
}

// Update the sensors at each time step.
void RobotPlugin::update_sensors(ros::Time current_time, bool is_controller_step)
{
    if (!sensors_initialized_) return; // Don't try to use sensors until initialization finishes.

    // Update all of the sensors and fill in the sample.
    for (int sensor = 0; sensor < sensors_.size(); sensor++)
    {
        sensors_[sensor]->update(this, current_time, is_controller_step);
        if (trial_controller_ != NULL){
            sensors_[sensor]->set_sample_data(current_time_step_sample_,
                trial_controller_->get_step_counter());
        }
        else {
            sensors_[sensor]->set_sample_data(current_time_step_sample_, 0);
        }
    }

    // Update all of the auxiliary sensors and fill in the sample.
    for (int sensor = 0; sensor < aux_sensors_.size(); sensor++)
    {
        aux_sensors_[sensor]->update(this, current_time, is_controller_step);
        aux_sensors_[sensor]->set_sample_data(aux_current_time_step_sample_, 0);
    }

    // If a data request is waiting, publish the sample.
    if (trial_data_request_waiting_) {
        publish_sample_report(current_time_step_sample_);
        trial_data_request_waiting_ = false;
    }

    if (aux_data_request_waiting_) {
        publish_sample_report(aux_current_time_step_sample_);
        aux_data_request_waiting_ = false;
    }
}

// Update the controllers at each time step.
void RobotPlugin::update_controllers(ros::Time current_time, bool is_controller_step)
{
    // Update right arm controller.
    // TODO - don't pass in wrong sample if used
    right_pos_controller_->update(this, current_time, current_time_step_sample_, right_arm_torques_);

    bool trial_init = trial_controller_ != NULL && trial_controller_->is_configured() && controller_initialized_;
    if(!is_controller_step && trial_init){
        return;
    }

    // If we have a trial controller, update that, otherwise update position controller.
    if (trial_init) trial_controller_->update(this, current_time, current_time_step_sample_, left_arm_torques_);
    else left_pos_controller_->update(this, current_time, current_time_step_sample_, left_arm_torques_);

    // Check if the trial controller finished and delete it.
    if (trial_init && trial_controller_->is_finished()) {

        // Publish sample after trial completion
        publish_sample_report(current_time_step_sample_, trial_controller_->get_trial_length());
        //Clear the trial controller.
        trial_controller_->reset(current_time);
        trial_controller_.reset(NULL);

        // Set the left arm controller to NO_CONTROL.
        OptionsMap options;
        options["mode"] = egmri::NO_CONTROL;
        left_pos_controller_->configure_controller(options);

        // Switch the sensors to run at full frequency.
        for (int sensor = 0; sensor < TotalSensorTypes; sensor++)
        {
            //sensors_[sensor]->set_update(left_pos_controller_->get_update_delay());
        }
        ROS_INFO_STREAM("Set the left arm controller to NO_CONTROL");
    }
    if (left_pos_controller_->report_waiting){
        if (left_pos_controller_->is_finished()){
          // ROS_INFO_STREAM("left_pos_controller_report_init");
            publish_sample_report(current_time_step_sample_);
            left_pos_controller_->report_waiting = false;
        }
    }
    if (right_pos_controller_->report_waiting){
        if (right_pos_controller_->is_finished()){
            publish_sample_report(current_time_step_sample_);
            right_pos_controller_->report_waiting = false;
        }
    }

}

void RobotPlugin::publish_sample_report(boost::scoped_ptr<Sample>& sample, int T /*=1*/){
    while(!report_publisher_->trylock());
    std::vector<egmri::SampleType> dtypes;
    sample->get_available_dtypes(dtypes);

    report_publisher_->msg_.sensor_data.resize(dtypes.size());
    for(int d=0; d<dtypes.size(); d++){ //Fill in each sample type
        report_publisher_->msg_.sensor_data[d].data_type = dtypes[d];
        Eigen::VectorXd tmp_data;
        sample->get_data(T, tmp_data, (egmri::SampleType)dtypes[d]);
        report_publisher_->msg_.sensor_data[d].data.resize(tmp_data.size());


        std::vector<int> shape;
        sample->get_shape((egmri::SampleType)dtypes[d], shape);
        shape.insert(shape.begin(), T);
        report_publisher_->msg_.sensor_data[d].shape.resize(shape.size());
        int total_expected_shape = 1;
        for(int i=0; i< shape.size(); i++){
            report_publisher_->msg_.sensor_data[d].shape[i] = shape[i];
            total_expected_shape *= shape[i];
        }
        if(total_expected_shape != tmp_data.size()){
            ROS_ERROR("Data stored in sample has different length than expected (%d vs %d)",
                    tmp_data.size(), total_expected_shape);
        }
        for(int i=0; i<tmp_data.size(); i++){
            report_publisher_->msg_.sensor_data[d].data[i] = tmp_data[i];
        }
    }
    report_publisher_->unlockAndPublish();
    ROS_INFO_STREAM("report_published");
}
//
void RobotPlugin::position_subscriber_callback(const egmri_controller_pkg::PositionCommand::ConstPtr& msg){

    ROS_INFO_STREAM("received position command");
    OptionsMap params;
    int8_t arm = msg->arm;
    params["mode"] = msg->mode;
    Eigen::VectorXd data;
    data.resize(msg->data.size());
    for(int i=0; i<data.size(); i++){
        data[i] = msg->data[i];
    }
    params["data"] = data;

    Eigen::MatrixXd pd_gains;
    pd_gains.resize(msg->pd_gains.size() / 4, 4);
    for(int i=0; i<pd_gains.rows(); i++){
        for(int j=0; j<4; j++){
            pd_gains(i, j) = msg->pd_gains[i * 4 + j];
        }
    }
    params["pd_gains"] = pd_gains;

    if(arm == egmri::LEFT_ARM){
        left_pos_controller_->configure_controller(params);
    }else if (arm == egmri::RIGHT_ARM){
        right_pos_controller_->configure_controller(params);
    }else{
        ROS_ERROR("Unknown position controller arm type");
    }
}
//
void RobotPlugin::trial_subscriber_callback(const egmri_controller_pkg::TrialCommand::ConstPtr& msg){

    OptionsMap controller_params;
    ROS_INFO_STREAM("received trial command");

    controller_initialized_ = false;

    //Read out trial information
    uint32_t T = msg->T;  // Trial length
    if (T > MAX_TRIAL_LENGTH) {
        ROS_FATAL("Trial length specified is longer than maximum trial length (%d vs %d)",
                T, MAX_TRIAL_LENGTH);
    }

    initialize_sample(current_time_step_sample_, egmri::LEFT_ARM);

    float frequency = msg->frequency;  // Controller frequency

    // Update sensor frequency
    for (int sensor = 0; sensor < sensors_.size(); sensor++)
    {
        sensors_[sensor]->set_update(1.0/frequency);
    }

    std::vector<int> state_datatypes, obs_datatypes;
    int arm_datatype;
    state_datatypes.resize(msg->state_datatypes.size());
    for(int i=0; i<state_datatypes.size(); i++){
        state_datatypes[i] = msg->state_datatypes[i];
    }
    controller_params["state_datatypes"] = state_datatypes;
    obs_datatypes.resize(msg->obs_datatypes.size());
    for(int i=0; i<obs_datatypes.size(); i++){
        obs_datatypes[i] = msg->obs_datatypes[i];
    }
    controller_params["obs_datatypes"] = obs_datatypes;
    arm_datatype = msg->arm_datatype;
    controller_params["arm_datatype"] = arm_datatype;
    if (msg->controller.controller_to_execute == egmri::TF_CONTROLLER) {
        trial_controller_.reset(new TfController());
        controller_params["T"] = (int)msg->T;
        egmri_controller_pkg::TfParams tfparams = msg->controller.tf;
        int dU = (int) tfparams.dU;
        controller_params["dU"] = dU;
        trial_controller_-> configure_controller(controller_params);
    }
    else if (msg->controller.controller_to_execute == egmri::IMP_CONTROLLER) {
      // initialize and configure the impedance controller
    }
    else{
        ROS_ERROR("Unknown trial controller arm type");
    }

    configure_sensors();

    controller_initialized_ = true;
}
//
void RobotPlugin::test_callback(const std_msgs::Empty::ConstPtr& msg){
    ROS_INFO_STREAM("Received test message");
}
//
void RobotPlugin::relax_subscriber_callback(const egmri_controller_pkg::RelaxCommand::ConstPtr& msg){

    ROS_INFO_STREAM("received relax command");
    OptionsMap params;
    int8_t arm = msg->arm;
    params["mode"] = egmri::NO_CONTROL;

    if(arm == egmri::LEFT_ARM){
        left_pos_controller_->configure_controller(params);
    }else if (arm == egmri::RIGHT_ARM){
        right_pos_controller_->configure_controller(params);
    }else{
        ROS_ERROR("Unknown position controller arm type");
    }
}
//
void RobotPlugin::data_request_subscriber_callback(const egmri_controller_pkg::DataRequest::ConstPtr& msg) {
    ROS_INFO_STREAM("received data request");
    OptionsMap params;
    int arm = msg->arm;
    if (arm < 2 && arm >= 0)
    {
        egmri::ActuatorType arm_type = (egmri::ActuatorType) arm;
        if (arm_type == egmri::LEFT_ARM)
        {
            trial_data_request_waiting_ = true;
        }
        else if (arm_type == egmri::RIGHT_ARM)
        {
            aux_data_request_waiting_ = true;
        }
    }
    else
    {
        ROS_INFO("Data request arm type not valid: %d", arm);
    }
}

// Get sensor.
Sensor *RobotPlugin::get_sensor(SensorType sensor, egmri::ActuatorType actuator_type)
{
    // TODO: ZDM: make this work for multiple sensors of each type -- pass in int instead of sensortype?
    if(actuator_type == egmri::LEFT_ARM)
    {
        assert(sensor < TotalSensorTypes);
        return sensors_[sensor].get();
    }
    else if (actuator_type == egmri::RIGHT_ARM)
    {
        assert((int)sensor < aux_sensors_.size());
        return aux_sensors_[sensor].get();
    }
}

// // Get forward kinematics solver.
// void RobotPlugin::get_fk_solver(boost::shared_ptr<KDL::ChainFkSolverPos> &fk_solver, boost::shared_ptr<KDL::ChainJntToJacSolver> &jac_solver, egmri::ActuatorType arm)
// {
//     if (arm == egmri::RIGHT_ARM)
//     {
//         fk_solver = right_arm_fk_solver_;
//         jac_solver = right_arm_jac_solver_;
//     }
//     else if (arm == egmri::LEFT_ARM)
//     {
//         fk_solver = left_arm_fk_solver_;
//         jac_solver = left_arm_jac_solver_;
//     }
//     else
//     {
//         ROS_ERROR("Unknown ArmType %i requested for joint encoder readings!",arm);
//     }
// }

void RobotPlugin::tf_robot_action_command_callback(const egmri_controller_pkg::TfActionCommand::ConstPtr& msg){

    bool trial_init = trial_controller_ != NULL && trial_controller_->is_configured();
    if(trial_init){
        // Unpack the action vector
        int idx = 0;
        int dU = (int)msg->dU;
        Eigen::VectorXd latest_action_command;
        latest_action_command.resize(dU);
        for (int i = 0; i < dU; ++i)
        {
            latest_action_command[i] = msg->action[i];
            idx++;
        }
        int last_command_id_received = msg ->id;
        trial_controller_->update_action_command(last_command_id_received, latest_action_command);

    }

}
//
void RobotPlugin::tf_publish_obs(Eigen::VectorXd obs){
    while(!tf_publisher_->trylock());
    tf_publisher_->msg_.data.resize(obs.size());
    for(int i=0; i<obs.size(); i++) {
        tf_publisher_->msg_.data[i] = obs[i];
    }
    tf_publisher_->unlockAndPublish();
}
