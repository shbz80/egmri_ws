#include "egmri_controller_pkg/encodersensor.h"
#include "egmri_controller_pkg/robotplugin.h"

using namespace egmri_control;

// Constructor.
EncoderSensor::EncoderSensor(ros::NodeHandle& n, RobotPlugin *plugin, egmri::ActuatorType actuator_type): Sensor(n, plugin)
{
    // Set internal arm
    actuator_type_ = actuator_type;

    // Get current joint angles.
    plugin->get_joint_encoder_readings(previous_angles_, actuator_type);

    // Initialize velocities.
    previous_velocities_.resize(previous_angles_.size());

    // Initialize temporary angles.
    temp_joint_angles_.resize(previous_angles_.size());

    // Set time.
    previous_angles_time_ = ros::Time(0.0); // This ignores the velocities on the first step.

    // Initialize and configure Kalman filter
    joint_filter_.reset(new EncoderFilter(n, previous_angles_));
}

// Destructor.
EncoderSensor::~EncoderSensor()
{
    // Nothing to do here.
}

// Update the sensor (called every tick).
void EncoderSensor::update(RobotPlugin *plugin, ros::Time current_time, bool is_controller_step)
{
    double update_time = current_time.toSec() - previous_angles_time_.toSec();

    //REMOVE
    //Eigen::VectorXd temp_temp_joint_angles_, err_temp_joint_angles;
    // Get new vector of joint angles from plugin.
    plugin->get_joint_encoder_readings(temp_joint_angles_, actuator_type_);
    // temp_temp_joint_angles_ = temp_joint_angles_;
    joint_filter_->update(update_time, temp_joint_angles_);
    // joint_filter_->get_state(temp_joint_angles_);
    // err_temp_joint_angles = temp_temp_joint_angles_ - temp_joint_angles_;
    // ROS_DEBUG_STREAM_NAMED("ekf_filt_pos_error",err_temp_joint_angles);
    if (is_controller_step)
    {
        // Get filtered joint angles
        joint_filter_->get_state(temp_joint_angles_);

        // Compute velocities.
        // Note that we can't assume the last angles are actually from one step ago, so we check first.
        // If they are roughly from one step ago, assume the step is correct, otherwise use actual time.

        double update_time = current_time.toSec() - previous_angles_time_.toSec();
        if (!previous_angles_time_.isZero())
        { // Only compute velocities if we have a previous sample.
            if (fabs(update_time)/sensor_step_length_ >= 0.5 &&
                fabs(update_time)/sensor_step_length_ <= 2.0)
            {
                // previous_end_effector_point_velocities_ = (temp_end_effector_points_ - previous_end_effector_points_)/sensor_step_length_;
                for (unsigned i = 0; i < previous_velocities_.size(); i++){
                    previous_velocities_[i] = (temp_joint_angles_[i] - previous_angles_[i])/sensor_step_length_;
                }
            }
            else
            {
                // previous_end_effector_point_velocities_ = (temp_end_effector_points_ - previous_end_effector_points_)/update_time;
                for (unsigned i = 0; i < previous_velocities_.size(); i++){
                    previous_velocities_[i] = (temp_joint_angles_[i] - previous_angles_[i])/update_time;
                }
            }
        }

        // Move temporaries into the previous joint angles.
        // previous_end_effector_points_ = temp_end_effector_points_;
        for (unsigned i = 0; i < previous_angles_.size(); i++){
            previous_angles_[i] = temp_joint_angles_[i];
        }

        // Update stored time.
        previous_angles_time_ = current_time;
    }
}

// Set data format and meta data on the provided sample.
void EncoderSensor::set_sample_data_format(boost::scoped_ptr<Sample>& sample)
{
    // Set joint angles size and format.
    OptionsMap joints_metadata;
    sample->set_meta_data(egmri::JOINT_ANGLES,previous_angles_.size(),SampleDataFormatEigenVector,joints_metadata);

    // Set joint velocities size and format.
    OptionsMap velocities_metadata;
    sample->set_meta_data(egmri::JOINT_VELOCITIES,previous_velocities_.size(),SampleDataFormatEigenVector,joints_metadata);
}

// Set data on the provided sample.
void EncoderSensor::set_sample_data(boost::scoped_ptr<Sample>& sample, int t)
{
    // Set joint angles.
    sample->set_data_vector(t,egmri::JOINT_ANGLES,previous_angles_.data(),previous_angles_.size(),SampleDataFormatEigenVector);

    // Set joint velocities.
    sample->set_data_vector(t,egmri::JOINT_VELOCITIES,previous_velocities_.data(),previous_velocities_.size(),SampleDataFormatEigenVector);
}
