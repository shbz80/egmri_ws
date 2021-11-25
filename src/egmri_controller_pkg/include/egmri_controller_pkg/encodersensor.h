/*
Joint encoder sensor: returns joint angles and, optionally, their velocities.
*/
#pragma once

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

#include "egmri/proto/egmri.pb.h"

// Superclass.
#include "egmri_controller_pkg/sensor.h"
#include "egmri_controller_pkg/sample.h"
#include "egmri_controller_pkg/encoderfilter.h"

// This sensor writes to the following data types:
// JointAngle
// JointVelocity

namespace egmri_control
{

class EncoderSensor: public Sensor
{
private:
    // Previous joint angles.
    Eigen::VectorXd previous_angles_;
    // Previous joint velocities.
    Eigen::VectorXd previous_velocities_;
    // Temporary storage for joint angles.
    Eigen::VectorXd temp_joint_angles_;

    boost::scoped_ptr<EncoderFilter> joint_filter_;
    // Time from last update when the previous angles were recorded (necessary to compute velocities).
    ros::Time previous_angles_time_;
    // which arm is this EncoderSensor for?
    egmri::ActuatorType actuator_type_;
public:
    // Constructor.
    EncoderSensor(ros::NodeHandle& n, RobotPlugin *plugin, egmri::ActuatorType actuator_type);
    // Destructor.
    virtual ~EncoderSensor();
    // Update the sensor (called every tick).
    virtual void update(RobotPlugin *plugin, ros::Time current_time, bool is_controller_step);
    // Set data format and meta data on the provided sample.
    virtual void set_sample_data_format(boost::scoped_ptr<Sample>& sample);
    // Set data on the provided sample.
    virtual void set_sample_data(boost::scoped_ptr<Sample>& sample, int t);
};

}
