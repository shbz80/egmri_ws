/*
Controller that executes a trial, using a control strategy that is defined in
a subclass.
*/
#pragma once

// Headers.
#include <vector>
#include <Eigen/Dense>
#include <boost/scoped_ptr.hpp>

#include "egmri/proto/egmri.pb.h"

// Superclass.
#include "egmri_controller_pkg/controller.h"

namespace egmri_control
{

class DualTrialController : public Controller
{
private:
    // Current time step.
    int t_;
    // Holds the last step of a trial
    int trial_end_step_;
    // Current time step.
    boost::scoped_ptr<Sample> current_step_left_;
    boost::scoped_ptr<Sample> current_step_right_;
    // Trajectory sample.
    boost::scoped_ptr<Sample> sample_left_;
    boost::scoped_ptr<Sample> sample_right_;
    // State and obs datatypes
    std::vector<egmri::SampleType> state_datatypes_;
    std::vector<egmri::SampleType> obs_datatypes_;
    // end effector target (subtracted before control is computed)
    // Eigen::VectorXd ee_tgt_;


protected:
    bool is_configured_;

public:
    ros::Time last_update_time_;
    // Counter for time step increment.
    int step_counter_;
    egmri::ActuatorType arm_;
    // Constructor.
    DualTrialController();
    // Destructor.
    virtual ~DualTrialController();
    // Compute the action at the current time step.
    // virtual void get_action(int t, const Eigen::VectorXd &X, const Eigen::VectorXd &obs, Eigen::VectorXd &U) = 0;
    // Update the controller (take an action).
    virtual void update(RobotPlugin *plugin,
                        ros::Time current_time,
                        boost::scoped_ptr<Sample>& sample_left,
                        boost::scoped_ptr<Sample>& sample_right,
                        Eigen::VectorXd &torques_left,
                        Eigen::VectorXd &torques_right);
    // Update the controller
    virtual void update_control_command(const int &step_count,
                                        ros::Time current_time,
                                        const Eigen::VectorXd &X_left,
                                        const Eigen::VectorXd &X_right,
                                        Eigen::VectorXd &torques_left,
                                        Eigen::VectorXd &torques_right) = 0;
    // Configure the controller.
    virtual void configure_controller(OptionsMap &options);
    // Check if controller is finished with its current task.
    virtual bool is_finished() const;
    // Return trial step index
    virtual int get_step_counter();
    // Return length of trial.
    virtual int get_trial_length();
    // Called when controller is turned on
    virtual void reset(ros::Time update_time);

    const bool is_configured(){
        return is_configured_;
    }

};

}
