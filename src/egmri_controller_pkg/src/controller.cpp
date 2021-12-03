#include "egmri_controller_pkg/controller.h"

using namespace egmri_control;

// Constructors.
Controller::Controller(ros::NodeHandle& n, egmri::ActuatorType arm, int size)
{
}

Controller::Controller()
{
}

// Destructor.
Controller::~Controller()
{
}

void Controller::update(RobotPlugin *plugin, ros::Time current_time, boost::scoped_ptr<Sample>& sample, Eigen::VectorXd &torques)
{
}

void Controller::configure_controller(OptionsMap &options)
{
}

void Controller::set_update_delay(double new_step_length)
{
}

double Controller::get_update_delay()
{
    return 1.0;
}

void Controller::reset(ros::Time update_time)
{
}
