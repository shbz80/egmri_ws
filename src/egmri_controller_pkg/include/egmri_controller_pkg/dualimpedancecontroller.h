/*
Controller that executes a trial, using a control strategy that is defined in
a subclass.
*/
#pragma once

// Headers.
#include <vector>
#include <Eigen/Dense>
// #include <boost/scoped_ptr.hpp>
#include "egmri/proto/egmri.pb.h"

#include "egmri_controller_pkg/dualtrialcontroller.h"

namespace egmri_control
{

      class DualImpedanceController : public DualTrialController
      {
      private:
          // P gains.
          Eigen::VectorXd pd_gains_p_;
          // D gains.
          Eigen::VectorXd pd_gains_d_;
          // I gains.
          Eigen::VectorXd pd_gains_i_;
          // Integral terms.
          Eigen::VectorXd pd_integral_left_;
          Eigen::VectorXd pd_integral_right_;
          Eigen::VectorXd i_clamp_;
          // Maximum joint velocities.
          Eigen::VectorXd max_velocities_;
          // Temporary storage for joint angle offset.
          Eigen::VectorXd temp_angles_left_;
          Eigen::VectorXd temp_angles_right_;
          // Current target (joint space).
          Eigen::VectorXd target_angles_left_;
          Eigen::VectorXd target_angles_right_;
          Eigen::VectorXd dest_angles_left_;
          Eigen::VectorXd dest_angles_right_;


          // Latest joint angles.
          Eigen::VectorXd current_angles_left_;
          Eigen::VectorXd current_angles_right_;
          // Latest joint angle velocities.
          Eigen::VectorXd current_angle_velocities_left_;
          Eigen::VectorXd current_angle_velocities_right_;

      public:
          // Constructor.
          DualImpedanceController();
          // Destructor.
          virtual ~DualImpedanceController();
          // Compute the action at the current time step.
          // virtual void get_action(int t, const Eigen::VectorXd &X, const Eigen::VectorXd &obs, Eigen::VectorXd &U) = 0;
          // Update the controller (take an action).
          // Update the controller
          virtual void update_control_command(const int &step_count,
                                              ros::Time current_time,
                                              const Eigen::VectorXd &X_left,
                                              const Eigen::VectorXd &X_right,
                                              Eigen::VectorXd &torques_left,
                                              Eigen::VectorXd &torques_right);
          // Configure the controller.
          virtual void configure_controller(OptionsMap &options);
          // Check if controller is finished with its current task.
      };

}
