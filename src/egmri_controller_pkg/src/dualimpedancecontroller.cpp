#include "egmri_controller_pkg/robotplugin.h"
#include "egmri_controller_pkg/dualimpedancecontroller.h"
#include "egmri_controller_pkg/util.h"
#include "controller.h"

using namespace egmri_control;

// Constructor.
DualImpedanceController::DualImpedanceController() : DualTrialController()
{
    // Set initial time.
    is_configured_ = false;
    // temp pid stuff
    pd_gains_p_.resize(7);
    pd_gains_d_.resize(7);
    pd_gains_i_.resize(7);

    // Initialize velocity bounds.
    max_velocities_.resize(7);

    // Initialize integral terms to zero.
    pd_integral_left_.resize(7);
    pd_integral_right_.resize(7);
    i_clamp_.resize(7);
    pd_integral_left_.fill(0.0);
    pd_integral_right_.fill(0.0);

    // Initialize current angle and position.
    current_angles_left_.resize(7);
    current_angle_velocities_left_.resize(7);
    current_angle_velocities_left_.setZero();
    current_angles_right_.resize(7);
    current_angle_velocities_right_.resize(7);
    current_angle_velocities_right_.setZero();


    // Initialize target angle and position.
    target_angles_left_.resize(7);
    dest_angles_left_.resize(7);
    target_angles_left_.setZero();
    dest_angles_left_.setZero();
    target_angles_right_.resize(7);
    dest_angles_right_.resize(7);
    target_angles_right_.setZero();
    dest_angles_right_.setZero();

        // Initialize joints temporary storage.
    temp_angles_left_.resize(7);
    temp_angles_right_.resize(7);

    last_update_time_ = ros::Time(0.0);
    // temp pid stuff




}

// Destructor.
DualImpedanceController::~DualImpedanceController() {
}


void DualImpedanceController::configure_controller(OptionsMap &options)
{
  //Call superclass
  DualTrialController::configure_controller(options);
  // temp pid stuff
  Eigen::VectorXd pos_left = boost::get<Eigen::VectorXd>(options["pos_left"]);
  ROS_INFO_STREAM("target left: "<<pos_left);
  Eigen::VectorXd pos_right = boost::get<Eigen::VectorXd>(options["pos_right"]);
  ROS_INFO_STREAM("target right: "<<pos_right);
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
  dest_angles_left_ = pos_left;
  dest_angles_right_ = pos_right;
  // dest_angles_left_ << -1.5924f, -1.0417f,  1.4975f,  0.1163f,  2.0738f,  1.1892f, -2.4352f;
  // dest_angles_right_ << 1.5924f, -1.0417f,  -1.4975f,  0.1163f,  -2.0738f,  1.1892f, -2.4352f;
  // dest_angles_right_ = {0,0,0,0,0,0,0};
  // dest_angles_right_ = pos_right;

  // temp pid stuff
  is_configured_ = true;

}

void DualImpedanceController::update_control_command(const int &step_count,
                                    ros::Time current_time,
                                    const Eigen::VectorXd &X_left,
                                    const Eigen::VectorXd &X_right,
                                    Eigen::VectorXd &torques_left,
                                    Eigen::VectorXd &torques_right)
{
  // arm_   variable that holds values LEFT_ARM, RIGHT_ARM or BOTH according to
  // the recieved command. You can check for the ritgh type as
  // if (arm_ == egmri::LEFT_ARM) {}
  // use method get_trial_length() to get the trial length in steps.
  // control frequecy is setup in yumitplugin.cpp
  // X_left and X_right are readonly state vectors
  // write computed torques to torques_left and torques_right

   ROS_INFO_STREAM("PD_PLUS begin");

  // for (int i = 0; i < 7; i++){
  //   temp_angles_left_(i) = X_left(i);
  //   current_angle_velocities_left_(i) = X_left(7+i);
  //   temp_angles_right_(i) = X_right(i);
  //   current_angle_velocities_right_(i) = X_right(7+i);
  // }
  //
  // // Estimate joint angle velocities.
  // double update_time = current_time.toSec() - last_update_time_.toSec();
  //
  // if (arm_==egmri::LEFT_ARM || arm_==egmri::BOTH){
  //   if (!last_update_time_.isZero())
  //   { // Only compute velocities if we have a previous sample.
  //       // current_angle_velocities_left_ = (temp_angles_left_ - current_angles_left_)/update_time;
  //   }
  //   else{
  //       update_time = 0.001;
  //   }
  //     // ROS_INFO_STREAM_THROTTLE(1, "update_time:"<<update_time);
  //   // Store new angles.
  //   current_angles_left_ = temp_angles_left_;
  //
  //   // computer current angle to dest. This is added to slow down the speed
  //   temp_angles_left_ = dest_angles_left_ - current_angles_left_;
  //   for(int i=0;i<7;i++){
  //     target_angles_left_(i) = std::min(std::abs(temp_angles_left_(i)),ANGLE)*((temp_angles_left_(i) >= 0.0) ? 1.0 : -1.0) + current_angles_left_(i);
  //   }
  //
  //   // Compute error.
  //   temp_angles_left_ = current_angles_left_ - target_angles_left_;
  //   // ROS_INFO_STREAM_THROTTLE(0.5, "error angles:"<<temp_angles_);
  //
  //   // Add to integral term.
  //   pd_integral_left_ += temp_angles_left_ * update_time;
  //
  //   // Clamp integral term
  //   for (int i = 0; i < temp_angles_left_.rows(); i++){
  //       if (pd_integral_left_(i) * pd_gains_i_(i) > i_clamp_(i)) {
  //           pd_integral_left_(i) = i_clamp_(i) / pd_gains_i_(i);
  //       }
  //       else if (pd_integral_left_(i) * pd_gains_i_(i) < -i_clamp_(i)) {
  //           pd_integral_left_(i) = -i_clamp_(i) / pd_gains_i_(i);
  //       }
  //   }
  //
  //   // Compute torques.
  //   torques_left = -((pd_gains_p_.array() * temp_angles_left_.array()) +
  //               (pd_gains_d_.array() * current_angle_velocities_left_.array()) +
  //               (pd_gains_i_.array() * pd_integral_left_.array())).matrix();
  // }
  //
  // if (arm_==egmri::RIGHT_ARM || arm_==egmri::BOTH){
  //   if (!last_update_time_.isZero())
  //   { // Only compute velocities if we have a previous sample.
  //       // current_angle_velocities_right_ = (temp_angles_right_ - current_angles_right_)/update_time;
  //   }
  //   else{
  //       update_time = 0.001;
  //   }
  //   // Store new angles.
  //   current_angles_right_ = temp_angles_right_;
  //
  //   // computer current angle to dest. This is added to slow down the speed
  //   temp_angles_right_ = dest_angles_right_ - current_angles_right_;
  //   for(int i=0;i<7;i++){
  //     target_angles_right_(i) = std::min(std::abs(temp_angles_right_(i)),ANGLE)*((temp_angles_right_(i) >= 0.0) ? 1.0 : -1.0) + current_angles_right_(i);
  //   }
  //
  //   // Compute error.
  //   temp_angles_right_ = current_angles_right_ - target_angles_right_;
  //   // ROS_INFO_STREAM_THROTTLE(0.5, "error angles:"<<temp_angles_);
  //
  //   // Add to integral term.
  //   pd_integral_right_ += temp_angles_right_ * update_time;
  //
  //   // Clamp integral term
  //   for (int i = 0; i < temp_angles_right_.rows(); i++){
  //       if (pd_integral_right_(i) * pd_gains_i_(i) > i_clamp_(i)) {
  //           pd_integral_right_(i) = i_clamp_(i) / pd_gains_i_(i);
  //       }
  //       else if (pd_integral_right_(i) * pd_gains_i_(i) < -i_clamp_(i)) {
  //           pd_integral_right_(i) = -i_clamp_(i) / pd_gains_i_(i);
  //       }
  //   }
  //   // Compute torques.
  //   torques_right = -((pd_gains_p_.array() * temp_angles_right_.array()) +
  //               (pd_gains_d_.array() * current_angle_velocities_right_.array()) +
  //               (pd_gains_i_.array() * pd_integral_right_.array())).matrix();
  //
  // }

  // pd_plus
  // desired joint vel


  for (int i=0; i<18; i++) {
    dqd[i] = 0.0;
  }

  ROS_INFO_STREAM("CP1");
  for (int i=0; i<7; i++) {
    if (arm_==egmri::BOTH || arm_==egmri::RIGHT_ARM) qd[i] = dest_angles_right_[i];
    if (arm_==egmri::BOTH || arm_==egmri::LEFT_ARM) qd[i+9] = dest_angles_left_[i];
    // qd[i] = 0;
    // qd[i+9] = 0;
  }

  ROS_INFO_STREAM("CP2");
  qd[7] = 0.0;
  qd[8] = 0.0;
  qd[16] = 0.0;
  qd[17] = 0.0;

ROS_INFO_STREAM("CP3");
  for (int i=0; i<7; i++) {
    q[i] = X_right[i];
    q[i+9] = X_left[i];
    dq[i] = X_right[i+7];
    dq[i+9] = X_left[i+7];
  }

ROS_INFO_STREAM("CP4");
  q[7] = 0.0;
  q[8] = 0.0;
  q[16] = 0.0;
  q[17] = 0.0;
  dq[7] = 0.0;
  dq[8] = 0.0;
  dq[16] = 0.0;
  dq[17] = 0.0;

  ROS_INFO_STREAM("PD_PLUS Init");

  controller(parent, dv1, dv2, dv3, dv4, q, dq, qd, dqd, tau);

  ROS_INFO_STREAM("controller called");

  for (int i=0; i<7; i++) {
    torques_right[i] = tau[i];
    torques_left[i] = tau[i+9];

  }

  ROS_INFO_STREAM("torques ready");

  // Update last update time.
  last_update_time_ = current_time;
}
