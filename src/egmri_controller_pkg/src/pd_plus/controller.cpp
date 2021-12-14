//
// File: controller.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 01-Dec-2021 14:28:21
//

// Include Files
#include "controller.h"
#include "DynamicMatrices.h"
#include "ForwardKinematics.h"
#include "executePlan.h"

// Function Definitions

//
// Arguments    : const double parent[19]
//                const double g0[4][76]
//                const double csi[18][114]
//                const double com[76]
//                const double Mass[114][114]
//                const double q[18]
//                const double dq[18]
//                const double qd[18]
//                const double dqd[18]
//                double tau[18]
// Return Type  : void
//
void controller(const double parent[19], const double g0[4][76], const double
                csi[18][114], const double com[76], const double Mass[114][114],
                const double q[18], const double dq[18], const double qd[18],
                const double dqd[18], double tau[18])
{
  int i;
  double Htm[4][76];
  double J[18][114];
  double q_[18];
  double xg[4];
  double L[18][6];
  double M[18][18];
  double dq_[18];
  for (i = 0; i < 18; i++) {
    q_[i] = q[i] - qd[i];
    dq_[i] = dq[i] - dqd[i];
  }

  double p_gains[18] = {45, 45, 24, 24, 3, 3, 3, 0, 0, 45, 45, 24, 24, 3, 3, 3, 0, 0};
  double d_gains[18] = {4.5, 4.5, 3, 3, 0.6, 0.6, 0.45, 0, 0, 4.5, 4.5, 3, 3, 0.6, 0.6, 0.45, 0, 0};
  //p_gains = p_gains/3.0 ;
  //d_gains = d_gains/3.0 ;

  ForwardKinematics(false, parent, g0, csi, q, Htm, J);
  DynamicMatrices(com, Mass, Htm, J, xg, L, M, tau);
  for (i = 0; i < 18; i++) {
    tau[i] = (-d_gains[i]/7.5 * dq_[i]) - p_gains[i]/7.5 * q_[i];
    // tau[i] = (tau[i] + 20.0 * dq_[i]) + 400.0 * q_[i];
  }
}

//
// File trailer for controller.cpp
//
// [EOF]
//
