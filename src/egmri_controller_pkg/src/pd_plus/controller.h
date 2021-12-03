//
// File: controller.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 01-Dec-2021 14:28:21
//
#ifndef CONTROLLER_H
#define CONTROLLER_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "executePlan_types.h"

// Function Declarations
extern void controller(const double parent[19], const double g0[4][76], const
  double csi[18][114], const double com[76], const double Mass[114][114], const
  double q[18], const double dq[18], const double qd[18], const double dqd[18],
  double tau[18]);

#endif

//
// File trailer for controller.h
//
// [EOF]
//
