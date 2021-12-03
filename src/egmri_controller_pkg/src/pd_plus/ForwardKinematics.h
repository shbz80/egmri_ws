//
// File: ForwardKinematics.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 01-Dec-2021 14:28:21
//
#ifndef FORWARDKINEMATICS_H
#define FORWARDKINEMATICS_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "executePlan_types.h"

// Function Declarations
extern void ForwardKinematics(boolean_T useTwist, const double parent[19], const
  double g0[4][76], const double csi[18][114], const double varargin_1[18],
  double Htm[4][76], double varargout_1[18][114]);

#endif

//
// File trailer for ForwardKinematics.h
//
// [EOF]
//
