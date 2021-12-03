//
// File: Joint.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 01-Dec-2021 14:28:21
//
#ifndef JOINT_H
#define JOINT_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "executePlan_types.h"

// Function Declarations
extern void Joint(boolean_T useTwist, const double csi[18][6], const double
                  varargin_1[18], double H[4][4], double varargout_1[18][6]);

#endif

//
// File trailer for Joint.h
//
// [EOF]
//
