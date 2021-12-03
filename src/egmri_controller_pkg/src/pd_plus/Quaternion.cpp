//
// File: Quaternion.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 01-Dec-2021 14:28:21
//

// Include Files
#include "Quaternion.h"
#include "Skew.h"
#include "executePlan.h"
#include "executePlan_data.h"

// Function Definitions

//
// Quaternion  Description.
//    [R,w,...]=Quaternion(Q,dQ,...) or [Q,dQ,...]=Quaternion(R,w,...).
//
//    See also: .
//    Source: Siciliano et al. - Robotics (book)
//    Implemented by Gianluca Garofalo.
// Arguments    : const double varargin_1[4]
//                double varargout_1[3][3]
// Return Type  : void
//
void Quaternion(const double varargin_1[4], double varargout_1[3][3])
{
  double a;
  double dv[3][3];
  a = 2.0 * (varargin_1[0] * varargin_1[0]) - 1.0;
  Skew(*(double (*)[3])&varargin_1[1], dv);
  for (int i = 0; i < 3; i++) {
    double varargout_1_tmp;
    varargout_1_tmp = varargin_1[i + 1];
    varargout_1[i][0] = a * static_cast<double>(iv[i][0]) + 2.0 * (varargin_1[1]
      * varargout_1_tmp + varargin_1[0] * dv[i][0]);
    varargout_1[i][1] = a * static_cast<double>(iv[i][1]) + 2.0 * (varargin_1[2]
      * varargout_1_tmp + varargin_1[0] * dv[i][1]);
    varargout_1[i][2] = a * static_cast<double>(iv[i][2]) + 2.0 * (varargin_1[3]
      * varargout_1_tmp + varargin_1[0] * dv[i][2]);
  }
}

//
// File trailer for Quaternion.cpp
//
// [EOF]
//
