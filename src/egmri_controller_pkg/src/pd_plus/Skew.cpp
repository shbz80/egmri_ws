//
// File: Skew.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 01-Dec-2021 14:28:21
//

// Include Files
#include "Skew.h"
#include "executePlan.h"

// Function Definitions

//
// Skew  Skew symmetric matrix.
//    out=Skew(in) calculates the skew symmetric matrix of a 3D vector.
//
//    See also: .
//    Source: Murray et al. - A Mathematical Introduction to Robotic
//    Manipulation (book)
//    Implemented by Gianluca Garofalo.
// Arguments    : const double in[3]
//                double out[3][3]
// Return Type  : void
//
void Skew(const double in[3], double out[3][3])
{
  out[0][0] = 0.0;
  out[1][0] = -in[2];
  out[2][0] = in[1];
  out[0][1] = in[2];
  out[1][1] = 0.0;
  out[2][1] = -in[0];
  out[0][2] = -in[1];
  out[1][2] = in[0];
  out[2][2] = 0.0;
}

//
// File trailer for Skew.cpp
//
// [EOF]
//
