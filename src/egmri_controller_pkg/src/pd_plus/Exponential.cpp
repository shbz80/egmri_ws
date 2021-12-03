//
// File: Exponential.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 01-Dec-2021 14:28:21
//

// Include Files
#include "Exponential.h"
#include "Skew.h"
#include "all.h"
#include "cos.h"
#include "executePlan.h"
#include "eye.h"
#include "sin.h"

// Function Definitions

//
// Exponential  Description.
//    Description.
//
//    See also: .
//    Source: Murray et al. - A Mathematical Introduction to Robotic
//    Manipulation (book)
//    Implemented by Gianluca Garofalo.
// Arguments    : const double csi[6]
//                double theta
//                double g[4][4]
// Return Type  : void
//
void Exponential(const double csi[6], double theta, double g[4][4])
{
  double b_I[3][3];
  boolean_T b_csi[3];
  double w_hat[3][3];
  double b;
  double d;
  double R_minus_I[3][3];
  double b_R_minus_I[3][3];
  b_eye(b_I);
  b_csi[0] = (csi[3] == 0.0);
  b_csi[1] = (csi[4] == 0.0);
  b_csi[2] = (csi[5] == 0.0);
  if (d_all(b_csi)) {
    for (int i = 0; i < 3; i++) {
      g[i][0] = b_I[i][0];
      g[i][1] = b_I[i][1];
      g[i][2] = b_I[i][2];
      g[3][i] = csi[i] * theta;
    }

    g[0][3] = 0.0;
    g[1][3] = 0.0;
    g[2][3] = 0.0;
    g[3][3] = 1.0;
  } else {
    double c_csi;
    int i;
    int i1;
    c_csi = (csi[3] * csi[0] + csi[4] * csi[1]) + csi[5] * csi[2];
    Skew(*(double (*)[3])&csi[3], w_hat);
    b = theta;
    b_sin(&b);
    d = theta;
    b_cos(&d);
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < 3; i1++) {
        R_minus_I[i1][i] = (w_hat[0][i] * w_hat[i1][0] + w_hat[1][i] * w_hat[i1]
                            [1]) + w_hat[2][i] * w_hat[i1][2];
      }
    }

    for (i = 0; i < 3; i++) {
      double d1;
      d1 = w_hat[i][0] * b + R_minus_I[i][0] * (1.0 - d);
      R_minus_I[i][0] = d1;
      b_R_minus_I[i][0] = -d1;
      d1 = w_hat[i][1] * b + R_minus_I[i][1] * (1.0 - d);
      R_minus_I[i][1] = d1;
      b_R_minus_I[i][1] = -d1;
      d1 = w_hat[i][2] * b + R_minus_I[i][2] * (1.0 - d);
      R_minus_I[i][2] = d1;
      b_R_minus_I[i][2] = -d1;
    }

    for (i = 0; i < 3; i++) {
      d = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        d += ((b_R_minus_I[0][i] * w_hat[i1][0] + b_R_minus_I[1][i] * w_hat[i1]
               [1]) + b_R_minus_I[2][i] * w_hat[i1][2]) * csi[i1];
        g[i][i1] = b_I[i][i1] + R_minus_I[i][i1];
      }

      g[3][i] = d + c_csi * csi[i + 3] * theta;
    }

    g[0][3] = 0.0;
    g[1][3] = 0.0;
    g[2][3] = 0.0;
    g[3][3] = 1.0;
  }
}

//
// File trailer for Exponential.cpp
//
// [EOF]
//
