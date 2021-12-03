//
// File: InvAdjoint.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 01-Dec-2021 14:28:21
//

// Include Files
#include "InvAdjoint.h"
#include "Skew.h"
#include "executePlan.h"

// Function Definitions

//
// InvAdjoint  Description.
//    iAd=InvAdjoint(Htm) or [iAd,diAd,ddiAd,...]=InvAdjoint(Htm,ad,dad,...).
//
//    See also: .
//    Source: Murray et al. - A Mathematical Introduction to Robotic
//    Manipulation (book)
//    Implemented by Gianluca Garofalo.
// Arguments    : const double Htm[4][4]
//                double varargout_1[6][6]
// Return Type  : void
//
void InvAdjoint(const double Htm[4][4], double varargout_1[6][6])
{
  double dv[3][3];
  int i;
  double R_tra[3][3];
  double b_R_tra[3][3];
  double c_R_tra[3][3];
  Skew(*(double (*)[3])&Htm[3][0], dv);
  for (i = 0; i < 3; i++) {
    R_tra[i][0] = Htm[0][i];
    b_R_tra[i][0] = -Htm[0][i];
    R_tra[i][1] = Htm[1][i];
    b_R_tra[i][1] = -Htm[1][i];
    R_tra[i][2] = Htm[2][i];
    b_R_tra[i][2] = -Htm[2][i];
  }

  for (i = 0; i < 3; i++) {
    for (int i1 = 0; i1 < 3; i1++) {
      c_R_tra[i1][i] = (b_R_tra[0][i] * dv[i1][0] + b_R_tra[1][i] * dv[i1][1]) +
        b_R_tra[2][i] * dv[i1][2];
      varargout_1[i][i1] = R_tra[i][i1];
    }
  }

  for (i = 0; i < 3; i++) {
    varargout_1[i + 3][0] = c_R_tra[i][0];
    varargout_1[i][3] = 0.0;
    varargout_1[i + 3][3] = R_tra[i][0];
    varargout_1[i + 3][1] = c_R_tra[i][1];
    varargout_1[i][4] = 0.0;
    varargout_1[i + 3][4] = R_tra[i][1];
    varargout_1[i + 3][2] = c_R_tra[i][2];
    varargout_1[i][5] = 0.0;
    varargout_1[i + 3][5] = R_tra[i][2];
  }

  //  Computing derivatives: - ad * iAd, ...
}

//
// File trailer for InvAdjoint.cpp
//
// [EOF]
//
