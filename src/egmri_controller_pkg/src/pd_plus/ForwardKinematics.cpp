//
// File: ForwardKinematics.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 01-Dec-2021 14:28:21
//

// Include Files
#include "ForwardKinematics.h"
#include "DerProd.h"
#include "DerSum.h"
#include "InvAdjoint.h"
#include "Joint.h"
#include "ListGet.h"
#include "ListSet.h"
#include "executePlan.h"
#include "eye.h"
#include <cstring>

// Function Definitions

//
// ForwardKinematics  Description.
//    [Htm,Jb,dJb,...]=ForwardKinematics(useTwist,parent,g0,csi,y,dy,...).
//
//    See also: .
//    Source: Garofalo et al. - On the closed form computation of the dynamic
//    matrices and their differentiations (paper)
//    Implemented by Gianluca Garofalo.
// Arguments    : boolean_T useTwist
//                const double parent[19]
//                const double g0[4][76]
//                const double csi[18][114]
//                const double varargin_1[18]
//                double Htm[4][76]
//                double varargout_1[18][114]
// Return Type  : void
//
void ForwardKinematics(boolean_T useTwist, const double parent[19], const double
  g0[4][76], const double csi[18][114], const double varargin_1[18], double Htm
  [4][76], double varargout_1[18][114])
{
  signed char ind_4c[4];
  double ind_6c[6];
  double varargout_4[4][4];
  double ind_4p[4];
  double varargout_3[18][6];
  double b_csi[18][6];
  double varargout_5[18][6];
  double dv[6];
  double dv1[18];
  double b_g0[4][4];
  double gc[4][4];
  double dv2[6][6];

  //  Is OverloadSym needed?
  //  Initialization
  std::memcpy(&Htm[0][0], &g0[0][0], 304U * sizeof(double));
  std::memcpy(&varargout_1[0][0], &csi[0][0], 2052U * sizeof(double));
  for (int k = 0; k < 19; k++) {
    signed char i;
    int i1;
    int i2;
    i = static_cast<signed char>(((k + 1) << 2) - 3);
    ind_4c[0] = i;
    ind_4c[1] = static_cast<signed char>(i + 1);
    ind_4c[2] = static_cast<signed char>(i + 2);
    ind_4c[3] = static_cast<signed char>(i + 3);
    i1 = 6 * (k + 1);
    for (i2 = 0; i2 < 6; i2++) {
      ind_6c[i2] = (i1 + i2) - 5;
    }

    //  Base (link's parent = world)
    if (parent[k] == 0.0) {
      eye(varargout_4);
      for (i1 = 0; i1 < 4; i1++) {
        ind_4p[i1] = ind_4c[i1];
        Htm[i1][i - 1] = varargout_4[i1][0];
        Htm[i1][ind_4c[1] - 1] = varargout_4[i1][1];
        Htm[i1][ind_4c[2] - 1] = varargout_4[i1][2];
        Htm[i1][ind_4c[3] - 1] = varargout_4[i1][3];
      }

      std::memset(&varargout_3[0][0], 0, 108U * sizeof(double));
    } else {
      double d;
      d = 4.0 * parent[k];
      ind_4p[0] = d + -3.0;
      ind_4p[1] = d + -2.0;
      ind_4p[2] = d + -1.0;
      ind_4p[3] = d;
      d = 6.0 * parent[k];
      for (i1 = 0; i1 < 6; i1++) {
        dv[i1] = d + (static_cast<double>(i1) + -5.0);
      }

      for (i1 = 0; i1 < 18; i1++) {
        dv1[i1] = static_cast<double>(i1) + 1.0;
      }

      ListGet(dv, dv1, varargout_1, varargout_3);
    }

    for (i1 = 0; i1 < 18; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        b_csi[i1][i2] = csi[i1][static_cast<int>(ind_6c[i2]) - 1];
      }
    }

    Joint(useTwist, b_csi, varargin_1, varargout_4, varargout_5);
    for (i1 = 0; i1 < 4; i1++) {
      b_g0[i1][0] = g0[i1][i - 1];
      b_g0[i1][1] = g0[i1][ind_4c[1] - 1];
      b_g0[i1][2] = g0[i1][ind_4c[2] - 1];
      b_g0[i1][3] = g0[i1][ind_4c[3] - 1];
    }

    for (i1 = 0; i1 < 4; i1++) {
      for (i2 = 0; i2 < 4; i2++) {
        gc[i2][i1] = ((b_g0[0][i1] * varargout_4[i2][0] + b_g0[1][i1] *
                       varargout_4[i2][1]) + b_g0[2][i1] * varargout_4[i2][2]) +
          b_g0[3][i1] * varargout_4[i2][3];
      }
    }

    InvAdjoint(gc, dv2);
    std::memcpy(&b_csi[0][0], &varargout_3[0][0], 108U * sizeof(double));
    DerProd(dv2, b_csi, varargout_3);
    std::memcpy(&b_csi[0][0], &varargout_3[0][0], 108U * sizeof(double));
    DerSum(b_csi, varargout_5, varargout_3);
    for (i1 = 0; i1 < 4; i1++) {
      varargout_4[i1][0] = Htm[i1][static_cast<int>(ind_4p[0]) - 1];
      varargout_4[i1][1] = Htm[i1][static_cast<int>(ind_4p[1]) - 1];
      varargout_4[i1][2] = Htm[i1][static_cast<int>(ind_4p[2]) - 1];
      varargout_4[i1][3] = Htm[i1][static_cast<int>(ind_4p[3]) - 1];
    }

    for (i1 = 0; i1 < 4; i1++) {
      int Htm_tmp;
      Htm_tmp = ind_4c[i1] - 1;
      for (i2 = 0; i2 < 4; i2++) {
        Htm[i2][Htm_tmp] = 0.0;
        Htm[i2][Htm_tmp] += varargout_4[0][i1] * gc[i2][0];
        Htm[i2][Htm_tmp] += varargout_4[1][i1] * gc[i2][1];
        Htm[i2][Htm_tmp] += varargout_4[2][i1] * gc[i2][2];
        Htm[i2][Htm_tmp] += varargout_4[3][i1] * gc[i2][3];
      }
    }

    for (i1 = 0; i1 < 18; i1++) {
      dv1[i1] = static_cast<double>(i1) + 1.0;
    }

    ListSet(ind_6c, dv1, varargout_1, varargout_3);
  }
}

//
// File trailer for ForwardKinematics.cpp
//
// [EOF]
//
