//
// File: DynamicMatrices.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 01-Dec-2021 14:28:21
//

// Include Files
#include "DynamicMatrices.h"
#include "DerProd.h"
#include "DerSum.h"
#include "InvAdjoint.h"
#include "ListGet.h"
#include "executePlan.h"
#include <cstring>

// Function Definitions

//
// DynamicMatrices  Description.
//    [xg,L,dL,...,M,dM,..,C,...,g,dg,...]=
//    DynamicMatrices(choice,com,Mass,Htm,J,dJ,...,dy,...), where xg is the
//    position of the CoM with the 4th element equal to the total mass and L
//    is a matrix that provides the total spatial momentum when multiplied by
//    the velocity.
//
//    See also: .
//    Source: Garofalo et al. - On the closed form computation of the dynamic
//    matrices and their differentiations (paper)
//    Implemented by Gianluca Garofalo.
// Arguments    : const double com[76]
//                const double Mass[114][114]
//                const double Htm[4][76]
//                const double varargin_1[18][114]
//                double xg[4]
//                double varargout_1[18][6]
//                double varargout_2[18][18]
//                double varargout_3[18]
// Return Type  : void
//
void DynamicMatrices(const double com[76], const double Mass[114][114], const
                     double Htm[4][76], const double varargin_1[18][114], double
                     xg[4], double varargout_1[18][6], double varargout_2[18][18],
                     double varargout_3[18])
{
  double com_s[4];
  int i;
  int Mass_tmp;
  signed char ind_4c[4];
  double dv[18];
  double ind_6c[6];
  signed char B_tmp[6];
  double B[6][6];
  double b_Mass[4][4];
  double varargout_6[18][6];
  double b_com[4];
  double d;
  double dv1[6][6];
  double b_B[18][6];
  double dv2[6][6];

  //  Is OverloadSym needed?
  //  Initialization
  com_s[0] = 0.0;
  com_s[1] = 0.0;
  com_s[2] = 0.0;
  com_s[3] = 0.0;
  for (i = 0; i < 18; i++) {
    for (Mass_tmp = 0; Mass_tmp < 18; Mass_tmp++) {
      varargout_2[i][Mass_tmp] = 0.0;
    }

    for (Mass_tmp = 0; Mass_tmp < 6; Mass_tmp++) {
      varargout_1[i][Mass_tmp] = 0.0;
    }

    dv[i] = static_cast<double>(i) + 1.0;
  }

  for (int k = 0; k < 19; k++) {
    signed char i1;
    int i2;
    i1 = static_cast<signed char>(((k + 1) << 2) - 3);
    ind_4c[0] = i1;
    ind_4c[1] = static_cast<signed char>(i1 + 1);
    ind_4c[2] = static_cast<signed char>(i1 + 2);
    ind_4c[3] = static_cast<signed char>(i1 + 3);
    i = 6 * (k + 1);
    for (Mass_tmp = 0; Mass_tmp < 6; Mass_tmp++) {
      i2 = (i + Mass_tmp) - 5;
      ind_6c[Mass_tmp] = i2;
      B_tmp[Mass_tmp] = static_cast<signed char>(i2);
    }

    for (i = 0; i < 6; i++) {
      for (Mass_tmp = 0; Mass_tmp < 6; Mass_tmp++) {
        B[i][Mass_tmp] = Mass[B_tmp[i] - 1][B_tmp[Mass_tmp] - 1];
      }
    }

    Mass_tmp = B_tmp[0] - 1;
    for (i = 0; i < 4; i++) {
      b_Mass[i][0] = Mass[Mass_tmp][Mass_tmp] * Htm[i][i1 - 1];
      b_Mass[i][1] = Mass[Mass_tmp][Mass_tmp] * Htm[i][ind_4c[1] - 1];
      b_Mass[i][2] = Mass[Mass_tmp][Mass_tmp] * Htm[i][ind_4c[2] - 1];
      b_Mass[i][3] = Mass[Mass_tmp][Mass_tmp] * Htm[i][ind_4c[3] - 1];
      b_com[i] = com[ind_4c[i] - 1];
    }

    for (i = 0; i < 4; i++) {
      com_s[i] += ((b_Mass[0][i] * b_com[0] + b_Mass[1][i] * b_com[1]) + b_Mass
                   [2][i] * b_com[2]) + b_Mass[3][i] * b_com[3];
    }

    ListGet(ind_6c, dv, varargin_1, varargout_6);
    for (i = 0; i < 6; i++) {
      for (Mass_tmp = 0; Mass_tmp < 18; Mass_tmp++) {
        d = 0.0;
        for (i2 = 0; i2 < 6; i2++) {
          d += B[i2][i] * varargout_6[Mass_tmp][i2];
        }

        b_B[Mass_tmp][i] = d;
      }
    }

    for (i = 0; i < 18; i++) {
      for (Mass_tmp = 0; Mass_tmp < 18; Mass_tmp++) {
        d = 0.0;
        for (i2 = 0; i2 < 6; i2++) {
          d += varargout_6[i][i2] * b_B[Mass_tmp][i2];
        }

        varargout_2[Mass_tmp][i] += d;
      }
    }

    for (i = 0; i < 4; i++) {
      b_Mass[i][0] = Htm[i][i1 - 1];
      b_Mass[i][1] = Htm[i][ind_4c[1] - 1];
      b_Mass[i][2] = Htm[i][ind_4c[2] - 1];
      b_Mass[i][3] = Htm[i][ind_4c[3] - 1];
    }

    InvAdjoint(b_Mass, dv1);
    for (i = 0; i < 6; i++) {
      for (Mass_tmp = 0; Mass_tmp < 6; Mass_tmp++) {
        d = 0.0;
        for (i2 = 0; i2 < 6; i2++) {
          d += dv1[i][i2] * B[Mass_tmp][i2];
        }

        dv2[Mass_tmp][i] = d;
      }
    }

    std::memcpy(&b_B[0][0], &varargout_6[0][0], 108U * sizeof(double));
    DerProd(dv2, b_B, varargout_6);
    std::memcpy(&b_B[0][0], &varargout_1[0][0], 108U * sizeof(double));
    DerSum(b_B, varargout_6, varargout_1);
  }

  xg[0] = com_s[0] / com_s[3];
  xg[1] = com_s[1] / com_s[3];
  xg[2] = com_s[2] / com_s[3];
  xg[3] = com_s[3];

  //  com_s(4) = total mass of the robot
  for (i = 0; i < 18; i++) {
    varargout_3[i] = 9.81 * varargout_1[i][2];
  }
}

//
// File trailer for DynamicMatrices.cpp
//
// [EOF]
//
