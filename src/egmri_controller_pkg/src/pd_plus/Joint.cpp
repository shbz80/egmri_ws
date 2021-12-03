//
// File: Joint.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 01-Dec-2021 14:28:21
//

// Include Files
#include "Joint.h"
#include "Exponential.h"
#include "InvAdjoint.h"
#include "Quaternion.h"
#include "Skew.h"
#include "abs.h"
#include "all.h"
#include "any1.h"
#include "deal.h"
#include "executePlan.h"
#include "executePlan_data.h"
#include "eye.h"
#include "mtimes.h"
#include "sqrt.h"
#include <cstring>

// Function Definitions

//
// Joint  Description.
//    [H,T,dT,ddT,...]=Joint(csi,q,dq,ddq,...).
//
//    See also: .
//    Implemented by Gianluca Garofalo.
// Arguments    : boolean_T useTwist
//                const double csi[18][6]
//                const double varargin_1[18]
//                double H[4][4]
//                double varargout_1[18][6]
// Return Type  : void
//
void Joint(boolean_T useTwist, const double csi[18][6], const double varargin_1
           [18], double H[4][4], double varargout_1[18][6])
{
  int i;
  static const signed char b_iv[4][4] = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0,
      1, 0 }, { 0, 0, 0, 1 } };

  int b_i;
  int n;
  int k;
  signed char ind[6];
  boolean_T hasSphericalJoint;
  boolean_T firstSpherical;
  boolean_T b_csi[6];
  double testSpherical[3];
  double idx[3];
  boolean_T c_csi[3][3];
  boolean_T b_ind[3];
  int b_k;
  double Q[4];
  double dv[6][6];
  double R[3][3];
  double dv1[4][4];
  double E[3][3];
  double dv2[4][4];
  double Hr[4][4];
  double Ht[4][4];

  //  Is OverloadSym needed?
  //  Initialization
  for (i = 0; i < 4; i++) {
    H[i][0] = b_iv[i][0];
    H[i][1] = b_iv[i][1];
    H[i][2] = b_iv[i][2];
    H[i][3] = b_iv[i][3];
  }

  for (i = 0; i < 18; i++) {
    for (b_i = 0; b_i < 6; b_i++) {
      varargout_1[i][b_i] = 0.0 * csi[i][b_i];
    }
  }

  //  Find requires variable size array, therefore here replaced with loop
  n = -1;
  for (b_i = 0; b_i < 6; b_i++) {
    ind[b_i] = 0;
  }

  for (k = 0; k < 18; k++) {
    for (i = 0; i < 6; i++) {
      b_csi[i] = (csi[k][i] != 0.0);
    }

    if (any(b_csi)) {
      n++;
      ind[n] = static_cast<signed char>(k + 1);
    }
  }

  //  There is a spherical joint if csi has 3 orthogonal rotational axis at the
  //  beginning or the end of its columns
  deal(false, &hasSphericalJoint, &firstSpherical);

  //  ismember requires variable size array, therefore replaced by loop
  //  testSpherical = ismember( csi(4:end,:).', E, 'rows' );
  testSpherical[0] = 0.0;
  testSpherical[1] = 0.0;
  testSpherical[2] = 0.0;
  for (b_i = 0; b_i < 16; b_i++) {
    idx[0] = static_cast<double>(b_i) + 1.0;
    idx[1] = (static_cast<double>(b_i) + 1.0) + 1.0;
    idx[2] = (static_cast<double>(b_i) + 1.0) + 2.0;
    for (i = 0; i < 3; i++) {
      b_k = static_cast<int>(idx[i]) - 1;
      c_csi[i][0] = (csi[b_k][3] == iv[i][0]);
      c_csi[i][1] = (csi[b_k][4] == iv[i][1]);
      c_csi[i][2] = (csi[b_k][5] == iv[i][2]);
    }

    all(c_csi, b_ind);
    if (b_all(b_ind)) {
      testSpherical[0] = static_cast<double>(b_i) + 1.0;
      testSpherical[1] = (static_cast<double>(b_i) + 1.0) + 1.0;
      testSpherical[2] = (static_cast<double>(b_i) + 1.0) + 2.0;
    }
  }

  if (c_all(testSpherical)) {
    for (i = 0; i < 3; i++) {
      b_k = ind[i] - 1;
      c_csi[i][0] = (csi[b_k][3] == iv[i][0]);
      c_csi[i][1] = (csi[b_k][4] == iv[i][1]);
      c_csi[i][2] = (csi[b_k][5] == iv[i][2]);
    }

    all(c_csi, b_ind);
    firstSpherical = b_all(b_ind);
    if (firstSpherical) {
      hasSphericalJoint = true;
    } else {
      for (i = 0; i < 3; i++) {
        b_k = ind[(i + n) - 2] - 1;
        c_csi[i][0] = (csi[b_k][3] == iv[i][0]);
        c_csi[i][1] = (csi[b_k][4] == iv[i][1]);
        c_csi[i][2] = (csi[b_k][5] == iv[i][2]);
      }

      all(c_csi, b_ind);
      if (b_all(b_ind)) {
        hasSphericalJoint = true;
      } else {
        hasSphericalJoint = false;
      }
    }
  }

  if (useTwist && hasSphericalJoint) {
    idx[0] = 0.0;
    idx[1] = 0.0;
    idx[2] = 0.0;
    Q[0] = 0.0;
    Q[1] = 0.0;
    Q[2] = 0.0;
    Q[3] = 0.0;

    //  Find the center of rotation
    //  Compute the quaternion and associated rotation matrix
    for (k = 0; k < 3; k++) {
      b_i = static_cast<int>(testSpherical[k]) - 1;
      Skew(*(double (*)[3])&csi[b_i][0], R);
      for (i = 0; i < 3; i++) {
        idx[i] -= (0.5 * R[0][i] * static_cast<double>(iv[k][0]) + 0.5 * R[1][i]
                   * static_cast<double>(iv[k][1])) + 0.5 * R[2][i] *
          static_cast<double>(iv[k][2]);
      }

      Q[k + 1] = varargin_1[b_i];
    }

    Q[0] = b_abs(1.0 - mtimes(*(double (*)[3])&Q[1], *(double (*)[3])&Q[1]));
    b_sqrt(&Q[0]);
    Quaternion(Q, R);

    //  Compute HTM of the sperical joint
    for (i = 0; i < 3; i++) {
      E[i][0] = static_cast<double>(iv[i][0]) - R[i][0];
      E[i][1] = static_cast<double>(iv[i][1]) - R[i][1];
      E[i][2] = static_cast<double>(iv[i][2]) - R[i][2];
    }

    for (i = 0; i < 3; i++) {
      Hr[i][0] = R[i][0];
      Hr[i][1] = R[i][1];
      Hr[i][2] = R[i][2];
      Hr[3][i] = (E[0][i] * idx[0] + E[1][i] * idx[1]) + E[2][i] * idx[2];
    }

    Hr[0][3] = 0.0;
    Hr[1][3] = 0.0;
    Hr[2][3] = 0.0;
    Hr[3][3] = 1.0;

    //  Compute HTM of the prismatic joint
    eye(Ht);
    i = static_cast<int>(((-1.0 - static_cast<double>(n + 1)) + 1.0) / -1.0);
    for (k = 0; k < i; k++) {
      b_i = n - k;
      b_ind[0] = (ind[b_i] == static_cast<int>(testSpherical[0]));
      b_ind[1] = (ind[b_i] == static_cast<int>(testSpherical[1]));
      b_ind[2] = (ind[b_i] == static_cast<int>(testSpherical[2]));
      if (!b_any(b_ind)) {
        b_i = ind[b_i] - 1;
        Exponential(*(double (*)[6])&csi[b_i][0], varargin_1[b_i], dv1);
        for (b_i = 0; b_i < 4; b_i++) {
          for (int i1 = 0; i1 < 4; i1++) {
            dv2[i1][b_i] = ((dv1[0][b_i] * Ht[i1][0] + dv1[1][b_i] * Ht[i1][1])
                            + dv1[2][b_i] * Ht[i1][2]) + dv1[3][b_i] * Ht[i1][3];
          }
        }

        for (b_i = 0; b_i < 4; b_i++) {
          Ht[b_i][0] = dv2[b_i][0];
          Ht[b_i][1] = dv2[b_i][1];
          Ht[b_i][2] = dv2[b_i][2];
          Ht[b_i][3] = dv2[b_i][3];
        }
      }
    }

    if (firstSpherical) {
      for (i = 0; i < 4; i++) {
        for (b_i = 0; b_i < 4; b_i++) {
          H[b_i][i] = ((Hr[0][i] * Ht[b_i][0] + Hr[1][i] * Ht[b_i][1]) + Hr[2][i]
                       * Ht[b_i][2]) + Hr[3][i] * Ht[b_i][3];
        }
      }
    } else {
      for (i = 0; i < 4; i++) {
        for (b_i = 0; b_i < 4; b_i++) {
          H[b_i][i] = ((Ht[0][i] * Hr[b_i][0] + Ht[1][i] * Hr[b_i][1]) + Ht[2][i]
                       * Hr[b_i][2]) + Ht[3][i] * Hr[b_i][3];
        }
      }
    }

    //  csi is already the Jacobian matrix
    std::memcpy(&varargout_1[0][0], &csi[0][0], 108U * sizeof(double));
  } else {
    i = static_cast<int>(((-1.0 - static_cast<double>(n + 1)) + 1.0) / -1.0);
    for (k = 0; k < i; k++) {
      int varargout_1_tmp;
      int i1;
      b_k = n - k;

      //  Step 1: twist = T*dq
      //  Step 2: ad = LieBracket(twist)
      //  Step 3: iAd, -ad*iAd, ....
      //  Step 4: dT(:,ind(k)) = -ad*iAd*csi(:,ind(k))
      InvAdjoint(H, dv);
      varargout_1_tmp = ind[b_k] - 1;
      for (b_i = 0; b_i < 6; b_i++) {
        varargout_1[varargout_1_tmp][b_i] = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          varargout_1[varargout_1_tmp][b_i] += dv[i1][b_i] * csi[varargout_1_tmp]
            [i1];
        }
      }

      b_i = ind[b_k] - 1;
      Exponential(*(double (*)[6])&csi[b_i][0], varargin_1[b_i], dv1);
      for (b_i = 0; b_i < 4; b_i++) {
        for (i1 = 0; i1 < 4; i1++) {
          dv2[i1][b_i] = ((dv1[0][b_i] * H[i1][0] + dv1[1][b_i] * H[i1][1]) +
                          dv1[2][b_i] * H[i1][2]) + dv1[3][b_i] * H[i1][3];
        }
      }

      for (b_i = 0; b_i < 4; b_i++) {
        H[b_i][0] = dv2[b_i][0];
        H[b_i][1] = dv2[b_i][1];
        H[b_i][2] = dv2[b_i][2];
        H[b_i][3] = dv2[b_i][3];
      }
    }
  }
}

//
// File trailer for Joint.cpp
//
// [EOF]
//
