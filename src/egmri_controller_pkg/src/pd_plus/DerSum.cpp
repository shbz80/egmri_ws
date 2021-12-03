//
// File: DerSum.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 01-Dec-2021 14:28:21
//

// Include Files
#include "DerSum.h"
#include "executePlan.h"

// Function Definitions

//
// DerSum  Description.
//    [p,dp,ddp,...]=DerSum(a,da,dda,...,b,db,ddb,...,c,dc,ddc,...).
//
//    See also: .
//    Implemented by Gianluca Garofalo.
// Arguments    : const double varargin_1[18][6]
//                const double varargin_2[18][6]
//                double varargout_1[18][6]
// Return Type  : void
//
void DerSum(const double varargin_1[18][6], const double varargin_2[18][6],
            double varargout_1[18][6])
{
  for (int i = 0; i < 18; i++) {
    for (int i1 = 0; i1 < 6; i1++) {
      varargout_1[i][i1] = (0.0 * varargin_1[i][i1] + varargin_1[i][i1]) +
        varargin_2[i][i1];
    }
  }
}

//
// File trailer for DerSum.cpp
//
// [EOF]
//
