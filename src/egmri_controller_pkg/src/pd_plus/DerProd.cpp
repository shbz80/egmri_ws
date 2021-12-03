//
// File: DerProd.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 01-Dec-2021 14:28:21
//

// Include Files
#include "DerProd.h"
#include "executePlan.h"

// Function Declarations
static void DerProd_core(const double varargin_1[6][6], const double varargin_2
  [18][6], double varargout_1[18][6]);

// Function Definitions

//
// DerProd_core  Description.
//    P=DerProd_core(order,a,da,dda,...,b,db,ddb,...,c,dc,ddc,...) where
//    P=[p;dp;ddp;...].
//
//    See also: .
//    Implemented by Gianluca Garofalo.
// Arguments    : const double varargin_1[6][6]
//                const double varargin_2[18][6]
//                double varargout_1[18][6]
// Return Type  : void
//
static void DerProd_core(const double varargin_1[6][6], const double varargin_2
  [18][6], double varargout_1[18][6])
{
  double d;

  //  number of a,da,dda,...
  //  number of a,b,c,...
  //  last factor and derivatives
  //  and their dimensions
  //  Is OverloadSym needed?
  for (int i = 0; i < 6; i++) {
    for (int i1 = 0; i1 < 18; i1++) {
      d = 0.0;
      for (int i2 = 0; i2 < 6; i2++) {
        d += varargin_1[i2][i] * varargin_2[i1][i2];
      }

      varargout_1[i1][i] = d;
    }
  }
}

//
// DerProd  Description.
//    [p,dp,ddp,...]=DerProd(a,da,dda,...,b,db,ddb,...,c,dc,ddc,...).
//
//    See also: .
//    Implemented by Gianluca Garofalo.
// Arguments    : const double varargin_1[6][6]
//                const double varargin_2[18][6]
//                double varargout_1[18][6]
// Return Type  : void
//
void DerProd(const double varargin_1[6][6], const double varargin_2[18][6],
             double varargout_1[18][6])
{
  DerProd_core(varargin_1, varargin_2, varargout_1);
}

//
// File trailer for DerProd.cpp
//
// [EOF]
//
