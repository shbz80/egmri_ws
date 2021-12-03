//
// File: ListGet.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 01-Dec-2021 14:28:21
//

// Include Files
#include "ListGet.h"
#include "executePlan.h"

// Function Definitions

//
// ListGet  Description.
//    [A(i1,i2,...),B(i1,i2,...),...]=ListGet(i1,i2,...,A,B,...).
//
//    See also: .
//    Implemented by Gianluca Garofalo.
// Arguments    : const double varargin_1[6]
//                const double varargin_2[18]
//                const double varargin_3[18][114]
//                double varargout_1[18][6]
// Return Type  : void
//
void ListGet(const double varargin_1[6], const double varargin_2[18], const
             double varargin_3[18][114], double varargout_1[18][6])
{
  for (int i = 0; i < 18; i++) {
    for (int i1 = 0; i1 < 6; i1++) {
      varargout_1[i][i1] = varargin_3[static_cast<int>(varargin_2[i]) - 1][
        static_cast<int>(varargin_1[i1]) - 1];
    }
  }
}

//
// File trailer for ListGet.cpp
//
// [EOF]
//
