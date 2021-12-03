//
// File: ListSet.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 01-Dec-2021 14:28:21
//

// Include Files
#include "ListSet.h"
#include "executePlan.h"

// Function Definitions

//
// ListSet  Description.
//    [A,B,...]=ListSet(i1,i2,...,A,B,...,a,b,...), where A(i1,i2,...)=a and
//    so on.
//
//    See also: IndMatrixBlock.
//    Implemented by Gianluca Garofalo.
// Arguments    : const double varargin_1[6]
//                const double varargin_2[18]
//                double varargin_3[18][114]
//                const double varargin_4[18][6]
// Return Type  : void
//
void ListSet(const double varargin_1[6], const double varargin_2[18], double
             varargin_3[18][114], const double varargin_4[18][6])
{
  for (int i = 0; i < 18; i++) {
    for (int i1 = 0; i1 < 6; i1++) {
      varargin_3[static_cast<int>(varargin_2[i]) - 1][static_cast<int>
        (varargin_1[i1]) - 1] = varargin_4[i][i1];
    }
  }
}

//
// File trailer for ListSet.cpp
//
// [EOF]
//
