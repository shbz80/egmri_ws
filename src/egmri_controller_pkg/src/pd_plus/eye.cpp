//
// File: eye.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 01-Dec-2021 14:28:21
//

// Include Files
#include "eye.h"
#include "executePlan.h"

// Function Definitions

//
// Arguments    : double b_I[3][3]
// Return Type  : void
//
void b_eye(double b_I[3][3])
{
  for (int i = 0; i < 3; i++) {
    b_I[i][0] = 0.0;
    b_I[i][1] = 0.0;
    b_I[i][2] = 0.0;
  }

  b_I[0][0] = 1.0;
  b_I[1][1] = 1.0;
  b_I[2][2] = 1.0;
}

//
// Arguments    : double b_I[4][4]
// Return Type  : void
//
void eye(double b_I[4][4])
{
  for (int i = 0; i < 4; i++) {
    b_I[i][0] = 0.0;
    b_I[i][1] = 0.0;
    b_I[i][2] = 0.0;
    b_I[i][3] = 0.0;
  }

  b_I[0][0] = 1.0;
  b_I[1][1] = 1.0;
  b_I[2][2] = 1.0;
  b_I[3][3] = 1.0;
}

//
// File trailer for eye.cpp
//
// [EOF]
//
