//
// File: all.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 01-Dec-2021 14:28:21
//

// Include Files
#include "all.h"
#include "allOrAny.h"
#include "executePlan.h"

// Function Definitions

//
// Arguments    : const boolean_T x[3][3]
//                boolean_T y[3]
// Return Type  : void
//
void all(const boolean_T x[3][3], boolean_T y[3])
{
  c_coder_internal_anonymous_func indToSub;
  boolean_T b_y[3];
  y[0] = false;
  y[1] = false;
  y[2] = false;
  indToSub.tunableEnvironment[0].f1[0] = 1;
  indToSub.tunableEnvironment[0].f1[1] = 1;
  b_y[0] = y[0];
  b_y[1] = y[1];
  b_y[2] = y[2];
  c___anon_fcn(x, indToSub, b_y, indToSub, y);
  indToSub.tunableEnvironment[0].f1[1] = 2;
  b_y[0] = y[0];
  b_y[1] = y[1];
  b_y[2] = y[2];
  c___anon_fcn(x, indToSub, b_y, indToSub, y);
  indToSub.tunableEnvironment[0].f1[1] = 3;
  b_y[0] = y[0];
  b_y[1] = y[1];
  b_y[2] = y[2];
  c___anon_fcn(x, indToSub, b_y, indToSub, y);
}

//
// Arguments    : const boolean_T x[3]
// Return Type  : boolean_T
//
boolean_T b_all(const boolean_T x[3])
{
  c_coder_internal_anonymous_func indToSub;
  indToSub.tunableEnvironment[0].f1[0] = 1;
  indToSub.tunableEnvironment[0].f1[1] = 1;
  return d___anon_fcn(x, indToSub, false);
}

//
// Arguments    : const double x[3]
// Return Type  : boolean_T
//
boolean_T c_all(const double x[3])
{
  c_coder_internal_anonymous_func indToSub;
  indToSub.tunableEnvironment[0].f1[0] = 1;
  indToSub.tunableEnvironment[0].f1[1] = 1;
  return f___anon_fcn(x, indToSub, false);
}

//
// Arguments    : const boolean_T x[3]
// Return Type  : boolean_T
//
boolean_T d_all(const boolean_T x[3])
{
  c_coder_internal_anonymous_func indToSub;
  indToSub.tunableEnvironment[0].f1[0] = 1;
  indToSub.tunableEnvironment[0].f1[1] = 1;
  return h___anon_fcn(x, indToSub, false);
}

//
// File trailer for all.cpp
//
// [EOF]
//
