//
// File: any1.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 01-Dec-2021 14:28:21
//

// Include Files
#include "any1.h"
#include "allOrAny.h"
#include "executePlan.h"

// Function Definitions

//
// Arguments    : const boolean_T x[6]
// Return Type  : boolean_T
//
boolean_T any(const boolean_T x[6])
{
  c_coder_internal_anonymous_func indToSub;
  indToSub.tunableEnvironment[0].f1[0] = 1;
  indToSub.tunableEnvironment[0].f1[1] = 1;
  return __anon_fcn(x, indToSub, false);
}

//
// Arguments    : const boolean_T x[3]
// Return Type  : boolean_T
//
boolean_T b_any(const boolean_T x[3])
{
  c_coder_internal_anonymous_func indToSub;
  indToSub.tunableEnvironment[0].f1[0] = 1;
  indToSub.tunableEnvironment[0].f1[1] = 1;
  return g___anon_fcn(x, indToSub, false);
}

//
// File trailer for any1.cpp
//
// [EOF]
//
