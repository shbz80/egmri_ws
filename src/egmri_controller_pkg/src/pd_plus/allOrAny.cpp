//
// File: allOrAny.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 01-Dec-2021 14:28:21
//

// Include Files
#include "allOrAny.h"
#include "applyVectorFunction.h"
#include "executePlan.h"

// Function Declarations
static void allFun(const boolean_T X[3][3], const
                   c_coder_internal_anonymous_func ind2SubX, boolean_T Y[3],
                   const c_coder_internal_anonymous_func ind2SubY);
static void anyFun(const boolean_T X[6], const c_coder_internal_anonymous_func
                   ind2SubX, boolean_T *Y);
static void b_allFun(const boolean_T X[3], const c_coder_internal_anonymous_func
                     ind2SubX, boolean_T *Y);
static void b_anyFun(const boolean_T X[3], const c_coder_internal_anonymous_func
                     ind2SubX, boolean_T *Y);
static void c_allFun(const double X[3], const c_coder_internal_anonymous_func
                     ind2SubX, boolean_T *Y);
static void d_allFun(const boolean_T X[3], const c_coder_internal_anonymous_func
                     ind2SubX, boolean_T *Y);

// Function Definitions

//
// Arguments    : const boolean_T X[3][3]
//                const c_coder_internal_anonymous_func ind2SubX
//                boolean_T Y[3]
//                const c_coder_internal_anonymous_func ind2SubY
// Return Type  : void
//
static void allFun(const boolean_T X[3][3], const
                   c_coder_internal_anonymous_func ind2SubX, boolean_T Y[3],
                   const c_coder_internal_anonymous_func ind2SubY)
{
  int subsY[2];
  int i;
  int k;
  boolean_T exitg1;
  b___anon_fcn(ind2SubY.tunableEnvironment[0].f1, 1, subsY);
  i = (subsY[0] + subsY[1]) - 2;
  Y[i] = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    b___anon_fcn(ind2SubX.tunableEnvironment[0].f1, k + 1, subsY);
    if (!X[subsY[1] - 1][subsY[0] - 1]) {
      Y[i] = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
}

//
// Arguments    : const boolean_T X[6]
//                const c_coder_internal_anonymous_func ind2SubX
//                boolean_T *Y
// Return Type  : void
//
static void anyFun(const boolean_T X[6], const c_coder_internal_anonymous_func
                   ind2SubX, boolean_T *Y)
{
  int k;
  boolean_T exitg1;
  int b_iv[2];
  *Y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 6)) {
    b___anon_fcn(ind2SubX.tunableEnvironment[0].f1, k + 1, b_iv);
    if (X[b_iv[0] - 1]) {
      *Y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }
}

//
// Arguments    : const boolean_T X[3]
//                const c_coder_internal_anonymous_func ind2SubX
//                boolean_T *Y
// Return Type  : void
//
static void b_allFun(const boolean_T X[3], const c_coder_internal_anonymous_func
                     ind2SubX, boolean_T *Y)
{
  int k;
  boolean_T exitg1;
  int subsX[2];
  *Y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    e___anon_fcn(ind2SubX.tunableEnvironment[0].f1, k + 1, subsX);
    if (!X[(subsX[0] + subsX[1]) - 2]) {
      *Y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
}

//
// Arguments    : const boolean_T X[3]
//                const c_coder_internal_anonymous_func ind2SubX
//                boolean_T *Y
// Return Type  : void
//
static void b_anyFun(const boolean_T X[3], const c_coder_internal_anonymous_func
                     ind2SubX, boolean_T *Y)
{
  int k;
  boolean_T exitg1;
  int b_iv[2];
  *Y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    b___anon_fcn(ind2SubX.tunableEnvironment[0].f1, k + 1, b_iv);
    if (X[b_iv[0] - 1]) {
      *Y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }
}

//
// Arguments    : const double X[3]
//                const c_coder_internal_anonymous_func ind2SubX
//                boolean_T *Y
// Return Type  : void
//
static void c_allFun(const double X[3], const c_coder_internal_anonymous_func
                     ind2SubX, boolean_T *Y)
{
  int k;
  boolean_T exitg1;
  int b_iv[2];
  *Y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    b___anon_fcn(ind2SubX.tunableEnvironment[0].f1, k + 1, b_iv);
    if (X[b_iv[0] - 1] == 0.0) {
      *Y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
}

//
// Arguments    : const boolean_T X[3]
//                const c_coder_internal_anonymous_func ind2SubX
//                boolean_T *Y
// Return Type  : void
//
static void d_allFun(const boolean_T X[3], const c_coder_internal_anonymous_func
                     ind2SubX, boolean_T *Y)
{
  int k;
  boolean_T exitg1;
  int b_iv[2];
  *Y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    b___anon_fcn(ind2SubX.tunableEnvironment[0].f1, k + 1, b_iv);
    if (!X[b_iv[0] - 1]) {
      *Y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
}

//
// Arguments    : const boolean_T X[6]
//                const c_coder_internal_anonymous_func indToSubX
//                boolean_T Y
// Return Type  : boolean_T
//
boolean_T __anon_fcn(const boolean_T X[6], const c_coder_internal_anonymous_func
                     indToSubX, boolean_T Y)
{
  boolean_T varargout_1;
  varargout_1 = Y;
  anyFun(X, indToSubX, &varargout_1);
  return varargout_1;
}

//
// Arguments    : const boolean_T X[3][3]
//                const c_coder_internal_anonymous_func indToSubX
//                const boolean_T Y[3]
//                const c_coder_internal_anonymous_func ind2SubY
//                boolean_T varargout_1[3]
// Return Type  : void
//
void c___anon_fcn(const boolean_T X[3][3], const c_coder_internal_anonymous_func
                  indToSubX, const boolean_T Y[3], const
                  c_coder_internal_anonymous_func ind2SubY, boolean_T
                  varargout_1[3])
{
  varargout_1[0] = Y[0];
  varargout_1[1] = Y[1];
  varargout_1[2] = Y[2];
  allFun(X, indToSubX, varargout_1, ind2SubY);
}

//
// Arguments    : const boolean_T X[3]
//                const c_coder_internal_anonymous_func indToSubX
//                boolean_T Y
// Return Type  : boolean_T
//
boolean_T d___anon_fcn(const boolean_T X[3], const
  c_coder_internal_anonymous_func indToSubX, boolean_T Y)
{
  boolean_T varargout_1;
  varargout_1 = Y;
  b_allFun(X, indToSubX, &varargout_1);
  return varargout_1;
}

//
// Arguments    : const double X[3]
//                const c_coder_internal_anonymous_func indToSubX
//                boolean_T Y
// Return Type  : boolean_T
//
boolean_T f___anon_fcn(const double X[3], const c_coder_internal_anonymous_func
  indToSubX, boolean_T Y)
{
  boolean_T varargout_1;
  varargout_1 = Y;
  c_allFun(X, indToSubX, &varargout_1);
  return varargout_1;
}

//
// Arguments    : const boolean_T X[3]
//                const c_coder_internal_anonymous_func indToSubX
//                boolean_T Y
// Return Type  : boolean_T
//
boolean_T g___anon_fcn(const boolean_T X[3], const
  c_coder_internal_anonymous_func indToSubX, boolean_T Y)
{
  boolean_T varargout_1;
  varargout_1 = Y;
  b_anyFun(X, indToSubX, &varargout_1);
  return varargout_1;
}

//
// Arguments    : const boolean_T X[3]
//                const c_coder_internal_anonymous_func indToSubX
//                boolean_T Y
// Return Type  : boolean_T
//
boolean_T h___anon_fcn(const boolean_T X[3], const
  c_coder_internal_anonymous_func indToSubX, boolean_T Y)
{
  boolean_T varargout_1;
  varargout_1 = Y;
  d_allFun(X, indToSubX, &varargout_1);
  return varargout_1;
}

//
// File trailer for allOrAny.cpp
//
// [EOF]
//
