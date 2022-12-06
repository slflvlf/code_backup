/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * checkX0.c
 *
 * Code generation for function 'checkX0'
 *
 */

/* Include files */
#include "checkX0.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRTEInfo h_emlrtRTEI = {
    1,         /* lineNo */
    1,         /* colNo */
    "checkX0", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\shared\\optimlib\\+optim\\+coder\\+"
    "validate\\checkX0.p" /* pName */
};

/* Function Definitions */
void checkX0(const emlrtStack *sp, const real_T x0[265])
{
  int32_T i;
  boolean_T b[265];
  boolean_T b_b[265];
  boolean_T exitg1;
  boolean_T y;
  for (i = 0; i < 265; i++) {
    real_T d;
    d = x0[i];
    b[i] = muDoubleScalarIsInf(d);
    b_b[i] = muDoubleScalarIsNaN(d);
  }
  y = true;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 265)) {
    if (b[i] || b_b[i]) {
      y = false;
      exitg1 = true;
    } else {
      i++;
    }
  }
  if (!y) {
    emlrtErrorWithMessageIdR2018a(
        sp, &h_emlrtRTEI, "optimlib_codegen:common:InfNaNComplexDetected",
        "optimlib_codegen:common:InfNaNComplexDetected", 3, 4, 2, "x0");
  }
}

/* End of code generation (checkX0.c) */
