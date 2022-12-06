/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * reshapeSizeChecks.c
 *
 * Code generation for function 'reshapeSizeChecks'
 *
 */

/* Include files */
#include "reshapeSizeChecks.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo cb_emlrtRSI = {
    109,               /* lineNo */
    "computeDimsData", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pathName */
};

static emlrtRTEInfo d_emlrtRTEI = {
    58,                   /* lineNo */
    23,                   /* colNo */
    "assertValidSizeArg", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\assertValidSizeArg.m" /* pName */
};

/* Function Definitions */
void computeDimsData(const emlrtStack *sp, real_T varargin_1)
{
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &cb_emlrtRSI;
  if ((varargin_1 != varargin_1) || muDoubleScalarIsInf(varargin_1)) {
    emlrtErrorWithMessageIdR2018a(
        &st, &d_emlrtRTEI, "Coder:MATLAB:NonIntegerInput",
        "Coder:MATLAB:NonIntegerInput", 4, 12, MIN_int32_T, 12, MAX_int32_T);
  }
}

/* End of code generation (reshapeSizeChecks.c) */
