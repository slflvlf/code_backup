/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * checkVectorNonFinite.c
 *
 * Code generation for function 'checkVectorNonFinite'
 *
 */

/* Include files */
#include "checkVectorNonFinite.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtBCInfo bb_emlrtBCI = {
    -1,                     /* iFirst */
    -1,                     /* iLast */
    1,                      /* lineNo */
    1,                      /* colNo */
    "",                     /* aName */
    "checkVectorNonFinite", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+utils\\+"
    "ObjNonlinEvaluator\\+internal\\checkVectorNonFinite.p", /* pName */
    0                                                        /* checkKind */
};

static emlrtBCInfo hb_emlrtBCI = {
    1,                      /* iFirst */
    192,                    /* iLast */
    1,                      /* lineNo */
    1,                      /* colNo */
    "",                     /* aName */
    "checkVectorNonFinite", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+utils\\+"
    "ObjNonlinEvaluator\\+internal\\checkVectorNonFinite.p", /* pName */
    0                                                        /* checkKind */
};

/* Function Definitions */
int32_T b_checkVectorNonFinite(const emlrtStack *sp, const real_T vec[192])
{
  int32_T idx_current;
  int32_T status;
  boolean_T allFinite;
  status = 1;
  allFinite = true;
  idx_current = 0;
  while (allFinite && (idx_current + 1 <= 192)) {
    allFinite = ((!muDoubleScalarIsInf(vec[idx_current])) &&
                 (!muDoubleScalarIsNaN(vec[idx_current])));
    idx_current++;
  }
  if (!allFinite) {
    if ((idx_current < 1) || (idx_current > 192)) {
      emlrtDynamicBoundsCheckR2012b(idx_current, 1, 192, &hb_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    if (muDoubleScalarIsNaN(vec[idx_current - 1])) {
      status = -3;
    } else if (vec[idx_current - 1] < 0.0) {
      status = -1;
    } else {
      status = -2;
    }
  }
  return status;
}

int32_T checkVectorNonFinite(const emlrtStack *sp, int32_T N,
                             const real_T vec_data[], int32_T vec_size,
                             int32_T iv0)
{
  int32_T idx_current;
  int32_T idx_end;
  int32_T status;
  boolean_T allFinite;
  status = 1;
  allFinite = true;
  idx_current = iv0 - 1;
  idx_end = (iv0 + N) - 1;
  while (allFinite && (idx_current + 1 <= idx_end)) {
    if ((idx_current + 1 < 1) || (idx_current + 1 > vec_size)) {
      emlrtDynamicBoundsCheckR2012b(idx_current + 1, 1, vec_size, &bb_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    allFinite = ((!muDoubleScalarIsInf(vec_data[idx_current])) &&
                 (!muDoubleScalarIsNaN(vec_data[idx_current])));
    idx_current++;
  }
  if (!allFinite) {
    real_T d;
    if ((idx_current < 1) || (idx_current > vec_size)) {
      emlrtDynamicBoundsCheckR2012b(idx_current, 1, vec_size, &bb_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    d = vec_data[idx_current - 1];
    if (muDoubleScalarIsNaN(d)) {
      status = -3;
    } else {
      if (idx_current > vec_size) {
        emlrtDynamicBoundsCheckR2012b(idx_current, 1, vec_size, &bb_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      if (d < 0.0) {
        status = -1;
      } else {
        status = -2;
      }
    }
  }
  return status;
}

/* End of code generation (checkVectorNonFinite.c) */
