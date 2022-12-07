/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeComplError.c
 *
 * Code generation for function 'computeComplError'
 *
 */

/* Include files */
#include "computeComplError.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo gd_emlrtRSI = {
    1,                   /* lineNo */
    "computeComplError", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "stopping\\computeComplError.p" /* pathName */
};

static emlrtBCInfo eb_emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    1,                   /* lineNo */
    1,                   /* colNo */
    "",                  /* aName */
    "computeComplError", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "stopping\\computeComplError.p", /* pName */
    0                                /* checkKind */
};

static emlrtBCInfo fb_emlrtBCI = {
    1,                   /* iFirst */
    265,                 /* iLast */
    1,                   /* lineNo */
    1,                   /* colNo */
    "",                  /* aName */
    "computeComplError", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "stopping\\computeComplError.p", /* pName */
    0                                /* checkKind */
};

/* Function Definitions */
real_T computeComplError(
    const emlrtStack *sp, int32_T fscales_lineq_constraint_size,
    int32_T fscales_cineq_constraint_size, const real_T xCurrent[265],
    int32_T mIneq, const real_T cIneq_data[], int32_T cIneq_size,
    const int32_T finiteLB_data[], int32_T finiteLB_size, int32_T mLB,
    const real_T lb[265], const int32_T finiteUB_data[], int32_T finiteUB_size,
    int32_T mUB, const real_T lambda_data[], int32_T lambda_size, int32_T iL0)
{
  emlrtStack b_st;
  emlrtStack st;
  real_T nlpComplError;
  int32_T idx;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  nlpComplError = 0.0;
  if ((mIneq + mLB) + mUB > 0) {
    real_T lbDelta;
    real_T lbLambda;
    int32_T i;
    int32_T iLineq0;
    int32_T ubOffset;
    st.site = &gd_emlrtRSI;
    for (idx = 0; idx < fscales_lineq_constraint_size; idx++) {
      if (idx + 1 > fscales_lineq_constraint_size) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, fscales_lineq_constraint_size,
                                      &eb_emlrtBCI, (emlrtCTX)sp);
      }
      if (idx + 1 > cIneq_size) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, cIneq_size, &eb_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      i = iL0 + idx;
      if ((i < 1) || (i > lambda_size)) {
        emlrtDynamicBoundsCheckR2012b(i, 1, lambda_size, &eb_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      if (idx + 1 > cIneq_size) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, cIneq_size, &eb_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      if (i > lambda_size) {
        emlrtDynamicBoundsCheckR2012b(i, 1, lambda_size, &eb_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      lbDelta = lambda_data[i - 1];
      nlpComplError = muDoubleScalarMax(
          nlpComplError,
          muDoubleScalarMin(
              muDoubleScalarAbs(cIneq_data[idx] * lbDelta),
              muDoubleScalarMin(muDoubleScalarAbs(cIneq_data[idx]), lbDelta)));
    }
    iLineq0 = (iL0 + fscales_lineq_constraint_size) - 1;
    st.site = &gd_emlrtRSI;
    for (idx = 0; idx < fscales_cineq_constraint_size; idx++) {
      if (idx + 1 > fscales_cineq_constraint_size) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, fscales_cineq_constraint_size,
                                      &eb_emlrtBCI, (emlrtCTX)sp);
      }
      i = (fscales_lineq_constraint_size + idx) + 1;
      if (i > cIneq_size) {
        emlrtDynamicBoundsCheckR2012b(i, 1, cIneq_size, &eb_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      ubOffset = (iLineq0 + idx) + 1;
      if ((ubOffset < 1) || (ubOffset > lambda_size)) {
        emlrtDynamicBoundsCheckR2012b(ubOffset, 1, lambda_size, &eb_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      if (i > cIneq_size) {
        emlrtDynamicBoundsCheckR2012b(i, 1, cIneq_size, &eb_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      if (ubOffset > lambda_size) {
        emlrtDynamicBoundsCheckR2012b(ubOffset, 1, lambda_size, &eb_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      lbDelta = lambda_data[ubOffset - 1];
      nlpComplError = muDoubleScalarMax(
          nlpComplError,
          muDoubleScalarMin(
              muDoubleScalarAbs(cIneq_data[i - 1] * lbDelta),
              muDoubleScalarMin(muDoubleScalarAbs(cIneq_data[i - 1]),
                                lbDelta)));
    }
    iLineq0 = iL0 + mIneq;
    ubOffset = iLineq0 + mLB;
    st.site = &gd_emlrtRSI;
    if (mLB > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx = 0; idx < mLB; idx++) {
      if ((idx + 1 < 1) || (idx + 1 > finiteLB_size)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, finiteLB_size, &eb_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      if ((finiteLB_data[idx] < 1) || (finiteLB_data[idx] > 265)) {
        emlrtDynamicBoundsCheckR2012b(finiteLB_data[idx], 1, 265, &fb_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      lbDelta = xCurrent[finiteLB_data[idx] - 1] - lb[finiteLB_data[idx] - 1];
      i = iLineq0 + idx;
      if ((i < 1) || (i > lambda_size)) {
        emlrtDynamicBoundsCheckR2012b(i, 1, lambda_size, &eb_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      lbLambda = lambda_data[i - 1];
      nlpComplError = muDoubleScalarMax(
          nlpComplError,
          muDoubleScalarMin(
              muDoubleScalarAbs(lbDelta * lbLambda),
              muDoubleScalarMin(muDoubleScalarAbs(lbDelta), lbLambda)));
    }
    st.site = &gd_emlrtRSI;
    if (mUB > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx = 0; idx < mUB; idx++) {
      if ((idx + 1 < 1) || (idx + 1 > finiteUB_size)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, finiteUB_size, &eb_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      i = ubOffset + idx;
      if ((i < 1) || (i > lambda_size)) {
        emlrtDynamicBoundsCheckR2012b(i, 1, lambda_size, &eb_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      lbLambda = lambda_data[i - 1];
      lbDelta = rtInf - xCurrent[finiteUB_data[idx] - 1];
      nlpComplError = muDoubleScalarMax(
          nlpComplError,
          muDoubleScalarMin(muDoubleScalarAbs(lbDelta * lbLambda),
                            muDoubleScalarMin(lbDelta, lbLambda)));
    }
  }
  return nlpComplError;
}

/* End of code generation (computeComplError.c) */
