/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeGradLag.c
 *
 * Code generation for function 'computeGradLag'
 *
 */

/* Include files */
#include "computeGradLag.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo bd_emlrtRSI = {
    1,                /* lineNo */
    "computeGradLag", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "stopping\\computeGradLag.p" /* pathName */
};

static emlrtBCInfo re_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    1,                /* lineNo */
    1,                /* colNo */
    "",               /* aName */
    "computeGradLag", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "stopping\\computeGradLag.p", /* pName */
    0                             /* checkKind */
};

/* Function Definitions */
void b_computeGradLag(const emlrtStack *sp, real_T workspace_data[],
                      const int32_T workspace_size[2], int32_T ldA,
                      int32_T nVar, const real_T grad_data[], int32_T grad_size,
                      int32_T mIneq, const real_T AineqTrans_data[],
                      const real_T AeqTrans_data[],
                      const int32_T finiteFixed_data[],
                      int32_T finiteFixed_size, int32_T mFixed,
                      const int32_T finiteLB_data[], int32_T finiteLB_size,
                      int32_T mLB, const int32_T finiteUB_data[],
                      int32_T finiteUB_size, int32_T mUB,
                      const real_T lambda_data[], int32_T lambda_size)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T i;
  int32_T iL0;
  int32_T idx;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &bd_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  if (nVar > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (iL0 = 0; iL0 < nVar; iL0++) {
    if ((iL0 + 1 < 1) || (iL0 + 1 > grad_size)) {
      emlrtDynamicBoundsCheckR2012b(iL0 + 1, 1, grad_size, &re_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    workspace_data[iL0] = grad_data[iL0];
  }
  st.site = &bd_emlrtRSI;
  if (mFixed > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = 0; idx < mFixed; idx++) {
    if ((idx + 1 < 1) || (idx + 1 > finiteFixed_size)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, finiteFixed_size, &re_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    if ((idx + 1 < 1) || (idx + 1 > lambda_size)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, lambda_size, &re_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    workspace_data[finiteFixed_data[idx] - 1] += lambda_data[idx];
  }
  st.site = &bd_emlrtRSI;
  xgemv(nVar, 60, AeqTrans_data, ldA, lambda_data, mFixed + 1, workspace_data);
  st.site = &bd_emlrtRSI;
  xgemv(nVar, mIneq, AineqTrans_data, ldA, lambda_data, mFixed + 61,
        workspace_data);
  iL0 = (mFixed + mIneq) + 61;
  st.site = &bd_emlrtRSI;
  if (mLB > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = 0; idx < mLB; idx++) {
    int32_T i1;
    if ((idx + 1 < 1) || (idx + 1 > finiteLB_size)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, finiteLB_size, &re_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    i = workspace_size[0] * workspace_size[1];
    if ((finiteLB_data[idx] < 1) || (finiteLB_data[idx] > i)) {
      emlrtDynamicBoundsCheckR2012b(finiteLB_data[idx], 1, i, &re_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    i1 = iL0 + idx;
    if ((i1 < 1) || (i1 > lambda_size)) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, lambda_size, &re_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    if ((finiteLB_data[idx] < 1) || (finiteLB_data[idx] > i)) {
      emlrtDynamicBoundsCheckR2012b(finiteLB_data[idx], 1, i, &re_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    workspace_data[finiteLB_data[idx] - 1] -= lambda_data[i1 - 1];
  }
  iL0 += mLB;
  st.site = &bd_emlrtRSI;
  if (mUB > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = 0; idx < mUB; idx++) {
    if ((idx + 1 < 1) || (idx + 1 > finiteUB_size)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, finiteUB_size, &re_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    i = iL0 + idx;
    if ((i < 1) || (i > lambda_size)) {
      emlrtDynamicBoundsCheckR2012b(i, 1, lambda_size, &re_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    workspace_data[finiteUB_data[idx] - 1] += lambda_data[i - 1];
  }
}

void computeGradLag(const emlrtStack *sp, real_T workspace_data[],
                    const int32_T *workspace_size, int32_T ldA, int32_T nVar,
                    const real_T grad_data[], int32_T grad_size, int32_T mIneq,
                    const real_T AineqTrans_data[],
                    const real_T AeqTrans_data[],
                    const int32_T finiteFixed_data[], int32_T finiteFixed_size,
                    int32_T mFixed, const int32_T finiteLB_data[],
                    int32_T finiteLB_size, int32_T mLB,
                    const int32_T finiteUB_data[], int32_T finiteUB_size,
                    int32_T mUB, const real_T lambda_data[],
                    int32_T lambda_size)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T b_i;
  int32_T i;
  int32_T iL0;
  int32_T idx;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &bd_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  if (nVar > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  i = *workspace_size;
  for (b_i = 0; b_i < nVar; b_i++) {
    if ((b_i + 1 < 1) || (b_i + 1 > grad_size)) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, grad_size, &re_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    if (b_i + 1 > i) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, i, &re_emlrtBCI, (emlrtCTX)sp);
    }
    workspace_data[b_i] = grad_data[b_i];
  }
  st.site = &bd_emlrtRSI;
  if (mFixed > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = 0; idx < mFixed; idx++) {
    if ((idx + 1 < 1) || (idx + 1 > finiteFixed_size)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, finiteFixed_size, &re_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    if ((idx + 1 < 1) || (idx + 1 > lambda_size)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, lambda_size, &re_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    workspace_data[finiteFixed_data[idx] - 1] += lambda_data[idx];
  }
  st.site = &bd_emlrtRSI;
  xgemv(nVar, 60, AeqTrans_data, ldA, lambda_data, mFixed + 1, workspace_data);
  st.site = &bd_emlrtRSI;
  xgemv(nVar, mIneq, AineqTrans_data, ldA, lambda_data, mFixed + 61,
        workspace_data);
  iL0 = (mFixed + mIneq) + 61;
  st.site = &bd_emlrtRSI;
  if (mLB > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = 0; idx < mLB; idx++) {
    if ((idx + 1 < 1) || (idx + 1 > finiteLB_size)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, finiteLB_size, &re_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    i = *workspace_size;
    if ((finiteLB_data[idx] < 1) || (finiteLB_data[idx] > i)) {
      emlrtDynamicBoundsCheckR2012b(finiteLB_data[idx], 1, i, &re_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    i = iL0 + idx;
    if ((i < 1) || (i > lambda_size)) {
      emlrtDynamicBoundsCheckR2012b(i, 1, lambda_size, &re_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    b_i = *workspace_size;
    if ((finiteLB_data[idx] < 1) || (finiteLB_data[idx] > b_i)) {
      emlrtDynamicBoundsCheckR2012b(finiteLB_data[idx], 1, b_i, &re_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    workspace_data[finiteLB_data[idx] - 1] -= lambda_data[i - 1];
  }
  iL0 += mLB;
  st.site = &bd_emlrtRSI;
  if (mUB > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = 0; idx < mUB; idx++) {
    if ((idx + 1 < 1) || (idx + 1 > finiteUB_size)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, finiteUB_size, &re_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    i = iL0 + idx;
    if ((i < 1) || (i > lambda_size)) {
      emlrtDynamicBoundsCheckR2012b(i, 1, lambda_size, &re_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    workspace_data[finiteUB_data[idx] - 1] += lambda_data[i - 1];
  }
}

/* End of code generation (computeGradLag.c) */
