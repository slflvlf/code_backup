/*
 * computePrimalFeasError.c
 *
 * Code generation for function 'computePrimalFeasError'
 *
 */

/* Include files */
#include "computePrimalFeasError.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo dd_emlrtRSI = {
    1,                        /* lineNo */
    "computePrimalFeasError", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "stopping\\computePrimalFeasError.p" /* pathName */
};

static emlrtBCInfo r_emlrtBCI = {
    -1,                       /* iFirst */
    -1,                       /* iLast */
    1,                        /* lineNo */
    1,                        /* colNo */
    "",                       /* aName */
    "computePrimalFeasError", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "stopping\\computePrimalFeasError.p", /* pName */
    0                                     /* checkKind */
};

static emlrtBCInfo s_emlrtBCI = {
    1,                        /* iFirst */
    85,                       /* iLast */
    1,                        /* lineNo */
    1,                        /* colNo */
    "",                       /* aName */
    "computePrimalFeasError", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "stopping\\computePrimalFeasError.p", /* pName */
    0                                     /* checkKind */
};

/* Function Definitions */
real_T computePrimalFeasError(
    const emlrtStack *sp, const real_T x[85], int32_T mLinIneq,
    int32_T mNonlinIneq, const real_T cIneq_data[], int32_T cIneq_size,
    const real_T cEq[60], const int32_T finiteLB_data[], int32_T finiteLB_size,
    int32_T mLB, const real_T lb[85], const int32_T finiteUB_data[],
    int32_T finiteUB_size, int32_T mUB)
{
  emlrtStack b_st;
  emlrtStack st;
  real_T feasError;
  int32_T idx;
  int32_T mIneq;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  feasError = 0.0;
  mIneq = mNonlinIneq + mLinIneq;
  st.site = &dd_emlrtRSI;
  for (idx = 0; idx < 60; idx++) {
    feasError = muDoubleScalarMax(feasError, muDoubleScalarAbs(cEq[idx]));
  }
  st.site = &dd_emlrtRSI;
  if (mIneq > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = 0; idx < mIneq; idx++) {
    if ((idx + 1 < 1) || (idx + 1 > cIneq_size)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, cIneq_size, &r_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    feasError = muDoubleScalarMax(feasError, cIneq_data[idx]);
  }
  st.site = &dd_emlrtRSI;
  if (mLB > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = 0; idx < mLB; idx++) {
    if ((idx + 1 < 1) || (idx + 1 > finiteLB_size)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, finiteLB_size, &r_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    mIneq = finiteLB_data[idx] - 1;
    if ((finiteLB_data[idx] < 1) || (finiteLB_data[idx] > 85)) {
      emlrtDynamicBoundsCheckR2012b(finiteLB_data[idx], 1, 85, &s_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    feasError = muDoubleScalarMax(feasError, lb[mIneq] - x[mIneq]);
  }
  st.site = &dd_emlrtRSI;
  if (mUB > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = 0; idx < mUB; idx++) {
    if ((idx + 1 < 1) || (idx + 1 > finiteUB_size)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, finiteUB_size, &r_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    feasError = muDoubleScalarMax(feasError, x[finiteUB_data[idx] - 1] - rtInf);
  }
  return feasError;
}

/* End of code generation (computePrimalFeasError.c) */
