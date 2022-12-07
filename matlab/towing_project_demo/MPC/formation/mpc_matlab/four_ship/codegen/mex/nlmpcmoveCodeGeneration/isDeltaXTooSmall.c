/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * isDeltaXTooSmall.c
 *
 * Code generation for function 'isDeltaXTooSmall'
 *
 */

/* Include files */
#include "isDeltaXTooSmall.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo hd_emlrtRSI = {
    1,                  /* lineNo */
    "isDeltaXTooSmall", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "stopping\\isDeltaXTooSmall.p" /* pathName */
};

static emlrtBCInfo gb_emlrtBCI = {
    1,                  /* iFirst */
    265,                /* iLast */
    1,                  /* lineNo */
    1,                  /* colNo */
    "",                 /* aName */
    "isDeltaXTooSmall", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "stopping\\isDeltaXTooSmall.p", /* pName */
    0                               /* checkKind */
};

/* Function Definitions */
boolean_T isDeltaXTooSmall(const emlrtStack *sp, const real_T xCurrent[265],
                           const real_T delta_x_data[], int32_T nVar)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T idx;
  boolean_T exitg1;
  boolean_T tf;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  tf = true;
  st.site = &hd_emlrtRSI;
  if (nVar > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= nVar - 1)) {
    if ((idx + 1 < 1) || (idx + 1 > 265)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, 265, &gb_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    if (1.0E-6 * muDoubleScalarMax(1.0, muDoubleScalarAbs(xCurrent[idx])) <=
        muDoubleScalarAbs(delta_x_data[idx])) {
      tf = false;
      exitg1 = true;
    } else {
      idx++;
    }
  }
  return tf;
}

/* End of code generation (isDeltaXTooSmall.c) */
