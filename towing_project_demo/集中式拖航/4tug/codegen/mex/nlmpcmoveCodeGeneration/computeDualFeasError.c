/*
 * computeDualFeasError.c
 *
 * Code generation for function 'computeDualFeasError'
 *
 */

/* Include files */
#include "computeDualFeasError.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ed_emlrtRSI = {
    1,                      /* lineNo */
    "computeDualFeasError", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "stopping\\computeDualFeasError.p" /* pathName */
};

static emlrtBCInfo l_emlrtBCI = {
    -1,                     /* iFirst */
    -1,                     /* iLast */
    1,                      /* lineNo */
    1,                      /* colNo */
    "",                     /* aName */
    "computeDualFeasError", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "stopping\\computeDualFeasError.p", /* pName */
    0                                   /* checkKind */
};

/* Function Definitions */
void b_computeDualFeasError(const emlrtStack *sp, int32_T nVar,
                            const real_T gradLag_data[], boolean_T *gradOK,
                            real_T *val)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T idx;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  *gradOK = true;
  *val = 0.0;
  st.site = &ed_emlrtRSI;
  if (nVar > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= nVar - 1)) {
    *gradOK = ((!muDoubleScalarIsInf(gradLag_data[idx])) &&
               (!muDoubleScalarIsNaN(gradLag_data[idx])));
    if (!*gradOK) {
      exitg1 = true;
    } else {
      *val = muDoubleScalarMax(*val, muDoubleScalarAbs(gradLag_data[idx]));
      idx++;
    }
  }
}

void computeDualFeasError(const emlrtStack *sp, int32_T nVar,
                          const real_T gradLag_data[], int32_T gradLag_size,
                          boolean_T *gradOK, real_T *val)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T idx;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  *gradOK = true;
  *val = 0.0;
  st.site = &ed_emlrtRSI;
  if (nVar > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= nVar - 1)) {
    if ((idx + 1 < 1) || (idx + 1 > gradLag_size)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, gradLag_size, &l_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    *gradOK = ((!muDoubleScalarIsInf(gradLag_data[idx])) &&
               (!muDoubleScalarIsNaN(gradLag_data[idx])));
    if (!*gradOK) {
      exitg1 = true;
    } else {
      if ((idx + 1 < 1) || (idx + 1 > gradLag_size)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, gradLag_size, &l_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      *val = muDoubleScalarMax(*val, muDoubleScalarAbs(gradLag_data[idx]));
      idx++;
    }
  }
}

/* End of code generation (computeDualFeasError.c) */
