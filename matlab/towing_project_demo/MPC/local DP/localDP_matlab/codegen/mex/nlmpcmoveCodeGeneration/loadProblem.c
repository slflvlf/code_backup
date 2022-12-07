/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * loadProblem.c
 *
 * Code generation for function 'loadProblem'
 *
 */

/* Include files */
#include "loadProblem.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ac_emlrtRSI = {
    1,             /* lineNo */
    "loadProblem", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\loadProblem.p" /* pathName */
};

static emlrtBCInfo ge_emlrtBCI = {
    -1,            /* iFirst */
    -1,            /* iLast */
    1,             /* lineNo */
    1,             /* colNo */
    "",            /* aName */
    "loadProblem", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\loadProblem.p", /* pName */
    0                            /* checkKind */
};

/* Function Definitions */
void loadProblem(const emlrtStack *sp, h_struct_T *obj, int32_T mIneq,
                 int32_T mLinIneq, const real_T Aineq_data[],
                 const int32_T Aineq_size[2], int32_T mLB, int32_T mConstrMax)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T i;
  int32_T i1;
  int32_T idx_col;
  int32_T idx_row;
  int32_T k;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  i = mIneq + mLB;
  obj->mConstr = i + 60;
  obj->mConstrOrig = i + 60;
  obj->mConstrMax = mConstrMax;
  obj->sizes[0] = 0;
  obj->sizes[1] = 60;
  obj->sizes[2] = mIneq;
  obj->sizes[3] = mLB;
  obj->sizes[4] = 0;
  obj->sizesPhaseOne[0] = 0;
  obj->sizesPhaseOne[1] = 60;
  obj->sizesPhaseOne[2] = mIneq;
  obj->sizesPhaseOne[3] = mLB + 1;
  obj->sizesPhaseOne[4] = 0;
  obj->sizesRegularized[0] = 0;
  obj->sizesRegularized[1] = 60;
  obj->sizesRegularized[2] = mIneq;
  obj->sizesRegularized[3] = i + 120;
  obj->sizesRegularized[4] = 0;
  obj->sizesRegPhaseOne[0] = 0;
  obj->sizesRegPhaseOne[1] = 60;
  obj->sizesRegPhaseOne[2] = mIneq;
  obj->sizesRegPhaseOne[3] = i + 121;
  obj->sizesRegPhaseOne[4] = 0;
  obj->isActiveIdxRegPhaseOne[0] = 1;
  obj->isActiveIdxRegPhaseOne[1] = 0;
  obj->isActiveIdxRegPhaseOne[2] = 60;
  obj->isActiveIdxRegPhaseOne[3] = mIneq;
  obj->isActiveIdxRegPhaseOne[4] = mLB;
  obj->isActiveIdxRegPhaseOne[5] = 0;
  for (k = 0; k < 5; k++) {
    obj->sizesNormal[k] = obj->sizes[k];
    obj->isActiveIdxRegPhaseOne[k + 1] += obj->isActiveIdxRegPhaseOne[k];
  }
  for (i1 = 0; i1 < 6; i1++) {
    obj->isActiveIdx[i1] = obj->isActiveIdxRegPhaseOne[i1];
    obj->isActiveIdxNormal[i1] = obj->isActiveIdxRegPhaseOne[i1];
  }
  obj->isActiveIdxRegPhaseOne[0] = 1;
  obj->isActiveIdxRegPhaseOne[1] = 0;
  obj->isActiveIdxRegPhaseOne[2] = 60;
  obj->isActiveIdxRegPhaseOne[3] = mIneq;
  obj->isActiveIdxRegPhaseOne[4] = mLB + 1;
  obj->isActiveIdxRegPhaseOne[5] = 0;
  for (k = 0; k < 5; k++) {
    obj->isActiveIdxRegPhaseOne[k + 1] += obj->isActiveIdxRegPhaseOne[k];
  }
  for (i1 = 0; i1 < 6; i1++) {
    obj->isActiveIdxPhaseOne[i1] = obj->isActiveIdxRegPhaseOne[i1];
  }
  obj->isActiveIdxRegPhaseOne[0] = 1;
  obj->isActiveIdxRegPhaseOne[1] = 0;
  obj->isActiveIdxRegPhaseOne[2] = 60;
  obj->isActiveIdxRegPhaseOne[3] = mIneq;
  obj->isActiveIdxRegPhaseOne[4] = i + 120;
  obj->isActiveIdxRegPhaseOne[5] = 0;
  for (k = 0; k < 5; k++) {
    obj->isActiveIdxRegPhaseOne[k + 1] += obj->isActiveIdxRegPhaseOne[k];
  }
  for (i1 = 0; i1 < 6; i1++) {
    obj->isActiveIdxRegularized[i1] = obj->isActiveIdxRegPhaseOne[i1];
  }
  obj->isActiveIdxRegPhaseOne[0] = 1;
  obj->isActiveIdxRegPhaseOne[1] = 0;
  obj->isActiveIdxRegPhaseOne[2] = 60;
  obj->isActiveIdxRegPhaseOne[3] = mIneq;
  obj->isActiveIdxRegPhaseOne[4] = i + 121;
  obj->isActiveIdxRegPhaseOne[5] = 0;
  for (k = 0; k < 5; k++) {
    obj->isActiveIdxRegPhaseOne[k + 1] += obj->isActiveIdxRegPhaseOne[k];
  }
  if (mIneq > 0) {
    st.site = &ac_emlrtRSI;
    if (mLinIneq > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx_col = 0; idx_col < mLinIneq; idx_col++) {
      st.site = &ac_emlrtRSI;
      i = Aineq_size[0];
      i1 = Aineq_size[1];
      i *= i1;
      for (idx_row = 0; idx_row < 79; idx_row++) {
        int32_T i2;
        i1 = (idx_col + mLinIneq * idx_row) + 1;
        if ((i1 < 1) || (i1 > i)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, i, &ge_emlrtBCI, (emlrtCTX)sp);
        }
        k = obj->Aineq.size[0];
        i2 = (idx_row + obj->ldA * idx_col) + 1;
        if ((i2 < 1) || (i2 > k)) {
          emlrtDynamicBoundsCheckR2012b(i2, 1, k, &ge_emlrtBCI, (emlrtCTX)sp);
        }
        obj->Aineq.data[i2 - 1] = Aineq_data[i1 - 1];
      }
    }
  }
}

/* End of code generation (loadProblem.c) */
