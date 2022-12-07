/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * setProblemType.c
 *
 * Code generation for function 'setProblemType'
 *
 */

/* Include files */
#include "setProblemType.h"
#include "eml_int_forloop_overflow_check.h"
#include "modifyOverheadPhaseOne_.h"
#include "modifyOverheadRegularized_.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo uc_emlrtRSI = {
    1,                /* lineNo */
    "setProblemType", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\setProblemType.p" /* pathName */
};

static emlrtBCInfo lb_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    1,                /* lineNo */
    1,                /* colNo */
    "",               /* aName */
    "setProblemType", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\setProblemType.p", /* pName */
    0                               /* checkKind */
};

/* Function Definitions */
void setProblemType(const emlrtStack *sp, h_struct_T *obj, int32_T PROBLEM_TYPE)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T i;
  int32_T idx;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  switch (PROBLEM_TYPE) {
  case 3: {
    obj->nVar = 265;
    obj->mConstr = obj->mConstrOrig;
    if (obj->nWConstr[4] > 0) {
      int32_T b;
      b = obj->sizesNormal[4];
      st.site = &uc_emlrtRSI;
      if (obj->sizesNormal[4] > 2147483646) {
        b_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }
      for (idx = 0; idx < b; idx++) {
        int32_T i1;
        int32_T i2;
        i = obj->isActiveConstr.size[0];
        i1 = obj->isActiveIdx[4] + idx;
        if ((i1 < 1) || (i1 > i)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, i, &lb_emlrtBCI, (emlrtCTX)sp);
        }
        i = obj->isActiveConstr.size[0];
        i2 = obj->isActiveIdxNormal[4] + idx;
        if ((i2 < 1) || (i2 > i)) {
          emlrtDynamicBoundsCheckR2012b(i2, 1, i, &lb_emlrtBCI, (emlrtCTX)sp);
        }
        obj->isActiveConstr.data[i2 - 1] = obj->isActiveConstr.data[i1 - 1];
      }
    }
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesNormal[i];
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxNormal[i];
    }
  } break;
  case 1:
    obj->nVar = 266;
    obj->mConstr = obj->mConstrOrig + 1;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesPhaseOne[i];
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxPhaseOne[i];
    }
    st.site = &uc_emlrtRSI;
    modifyOverheadPhaseOne_(&st, obj);
    break;
  case 2:
    obj->nVar = obj->nVarMax - 1;
    obj->mConstr = obj->mConstrMax - 1;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesRegularized[i];
    }
    if (obj->probType != 4) {
      st.site = &uc_emlrtRSI;
      modifyOverheadRegularized_(&st, obj);
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxRegularized[i];
    }
    break;
  default:
    obj->nVar = obj->nVarMax;
    obj->mConstr = obj->mConstrMax;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesRegPhaseOne[i];
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxRegPhaseOne[i];
    }
    st.site = &uc_emlrtRSI;
    modifyOverheadPhaseOne_(&st, obj);
    break;
  }
  obj->probType = PROBLEM_TYPE;
}

/* End of code generation (setProblemType.c) */
