/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * removeAllIneqConstr.c
 *
 * Code generation for function 'removeAllIneqConstr'
 *
 */

/* Include files */
#include "removeAllIneqConstr.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo pf_emlrtRSI = {
    1,                     /* lineNo */
    "removeAllIneqConstr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\removeAllIneqConstr.p" /* pathName */
};

static emlrtBCInfo tc_emlrtBCI = {
    -1,                    /* iFirst */
    -1,                    /* iLast */
    1,                     /* lineNo */
    1,                     /* colNo */
    "",                    /* aName */
    "removeAllIneqConstr", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\removeAllIneqConstr.p", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo uc_emlrtBCI = {
    1,                     /* iFirst */
    6,                     /* iLast */
    1,                     /* lineNo */
    1,                     /* colNo */
    "",                    /* aName */
    "removeAllIneqConstr", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\removeAllIneqConstr.p", /* pName */
    0                                    /* checkKind */
};

/* Function Definitions */
void removeAllIneqConstr(const emlrtStack *sp, h_struct_T *obj)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T idxEndIneq;
  int32_T idxStartIneq_tmp;
  int32_T idx_global;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  idxStartIneq_tmp = (obj->nWConstr[0] + obj->nWConstr[1]) + 1;
  idxEndIneq = obj->nActiveConstr;
  st.site = &pf_emlrtRSI;
  if (((obj->nWConstr[0] + obj->nWConstr[1]) + 1 <= obj->nActiveConstr) &&
      (obj->nActiveConstr > 2147483646)) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx_global = idxStartIneq_tmp; idx_global <= idxEndIneq; idx_global++) {
    int32_T i;
    int32_T i1;
    i = obj->Wid.size[0];
    if ((idx_global < 1) || (idx_global > i)) {
      emlrtDynamicBoundsCheckR2012b(idx_global, 1, i, &tc_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    i = obj->Wlocalidx.size[0];
    if (idx_global > i) {
      emlrtDynamicBoundsCheckR2012b(idx_global, 1, i, &tc_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    i = obj->Wid.data[idx_global - 1];
    if ((i < 1) || (i > 6)) {
      emlrtDynamicBoundsCheckR2012b(obj->Wid.data[idx_global - 1], 1, 6,
                                    &uc_emlrtBCI, (emlrtCTX)sp);
    }
    i1 = obj->isActiveConstr.size[0];
    i = (obj->isActiveIdx[i - 1] + obj->Wlocalidx.data[idx_global - 1]) - 1;
    if ((i < 1) || (i > i1)) {
      emlrtDynamicBoundsCheckR2012b(i, 1, i1, &tc_emlrtBCI, (emlrtCTX)sp);
    }
    obj->isActiveConstr.data[i - 1] = false;
  }
  obj->nWConstr[2] = 0;
  obj->nWConstr[3] = 0;
  obj->nWConstr[4] = 0;
  obj->nActiveConstr = obj->nWConstr[0] + obj->nWConstr[1];
}

/* End of code generation (removeAllIneqConstr.c) */
