/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * modifyOverheadPhaseOne_.c
 *
 * Code generation for function 'modifyOverheadPhaseOne_'
 *
 */

/* Include files */
#include "modifyOverheadPhaseOne_.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo vc_emlrtRSI = {
    1,                         /* lineNo */
    "modifyOverheadPhaseOne_", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\modifyOverheadPhaseOne_.p" /* pathName */
};

static emlrtBCInfo mb_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    1,                         /* lineNo */
    1,                         /* colNo */
    "",                        /* aName */
    "modifyOverheadPhaseOne_", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\modifyOverheadPhaseOne_.p", /* pName */
    0                                        /* checkKind */
};

/* Function Definitions */
void modifyOverheadPhaseOne_(const emlrtStack *sp, h_struct_T *obj)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T i;
  int32_T i1;
  int32_T idx;
  int32_T idxEq;
  int32_T idxStartIneq;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  idxEq = obj->sizes[0];
  st.site = &vc_emlrtRSI;
  if (obj->sizes[0] > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = 0; idx < idxEq; idx++) {
    i = obj->ATwset.size[0];
    i1 = obj->nVar + obj->ldA * idx;
    if ((i1 < 1) || (i1 > i)) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i, &mb_emlrtBCI, (emlrtCTX)sp);
    }
    obj->ATwset.data[i1 - 1] = 0.0;
  }
  st.site = &vc_emlrtRSI;
  for (idx = 0; idx < 192; idx++) {
    idxEq = obj->nVar + obj->ldA * idx;
    i = obj->Aeq.size[0];
    if ((idxEq < 1) || (idxEq > i)) {
      emlrtDynamicBoundsCheckR2012b(idxEq, 1, i, &mb_emlrtBCI, (emlrtCTX)sp);
    }
    obj->Aeq.data[idxEq - 1] = 0.0;
    i = obj->ATwset.size[0];
    i1 = idxEq + obj->ldA * (obj->isActiveIdx[1] - 1);
    if ((i1 < 1) || (i1 > i)) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i, &mb_emlrtBCI, (emlrtCTX)sp);
    }
    obj->ATwset.data[i1 - 1] = 0.0;
  }
  idxEq = obj->sizes[2];
  st.site = &vc_emlrtRSI;
  if (obj->sizes[2] > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = 0; idx < idxEq; idx++) {
    i = obj->Aineq.size[0];
    i1 = obj->nVar + obj->ldA * idx;
    if ((i1 < 1) || (i1 > i)) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i, &mb_emlrtBCI, (emlrtCTX)sp);
    }
    obj->Aineq.data[i1 - 1] = -1.0;
  }
  i = obj->indexLB.size[0];
  if ((obj->sizes[3] < 1) || (obj->sizes[3] > i)) {
    emlrtDynamicBoundsCheckR2012b(obj->sizes[3], 1, i, &mb_emlrtBCI,
                                  (emlrtCTX)sp);
  }
  obj->indexLB.data[obj->sizes[3] - 1] = obj->nVar;
  i = obj->lb.size[0];
  if ((obj->nVar < 1) || (obj->nVar > i)) {
    emlrtDynamicBoundsCheckR2012b(obj->nVar, 1, i, &mb_emlrtBCI, (emlrtCTX)sp);
  }
  obj->lb.data[obj->nVar - 1] = 1.0E-5;
  idxStartIneq = obj->isActiveIdx[2];
  idxEq = obj->nActiveConstr;
  st.site = &vc_emlrtRSI;
  if ((obj->isActiveIdx[2] <= obj->nActiveConstr) &&
      (obj->nActiveConstr > 2147483646)) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = idxStartIneq; idx <= idxEq; idx++) {
    i = obj->ATwset.size[0];
    i1 = obj->nVar + obj->ldA * (idx - 1);
    if ((i1 < 1) || (i1 > i)) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i, &mb_emlrtBCI, (emlrtCTX)sp);
    }
    obj->ATwset.data[i1 - 1] = -1.0;
  }
  if (obj->nWConstr[4] > 0) {
    idxEq = obj->sizesNormal[4];
    st.site = &vc_emlrtRSI;
    if (obj->sizesNormal[4] + 1 > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx = 0; idx <= idxEq; idx++) {
      i = obj->isActiveConstr.size[0];
      i1 = obj->isActiveIdx[4] + idx;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &mb_emlrtBCI, (emlrtCTX)sp);
      }
      obj->isActiveConstr.data[i1 - 1] = false;
    }
  }
  i = obj->isActiveConstr.size[0];
  i1 = obj->isActiveIdx[4] - 1;
  if ((i1 < 1) || (i1 > i)) {
    emlrtDynamicBoundsCheckR2012b(i1, 1, i, &mb_emlrtBCI, (emlrtCTX)sp);
  }
  obj->isActiveConstr.data[i1 - 1] = false;
}

/* End of code generation (modifyOverheadPhaseOne_.c) */
