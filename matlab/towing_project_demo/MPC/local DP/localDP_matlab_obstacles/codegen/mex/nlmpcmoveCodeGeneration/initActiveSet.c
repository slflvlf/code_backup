/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * initActiveSet.c
 *
 * Code generation for function 'initActiveSet'
 *
 */

/* Include files */
#include "initActiveSet.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo sc_emlrtRSI = {
    1,               /* lineNo */
    "initActiveSet", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\initActiveSet.p" /* pathName */
};

static emlrtBCInfo me_emlrtBCI = {
    -1,              /* iFirst */
    -1,              /* iLast */
    1,               /* lineNo */
    1,               /* colNo */
    "",              /* aName */
    "initActiveSet", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\initActiveSet.p", /* pName */
    0                              /* checkKind */
};

/* Function Definitions */
void initActiveSet(const emlrtStack *sp, h_struct_T *obj)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T b;
  int32_T b_i;
  int32_T colOffsetATw;
  int32_T i;
  int32_T i1;
  int32_T iATw0;
  int32_T idxFillStart;
  int32_T idx_local;
  int32_T nWFixed;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &sc_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  setProblemType(&st, obj, 3);
  idxFillStart = obj->isActiveIdx[2];
  b = obj->mConstrMax;
  st.site = &sc_emlrtRSI;
  if ((obj->isActiveIdx[2] <= obj->mConstrMax) &&
      (obj->mConstrMax > 2147483646)) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (colOffsetATw = idxFillStart; colOffsetATw <= b; colOffsetATw++) {
    i = obj->isActiveConstr.size[0];
    if ((colOffsetATw < 1) || (colOffsetATw > i)) {
      emlrtDynamicBoundsCheckR2012b(colOffsetATw, 1, i, &me_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    obj->isActiveConstr.data[colOffsetATw - 1] = false;
  }
  obj->nWConstr[0] = obj->sizes[0];
  obj->nWConstr[1] = 60;
  obj->nWConstr[2] = 0;
  obj->nWConstr[3] = 0;
  obj->nWConstr[4] = 0;
  obj->nActiveConstr = obj->nWConstr[0] + 60;
  nWFixed = obj->sizes[0] - 1;
  st.site = &sc_emlrtRSI;
  if (obj->sizes[0] > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx_local = 0; idx_local <= nWFixed; idx_local++) {
    i = obj->Wid.size[0];
    if ((idx_local + 1 < 1) || (idx_local + 1 > i)) {
      emlrtDynamicBoundsCheckR2012b(idx_local + 1, 1, i, &me_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    obj->Wid.data[idx_local] = 1;
    i = obj->Wlocalidx.size[0];
    if (idx_local + 1 > i) {
      emlrtDynamicBoundsCheckR2012b(idx_local + 1, 1, i, &me_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    obj->Wlocalidx.data[idx_local] = idx_local + 1;
    i = obj->isActiveConstr.size[0];
    if (idx_local + 1 > i) {
      emlrtDynamicBoundsCheckR2012b(idx_local + 1, 1, i, &me_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    obj->isActiveConstr.data[idx_local] = true;
    i = obj->indexFixed.size[0];
    if (idx_local + 1 > i) {
      emlrtDynamicBoundsCheckR2012b(idx_local + 1, 1, i, &me_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    idxFillStart = obj->indexFixed.data[idx_local];
    colOffsetATw = obj->ldA * idx_local;
    st.site = &sc_emlrtRSI;
    if (idxFillStart - 1 > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (b_i = 0; b_i <= idxFillStart - 2; b_i++) {
      i = obj->ATwset.size[0];
      i1 = (b_i + colOffsetATw) + 1;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &me_emlrtBCI, (emlrtCTX)sp);
      }
      obj->ATwset.data[i1 - 1] = 0.0;
    }
    i = obj->ATwset.size[0];
    i1 = idxFillStart + colOffsetATw;
    if ((i1 < 1) || (i1 > i)) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i, &me_emlrtBCI, (emlrtCTX)sp);
    }
    obj->ATwset.data[i1 - 1] = 1.0;
    iATw0 = idxFillStart + 1;
    b = obj->nVar;
    st.site = &sc_emlrtRSI;
    if ((idxFillStart + 1 <= obj->nVar) && (obj->nVar > 2147483646)) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (b_i = iATw0; b_i <= b; b_i++) {
      i = obj->ATwset.size[0];
      i1 = b_i + colOffsetATw;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &me_emlrtBCI, (emlrtCTX)sp);
      }
      obj->ATwset.data[i1 - 1] = 0.0;
    }
    i = obj->bwset.size[0];
    if (idx_local + 1 > i) {
      emlrtDynamicBoundsCheckR2012b(idx_local + 1, 1, i, &me_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    obj->bwset.data[idx_local] = obj->ub.data[idxFillStart - 1];
  }
  st.site = &sc_emlrtRSI;
  for (idx_local = 0; idx_local < 60; idx_local++) {
    idxFillStart = (nWFixed + idx_local) + 2;
    i = obj->Wid.size[0];
    if ((idxFillStart < 1) || (idxFillStart > i)) {
      emlrtDynamicBoundsCheckR2012b(idxFillStart, 1, i, &me_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    obj->Wid.data[idxFillStart - 1] = 2;
    i = obj->Wlocalidx.size[0];
    if (idxFillStart > i) {
      emlrtDynamicBoundsCheckR2012b(idxFillStart, 1, i, &me_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    obj->Wlocalidx.data[idxFillStart - 1] = idx_local + 1;
    i = obj->isActiveConstr.size[0];
    if (idxFillStart > i) {
      emlrtDynamicBoundsCheckR2012b(idxFillStart, 1, i, &me_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    obj->isActiveConstr.data[idxFillStart - 1] = true;
    colOffsetATw = obj->ldA * idx_local + 1;
    iATw0 = obj->ldA * (idxFillStart - 1) + 1;
    i = obj->nVar - 1;
    for (b_i = 0; b_i <= i; b_i++) {
      int32_T i2;
      i1 = obj->Aeq.size[0];
      b = colOffsetATw + b_i;
      if ((b < 1) || (b > i1)) {
        emlrtDynamicBoundsCheckR2012b(b, 1, i1, &me_emlrtBCI, (emlrtCTX)sp);
      }
      i1 = obj->ATwset.size[0];
      i2 = iATw0 + b_i;
      if ((i2 < 1) || (i2 > i1)) {
        emlrtDynamicBoundsCheckR2012b(i2, 1, i1, &me_emlrtBCI, (emlrtCTX)sp);
      }
      obj->ATwset.data[i2 - 1] = obj->Aeq.data[b - 1];
    }
    i = obj->bwset.size[0];
    if (idxFillStart > i) {
      emlrtDynamicBoundsCheckR2012b(idxFillStart, 1, i, &me_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    obj->bwset.data[idxFillStart - 1] = obj->beq[idx_local];
  }
}

/* End of code generation (initActiveSet.c) */
