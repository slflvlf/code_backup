/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * modifyOverheadRegularized_.c
 *
 * Code generation for function 'modifyOverheadRegularized_'
 *
 */

/* Include files */
#include "modifyOverheadRegularized_.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo wc_emlrtRSI = {
    1,                            /* lineNo */
    "modifyOverheadRegularized_", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\modifyOverheadRegularized_.p" /* pathName */
};

static emlrtBCInfo nb_emlrtBCI = {
    -1,                           /* iFirst */
    -1,                           /* iLast */
    1,                            /* lineNo */
    1,                            /* colNo */
    "",                           /* aName */
    "modifyOverheadRegularized_", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\modifyOverheadRegularized_.p", /* pName */
    0                                           /* checkKind */
};

/* Function Definitions */
void modifyOverheadRegularized_(const emlrtStack *sp, h_struct_T *obj)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T a;
  int32_T b;
  int32_T b_b;
  int32_T b_tmp;
  int32_T colOffsetATw;
  int32_T colOffsetAineq;
  int32_T i;
  int32_T i1;
  int32_T idx_col;
  int32_T idx_lb;
  int32_T idx_local;
  int32_T idx_row;
  int32_T mIneq_tmp;
  int32_T offsetEq2;
  boolean_T overflow;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  mIneq_tmp = obj->sizes[2] + 265;
  offsetEq2 = obj->sizes[2] + 457;
  b = obj->sizes[0];
  st.site = &wc_emlrtRSI;
  if (obj->sizes[0] > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx_col = 0; idx_col < b; idx_col++) {
    colOffsetATw = obj->ldA * idx_col;
    b_b = obj->nVar;
    st.site = &wc_emlrtRSI;
    if (obj->nVar > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx_row = 266; idx_row <= b_b; idx_row++) {
      i = obj->ATwset.size[0];
      i1 = idx_row + colOffsetATw;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
      }
      obj->ATwset.data[i1 - 1] = 0.0;
    }
  }
  st.site = &wc_emlrtRSI;
  if (obj->sizes[2] > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx_col = 0; idx_col <= mIneq_tmp - 266; idx_col++) {
    idx_lb = idx_col + 265;
    colOffsetAineq = obj->ldA * idx_col;
    st.site = &wc_emlrtRSI;
    if (idx_col + 265 > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx_row = 266; idx_row <= idx_lb; idx_row++) {
      i = obj->Aineq.size[0];
      i1 = idx_row + colOffsetAineq;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
      }
      obj->Aineq.data[i1 - 1] = 0.0;
    }
    i = obj->Aineq.size[0];
    i1 = (idx_col + colOffsetAineq) + 266;
    if ((i1 < 1) || (i1 > i)) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
    }
    obj->Aineq.data[i1 - 1] = -1.0;
    a = idx_col + 267;
    b = obj->nVar;
    st.site = &wc_emlrtRSI;
    if ((idx_col + 267 <= obj->nVar) && (obj->nVar > 2147483646)) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx_row = a; idx_row <= b; idx_row++) {
      i = obj->Aineq.size[0];
      i1 = idx_row + colOffsetAineq;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
      }
      obj->Aineq.data[i1 - 1] = 0.0;
    }
  }
  st.site = &wc_emlrtRSI;
  overflow = (mIneq_tmp > 2147483646);
  a = mIneq_tmp + 1;
  idx_lb = mIneq_tmp + 193;
  for (idx_col = 0; idx_col < 192; idx_col++) {
    colOffsetAineq = obj->ldA * idx_col;
    colOffsetATw = colOffsetAineq + obj->ldA * (obj->isActiveIdx[1] - 1);
    st.site = &wc_emlrtRSI;
    if (overflow) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx_row = 266; idx_row <= mIneq_tmp; idx_row++) {
      i = obj->Aeq.size[0];
      i1 = idx_row + colOffsetAineq;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
      }
      obj->Aeq.data[i1 - 1] = 0.0;
      i = obj->ATwset.size[0];
      i1 = idx_row + colOffsetATw;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
      }
      obj->ATwset.data[i1 - 1] = 0.0;
    }
    b_tmp = mIneq_tmp + idx_col;
    st.site = &wc_emlrtRSI;
    if ((mIneq_tmp + 1 <= b_tmp) && (b_tmp > 2147483646)) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx_row = a; idx_row <= b_tmp; idx_row++) {
      i = obj->Aeq.size[0];
      i1 = idx_row + colOffsetAineq;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
      }
      obj->Aeq.data[i1 - 1] = 0.0;
      i = obj->ATwset.size[0];
      i1 = idx_row + colOffsetATw;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
      }
      obj->ATwset.data[i1 - 1] = 0.0;
    }
    i = obj->Aeq.size[0];
    i1 = b_tmp + colOffsetAineq;
    if ((i1 + 1 < 1) || (i1 + 1 > i)) {
      emlrtDynamicBoundsCheckR2012b(i1 + 1, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
    }
    obj->Aeq.data[i1] = -1.0;
    i = obj->ATwset.size[0];
    b_b = (b_tmp + colOffsetATw) + 1;
    if ((b_b < 1) || (b_b > i)) {
      emlrtDynamicBoundsCheckR2012b(b_b, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
    }
    obj->ATwset.data[b_b - 1] = -1.0;
    idx_local = b_tmp + 2;
    st.site = &wc_emlrtRSI;
    if ((b_tmp + 2 <= mIneq_tmp + 192) && (mIneq_tmp + 192 > 2147483646)) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx_row = idx_local; idx_row <= offsetEq2; idx_row++) {
      i = obj->Aeq.size[0];
      b_b = idx_row + colOffsetAineq;
      if ((b_b < 1) || (b_b > i)) {
        emlrtDynamicBoundsCheckR2012b(b_b, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
      }
      obj->Aeq.data[b_b - 1] = 0.0;
      i = obj->ATwset.size[0];
      b_b = idx_row + colOffsetATw;
      if ((b_b < 1) || (b_b > i)) {
        emlrtDynamicBoundsCheckR2012b(b_b, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
      }
      obj->ATwset.data[b_b - 1] = 0.0;
    }
    b = b_tmp + 192;
    st.site = &wc_emlrtRSI;
    if ((offsetEq2 + 1 <= b_tmp + 192) && (b_tmp + 192 > 2147483646)) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx_row = idx_lb; idx_row <= b; idx_row++) {
      i = obj->Aeq.size[0];
      b_b = idx_row + colOffsetAineq;
      if ((b_b < 1) || (b_b > i)) {
        emlrtDynamicBoundsCheckR2012b(b_b, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
      }
      obj->Aeq.data[b_b - 1] = 0.0;
      i = obj->ATwset.size[0];
      b_b = idx_row + colOffsetATw;
      if ((b_b < 1) || (b_b > i)) {
        emlrtDynamicBoundsCheckR2012b(b_b, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
      }
      obj->ATwset.data[b_b - 1] = 0.0;
    }
    i = obj->Aeq.size[0];
    if ((i1 + 193 < 1) || (i1 + 193 > i)) {
      emlrtDynamicBoundsCheckR2012b(i1 + 193, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
    }
    obj->Aeq.data[i1 + 192] = 1.0;
    i = obj->ATwset.size[0];
    i1 = (b_tmp + colOffsetATw) + 193;
    if ((i1 < 1) || (i1 > i)) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
    }
    obj->ATwset.data[i1 - 1] = 1.0;
    idx_local = b_tmp + 194;
    b = obj->nVar;
    st.site = &wc_emlrtRSI;
    if ((b_tmp + 194 <= obj->nVar) && (obj->nVar > 2147483646)) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx_row = idx_local; idx_row <= b; idx_row++) {
      i = obj->Aeq.size[0];
      i1 = idx_row + colOffsetAineq;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
      }
      obj->Aeq.data[i1 - 1] = 0.0;
      i = obj->ATwset.size[0];
      i1 = idx_row + colOffsetATw;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
      }
      obj->ATwset.data[i1 - 1] = 0.0;
    }
  }
  idx_lb = 265;
  a = obj->sizesNormal[3] + 1;
  b = obj->sizesRegularized[3];
  st.site = &wc_emlrtRSI;
  if ((obj->sizesNormal[3] + 1 <= obj->sizesRegularized[3]) &&
      (obj->sizesRegularized[3] > 2147483646)) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (colOffsetAineq = a; colOffsetAineq <= b; colOffsetAineq++) {
    idx_lb++;
    i = obj->indexLB.size[0];
    if ((colOffsetAineq < 1) || (colOffsetAineq > i)) {
      emlrtDynamicBoundsCheckR2012b(colOffsetAineq, 1, i, &nb_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    obj->indexLB.data[colOffsetAineq - 1] = idx_lb;
  }
  if (obj->nWConstr[4] > 0) {
    b = obj->sizesRegularized[4];
    st.site = &wc_emlrtRSI;
    if (obj->sizesRegularized[4] > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (colOffsetAineq = 0; colOffsetAineq < b; colOffsetAineq++) {
      i = obj->isActiveConstr.size[0];
      i1 = obj->isActiveIdx[4] + colOffsetAineq;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
      }
      i = obj->isActiveConstr.size[0];
      b_b = (obj->isActiveIdxRegularized[4] + colOffsetAineq) + 1;
      if ((b_b < 1) || (b_b > i)) {
        emlrtDynamicBoundsCheckR2012b(b_b, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
      }
      obj->isActiveConstr.data[b_b - 1] = obj->isActiveConstr.data[i1 - 1];
    }
  }
  a = obj->isActiveIdx[4];
  b = obj->isActiveIdxRegularized[4] - 1;
  st.site = &wc_emlrtRSI;
  if ((obj->isActiveIdx[4] <= obj->isActiveIdxRegularized[4] - 1) &&
      (obj->isActiveIdxRegularized[4] - 1 > 2147483646)) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (colOffsetAineq = a; colOffsetAineq <= b; colOffsetAineq++) {
    i = obj->isActiveConstr.size[0];
    if ((colOffsetAineq < 1) || (colOffsetAineq > i)) {
      emlrtDynamicBoundsCheckR2012b(colOffsetAineq, 1, i, &nb_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    obj->isActiveConstr.data[colOffsetAineq - 1] = false;
  }
  b = obj->sizes[2] + 649;
  st.site = &wc_emlrtRSI;
  if (obj->sizes[2] + 649 > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (colOffsetAineq = 266; colOffsetAineq <= b; colOffsetAineq++) {
    i = obj->lb.size[0];
    if (colOffsetAineq > i) {
      emlrtDynamicBoundsCheckR2012b(colOffsetAineq, 1, i, &nb_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    obj->lb.data[colOffsetAineq - 1] = 0.0;
  }
  idx_lb = obj->isActiveIdx[2];
  b = obj->nActiveConstr;
  st.site = &wc_emlrtRSI;
  if ((obj->isActiveIdx[2] <= obj->nActiveConstr) &&
      (obj->nActiveConstr > 2147483646)) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx_col = idx_lb; idx_col <= b; idx_col++) {
    colOffsetATw = obj->ldA * (idx_col - 1);
    i = obj->Wid.size[0];
    if ((idx_col < 1) || (idx_col > i)) {
      emlrtDynamicBoundsCheckR2012b(idx_col, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
    }
    if (obj->Wid.data[idx_col - 1] == 3) {
      i = obj->Wlocalidx.size[0];
      if (idx_col > i) {
        emlrtDynamicBoundsCheckR2012b(idx_col, 1, i, &nb_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      colOffsetAineq = obj->Wlocalidx.data[idx_col - 1];
      idx_local = colOffsetAineq + 266;
      b_tmp = colOffsetAineq + 264;
      st.site = &wc_emlrtRSI;
      if (colOffsetAineq + 264 > 2147483646) {
        b_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }
      for (idx_row = 266; idx_row <= b_tmp; idx_row++) {
        i = obj->ATwset.size[0];
        i1 = idx_row + colOffsetATw;
        if ((i1 < 1) || (i1 > i)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
        }
        obj->ATwset.data[i1 - 1] = 0.0;
      }
      i = obj->ATwset.size[0];
      i1 = (colOffsetAineq + colOffsetATw) + 265;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
      }
      obj->ATwset.data[i1 - 1] = -1.0;
      b_b = obj->nVar;
      st.site = &wc_emlrtRSI;
      if ((colOffsetAineq + 266 <= obj->nVar) && (obj->nVar > 2147483646)) {
        b_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }
      for (idx_row = idx_local; idx_row <= b_b; idx_row++) {
        i = obj->ATwset.size[0];
        i1 = idx_row + colOffsetATw;
        if ((i1 < 1) || (i1 > i)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
        }
        obj->ATwset.data[i1 - 1] = 0.0;
      }
    } else {
      b_b = obj->nVar;
      st.site = &wc_emlrtRSI;
      if (obj->nVar > 2147483646) {
        b_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }
      for (idx_row = 266; idx_row <= b_b; idx_row++) {
        i = obj->ATwset.size[0];
        i1 = idx_row + colOffsetATw;
        if ((i1 < 1) || (i1 > i)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, i, &nb_emlrtBCI, (emlrtCTX)sp);
        }
        obj->ATwset.data[i1 - 1] = 0.0;
      }
    }
  }
}

/* End of code generation (modifyOverheadRegularized_.c) */
