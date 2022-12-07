/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * updateWorkingSetForNewQP.c
 *
 * Code generation for function 'updateWorkingSetForNewQP'
 *
 */

/* Include files */
#include "updateWorkingSetForNewQP.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo sc_emlrtRSI = {
    1,                          /* lineNo */
    "updateWorkingSetForNewQP", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "internal\\updateWorkingSetForNewQP.p" /* pathName */
};

static emlrtBCInfo ke_emlrtBCI = {
    -1,                         /* iFirst */
    -1,                         /* iLast */
    1,                          /* lineNo */
    1,                          /* colNo */
    "",                         /* aName */
    "updateWorkingSetForNewQP", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "internal\\updateWorkingSetForNewQP.p", /* pName */
    0                                       /* checkKind */
};

static emlrtBCInfo ne_emlrtBCI = {
    1,                          /* iFirst */
    265,                        /* iLast */
    1,                          /* lineNo */
    1,                          /* colNo */
    "",                         /* aName */
    "updateWorkingSetForNewQP", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "internal\\updateWorkingSetForNewQP.p", /* pName */
    0                                       /* checkKind */
};

/* Function Definitions */
void b_updateWorkingSetForNewQP(const emlrtStack *sp, const real_T xk[265],
                                h_struct_T *WorkingSet, int32_T mIneq,
                                int32_T mNonlinIneq, const real_T cIneq_data[],
                                int32_T cIneq_size, const real_T cEq[192],
                                int32_T mLB, const real_T lb[265], int32_T mUB,
                                int32_T mFixed)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack st;
  int32_T b;
  int32_T b_i;
  int32_T i;
  int32_T i1;
  int32_T iEq0;
  int32_T idx;
  int32_T iw0;
  int32_T nVar;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  nVar = WorkingSet->nVar;
  st.site = &sc_emlrtRSI;
  for (idx = 0; idx < 192; idx++) {
    WorkingSet->beq[idx] = -cEq[idx];
    i = WorkingSet->bwset.size[0];
    i1 = (mFixed + idx) + 1;
    if ((i1 < 1) || (i1 > i)) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i, &ke_emlrtBCI, (emlrtCTX)sp);
    }
    WorkingSet->bwset.data[i1 - 1] = WorkingSet->beq[idx];
  }
  iw0 = WorkingSet->ldA * mFixed + 1;
  iEq0 = 1;
  st.site = &sc_emlrtRSI;
  i = nVar - 1;
  for (idx = 0; idx < 192; idx++) {
    for (b_i = 0; b_i <= i; b_i++) {
      int32_T i2;
      i1 = WorkingSet->Aeq.size[0];
      b = iEq0 + b_i;
      if ((b < 1) || (b > i1)) {
        emlrtDynamicBoundsCheckR2012b(b, 1, i1, &ke_emlrtBCI, (emlrtCTX)sp);
      }
      i1 = WorkingSet->ATwset.size[0];
      i2 = iw0 + b_i;
      if ((i2 < 1) || (i2 > i1)) {
        emlrtDynamicBoundsCheckR2012b(i2, 1, i1, &ke_emlrtBCI, (emlrtCTX)sp);
      }
      WorkingSet->ATwset.data[i2 - 1] = WorkingSet->Aeq.data[b - 1];
    }
    iw0 += WorkingSet->ldA;
    iEq0 += WorkingSet->ldA;
  }
  st.site = &sc_emlrtRSI;
  if (mIneq > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = 0; idx < mIneq; idx++) {
    if ((idx + 1 < 1) || (idx + 1 > cIneq_size)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, cIneq_size, &ke_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    i = WorkingSet->bineq.size[0];
    if (idx + 1 > i) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &ke_emlrtBCI, (emlrtCTX)sp);
    }
    WorkingSet->bineq.data[idx] = -cIneq_data[idx];
  }
  st.site = &sc_emlrtRSI;
  if (mLB > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = 0; idx < mLB; idx++) {
    i = WorkingSet->indexLB.size[0];
    if ((idx + 1 < 1) || (idx + 1 > i)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &ke_emlrtBCI, (emlrtCTX)sp);
    }
    if ((WorkingSet->indexLB.data[idx] < 1) ||
        (WorkingSet->indexLB.data[idx] > 265)) {
      emlrtDynamicBoundsCheckR2012b(WorkingSet->indexLB.data[idx], 1, 265,
                                    &ne_emlrtBCI, (emlrtCTX)sp);
    }
    i = WorkingSet->lb.size[0];
    if ((WorkingSet->indexLB.data[idx] < 1) ||
        (WorkingSet->indexLB.data[idx] > i)) {
      emlrtDynamicBoundsCheckR2012b(WorkingSet->indexLB.data[idx], 1, i,
                                    &ke_emlrtBCI, (emlrtCTX)sp);
    }
    WorkingSet->lb.data[WorkingSet->indexLB.data[idx] - 1] =
        -lb[WorkingSet->indexLB.data[idx] - 1] +
        xk[WorkingSet->indexLB.data[idx] - 1];
  }
  st.site = &sc_emlrtRSI;
  if (mUB > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = 0; idx < mUB; idx++) {
    i = WorkingSet->indexUB.size[0];
    if ((idx + 1 < 1) || (idx + 1 > i)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &ke_emlrtBCI, (emlrtCTX)sp);
    }
    WorkingSet->ub.data[WorkingSet->indexUB.data[idx] - 1] =
        rtInf - xk[WorkingSet->indexUB.data[idx] - 1];
  }
  st.site = &sc_emlrtRSI;
  if (mFixed > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = 0; idx < mFixed; idx++) {
    i = WorkingSet->indexFixed.size[0];
    if ((idx + 1 < 1) || (idx + 1 > i)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &ke_emlrtBCI, (emlrtCTX)sp);
    }
    iw0 = WorkingSet->indexFixed.data[idx];
    WorkingSet->ub.data[WorkingSet->indexFixed.data[idx] - 1] =
        rtInf - xk[WorkingSet->indexFixed.data[idx] - 1];
    i = WorkingSet->bwset.size[0];
    if ((idx + 1 < 1) || (idx + 1 > i)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &ke_emlrtBCI, (emlrtCTX)sp);
    }
    WorkingSet->bwset.data[idx] = rtInf - xk[iw0 - 1];
  }
  if (WorkingSet->nActiveConstr > mFixed + 192) {
    iEq0 = mFixed + 193;
    b = WorkingSet->nActiveConstr;
    st.site = &sc_emlrtRSI;
    if ((mFixed + 193 <= WorkingSet->nActiveConstr) &&
        (WorkingSet->nActiveConstr > 2147483646)) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx = iEq0; idx <= b; idx++) {
      i = WorkingSet->Wlocalidx.size[0];
      if ((idx < 1) || (idx > i)) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ke_emlrtBCI, (emlrtCTX)sp);
      }
      iw0 = WorkingSet->Wlocalidx.data[idx - 1];
      i = WorkingSet->Wid.size[0];
      if (idx > i) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ke_emlrtBCI, (emlrtCTX)sp);
      }
      switch (WorkingSet->Wid.data[idx - 1]) {
      case 4:
        i = WorkingSet->indexLB.size[0];
        if ((iw0 < 1) || (iw0 > i)) {
          emlrtDynamicBoundsCheckR2012b(WorkingSet->Wlocalidx.data[idx - 1], 1,
                                        i, &ke_emlrtBCI, (emlrtCTX)sp);
        }
        i = WorkingSet->lb.size[0];
        i1 = WorkingSet->indexLB.data[iw0 - 1];
        if ((i1 < 1) || (i1 > i)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, i, &ke_emlrtBCI, (emlrtCTX)sp);
        }
        i = WorkingSet->bwset.size[0];
        if (idx > i) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ke_emlrtBCI, (emlrtCTX)sp);
        }
        WorkingSet->bwset.data[idx - 1] = WorkingSet->lb.data[i1 - 1];
        break;
      case 5:
        i = WorkingSet->indexUB.size[0];
        if ((iw0 < 1) || (iw0 > i)) {
          emlrtDynamicBoundsCheckR2012b(WorkingSet->Wlocalidx.data[idx - 1], 1,
                                        i, &ke_emlrtBCI, (emlrtCTX)sp);
        }
        i = WorkingSet->bwset.size[0];
        if (idx > i) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ke_emlrtBCI, (emlrtCTX)sp);
        }
        WorkingSet->bwset.data[idx - 1] =
            WorkingSet->ub
                .data[WorkingSet->indexUB
                          .data[WorkingSet->Wlocalidx.data[idx - 1] - 1] -
                      1];
        break;
      default:
        i = WorkingSet->bineq.size[0];
        if ((iw0 < 1) || (iw0 > i)) {
          emlrtDynamicBoundsCheckR2012b(WorkingSet->Wlocalidx.data[idx - 1], 1,
                                        i, &ke_emlrtBCI, (emlrtCTX)sp);
        }
        i = WorkingSet->bwset.size[0];
        if (idx > i) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ke_emlrtBCI, (emlrtCTX)sp);
        }
        WorkingSet->bwset.data[idx - 1] =
            WorkingSet->bineq.data[WorkingSet->Wlocalidx.data[idx - 1] - 1];
        if ((mNonlinIneq > 0) && (iw0 >= mNonlinIneq)) {
          st.site = &sc_emlrtRSI;
          n_t = (ptrdiff_t)nVar;
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          dcopy(&n_t, &WorkingSet->Aineq.data[WorkingSet->ldA * (iw0 - 1)],
                &incx_t, &WorkingSet->ATwset.data[WorkingSet->ldA * (idx - 1)],
                &incy_t);
        }
        break;
      }
    }
  }
}

void updateWorkingSetForNewQP(const emlrtStack *sp, const real_T xk[265],
                              h_struct_T *WorkingSet, int32_T mIneq,
                              const real_T cIneq_data[], int32_T cIneq_size,
                              const real_T cEq[192], int32_T mLB,
                              const real_T lb[265])
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T b_i;
  int32_T i;
  int32_T iEq0;
  int32_T idx;
  int32_T iw0;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  for (idx = 0; idx < 192; idx++) {
    WorkingSet->beq[idx] = -cEq[idx];
    i = WorkingSet->bwset.size[0];
    if (idx + 1 > i) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &ke_emlrtBCI, (emlrtCTX)sp);
    }
    WorkingSet->bwset.data[idx] = WorkingSet->beq[idx];
  }
  iw0 = 1;
  iEq0 = 1;
  st.site = &sc_emlrtRSI;
  for (idx = 0; idx < 192; idx++) {
    for (b_i = 0; b_i < 265; b_i++) {
      int32_T i1;
      int32_T i2;
      i = WorkingSet->Aeq.size[0];
      i1 = iEq0 + b_i;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &ke_emlrtBCI, (emlrtCTX)sp);
      }
      i = WorkingSet->ATwset.size[0];
      i2 = iw0 + b_i;
      if ((i2 < 1) || (i2 > i)) {
        emlrtDynamicBoundsCheckR2012b(i2, 1, i, &ke_emlrtBCI, (emlrtCTX)sp);
      }
      WorkingSet->ATwset.data[i2 - 1] = WorkingSet->Aeq.data[i1 - 1];
    }
    iw0 += WorkingSet->ldA;
    iEq0 += WorkingSet->ldA;
  }
  st.site = &sc_emlrtRSI;
  if (mIneq > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = 0; idx < mIneq; idx++) {
    if ((idx + 1 < 1) || (idx + 1 > cIneq_size)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, cIneq_size, &ke_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    i = WorkingSet->bineq.size[0];
    if (idx + 1 > i) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &ke_emlrtBCI, (emlrtCTX)sp);
    }
    WorkingSet->bineq.data[idx] = -cIneq_data[idx];
  }
  st.site = &sc_emlrtRSI;
  if (mLB > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = 0; idx < mLB; idx++) {
    i = WorkingSet->indexLB.size[0];
    if ((idx + 1 < 1) || (idx + 1 > i)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &ke_emlrtBCI, (emlrtCTX)sp);
    }
    WorkingSet->lb.data[WorkingSet->indexLB.data[idx] - 1] =
        -lb[WorkingSet->indexLB.data[idx] - 1] +
        xk[WorkingSet->indexLB.data[idx] - 1];
  }
  st.site = &sc_emlrtRSI;
  st.site = &sc_emlrtRSI;
}

/* End of code generation (updateWorkingSetForNewQP.c) */
