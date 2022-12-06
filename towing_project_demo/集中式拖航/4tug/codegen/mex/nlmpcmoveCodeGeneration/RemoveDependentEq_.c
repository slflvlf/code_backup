/*
 * RemoveDependentEq_.c
 *
 * Code generation for function 'RemoveDependentEq_'
 *
 */

/* Include files */
#include "RemoveDependentEq_.h"
#include "computeQ_.h"
#include "countsort.h"
#include "eml_int_forloop_overflow_check.h"
#include "moveConstraint_.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "xgeqp3.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ne_emlrtRSI = {
    1,                    /* lineNo */
    "RemoveDependentEq_", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "initialize\\RemoveDependentEq_.p" /* pathName */
};

static emlrtRSInfo oe_emlrtRSI = {
    1,                        /* lineNo */
    "ComputeNumDependentEq_", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "initialize\\ComputeNumDependentEq_.p" /* pathName */
};

static emlrtRSInfo pe_emlrtRSI = {
    1,                     /* lineNo */
    "IndexOfDependentEq_", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "initialize\\IndexOfDependentEq_.p" /* pathName */
};

static emlrtRSInfo re_emlrtRSI = {
    1,                /* lineNo */
    "removeEqConstr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\removeEqConstr.p" /* pathName */
};

static emlrtBCInfo bc_emlrtBCI = {
    -1,                       /* iFirst */
    -1,                       /* iLast */
    1,                        /* lineNo */
    1,                        /* colNo */
    "",                       /* aName */
    "ComputeNumDependentEq_", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "initialize\\ComputeNumDependentEq_.p", /* pName */
    0                                       /* checkKind */
};

static emlrtBCInfo cc_emlrtBCI = {
    -1,                   /* iFirst */
    -1,                   /* iLast */
    1,                    /* lineNo */
    1,                    /* colNo */
    "",                   /* aName */
    "RemoveDependentEq_", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "initialize\\RemoveDependentEq_.p", /* pName */
    0                                   /* checkKind */
};

static emlrtBCInfo dc_emlrtBCI = {
    1,                /* iFirst */
    6,                /* iLast */
    1,                /* lineNo */
    1,                /* colNo */
    "",               /* aName */
    "removeEqConstr", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\removeEqConstr.p", /* pName */
    0                               /* checkKind */
};

static emlrtBCInfo gc_emlrtBCI = {
    -1,                    /* iFirst */
    -1,                    /* iLast */
    1,                     /* lineNo */
    1,                     /* colNo */
    "",                    /* aName */
    "IndexOfDependentEq_", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "initialize\\IndexOfDependentEq_.p", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo hc_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    1,                /* lineNo */
    1,                /* colNo */
    "",               /* aName */
    "removeEqConstr", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\removeEqConstr.p", /* pName */
    0                               /* checkKind */
};

static emlrtBCInfo ic_emlrtBCI = {
    1,                /* iFirst */
    60,               /* iLast */
    1,                /* lineNo */
    1,                /* colNo */
    "",               /* aName */
    "removeEqConstr", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\removeEqConstr.p", /* pName */
    3                               /* checkKind */
};

static emlrtBCInfo jc_emlrtBCI = {
    1,                /* iFirst */
    5,                /* iLast */
    1,                /* lineNo */
    1,                /* colNo */
    "",               /* aName */
    "removeEqConstr", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\removeEqConstr.p", /* pName */
    3                               /* checkKind */
};

/* Function Definitions */
int32_T RemoveDependentEq_(const emlrtStack *sp, g_struct_T *memspace,
                           h_struct_T *workingset, i_struct_T *qrmanager)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T idxDiag;
  int32_T idx_col;
  int32_T mTotalWorkingEq_tmp_tmp;
  int32_T mWorkingFixed;
  int32_T nDepInd;
  int32_T nVar_tmp;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  nVar_tmp = workingset->nVar;
  mWorkingFixed = workingset->nWConstr[0];
  mTotalWorkingEq_tmp_tmp = workingset->nWConstr[0] + workingset->nWConstr[1];
  nDepInd = 0;
  if (mTotalWorkingEq_tmp_tmp > 0) {
    real_T tol;
    int32_T TYPE;
    int32_T i;
    int32_T i1;
    int32_T i2;
    int32_T i3;
    boolean_T exitg1;
    boolean_T overflow;
    st.site = &ne_emlrtRSI;
    if (mTotalWorkingEq_tmp_tmp > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    overflow = (nVar_tmp > 2147483646);
    i = workingset->ATwset.size[0];
    for (idxDiag = 0; idxDiag < mTotalWorkingEq_tmp_tmp; idxDiag++) {
      st.site = &ne_emlrtRSI;
      if (overflow) {
        b_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }
      for (idx_col = 0; idx_col < nVar_tmp; idx_col++) {
        i1 = (idx_col + workingset->ldA * idxDiag) + 1;
        if ((i1 < 1) || (i1 > i)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, i, &cc_emlrtBCI, (emlrtCTX)sp);
        }
        i2 = qrmanager->QR.size[0] * qrmanager->QR.size[1];
        i3 = (idxDiag + qrmanager->ldq * idx_col) + 1;
        if ((i3 < 1) || (i3 > i2)) {
          emlrtDynamicBoundsCheckR2012b(i3, 1, i2, &cc_emlrtBCI, (emlrtCTX)sp);
        }
        qrmanager->QR.data[i3 - 1] = workingset->ATwset.data[i1 - 1];
      }
    }
    st.site = &ne_emlrtRSI;
    idxDiag = mTotalWorkingEq_tmp_tmp - workingset->nVar;
    nDepInd = muIntScalarMax_sint32(0, idxDiag);
    b_st.site = &oe_emlrtRSI;
    if (workingset->nVar > 2147483646) {
      c_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&c_st);
    }
    for (idx_col = 0; idx_col < nVar_tmp; idx_col++) {
      i = qrmanager->jpvt.size[0];
      if ((idx_col + 1 < 1) || (idx_col + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(idx_col + 1, 1, i, &bc_emlrtBCI, &st);
      }
      qrmanager->jpvt.data[idx_col] = 0;
    }
    b_st.site = &oe_emlrtRSI;
    i = mTotalWorkingEq_tmp_tmp * workingset->nVar;
    if (i == 0) {
      qrmanager->mrows = mTotalWorkingEq_tmp_tmp;
      qrmanager->ncols = workingset->nVar;
      qrmanager->minRowCol = 0;
    } else {
      qrmanager->usedPivoting = true;
      qrmanager->mrows = mTotalWorkingEq_tmp_tmp;
      qrmanager->ncols = workingset->nVar;
      qrmanager->minRowCol =
          muIntScalarMin_sint32(mTotalWorkingEq_tmp_tmp, workingset->nVar);
      c_st.site = &id_emlrtRSI;
      xgeqp3(&c_st, qrmanager->QR.data, qrmanager->QR.size,
             mTotalWorkingEq_tmp_tmp, workingset->nVar, qrmanager->jpvt.data,
             &qrmanager->jpvt.size[0], qrmanager->tau.data,
             &qrmanager->tau.size[0]);
    }
    tol = 100.0 * (real_T)workingset->nVar * 2.2204460492503131E-16;
    TYPE = muIntScalarMin_sint32(workingset->nVar, mTotalWorkingEq_tmp_tmp);
    idxDiag = TYPE + qrmanager->ldq * (TYPE - 1);
    exitg1 = false;
    while ((!exitg1) && (idxDiag > 0)) {
      i1 = qrmanager->QR.size[0] * qrmanager->QR.size[1];
      if (idxDiag > i1) {
        emlrtDynamicBoundsCheckR2012b(idxDiag, 1, i1, &bc_emlrtBCI, &st);
      }
      if (muDoubleScalarAbs(qrmanager->QR.data[idxDiag - 1]) < tol) {
        idxDiag = (idxDiag - qrmanager->ldq) - 1;
        nDepInd++;
      } else {
        exitg1 = true;
      }
    }
    if (nDepInd > 0) {
      b_st.site = &oe_emlrtRSI;
      c_st.site = &yd_emlrtRSI;
      computeQ_(&c_st, qrmanager, qrmanager->mrows);
      b_st.site = &oe_emlrtRSI;
      if (nDepInd > 2147483646) {
        c_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      idx_col = 0;
      exitg1 = false;
      while ((!exitg1) && (idx_col <= nDepInd - 1)) {
        real_T x;
        n_t = (ptrdiff_t)mTotalWorkingEq_tmp_tmp;
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        x = ddot(&n_t,
                 &qrmanager->Q.data[qrmanager->ldq *
                                    ((mTotalWorkingEq_tmp_tmp - idx_col) - 1)],
                 &incx_t, &workingset->bwset.data[0], &incy_t);
        if (muDoubleScalarAbs(x) >= tol) {
          nDepInd = -1;
          exitg1 = true;
        } else {
          idx_col++;
        }
      }
    }
    if (nDepInd > 0) {
      st.site = &ne_emlrtRSI;
      n_t = (ptrdiff_t)nVar_tmp;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      for (idx_col = 0; idx_col < mTotalWorkingEq_tmp_tmp; idx_col++) {
        st.site = &ne_emlrtRSI;
        b_st.site = &mc_emlrtRSI;
        dcopy(&n_t, &workingset->ATwset.data[workingset->ldA * idx_col],
              &incx_t, &qrmanager->QR.data[qrmanager->ldq * idx_col], &incy_t);
      }
      st.site = &ne_emlrtRSI;
      b_st.site = &pe_emlrtRSI;
      if (workingset->nWConstr[0] > 2147483646) {
        c_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (idx_col = 0; idx_col < mWorkingFixed; idx_col++) {
        i1 = qrmanager->jpvt.size[0];
        if ((idx_col + 1 < 1) || (idx_col + 1 > i1)) {
          emlrtDynamicBoundsCheckR2012b(idx_col + 1, 1, i1, &gc_emlrtBCI, &st);
        }
        qrmanager->jpvt.data[idx_col] = 1;
      }
      idxDiag = workingset->nWConstr[0] + 1;
      b_st.site = &pe_emlrtRSI;
      for (idx_col = idxDiag; idx_col <= mTotalWorkingEq_tmp_tmp; idx_col++) {
        i1 = qrmanager->jpvt.size[0];
        if ((idx_col < 1) || (idx_col > i1)) {
          emlrtDynamicBoundsCheckR2012b(idx_col, 1, i1, &gc_emlrtBCI, &st);
        }
        qrmanager->jpvt.data[idx_col - 1] = 0;
      }
      b_st.site = &pe_emlrtRSI;
      if (i == 0) {
        qrmanager->mrows = workingset->nVar;
        qrmanager->ncols = mTotalWorkingEq_tmp_tmp;
        qrmanager->minRowCol = 0;
      } else {
        qrmanager->usedPivoting = true;
        qrmanager->mrows = workingset->nVar;
        qrmanager->ncols = mTotalWorkingEq_tmp_tmp;
        qrmanager->minRowCol = TYPE;
        c_st.site = &id_emlrtRSI;
        xgeqp3(&c_st, qrmanager->QR.data, qrmanager->QR.size, workingset->nVar,
               mTotalWorkingEq_tmp_tmp, qrmanager->jpvt.data,
               &qrmanager->jpvt.size[0], qrmanager->tau.data,
               &qrmanager->tau.size[0]);
      }
      b_st.site = &pe_emlrtRSI;
      i = qrmanager->jpvt.size[0];
      for (idx_col = 0; idx_col < nDepInd; idx_col++) {
        i1 = ((mTotalWorkingEq_tmp_tmp - nDepInd) + idx_col) + 1;
        if ((i1 < 1) || (i1 > i)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, i, &gc_emlrtBCI, &st);
        }
        i2 = memspace->workspace_int.size[0];
        if (idx_col + 1 > i2) {
          emlrtDynamicBoundsCheckR2012b(idx_col + 1, 1, i2, &gc_emlrtBCI, &st);
        }
        memspace->workspace_int.data[idx_col] = qrmanager->jpvt.data[i1 - 1];
      }
      st.site = &ne_emlrtRSI;
      countsort(&st, memspace->workspace_int.data,
                &memspace->workspace_int.size[0], nDepInd,
                memspace->workspace_sort.data,
                &memspace->workspace_sort.size[0], 1, mTotalWorkingEq_tmp_tmp);
      i = memspace->workspace_int.size[0];
      for (idx_col = nDepInd; idx_col >= 1; idx_col--) {
        st.site = &ne_emlrtRSI;
        if (idx_col > i) {
          emlrtDynamicBoundsCheckR2012b(idx_col, 1, i, &cc_emlrtBCI, &st);
        }
        i1 = workingset->nWConstr[0] + workingset->nWConstr[1];
        if (i1 != 0) {
          i2 = memspace->workspace_int.data[idx_col - 1];
          if (i2 <= i1) {
            if ((workingset->nActiveConstr == i1) || (i2 == i1)) {
              workingset->mEqRemoved++;
              i1 = workingset->Wlocalidx.size[0];
              if ((i2 < 1) || (i2 > i1)) {
                emlrtDynamicBoundsCheckR2012b(
                    memspace->workspace_int.data[idx_col - 1], 1, i1,
                    &hc_emlrtBCI, &st);
              }
              if ((workingset->mEqRemoved < 1) ||
                  (workingset->mEqRemoved > 60)) {
                emlrtDynamicBoundsCheckR2012b(workingset->mEqRemoved, 1, 60,
                                              &ic_emlrtBCI, &st);
              }
              i1 = memspace->workspace_int.data[idx_col - 1] - 1;
              workingset->indexEqRemoved[workingset->mEqRemoved - 1] =
                  workingset->Wlocalidx.data[i1];
              b_st.site = &re_emlrtRSI;
              i3 = workingset->Wid.size[0];
              if (i2 > i3) {
                emlrtDynamicBoundsCheckR2012b(
                    memspace->workspace_int.data[idx_col - 1], 1, i3,
                    &ec_emlrtBCI, &b_st);
              }
              idxDiag = workingset->Wid.data[i1];
              i3 = workingset->Wlocalidx.size[0];
              if (i2 > i3) {
                emlrtDynamicBoundsCheckR2012b(
                    memspace->workspace_int.data[idx_col - 1], 1, i3,
                    &ec_emlrtBCI, &b_st);
              }
              if ((idxDiag < 1) || (idxDiag > 6)) {
                emlrtDynamicBoundsCheckR2012b(idxDiag, 1, 6, &fc_emlrtBCI,
                                              &b_st);
              }
              i2 = workingset->isActiveConstr.size[0];
              i1 = (workingset->isActiveIdx[workingset->Wid.data[i1] - 1] +
                    workingset->Wlocalidx.data[i1]) -
                   1;
              if ((i1 < 1) || (i1 > i2)) {
                emlrtDynamicBoundsCheckR2012b(i1, 1, i2, &ec_emlrtBCI, &b_st);
              }
              workingset->isActiveConstr.data[i1 - 1] = false;
              c_st.site = &se_emlrtRSI;
              moveConstraint_(&c_st, workingset, workingset->nActiveConstr,
                              memspace->workspace_int.data[idx_col - 1]);
              workingset->nActiveConstr--;
              if (idxDiag > 5) {
                emlrtDynamicBoundsCheckR2012b(6, 1, 5, &kc_emlrtBCI, &b_st);
              }
              workingset->nWConstr[idxDiag - 1]--;
            } else {
              workingset->mEqRemoved++;
              i3 = workingset->Wid.size[0];
              if ((i2 < 1) || (i2 > i3)) {
                emlrtDynamicBoundsCheckR2012b(
                    memspace->workspace_int.data[idx_col - 1], 1, i3,
                    &hc_emlrtBCI, &st);
              }
              TYPE = workingset->Wid.data[i2 - 1];
              i3 = workingset->Wlocalidx.size[0];
              if (i2 > i3) {
                emlrtDynamicBoundsCheckR2012b(
                    memspace->workspace_int.data[idx_col - 1], 1, i3,
                    &hc_emlrtBCI, &st);
              }
              idxDiag = workingset->Wlocalidx.data[i2 - 1];
              if ((workingset->mEqRemoved < 1) ||
                  (workingset->mEqRemoved > 60)) {
                emlrtDynamicBoundsCheckR2012b(workingset->mEqRemoved, 1, 60,
                                              &ic_emlrtBCI, &st);
              }
              workingset->indexEqRemoved[workingset->mEqRemoved - 1] = idxDiag;
              if ((TYPE < 1) || (TYPE > 6)) {
                emlrtDynamicBoundsCheckR2012b(TYPE, 1, 6, &dc_emlrtBCI, &st);
              }
              i2 = workingset->isActiveConstr.size[0];
              i3 = (workingset->isActiveIdx[TYPE - 1] + idxDiag) - 1;
              if ((i3 < 1) || (i3 > i2)) {
                emlrtDynamicBoundsCheckR2012b(i3, 1, i2, &hc_emlrtBCI, &st);
              }
              workingset->isActiveConstr.data[i3 - 1] = false;
              b_st.site = &re_emlrtRSI;
              moveConstraint_(&b_st, workingset, i1,
                              memspace->workspace_int.data[idx_col - 1]);
              b_st.site = &re_emlrtRSI;
              moveConstraint_(&b_st, workingset, workingset->nActiveConstr, i1);
              workingset->nActiveConstr--;
              if (TYPE > 5) {
                emlrtDynamicBoundsCheckR2012b(6, 1, 5, &jc_emlrtBCI, &st);
              }
              workingset->nWConstr[TYPE - 1]--;
            }
          }
        }
      }
    }
  }
  return nDepInd;
}

/* End of code generation (RemoveDependentEq_.c) */
