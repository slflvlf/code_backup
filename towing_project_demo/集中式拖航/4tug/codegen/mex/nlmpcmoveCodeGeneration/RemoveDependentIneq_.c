/*
 * RemoveDependentIneq_.c
 *
 * Code generation for function 'RemoveDependentIneq_'
 *
 */

/* Include files */
#include "RemoveDependentIneq_.h"
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
static emlrtRSInfo ue_emlrtRSI = {
    1,                      /* lineNo */
    "RemoveDependentIneq_", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "initialize\\RemoveDependentIneq_.p" /* pathName */
};

static emlrtBCInfo nc_emlrtBCI = {
    -1,                     /* iFirst */
    -1,                     /* iLast */
    1,                      /* lineNo */
    1,                      /* colNo */
    "",                     /* aName */
    "RemoveDependentIneq_", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "initialize\\RemoveDependentIneq_.p", /* pName */
    0                                     /* checkKind */
};

/* Function Definitions */
void RemoveDependentIneq_(const emlrtStack *sp, h_struct_T *workingset,
                          i_struct_T *qrmanager, g_struct_T *memspace)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack st;
  int32_T idx;
  int32_T idxDiag;
  int32_T nActiveConstr_tmp;
  int32_T nFixedConstr;
  int32_T nVar;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  nActiveConstr_tmp = workingset->nActiveConstr;
  nFixedConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  nVar = workingset->nVar;
  if ((workingset->nWConstr[2] + workingset->nWConstr[3]) +
          workingset->nWConstr[4] >
      0) {
    real_T tol;
    int32_T i;
    int32_T nDepIneq;
    tol = 100.0 * (real_T)workingset->nVar * 2.2204460492503131E-16;
    st.site = &ue_emlrtRSI;
    if (nFixedConstr > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx = 0; idx < nFixedConstr; idx++) {
      i = qrmanager->jpvt.size[0];
      if ((idx + 1 < 1) || (idx + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &nc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      qrmanager->jpvt.data[idx] = 1;
    }
    idxDiag = nFixedConstr + 1;
    st.site = &ue_emlrtRSI;
    if ((nFixedConstr + 1 <= workingset->nActiveConstr) &&
        (workingset->nActiveConstr > 2147483646)) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx = idxDiag; idx <= nActiveConstr_tmp; idx++) {
      i = qrmanager->jpvt.size[0];
      if ((idx < 1) || (idx > i)) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &nc_emlrtBCI, (emlrtCTX)sp);
      }
      qrmanager->jpvt.data[idx - 1] = 0;
    }
    st.site = &ue_emlrtRSI;
    if (workingset->nActiveConstr > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idxDiag = 0; idxDiag < nActiveConstr_tmp; idxDiag++) {
      st.site = &ue_emlrtRSI;
      n_t = (ptrdiff_t)nVar;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      dcopy(&n_t, &workingset->ATwset.data[workingset->ldA * idxDiag], &incx_t,
            &qrmanager->QR.data[qrmanager->ldq * idxDiag], &incy_t);
    }
    st.site = &ue_emlrtRSI;
    if (workingset->nVar * workingset->nActiveConstr == 0) {
      qrmanager->mrows = workingset->nVar;
      qrmanager->ncols = workingset->nActiveConstr;
      qrmanager->minRowCol = 0;
    } else {
      qrmanager->usedPivoting = true;
      qrmanager->mrows = workingset->nVar;
      qrmanager->ncols = workingset->nActiveConstr;
      qrmanager->minRowCol =
          muIntScalarMin_sint32(workingset->nVar, workingset->nActiveConstr);
      b_st.site = &id_emlrtRSI;
      xgeqp3(&b_st, qrmanager->QR.data, qrmanager->QR.size, workingset->nVar,
             workingset->nActiveConstr, qrmanager->jpvt.data,
             &qrmanager->jpvt.size[0], qrmanager->tau.data,
             &qrmanager->tau.size[0]);
    }
    nDepIneq = 0;
    for (idx = workingset->nActiveConstr; idx > nVar; idx--) {
      nDepIneq++;
      i = qrmanager->jpvt.size[0];
      if ((idx < 1) || (idx > i)) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &nc_emlrtBCI, (emlrtCTX)sp);
      }
      i = memspace->workspace_int.size[0];
      if (nDepIneq > i) {
        emlrtDynamicBoundsCheckR2012b(nDepIneq, 1, i, &nc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      memspace->workspace_int.data[nDepIneq - 1] =
          qrmanager->jpvt.data[idx - 1];
    }
    if (idx <= workingset->nVar) {
      boolean_T exitg1;
      idxDiag = idx + qrmanager->ldq * (idx - 1);
      exitg1 = false;
      while ((!exitg1) && (idx > nFixedConstr)) {
        i = qrmanager->QR.size[0] * qrmanager->QR.size[1];
        if ((idxDiag < 1) || (idxDiag > i)) {
          emlrtDynamicBoundsCheckR2012b(idxDiag, 1, i, &nc_emlrtBCI,
                                        (emlrtCTX)sp);
        }
        if (muDoubleScalarAbs(qrmanager->QR.data[idxDiag - 1]) < tol) {
          nDepIneq++;
          i = qrmanager->jpvt.size[0];
          if ((idx < 1) || (idx > i)) {
            emlrtDynamicBoundsCheckR2012b(idx, 1, i, &nc_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          i = memspace->workspace_int.size[0];
          if (nDepIneq > i) {
            emlrtDynamicBoundsCheckR2012b(nDepIneq, 1, i, &nc_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          memspace->workspace_int.data[nDepIneq - 1] =
              qrmanager->jpvt.data[idx - 1];
          idx--;
          idxDiag = (idxDiag - qrmanager->ldq) - 1;
        } else {
          exitg1 = true;
        }
      }
    }
    st.site = &ue_emlrtRSI;
    countsort(&st, memspace->workspace_int.data,
              &memspace->workspace_int.size[0], nDepIneq,
              memspace->workspace_sort.data, &memspace->workspace_sort.size[0],
              nFixedConstr + 1, workingset->nActiveConstr);
    for (idx = nDepIneq; idx >= 1; idx--) {
      st.site = &ue_emlrtRSI;
      i = memspace->workspace_int.size[0];
      if (idx > i) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &nc_emlrtBCI, &st);
      }
      i = workingset->Wid.size[0];
      idxDiag = memspace->workspace_int.data[idx - 1];
      if ((idxDiag < 1) || (idxDiag > i)) {
        emlrtDynamicBoundsCheckR2012b(memspace->workspace_int.data[idx - 1], 1,
                                      i, &ec_emlrtBCI, &st);
      }
      nActiveConstr_tmp = workingset->Wid.data[idxDiag - 1];
      i = workingset->Wlocalidx.size[0];
      if (idxDiag > i) {
        emlrtDynamicBoundsCheckR2012b(memspace->workspace_int.data[idx - 1], 1,
                                      i, &ec_emlrtBCI, &st);
      }
      if ((nActiveConstr_tmp < 1) || (nActiveConstr_tmp > 6)) {
        emlrtDynamicBoundsCheckR2012b(nActiveConstr_tmp, 1, 6, &fc_emlrtBCI,
                                      &st);
      }
      i = workingset->isActiveConstr.size[0];
      idxDiag = (workingset->isActiveIdx[nActiveConstr_tmp - 1] +
                 workingset->Wlocalidx.data[idxDiag - 1]) -
                1;
      if ((idxDiag < 1) || (idxDiag > i)) {
        emlrtDynamicBoundsCheckR2012b(idxDiag, 1, i, &ec_emlrtBCI, &st);
      }
      workingset->isActiveConstr.data[idxDiag - 1] = false;
      b_st.site = &se_emlrtRSI;
      moveConstraint_(&b_st, workingset, workingset->nActiveConstr,
                      memspace->workspace_int.data[idx - 1]);
      workingset->nActiveConstr--;
      if (nActiveConstr_tmp > 5) {
        emlrtDynamicBoundsCheckR2012b(6, 1, 5, &kc_emlrtBCI, &st);
      }
      workingset->nWConstr[nActiveConstr_tmp - 1]--;
    }
  }
}

void b_RemoveDependentIneq_(const emlrtStack *sp, h_struct_T *workingset,
                            i_struct_T *qrmanager, g_struct_T *memspace)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack st;
  int32_T idx;
  int32_T idxDiag;
  int32_T nActiveConstr_tmp;
  int32_T nFixedConstr;
  int32_T nVar;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  nActiveConstr_tmp = workingset->nActiveConstr;
  nFixedConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  nVar = workingset->nVar;
  if ((workingset->nWConstr[2] + workingset->nWConstr[3]) +
          workingset->nWConstr[4] >
      0) {
    real_T tol;
    int32_T i;
    int32_T nDepIneq;
    tol = 1000.0 * (real_T)workingset->nVar * 2.2204460492503131E-16;
    st.site = &ue_emlrtRSI;
    if (nFixedConstr > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx = 0; idx < nFixedConstr; idx++) {
      i = qrmanager->jpvt.size[0];
      if ((idx + 1 < 1) || (idx + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &nc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      qrmanager->jpvt.data[idx] = 1;
    }
    idxDiag = nFixedConstr + 1;
    st.site = &ue_emlrtRSI;
    if ((nFixedConstr + 1 <= workingset->nActiveConstr) &&
        (workingset->nActiveConstr > 2147483646)) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx = idxDiag; idx <= nActiveConstr_tmp; idx++) {
      i = qrmanager->jpvt.size[0];
      if ((idx < 1) || (idx > i)) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &nc_emlrtBCI, (emlrtCTX)sp);
      }
      qrmanager->jpvt.data[idx - 1] = 0;
    }
    st.site = &ue_emlrtRSI;
    if (workingset->nActiveConstr > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idxDiag = 0; idxDiag < nActiveConstr_tmp; idxDiag++) {
      st.site = &ue_emlrtRSI;
      n_t = (ptrdiff_t)nVar;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      dcopy(&n_t, &workingset->ATwset.data[workingset->ldA * idxDiag], &incx_t,
            &qrmanager->QR.data[qrmanager->ldq * idxDiag], &incy_t);
    }
    st.site = &ue_emlrtRSI;
    if (workingset->nVar * workingset->nActiveConstr == 0) {
      qrmanager->mrows = workingset->nVar;
      qrmanager->ncols = workingset->nActiveConstr;
      qrmanager->minRowCol = 0;
    } else {
      qrmanager->usedPivoting = true;
      qrmanager->mrows = workingset->nVar;
      qrmanager->ncols = workingset->nActiveConstr;
      qrmanager->minRowCol =
          muIntScalarMin_sint32(workingset->nVar, workingset->nActiveConstr);
      b_st.site = &id_emlrtRSI;
      xgeqp3(&b_st, qrmanager->QR.data, qrmanager->QR.size, workingset->nVar,
             workingset->nActiveConstr, qrmanager->jpvt.data,
             &qrmanager->jpvt.size[0], qrmanager->tau.data,
             &qrmanager->tau.size[0]);
    }
    nDepIneq = 0;
    for (idx = workingset->nActiveConstr; idx > nVar; idx--) {
      nDepIneq++;
      i = qrmanager->jpvt.size[0];
      if ((idx < 1) || (idx > i)) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &nc_emlrtBCI, (emlrtCTX)sp);
      }
      i = memspace->workspace_int.size[0];
      if (nDepIneq > i) {
        emlrtDynamicBoundsCheckR2012b(nDepIneq, 1, i, &nc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      memspace->workspace_int.data[nDepIneq - 1] =
          qrmanager->jpvt.data[idx - 1];
    }
    if (idx <= workingset->nVar) {
      boolean_T exitg1;
      idxDiag = idx + qrmanager->ldq * (idx - 1);
      exitg1 = false;
      while ((!exitg1) && (idx > nFixedConstr)) {
        i = qrmanager->QR.size[0] * qrmanager->QR.size[1];
        if ((idxDiag < 1) || (idxDiag > i)) {
          emlrtDynamicBoundsCheckR2012b(idxDiag, 1, i, &nc_emlrtBCI,
                                        (emlrtCTX)sp);
        }
        if (muDoubleScalarAbs(qrmanager->QR.data[idxDiag - 1]) < tol) {
          nDepIneq++;
          i = qrmanager->jpvt.size[0];
          if ((idx < 1) || (idx > i)) {
            emlrtDynamicBoundsCheckR2012b(idx, 1, i, &nc_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          i = memspace->workspace_int.size[0];
          if (nDepIneq > i) {
            emlrtDynamicBoundsCheckR2012b(nDepIneq, 1, i, &nc_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          memspace->workspace_int.data[nDepIneq - 1] =
              qrmanager->jpvt.data[idx - 1];
          idx--;
          idxDiag = (idxDiag - qrmanager->ldq) - 1;
        } else {
          exitg1 = true;
        }
      }
    }
    st.site = &ue_emlrtRSI;
    countsort(&st, memspace->workspace_int.data,
              &memspace->workspace_int.size[0], nDepIneq,
              memspace->workspace_sort.data, &memspace->workspace_sort.size[0],
              nFixedConstr + 1, workingset->nActiveConstr);
    for (idx = nDepIneq; idx >= 1; idx--) {
      st.site = &ue_emlrtRSI;
      i = memspace->workspace_int.size[0];
      if (idx > i) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &nc_emlrtBCI, &st);
      }
      i = workingset->Wid.size[0];
      idxDiag = memspace->workspace_int.data[idx - 1];
      if ((idxDiag < 1) || (idxDiag > i)) {
        emlrtDynamicBoundsCheckR2012b(memspace->workspace_int.data[idx - 1], 1,
                                      i, &ec_emlrtBCI, &st);
      }
      nActiveConstr_tmp = workingset->Wid.data[idxDiag - 1];
      i = workingset->Wlocalidx.size[0];
      if (idxDiag > i) {
        emlrtDynamicBoundsCheckR2012b(memspace->workspace_int.data[idx - 1], 1,
                                      i, &ec_emlrtBCI, &st);
      }
      if ((nActiveConstr_tmp < 1) || (nActiveConstr_tmp > 6)) {
        emlrtDynamicBoundsCheckR2012b(nActiveConstr_tmp, 1, 6, &fc_emlrtBCI,
                                      &st);
      }
      i = workingset->isActiveConstr.size[0];
      idxDiag = (workingset->isActiveIdx[nActiveConstr_tmp - 1] +
                 workingset->Wlocalidx.data[idxDiag - 1]) -
                1;
      if ((idxDiag < 1) || (idxDiag > i)) {
        emlrtDynamicBoundsCheckR2012b(idxDiag, 1, i, &ec_emlrtBCI, &st);
      }
      workingset->isActiveConstr.data[idxDiag - 1] = false;
      b_st.site = &se_emlrtRSI;
      moveConstraint_(&b_st, workingset, workingset->nActiveConstr,
                      memspace->workspace_int.data[idx - 1]);
      workingset->nActiveConstr--;
      if (nActiveConstr_tmp > 5) {
        emlrtDynamicBoundsCheckR2012b(6, 1, 5, &kc_emlrtBCI, &st);
      }
      workingset->nWConstr[nActiveConstr_tmp - 1]--;
    }
  }
}

/* End of code generation (RemoveDependentIneq_.c) */
