/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * assignResidualsToXSlack.c
 *
 * Code generation for function 'assignResidualsToXSlack'
 *
 */

/* Include files */
#include "assignResidualsToXSlack.h"
#include "addBoundToActiveSetMatrix_.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo oh_emlrtRSI = {
    1,                         /* lineNo */
    "assignResidualsToXSlack", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\+relaxed\\assignResidualsToXSlack.p" /* pathName */
};

static emlrtBCInfo td_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    1,                         /* lineNo */
    1,                         /* colNo */
    "",                        /* aName */
    "assignResidualsToXSlack", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\+relaxed\\assignResidualsToXSlack.p", /* pName */
    0                                            /* checkKind */
};

/* Function Definitions */
void assignResidualsToXSlack(const emlrtStack *sp, int32_T nVarOrig,
                             h_struct_T *WorkingSet, e_struct_T *TrialState,
                             g_struct_T *memspace)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack st;
  int32_T i;
  int32_T i1;
  int32_T idx;
  int32_T mIneq;
  int32_T mLBOrig;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  mIneq = WorkingSet->sizes[2] - 1;
  mLBOrig = (WorkingSet->sizes[3] - WorkingSet->sizes[2]) - 384;
  st.site = &oh_emlrtRSI;
  if (WorkingSet->sizes[2] >= 1) {
    n_t = (ptrdiff_t)WorkingSet->sizes[2];
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dcopy(&n_t, &WorkingSet->bineq.data[0], &incx_t,
          &memspace->workspace_double.data[0], &incy_t);
  }
  st.site = &oh_emlrtRSI;
  d_xgemv(nVarOrig, WorkingSet->sizes[2], WorkingSet->Aineq.data,
          WorkingSet->ldA, TrialState->xstar.data,
          memspace->workspace_double.data);
  st.site = &oh_emlrtRSI;
  if (WorkingSet->sizes[2] > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = 0; idx <= mIneq; idx++) {
    i = TrialState->xstar.size[0];
    i1 = (nVarOrig + idx) + 1;
    if ((i1 < 1) || (i1 > i)) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i, &td_emlrtBCI, (emlrtCTX)sp);
    }
    TrialState->xstar.data[i1 - 1] =
        (real_T)(memspace->workspace_double.data[idx] > 0.0) *
        memspace->workspace_double.data[idx];
  }
  st.site = &oh_emlrtRSI;
  memcpy(&memspace->workspace_double.data[0], &WorkingSet->beq[0],
         192U * sizeof(real_T));
  st.site = &oh_emlrtRSI;
  d_xgemv(nVarOrig, 192, WorkingSet->Aeq.data, WorkingSet->ldA,
          TrialState->xstar.data, memspace->workspace_double.data);
  st.site = &oh_emlrtRSI;
  for (idx = 0; idx < 192; idx++) {
    int32_T idx_positive_tmp;
    idx_positive_tmp = mIneq + idx;
    if (memspace->workspace_double.data[idx] <= 0.0) {
      i = TrialState->xstar.size[0];
      i1 = (nVarOrig + idx_positive_tmp) + 2;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &td_emlrtBCI, (emlrtCTX)sp);
      }
      TrialState->xstar.data[i1 - 1] = 0.0;
      i = TrialState->xstar.size[0];
      i1 = (nVarOrig + idx_positive_tmp) + 194;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &td_emlrtBCI, (emlrtCTX)sp);
      }
      TrialState->xstar.data[i1 - 1] = -memspace->workspace_double.data[idx];
      st.site = &oh_emlrtRSI;
      b_st.site = &dh_emlrtRSI;
      addBoundToActiveSetMatrix_(&b_st, WorkingSet, 4,
                                 (mLBOrig + idx_positive_tmp) + 2);
      if (memspace->workspace_double.data[idx] >= -1.0E-6) {
        st.site = &oh_emlrtRSI;
        b_st.site = &dh_emlrtRSI;
        addBoundToActiveSetMatrix_(&b_st, WorkingSet, 4,
                                   (mLBOrig + idx_positive_tmp) + 194);
      }
    } else {
      i = TrialState->xstar.size[0];
      i1 = nVarOrig + idx_positive_tmp;
      if ((i1 + 2 < 1) || (i1 + 2 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1 + 2, 1, i, &td_emlrtBCI, (emlrtCTX)sp);
      }
      TrialState->xstar.data[i1 + 1] = memspace->workspace_double.data[idx];
      i = TrialState->xstar.size[0];
      if ((i1 + 194 < 1) || (i1 + 194 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1 + 194, 1, i, &td_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      TrialState->xstar.data[i1 + 193] = 0.0;
      st.site = &oh_emlrtRSI;
      b_st.site = &dh_emlrtRSI;
      addBoundToActiveSetMatrix_(&b_st, WorkingSet, 4,
                                 (mLBOrig + idx_positive_tmp) + 194);
      if (memspace->workspace_double.data[idx] <= 1.0E-6) {
        st.site = &oh_emlrtRSI;
        b_st.site = &dh_emlrtRSI;
        addBoundToActiveSetMatrix_(&b_st, WorkingSet, 4,
                                   (mLBOrig + idx_positive_tmp) + 2);
      }
    }
  }
}

/* End of code generation (assignResidualsToXSlack.c) */
