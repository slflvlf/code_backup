/*
 * updateWorkingSet.c
 *
 * Code generation for function 'updateWorkingSet'
 *
 */

/* Include files */
#include "updateWorkingSet.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo sh_emlrtRSI = {
    1,                  /* lineNo */
    "updateWorkingSet", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\+soc\\updateWorkingSet.p" /* pathName */
};

static emlrtBCInfo ce_emlrtBCI = {
    -1,                 /* iFirst */
    -1,                 /* iLast */
    1,                  /* lineNo */
    1,                  /* colNo */
    "",                 /* aName */
    "updateWorkingSet", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\+soc\\updateWorkingSet.p", /* pName */
    0                                 /* checkKind */
};

/* Function Definitions */
void updateWorkingSet(const emlrtStack *sp, h_struct_T *WorkingSet,
                      const real_T TrialState_cIneq_data[],
                      int32_T TrialState_cIneq_size,
                      const real_T TrialState_cEq[60],
                      const real_T TrialState_searchDir_data[],
                      int32_T workspace_int_data[],
                      const int32_T *workspace_int_size)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack st;
  real_T y_data[1231];
  real_T alpha1;
  real_T beta1;
  int32_T idx;
  int32_T idxIneqOffset;
  int32_T idx_lower;
  int32_T mIneq;
  char_T TRANSA;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  mIneq = WorkingSet->sizes[2];
  idxIneqOffset = WorkingSet->isActiveIdx[2];
  st.site = &sh_emlrtRSI;
  for (idx = 0; idx < 60; idx++) {
    WorkingSet->beq[idx] = -TrialState_cEq[idx];
  }
  st.site = &sh_emlrtRSI;
  alpha1 = 1.0;
  beta1 = 1.0;
  TRANSA = 'T';
  m_t = (ptrdiff_t)WorkingSet->nVar;
  n_t = (ptrdiff_t)60;
  lda_t = (ptrdiff_t)WorkingSet->ldA;
  incx_t = (ptrdiff_t)1;
  incy_t = (ptrdiff_t)1;
  dgemv(&TRANSA, &m_t, &n_t, &alpha1, &WorkingSet->Aeq.data[0], &lda_t,
        &TrialState_searchDir_data[0], &incx_t, &beta1, &WorkingSet->beq[0],
        &incy_t);
  st.site = &sh_emlrtRSI;
  for (idx_lower = 0; idx_lower < 60; idx_lower++) {
    WorkingSet->bwset.data[WorkingSet->sizes[0] + idx_lower] =
        WorkingSet->beq[idx_lower];
  }
  if (WorkingSet->sizes[2] > 0) {
    int32_T b;
    int32_T i;
    int32_T idx_upper;
    int32_T y_size;
    st.site = &sh_emlrtRSI;
    if (WorkingSet->sizes[2] > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx = 0; idx < mIneq; idx++) {
      if ((idx + 1 < 1) || (idx + 1 > TrialState_cIneq_size)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, TrialState_cIneq_size,
                                      &ce_emlrtBCI, (emlrtCTX)sp);
      }
      i = WorkingSet->bineq.size[0];
      if (idx + 1 > i) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &ce_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      WorkingSet->bineq.data[idx] = -TrialState_cIneq_data[idx];
    }
    st.site = &sh_emlrtRSI;
    y_size = WorkingSet->bineq.size[0];
    mIneq = WorkingSet->bineq.size[0];
    if (mIneq - 1 >= 0) {
      memcpy(&y_data[0], &WorkingSet->bineq.data[0], mIneq * sizeof(real_T));
    }
    alpha1 = 1.0;
    beta1 = 1.0;
    TRANSA = 'T';
    m_t = (ptrdiff_t)WorkingSet->nVar;
    n_t = (ptrdiff_t)WorkingSet->sizes[2];
    lda_t = (ptrdiff_t)WorkingSet->ldA;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dgemv(&TRANSA, &m_t, &n_t, &alpha1, &WorkingSet->Aineq.data[0], &lda_t,
          &TrialState_searchDir_data[0], &incx_t, &beta1, &y_data[0], &incy_t);
    WorkingSet->bineq.size[0] = y_size;
    if (y_size - 1 >= 0) {
      memcpy(&WorkingSet->bineq.data[0], &y_data[0], y_size * sizeof(real_T));
    }
    mIneq = 1;
    idx_lower = WorkingSet->sizes[2] + 1;
    idx_upper = (WorkingSet->sizes[2] + WorkingSet->sizes[3]) + 1;
    b = WorkingSet->nActiveConstr;
    st.site = &sh_emlrtRSI;
    if ((WorkingSet->isActiveIdx[2] <= WorkingSet->nActiveConstr) &&
        (WorkingSet->nActiveConstr > 2147483646)) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx = idxIneqOffset; idx <= b; idx++) {
      int32_T idx_IneqLocal_tmp;
      int32_T idx_Partition;
      i = WorkingSet->Wid.size[0];
      if ((idx < 1) || (idx > i)) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ce_emlrtBCI, (emlrtCTX)sp);
      }
      i = WorkingSet->Wlocalidx.size[0];
      if (idx > i) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ce_emlrtBCI, (emlrtCTX)sp);
      }
      idx_IneqLocal_tmp = WorkingSet->Wlocalidx.data[idx - 1];
      switch (WorkingSet->Wid.data[idx - 1]) {
      case 3:
        idx_Partition = mIneq;
        mIneq++;
        if ((idx_IneqLocal_tmp < 1) || (idx_IneqLocal_tmp > y_size)) {
          emlrtDynamicBoundsCheckR2012b(WorkingSet->Wlocalidx.data[idx - 1], 1,
                                        y_size, &ce_emlrtBCI, (emlrtCTX)sp);
        }
        i = WorkingSet->bwset.size[0];
        if (idx > i) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ce_emlrtBCI, (emlrtCTX)sp);
        }
        WorkingSet->bwset.data[idx - 1] = y_data[idx_IneqLocal_tmp - 1];
        break;
      case 4:
        idx_Partition = idx_lower;
        idx_lower++;
        break;
      default:
        idx_Partition = idx_upper;
        idx_upper++;
        break;
      }
      i = *workspace_int_size;
      if ((idx_Partition < 1) || (idx_Partition > i)) {
        emlrtDynamicBoundsCheckR2012b(idx_Partition, 1, i, &ce_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      workspace_int_data[idx_Partition - 1] = idx_IneqLocal_tmp;
    }
  }
}

/* End of code generation (updateWorkingSet.c) */
