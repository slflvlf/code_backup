/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * feasibleratiotest.c
 *
 * Code generation for function 'feasibleratiotest'
 *
 */

/* Include files */
#include "feasibleratiotest.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo bh_emlrtRSI = {
    1,                   /* lineNo */
    "feasibleratiotest", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "qpactiveset\\feasibleratiotest.p" /* pathName */
};

static emlrtBCInfo re_emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    1,                   /* lineNo */
    1,                   /* colNo */
    "",                  /* aName */
    "feasibleratiotest", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "qpactiveset\\feasibleratiotest.p", /* pName */
    0                                   /* checkKind */
};

/* Function Definitions */
void feasibleratiotest(
    const emlrtStack *sp, const real_T solution_xstar_data[],
    int32_T solution_xstar_size, const real_T solution_searchDir_data[],
    int32_T solution_searchDir_size, real_T workspace_data[],
    const int32_T workspace_size[2], int32_T workingset_nVar,
    int32_T workingset_ldA, const real_T workingset_Aineq_data[],
    const real_T workingset_bineq_data[], const real_T workingset_lb_data[],
    int32_T workingset_lb_size, const real_T workingset_ub_data[],
    const int32_T workingset_indexLB_data[], int32_T workingset_indexLB_size,
    const int32_T workingset_indexUB_data[], int32_T workingset_indexUB_size,
    const int32_T workingset_sizes[5], const int32_T workingset_isActiveIdx[6],
    const boolean_T workingset_isActiveConstr_data[],
    int32_T workingset_isActiveConstr_size,
    const int32_T workingset_nWConstr[5], boolean_T isPhaseOne, real_T *alpha,
    boolean_T *newBlocking, int32_T *constrType, int32_T *constrIdx)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack st;
  real_T alphaTemp;
  real_T denomTol;
  real_T phaseOneCorrectionP;
  real_T phaseOneCorrectionX;
  real_T pk_corrected;
  real_T ratio;
  int32_T i;
  int32_T i1;
  int32_T idx;
  int32_T totalIneq;
  int32_T totalUB;
  char_T TRANSA;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  totalIneq = workingset_sizes[2];
  totalUB = workingset_sizes[4];
  *alpha = 1.0E+30;
  *newBlocking = false;
  *constrType = 0;
  *constrIdx = 0;
  if (workingset_nVar < 1) {
    alphaTemp = 0.0;
  } else {
    n_t = (ptrdiff_t)workingset_nVar;
    incx_t = (ptrdiff_t)1;
    alphaTemp = dnrm2(&n_t, &solution_searchDir_data[0], &incx_t);
  }
  denomTol = 2.2204460492503131E-13 * alphaTemp;
  if (workingset_nWConstr[2] < workingset_sizes[2]) {
    int32_T ldw;
    st.site = &bh_emlrtRSI;
    if (workingset_sizes[2] >= 1) {
      n_t = (ptrdiff_t)workingset_sizes[2];
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      dcopy(&n_t, &workingset_bineq_data[0], &incx_t, &workspace_data[0],
            &incy_t);
    }
    st.site = &bh_emlrtRSI;
    d_xgemv(workingset_nVar, workingset_sizes[2], workingset_Aineq_data,
            workingset_ldA, solution_xstar_data, workspace_data);
    ldw = workspace_size[0];
    st.site = &bh_emlrtRSI;
    if (workingset_sizes[2] >= 1) {
      alphaTemp = 1.0;
      pk_corrected = 0.0;
      TRANSA = 'T';
      m_t = (ptrdiff_t)workingset_nVar;
      n_t = (ptrdiff_t)workingset_sizes[2];
      lda_t = (ptrdiff_t)workingset_ldA;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      dgemv(&TRANSA, &m_t, &n_t, &alphaTemp, &workingset_Aineq_data[0], &lda_t,
            &solution_searchDir_data[0], &incx_t, &pk_corrected,
            &workspace_data[workspace_size[0]], &incy_t);
    }
    st.site = &bh_emlrtRSI;
    if (workingset_sizes[2] > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx = 0; idx < totalIneq; idx++) {
      i = ldw + idx;
      if (workspace_data[i] > denomTol) {
        st.site = &bh_emlrtRSI;
        i1 = workingset_isActiveIdx[2] + idx;
        if ((i1 < 1) || (i1 > workingset_isActiveConstr_size)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, workingset_isActiveConstr_size,
                                        &vc_emlrtBCI, &st);
        }
        if (!workingset_isActiveConstr_data[i1 - 1]) {
          alphaTemp = muDoubleScalarMin(muDoubleScalarAbs(workspace_data[idx]),
                                        1.0E-6 - workspace_data[idx]) /
                      workspace_data[i];
          if (alphaTemp < *alpha) {
            *alpha = alphaTemp;
            *constrType = 3;
            *constrIdx = idx + 1;
            *newBlocking = true;
          }
        }
      }
    }
  }
  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    if ((workingset_nVar < 1) || (workingset_nVar > solution_xstar_size)) {
      emlrtDynamicBoundsCheckR2012b(workingset_nVar, 1, solution_xstar_size,
                                    &re_emlrtBCI, (emlrtCTX)sp);
    }
    phaseOneCorrectionX =
        (real_T)isPhaseOne * solution_xstar_data[workingset_nVar - 1];
    if (workingset_nVar > solution_searchDir_size) {
      emlrtDynamicBoundsCheckR2012b(workingset_nVar, 1, solution_searchDir_size,
                                    &re_emlrtBCI, (emlrtCTX)sp);
    }
    phaseOneCorrectionP =
        (real_T)isPhaseOne * solution_searchDir_data[workingset_nVar - 1];
    i = workingset_sizes[3];
    st.site = &bh_emlrtRSI;
    for (idx = 0; idx <= i - 2; idx++) {
      if ((idx + 1 < 1) || (idx + 1 > workingset_indexLB_size)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, workingset_indexLB_size,
                                      &re_emlrtBCI, (emlrtCTX)sp);
      }
      if ((workingset_indexLB_data[idx] < 1) ||
          (workingset_indexLB_data[idx] > solution_searchDir_size)) {
        emlrtDynamicBoundsCheckR2012b(workingset_indexLB_data[idx], 1,
                                      solution_searchDir_size, &re_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      pk_corrected =
          -solution_searchDir_data[workingset_indexLB_data[idx] - 1] -
          phaseOneCorrectionP;
      if (pk_corrected > denomTol) {
        st.site = &bh_emlrtRSI;
        i1 = workingset_isActiveIdx[3] + idx;
        if ((i1 < 1) || (i1 > workingset_isActiveConstr_size)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, workingset_isActiveConstr_size,
                                        &vc_emlrtBCI, &st);
        }
        if (!workingset_isActiveConstr_data[i1 - 1]) {
          if ((workingset_indexLB_data[idx] < 1) ||
              (workingset_indexLB_data[idx] > solution_xstar_size)) {
            emlrtDynamicBoundsCheckR2012b(workingset_indexLB_data[idx], 1,
                                          solution_xstar_size, &re_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          if ((workingset_indexLB_data[idx] < 1) ||
              (workingset_indexLB_data[idx] > workingset_lb_size)) {
            emlrtDynamicBoundsCheckR2012b(workingset_indexLB_data[idx], 1,
                                          workingset_lb_size, &re_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          ratio = (-solution_xstar_data[workingset_indexLB_data[idx] - 1] -
                   workingset_lb_data[workingset_indexLB_data[idx] - 1]) -
                  phaseOneCorrectionX;
          alphaTemp =
              muDoubleScalarMin(muDoubleScalarAbs(ratio), 1.0E-6 - ratio) /
              pk_corrected;
          if (alphaTemp < *alpha) {
            *alpha = alphaTemp;
            *constrType = 4;
            *constrIdx = idx + 1;
            *newBlocking = true;
          }
        }
      }
    }
    if ((workingset_sizes[3] < 1) ||
        (workingset_sizes[3] > workingset_indexLB_size)) {
      emlrtDynamicBoundsCheckR2012b(workingset_sizes[3], 1,
                                    workingset_indexLB_size, &re_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    i = workingset_indexLB_data[workingset_sizes[3] - 1];
    if ((i < 1) || (i > solution_searchDir_size)) {
      emlrtDynamicBoundsCheckR2012b(i, 1, solution_searchDir_size, &re_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    alphaTemp = -solution_searchDir_data[i - 1];
    if (alphaTemp > denomTol) {
      st.site = &bh_emlrtRSI;
      i1 = workingset_isActiveIdx[3] + workingset_sizes[3];
      if ((i1 - 1 < 1) || (i1 - 1 > workingset_isActiveConstr_size)) {
        emlrtDynamicBoundsCheckR2012b(i1 - 1, 1, workingset_isActiveConstr_size,
                                      &vc_emlrtBCI, &st);
      }
      if (!workingset_isActiveConstr_data[i1 - 2]) {
        if (i > solution_xstar_size) {
          emlrtDynamicBoundsCheckR2012b(i, 1, solution_xstar_size, &re_emlrtBCI,
                                        (emlrtCTX)sp);
        }
        if (i > workingset_lb_size) {
          emlrtDynamicBoundsCheckR2012b(i, 1, workingset_lb_size, &re_emlrtBCI,
                                        (emlrtCTX)sp);
        }
        ratio = -solution_xstar_data[i - 1] - workingset_lb_data[i - 1];
        if (i > solution_searchDir_size) {
          emlrtDynamicBoundsCheckR2012b(i, 1, solution_searchDir_size,
                                        &re_emlrtBCI, (emlrtCTX)sp);
        }
        alphaTemp =
            muDoubleScalarMin(muDoubleScalarAbs(ratio), 1.0E-6 - ratio) /
            alphaTemp;
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 4;
          *constrIdx = workingset_sizes[3];
          *newBlocking = true;
        }
      }
    }
  }
  if (workingset_nWConstr[4] < workingset_sizes[4]) {
    if ((workingset_nVar < 1) || (workingset_nVar > solution_xstar_size)) {
      emlrtDynamicBoundsCheckR2012b(workingset_nVar, 1, solution_xstar_size,
                                    &re_emlrtBCI, (emlrtCTX)sp);
    }
    phaseOneCorrectionX =
        (real_T)isPhaseOne * solution_xstar_data[workingset_nVar - 1];
    if (workingset_nVar > solution_searchDir_size) {
      emlrtDynamicBoundsCheckR2012b(workingset_nVar, 1, solution_searchDir_size,
                                    &re_emlrtBCI, (emlrtCTX)sp);
    }
    phaseOneCorrectionP =
        (real_T)isPhaseOne * solution_searchDir_data[workingset_nVar - 1];
    st.site = &bh_emlrtRSI;
    if (workingset_sizes[4] > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx = 0; idx < totalUB; idx++) {
      if ((idx + 1 < 1) || (idx + 1 > workingset_indexUB_size)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, workingset_indexUB_size,
                                      &re_emlrtBCI, (emlrtCTX)sp);
      }
      pk_corrected = solution_searchDir_data[workingset_indexUB_data[idx] - 1] -
                     phaseOneCorrectionP;
      if (pk_corrected > denomTol) {
        st.site = &bh_emlrtRSI;
        i = workingset_isActiveIdx[4] + idx;
        if ((i < 1) || (i > workingset_isActiveConstr_size)) {
          emlrtDynamicBoundsCheckR2012b(i, 1, workingset_isActiveConstr_size,
                                        &vc_emlrtBCI, &st);
        }
        if (!workingset_isActiveConstr_data[i - 1]) {
          ratio = (solution_xstar_data[workingset_indexUB_data[idx] - 1] -
                   workingset_ub_data[workingset_indexUB_data[idx] - 1]) -
                  phaseOneCorrectionX;
          alphaTemp =
              muDoubleScalarMin(muDoubleScalarAbs(ratio), 1.0E-6 - ratio) /
              pk_corrected;
          if (alphaTemp < *alpha) {
            *alpha = alphaTemp;
            *constrType = 5;
            *constrIdx = idx + 1;
            *newBlocking = true;
          }
        }
      }
    }
  }
  if (!isPhaseOne) {
    if ((*newBlocking) && (*alpha > 1.0)) {
      *newBlocking = false;
    }
    *alpha = muDoubleScalarMin(*alpha, 1.0);
  }
}

/* End of code generation (feasibleratiotest.c) */
