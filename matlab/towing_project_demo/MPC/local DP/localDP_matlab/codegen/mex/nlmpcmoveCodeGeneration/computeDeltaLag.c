/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeDeltaLag.c
 *
 * Code generation for function 'computeDeltaLag'
 *
 */

/* Include files */
#include "computeDeltaLag.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo di_emlrtRSI = {
    1,                 /* lineNo */
    "computeDeltaLag", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "stopping\\computeDeltaLag.p" /* pathName */
};

static emlrtBCInfo qe_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    1,                 /* lineNo */
    1,                 /* colNo */
    "",                /* aName */
    "computeDeltaLag", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "stopping\\computeDeltaLag.p", /* pName */
    0                              /* checkKind */
};

/* Function Definitions */
void computeDeltaLag(const emlrtStack *sp, int32_T nVar, int32_T ldJ,
                     int32_T mNonlinIneq, real_T workspace_data[],
                     const int32_T *workspace_size, const real_T grad_data[],
                     int32_T grad_size, const real_T JacIneqTrans_data[],
                     int32_T ineqJ0, const real_T JacEqTrans_data[],
                     const real_T grad_old_data[],
                     const real_T JacIneqTrans_old_data[],
                     const real_T JacEqTrans_old_data[],
                     const real_T lambda_data[], int32_T ineqL0, int32_T eqL0)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack st;
  real_T a;
  real_T beta1;
  int32_T b_i;
  int32_T i;
  char_T TRANSA;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &di_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  if (nVar > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  i = *workspace_size;
  for (b_i = 0; b_i < nVar; b_i++) {
    if ((b_i + 1 < 1) || (b_i + 1 > grad_size)) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, grad_size, &qe_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    if (b_i + 1 > i) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, i, &qe_emlrtBCI, (emlrtCTX)sp);
    }
    workspace_data[b_i] = grad_data[b_i];
  }
  st.site = &di_emlrtRSI;
  if (nVar >= 1) {
    a = -1.0;
    n_t = (ptrdiff_t)nVar;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    daxpy(&n_t, &a, &grad_old_data[0], &incx_t, &workspace_data[0], &incy_t);
  }
  st.site = &di_emlrtRSI;
  a = 1.0;
  beta1 = 1.0;
  TRANSA = 'N';
  m_t = (ptrdiff_t)nVar;
  n_t = (ptrdiff_t)60;
  lda_t = (ptrdiff_t)ldJ;
  incx_t = (ptrdiff_t)1;
  incy_t = (ptrdiff_t)1;
  dgemv(&TRANSA, &m_t, &n_t, &a, &JacEqTrans_data[0], &lda_t,
        &lambda_data[eqL0 - 1], &incx_t, &beta1, &workspace_data[0], &incy_t);
  st.site = &di_emlrtRSI;
  a = -1.0;
  beta1 = 1.0;
  TRANSA = 'N';
  m_t = (ptrdiff_t)nVar;
  n_t = (ptrdiff_t)60;
  lda_t = (ptrdiff_t)ldJ;
  incx_t = (ptrdiff_t)1;
  incy_t = (ptrdiff_t)1;
  dgemv(&TRANSA, &m_t, &n_t, &a, &JacEqTrans_old_data[0], &lda_t,
        &lambda_data[eqL0 - 1], &incx_t, &beta1, &workspace_data[0], &incy_t);
  if (mNonlinIneq > 0) {
    st.site = &di_emlrtRSI;
    a = 1.0;
    beta1 = 1.0;
    TRANSA = 'N';
    m_t = (ptrdiff_t)nVar;
    n_t = (ptrdiff_t)mNonlinIneq;
    lda_t = (ptrdiff_t)ldJ;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dgemv(&TRANSA, &m_t, &n_t, &a, &JacIneqTrans_data[ldJ * (ineqJ0 - 1)],
          &lda_t, &lambda_data[ineqL0 - 1], &incx_t, &beta1, &workspace_data[0],
          &incy_t);
    st.site = &di_emlrtRSI;
    a = -1.0;
    beta1 = 1.0;
    TRANSA = 'N';
    m_t = (ptrdiff_t)nVar;
    n_t = (ptrdiff_t)mNonlinIneq;
    lda_t = (ptrdiff_t)ldJ;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dgemv(&TRANSA, &m_t, &n_t, &a, &JacIneqTrans_old_data[0], &lda_t,
          &lambda_data[ineqL0 - 1], &incx_t, &beta1, &workspace_data[0],
          &incy_t);
  }
}

/* End of code generation (computeDeltaLag.c) */
