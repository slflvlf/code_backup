/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xgemv.c
 *
 * Code generation for function 'xgemv'
 *
 */

/* Include files */
#include "xgemv.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void b_xgemv(int32_T m, int32_T n, const real_T A_data[], int32_T lda,
             const real_T x_data[], real_T y_data[])
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T alpha1;
  real_T beta1;
  char_T TRANSA;
  if (m >= 1) {
    alpha1 = 1.0;
    beta1 = 0.0;
    TRANSA = 'T';
    m_t = (ptrdiff_t)m;
    n_t = (ptrdiff_t)n;
    lda_t = (ptrdiff_t)lda;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dgemv(&TRANSA, &m_t, &n_t, &alpha1, &A_data[0], &lda_t, &x_data[0], &incx_t,
          &beta1, &y_data[0], &incy_t);
  }
}

void c_xgemv(int32_T m, int32_T n, const real_T A_data[], int32_T lda,
             const real_T x_data[], real_T y_data[])
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T alpha1;
  real_T beta1;
  char_T TRANSA;
  if (n >= 1) {
    alpha1 = -1.0;
    beta1 = 1.0;
    TRANSA = 'T';
    m_t = (ptrdiff_t)m;
    n_t = (ptrdiff_t)n;
    lda_t = (ptrdiff_t)lda;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dgemv(&TRANSA, &m_t, &n_t, &alpha1, &A_data[0], &lda_t, &x_data[0], &incx_t,
          &beta1, &y_data[0], &incy_t);
  }
}

void d_xgemv(int32_T m, int32_T n, const real_T A_data[], int32_T lda,
             const real_T x_data[], real_T y_data[])
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T alpha1;
  real_T beta1;
  char_T TRANSA;
  if (n >= 1) {
    alpha1 = 1.0;
    beta1 = -1.0;
    TRANSA = 'T';
    m_t = (ptrdiff_t)m;
    n_t = (ptrdiff_t)n;
    lda_t = (ptrdiff_t)lda;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dgemv(&TRANSA, &m_t, &n_t, &alpha1, &A_data[0], &lda_t, &x_data[0], &incx_t,
          &beta1, &y_data[0], &incy_t);
  }
}

void e_xgemv(int32_T m, int32_T n, const real_T A_data[], int32_T lda,
             const real_T x_data[], int32_T ix0, real_T y_data[])
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T alpha1;
  real_T beta1;
  char_T TRANSA;
  if (n >= 1) {
    alpha1 = 1.0;
    beta1 = -1.0;
    TRANSA = 'T';
    m_t = (ptrdiff_t)m;
    n_t = (ptrdiff_t)n;
    lda_t = (ptrdiff_t)lda;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dgemv(&TRANSA, &m_t, &n_t, &alpha1, &A_data[0], &lda_t, &x_data[ix0 - 1],
          &incx_t, &beta1, &y_data[0], &incy_t);
  }
}

void f_xgemv(int32_T m, int32_T n, const real_T A[6241], int32_T lda,
             const real_T x_data[], real_T y_data[])
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T alpha1;
  real_T beta1;
  char_T TRANSA;
  if ((m >= 1) && (n >= 1)) {
    alpha1 = 1.0;
    beta1 = 0.0;
    TRANSA = 'N';
    m_t = (ptrdiff_t)m;
    n_t = (ptrdiff_t)n;
    lda_t = (ptrdiff_t)lda;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dgemv(&TRANSA, &m_t, &n_t, &alpha1, &A[0], &lda_t, &x_data[0], &incx_t,
          &beta1, &y_data[0], &incy_t);
  }
}

void g_xgemv(int32_T m, int32_T n, const real_T A_data[], int32_T ia0,
             int32_T lda, const real_T x_data[], real_T y_data[])
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T alpha1;
  real_T beta1;
  char_T TRANSA;
  if (m >= 1) {
    alpha1 = 1.0;
    beta1 = 0.0;
    TRANSA = 'N';
    m_t = (ptrdiff_t)m;
    n_t = (ptrdiff_t)n;
    lda_t = (ptrdiff_t)lda;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dgemv(&TRANSA, &m_t, &n_t, &alpha1, &A_data[ia0 - 1], &lda_t, &x_data[0],
          &incx_t, &beta1, &y_data[0], &incy_t);
  }
}

void h_xgemv(int32_T n, const real_T A_data[], int32_T lda,
             const real_T x_data[], real_T y_data[])
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T alpha1;
  real_T beta1;
  char_T TRANSA;
  if (n >= 1) {
    alpha1 = 1.0;
    beta1 = -1.0;
    TRANSA = 'T';
    m_t = (ptrdiff_t)79;
    n_t = (ptrdiff_t)n;
    lda_t = (ptrdiff_t)lda;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dgemv(&TRANSA, &m_t, &n_t, &alpha1, &A_data[0], &lda_t, &x_data[0], &incx_t,
          &beta1, &y_data[0], &incy_t);
  }
}

void xgemv(int32_T m, int32_T n, const real_T A_data[], int32_T lda,
           const real_T x_data[], int32_T ix0, real_T y_data[])
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T alpha1;
  real_T beta1;
  char_T TRANSA;
  if (n >= 1) {
    alpha1 = 1.0;
    beta1 = 1.0;
    TRANSA = 'N';
    m_t = (ptrdiff_t)m;
    n_t = (ptrdiff_t)n;
    lda_t = (ptrdiff_t)lda;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dgemv(&TRANSA, &m_t, &n_t, &alpha1, &A_data[0], &lda_t, &x_data[ix0 - 1],
          &incx_t, &beta1, &y_data[0], &incy_t);
  }
}

/* End of code generation (xgemv.c) */
