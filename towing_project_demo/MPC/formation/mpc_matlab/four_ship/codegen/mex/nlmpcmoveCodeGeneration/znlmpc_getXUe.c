/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * znlmpc_getXUe.c
 *
 * Code generation for function 'znlmpc_getXUe'
 *
 */

/* Include files */
#include "znlmpc_getXUe.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void znlmpc_getXUe(const real_T z[265], const real_T x[24], real_T X[216],
                   real_T U[216], real_T *e)
{
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T Umv[216];
  real_T b_z[192];
  real_T alpha1;
  real_T beta1;
  int32_T b_i;
  int32_T i;
  char_T TRANSA1;
  char_T TRANSB1;
  memset(&Umv[0], 0, 216U * sizeof(real_T));
  TRANSB1 = 'T';
  TRANSA1 = 'N';
  alpha1 = 1.0;
  beta1 = 0.0;
  m_t = (ptrdiff_t)192;
  n_t = (ptrdiff_t)1;
  k_t = (ptrdiff_t)72;
  lda_t = (ptrdiff_t)192;
  ldb_t = (ptrdiff_t)1;
  ldc_t = (ptrdiff_t)192;
  dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &dv[0], &lda_t, &z[192],
        &ldb_t, &beta1, &b_z[0], &ldc_t);
  for (i = 0; i < 24; i++) {
    for (b_i = 0; b_i < 8; b_i++) {
      Umv[b_i + 9 * i] = b_z[i + 24 * b_i];
    }
  }
  *e = z[264];
  memcpy(&b_z[0], &z[0], 192U * sizeof(real_T));
  for (i = 0; i < 24; i++) {
    for (b_i = 0; b_i < 8; b_i++) {
      X[(b_i + 9 * i) + 1] = b_z[i + 24 * b_i];
    }
  }
  for (i = 0; i < 24; i++) {
    X[9 * i] = x[i];
    Umv[9 * i + 8] = Umv[9 * i + 7];
    memcpy(&U[i * 9], &Umv[i * 9], 9U * sizeof(real_T));
  }
}

/* End of code generation (znlmpc_getXUe.c) */
