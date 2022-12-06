/*
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
void znlmpc_getXUe(const real_T z[85], const real_T x[6], real_T X[66],
                   real_T U[88], real_T *e)
{
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T Umv[88];
  real_T b_dv[80];
  real_T b_z[60];
  real_T alpha1;
  real_T beta1;
  int32_T b_i;
  int32_T i;
  char_T TRANSA1;
  char_T TRANSB1;
  memset(&Umv[0], 0, 88U * sizeof(real_T));
  TRANSB1 = 'T';
  TRANSA1 = 'N';
  alpha1 = 1.0;
  beta1 = 0.0;
  m_t = (ptrdiff_t)80;
  n_t = (ptrdiff_t)1;
  k_t = (ptrdiff_t)24;
  lda_t = (ptrdiff_t)80;
  ldb_t = (ptrdiff_t)1;
  ldc_t = (ptrdiff_t)80;
  dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &dv[0], &lda_t, &z[60],
        &ldb_t, &beta1, &b_dv[0], &ldc_t);
  for (i = 0; i < 8; i++) {
    for (b_i = 0; b_i < 10; b_i++) {
      Umv[b_i + 11 * i] = b_dv[i + (b_i << 3)];
    }
  }
  *e = z[84];
  memcpy(&b_z[0], &z[0], 60U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    for (b_i = 0; b_i < 10; b_i++) {
      X[(b_i + 11 * i) + 1] = b_z[i + 6 * b_i];
    }
  }
  for (i = 0; i < 6; i++) {
    X[11 * i] = x[i];
  }
  for (i = 0; i < 8; i++) {
    Umv[11 * i + 10] = Umv[11 * i + 9];
    memcpy(&U[i * 11], &Umv[i * 11], 11U * sizeof(real_T));
  }
}

/* End of code generation (znlmpc_getXUe.c) */
