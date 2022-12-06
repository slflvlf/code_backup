/*
 * mtimes.c
 *
 * Code generation for function 'mtimes'
 *
 */

/* Include files */
#include "mtimes.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void mtimes(const real_T A_data[], const int32_T A_size[2], real_T C_data[],
            int32_T C_size[2])
{
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T alpha1;
  real_T beta1;
  char_T TRANSA1;
  char_T TRANSB1;
  if (A_size[0] == 0) {
    C_size[0] = 0;
    C_size[1] = 24;
  } else {
    TRANSB1 = 'N';
    TRANSA1 = 'N';
    alpha1 = 1.0;
    beta1 = 0.0;
    m_t = (ptrdiff_t)A_size[0];
    n_t = (ptrdiff_t)24;
    k_t = (ptrdiff_t)80;
    lda_t = (ptrdiff_t)A_size[0];
    ldb_t = (ptrdiff_t)80;
    ldc_t = (ptrdiff_t)A_size[0];
    C_size[0] = A_size[0];
    C_size[1] = 24;
    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &A_data[0], &lda_t,
          &dv[0], &ldb_t, &beta1, &C_data[0], &ldc_t);
  }
}

/* End of code generation (mtimes.c) */
