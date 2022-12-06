/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xtrsv.c
 *
 * Code generation for function 'xtrsv'
 *
 */

/* Include files */
#include "xtrsv.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void xtrsv(int32_T n, const real_T A_data[], int32_T lda, real_T x_data[])
{
  ptrdiff_t incx_t;
  ptrdiff_t lda_t;
  ptrdiff_t n_t;
  char_T DIAGA1;
  char_T TRANSA1;
  char_T UPLO1;
  if (n >= 1) {
    DIAGA1 = 'N';
    TRANSA1 = 'N';
    UPLO1 = 'U';
    n_t = (ptrdiff_t)n;
    lda_t = (ptrdiff_t)lda;
    incx_t = (ptrdiff_t)1;
    dtrsv(&UPLO1, &TRANSA1, &DIAGA1, &n_t, &A_data[0], &lda_t, &x_data[0],
          &incx_t);
  }
}

/* End of code generation (xtrsv.c) */
