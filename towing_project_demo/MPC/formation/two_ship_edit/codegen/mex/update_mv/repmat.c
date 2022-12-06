/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * repmat.c
 *
 * Code generation for function 'repmat'
 *
 */

/* Include files */
#include "repmat.h"
#include "rt_nonfinite.h"
#include "update_mv.h"

/* Function Definitions */
void repmat(const real_T a[6], real_T b[12])
{
  int32_T itilerow;
  int32_T ibcol;
  int32_T k;
  for (itilerow = 0; itilerow < 2; itilerow++) {
    ibcol = itilerow * 6;
    for (k = 0; k < 6; k++) {
      b[ibcol + k] = a[k];
    }
  }
}

/* End of code generation (repmat.c) */
