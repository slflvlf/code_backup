/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * checkMatrixNonFinite.c
 *
 * Code generation for function 'checkMatrixNonFinite'
 *
 */

/* Include files */
#include "checkMatrixNonFinite.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtBCInfo q_emlrtBCI = {
    -1,                     /* iFirst */
    -1,                     /* iLast */
    1,                      /* lineNo */
    1,                      /* colNo */
    "",                     /* aName */
    "checkMatrixNonFinite", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+utils\\+"
    "ObjNonlinEvaluator\\+internal\\checkMatrixNonFinite.p", /* pName */
    0                                                        /* checkKind */
};

/* Function Definitions */
int32_T checkMatrixNonFinite(const emlrtStack *sp, int32_T ncols,
                             const real_T mat_data[], int32_T mat_size,
                             int32_T col0, int32_T ldm)
{
  int32_T col;
  int32_T col_end;
  int32_T row;
  int32_T status;
  boolean_T allFinite;
  status = 1;
  allFinite = true;
  row = 0;
  col = col0;
  col_end = (col0 + ncols) - 1;
  while (allFinite && (col <= col_end)) {
    row = 0;
    while (allFinite && (row + 1 <= 79)) {
      int32_T idx_mat_tmp;
      idx_mat_tmp = row + ldm * (col - 1);
      if ((idx_mat_tmp + 1 < 1) || (idx_mat_tmp + 1 > mat_size)) {
        emlrtDynamicBoundsCheckR2012b(idx_mat_tmp + 1, 1, mat_size, &q_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      allFinite = ((!muDoubleScalarIsInf(mat_data[idx_mat_tmp])) &&
                   (!muDoubleScalarIsNaN(mat_data[idx_mat_tmp])));
      row++;
    }
    col++;
  }
  if (!allFinite) {
    real_T d;
    row += ldm * (col - 2);
    if ((row < 1) || (row > mat_size)) {
      emlrtDynamicBoundsCheckR2012b(row, 1, mat_size, &q_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    d = mat_data[row - 1];
    if (muDoubleScalarIsNaN(d)) {
      status = -3;
    } else {
      if (row > mat_size) {
        emlrtDynamicBoundsCheckR2012b(row, 1, mat_size, &q_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      if (d < 0.0) {
        status = -1;
      } else {
        status = -2;
      }
    }
  }
  return status;
}

/* End of code generation (checkMatrixNonFinite.c) */
