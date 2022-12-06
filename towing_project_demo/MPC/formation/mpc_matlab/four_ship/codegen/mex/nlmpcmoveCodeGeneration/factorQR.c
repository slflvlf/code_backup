/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * factorQR.c
 *
 * Code generation for function 'factorQR'
 *
 */

/* Include files */
#include "factorQR.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "xgeqrf.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void factorQR(const emlrtStack *sp, i_struct_T *obj, const real_T A_data[],
              int32_T mrows, int32_T ncols, int32_T ldA)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack st;
  int32_T i;
  int32_T idx;
  boolean_T guard1 = false;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  i = mrows * ncols;
  guard1 = false;
  if (i > 0) {
    st.site = &xe_emlrtRSI;
    if (ncols > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx = 0; idx < ncols; idx++) {
      st.site = &xe_emlrtRSI;
      n_t = (ptrdiff_t)mrows;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      dcopy(&n_t, &A_data[ldA * idx], &incx_t, &obj->QR.data[obj->ldq * idx],
            &incy_t);
    }
    guard1 = true;
  } else if (i == 0) {
    obj->mrows = mrows;
    obj->ncols = ncols;
    obj->minRowCol = 0;
  } else {
    guard1 = true;
  }
  if (guard1) {
    obj->usedPivoting = false;
    obj->mrows = mrows;
    obj->ncols = ncols;
    st.site = &xe_emlrtRSI;
    if (ncols > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx = 0; idx < ncols; idx++) {
      i = obj->jpvt.size[0];
      if ((idx + 1 < 1) || (idx + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &pc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      obj->jpvt.data[idx] = idx + 1;
    }
    obj->minRowCol = muIntScalarMin_sint32(mrows, ncols);
    st.site = &xe_emlrtRSI;
    xgeqrf(&st, obj->QR.data, obj->QR.size, mrows, ncols, obj->tau.data,
           &obj->tau.size[0]);
  }
}

/* End of code generation (factorQR.c) */
