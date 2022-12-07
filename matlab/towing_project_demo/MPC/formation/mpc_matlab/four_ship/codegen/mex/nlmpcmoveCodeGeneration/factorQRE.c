/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * factorQRE.c
 *
 * Code generation for function 'factorQRE'
 *
 */

/* Include files */
#include "factorQRE.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "xgeqp3.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void factorQRE(const emlrtStack *sp, i_struct_T *obj, const real_T A_data[],
               int32_T mrows, int32_T ncols, int32_T ldA)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack st;
  int32_T idx;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &jd_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  if (ncols > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  n_t = (ptrdiff_t)mrows;
  incx_t = (ptrdiff_t)1;
  incy_t = (ptrdiff_t)1;
  for (idx = 0; idx < ncols; idx++) {
    st.site = &jd_emlrtRSI;
    dcopy(&n_t, &A_data[ldA * idx], &incx_t, &obj->QR.data[obj->ldq * idx],
          &incy_t);
  }
  obj->usedPivoting = true;
  obj->mrows = mrows;
  obj->ncols = ncols;
  obj->minRowCol = muIntScalarMin_sint32(mrows, ncols);
  st.site = &jd_emlrtRSI;
  xgeqp3(&st, obj->QR.data, obj->QR.size, mrows, ncols, obj->jpvt.data,
         &obj->jpvt.size[0], obj->tau.data, &obj->tau.size[0]);
}

/* End of code generation (factorQRE.c) */
