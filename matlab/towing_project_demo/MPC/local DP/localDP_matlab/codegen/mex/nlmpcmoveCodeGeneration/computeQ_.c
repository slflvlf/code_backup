/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeQ_.c
 *
 * Code generation for function 'computeQ_'
 *
 */

/* Include files */
#include "computeQ_.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "lapacke.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ae_emlrtRSI = {
    1,           /* lineNo */
    "computeQ_", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "QRManager\\computeQ_.p" /* pathName */
};

static emlrtRSInfo be_emlrtRSI = {
    60,             /* lineNo */
    "ceval_xorgqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xorgqr.m" /* pathName */
};

static emlrtRSInfo ce_emlrtRSI = {
    14,       /* lineNo */
    "xorgqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xorgqr.m" /* pathName */
};

/* Function Definitions */
void computeQ_(const emlrtStack *sp, i_struct_T *obj, int32_T nrows)
{
  static const char_T fname[14] = {'L', 'A', 'P', 'A', 'C', 'K', 'E',
                                   '_', 'd', 'o', 'r', 'g', 'q', 'r'};
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T b;
  int32_T iQR0;
  int32_T idx;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  b = obj->minRowCol;
  st.site = &ae_emlrtRSI;
  if (obj->minRowCol > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = 0; idx < b; idx++) {
    int32_T n;
    iQR0 = (obj->ldq * idx + idx) + 1;
    st.site = &ae_emlrtRSI;
    n = (obj->mrows - idx) - 1;
    if (n >= 1) {
      b_st.site = &mc_emlrtRSI;
      n_t = (ptrdiff_t)n;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      dcopy(&n_t, &obj->QR.data[iQR0], &incx_t, &obj->Q.data[iQR0], &incy_t);
    }
  }
  st.site = &ae_emlrtRSI;
  b_st.site = &ce_emlrtRSI;
  n_t = LAPACKE_dorgqr(102, (ptrdiff_t)obj->mrows, (ptrdiff_t)nrows,
                       (ptrdiff_t)obj->minRowCol, &obj->Q.data[0],
                       (ptrdiff_t)obj->ldq, &obj->tau.data[0]);
  b = (int32_T)n_t;
  c_st.site = &be_emlrtRSI;
  if (b != 0) {
    boolean_T b_p;
    p = true;
    b_p = false;
    if (b == -7) {
      b_p = true;
    } else if (b == -5) {
      b_p = true;
    }
    if (!b_p) {
      if (b == -1010) {
        emlrtErrorWithMessageIdR2018a(&c_st, &p_emlrtRTEI, "MATLAB:nomem",
                                      "MATLAB:nomem", 0);
      } else {
        emlrtErrorWithMessageIdR2018a(
            &c_st, &o_emlrtRTEI, "Coder:toolbox:LAPACKCallErrorInfo",
            "Coder:toolbox:LAPACKCallErrorInfo", 5, 4, 14, &fname[0], 12, b);
      }
    }
  } else {
    p = false;
  }
  if (p) {
    b = obj->Q.size[0];
    iQR0 = obj->Q.size[1];
    obj->Q.size[0] = b;
    obj->Q.size[1] = iQR0;
    b *= iQR0;
    for (iQR0 = 0; iQR0 < b; iQR0++) {
      obj->Q.data[iQR0] = rtNaN;
    }
  }
}

/* End of code generation (computeQ_.c) */
