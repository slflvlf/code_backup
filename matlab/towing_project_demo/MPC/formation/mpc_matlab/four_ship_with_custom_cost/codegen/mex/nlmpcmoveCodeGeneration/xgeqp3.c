/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xgeqp3.c
 *
 * Code generation for function 'xgeqp3'
 *
 */

/* Include files */
#include "xgeqp3.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "rt_nonfinite.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo kd_emlrtRSI = {
    63,       /* lineNo */
    "xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

static emlrtRSInfo ld_emlrtRSI = {
    158,            /* lineNo */
    "ceval_xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

static emlrtRSInfo md_emlrtRSI = {
    154,            /* lineNo */
    "ceval_xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

static emlrtRSInfo nd_emlrtRSI = {
    151,            /* lineNo */
    "ceval_xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

static emlrtRSInfo od_emlrtRSI = {
    148,            /* lineNo */
    "ceval_xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

static emlrtRSInfo pd_emlrtRSI = {
    143,            /* lineNo */
    "ceval_xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

static emlrtRSInfo qd_emlrtRSI = {
    141,            /* lineNo */
    "ceval_xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

static emlrtRSInfo rd_emlrtRSI = {
    138,            /* lineNo */
    "ceval_xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

static emlrtRSInfo sd_emlrtRSI = {
    98,             /* lineNo */
    "ceval_xgeqp3", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqp3.m" /* pathName */
};

/* Function Definitions */
void xgeqp3(const emlrtStack *sp, real_T A_data[], const int32_T A_size[2],
            int32_T m, int32_T n, int32_T jpvt_data[], const int32_T *jpvt_size,
            real_T tau_data[], int32_T *tau_size)
{
  static const char_T fname[14] = {'L', 'A', 'P', 'A', 'C', 'K', 'E',
                                   '_', 'd', 'g', 'e', 'q', 'p', '3'};
  ptrdiff_t jpvt_t_data[3411];
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T i;
  int32_T ma;
  int32_T minmn;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &kd_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  ma = A_size[0];
  *tau_size = muIntScalarMin_sint32(ma, A_size[1]);
  if (n < 1) {
    if (*tau_size - 1 >= 0) {
      memset(&tau_data[0], 0, *tau_size * sizeof(real_T));
    }
    b_st.site = &sd_emlrtRSI;
    for (ma = 0; ma < n; ma++) {
      jpvt_data[ma] = ma + 1;
    }
  } else {
    boolean_T overflow;
    for (minmn = 0; minmn < *jpvt_size; minmn++) {
      jpvt_t_data[minmn] = (ptrdiff_t)jpvt_data[minmn];
    }
    ptrdiff_t info_t;
    info_t =
        LAPACKE_dgeqp3(102, (ptrdiff_t)m, (ptrdiff_t)n, &A_data[0],
                       (ptrdiff_t)A_size[0], &jpvt_t_data[0], &tau_data[0]);
    b_st.site = &rd_emlrtRSI;
    minmn = (int32_T)info_t;
    if (minmn != 0) {
      overflow = true;
      if (minmn != -4) {
        if (minmn == -1010) {
          emlrtErrorWithMessageIdR2018a(&b_st, &o_emlrtRTEI, "MATLAB:nomem",
                                        "MATLAB:nomem", 0);
        } else {
          emlrtErrorWithMessageIdR2018a(&b_st, &n_emlrtRTEI,
                                        "Coder:toolbox:LAPACKCallErrorInfo",
                                        "Coder:toolbox:LAPACKCallErrorInfo", 5,
                                        4, 14, &fname[0], 12, minmn);
        }
      }
    } else {
      overflow = false;
    }
    if (overflow) {
      b_st.site = &qd_emlrtRSI;
      if (n > 2147483646) {
        c_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      overflow = (m > 2147483646);
      for (minmn = 0; minmn < n; minmn++) {
        b_st.site = &pd_emlrtRSI;
        if (overflow) {
          c_st.site = &db_emlrtRSI;
          check_forloop_overflow_error(&c_st);
        }
        for (i = 0; i < m; i++) {
          A_data[minmn * ma + i] = rtNaN;
        }
      }
      minmn = muIntScalarMin_sint32(m, n) - 1;
      b_st.site = &od_emlrtRSI;
      for (ma = 0; ma <= minmn; ma++) {
        tau_data[ma] = rtNaN;
      }
      minmn += 2;
      b_st.site = &nd_emlrtRSI;
      if (minmn <= *tau_size) {
        memset(&tau_data[minmn + -1], 0,
               ((*tau_size - minmn) + 1) * sizeof(real_T));
      }
      b_st.site = &md_emlrtRSI;
      for (ma = 0; ma < n; ma++) {
        jpvt_data[ma] = ma + 1;
      }
    } else {
      b_st.site = &ld_emlrtRSI;
      if (n > 2147483646) {
        c_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (ma = 0; ma < n; ma++) {
        jpvt_data[ma] = (int32_T)jpvt_t_data[ma];
      }
    }
  }
}

/* End of code generation (xgeqp3.c) */
