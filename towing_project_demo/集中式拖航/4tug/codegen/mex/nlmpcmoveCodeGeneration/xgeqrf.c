/*
 * xgeqrf.c
 *
 * Code generation for function 'xgeqrf'
 *
 */

/* Include files */
#include "xgeqrf.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "rt_nonfinite.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo xe_emlrtRSI = {
    27,       /* lineNo */
    "xgeqrf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqrf.m" /* pathName */
};

static emlrtRSInfo ye_emlrtRSI = {
    102,            /* lineNo */
    "ceval_xgeqrf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqrf.m" /* pathName */
};

static emlrtRSInfo af_emlrtRSI = {
    99,             /* lineNo */
    "ceval_xgeqrf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqrf.m" /* pathName */
};

static emlrtRSInfo bf_emlrtRSI = {
    94,             /* lineNo */
    "ceval_xgeqrf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqrf.m" /* pathName */
};

static emlrtRSInfo cf_emlrtRSI = {
    93,             /* lineNo */
    "ceval_xgeqrf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqrf.m" /* pathName */
};

static emlrtRSInfo df_emlrtRSI = {
    91,             /* lineNo */
    "ceval_xgeqrf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeqrf.m" /* pathName */
};

/* Function Definitions */
void xgeqrf(const emlrtStack *sp, real_T A_data[], const int32_T A_size[2],
            int32_T m, int32_T n, real_T tau_data[], int32_T *tau_size)
{
  static const char_T fname[14] = {'L', 'A', 'P', 'A', 'C', 'K', 'E',
                                   '_', 'd', 'g', 'e', 'q', 'r', 'f'};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T i;
  int32_T ma;
  int32_T minmn;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &xe_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  ma = A_size[0];
  *tau_size = muIntScalarMin_sint32(ma, A_size[1]);
  if (n == 0) {
    if (*tau_size - 1 >= 0) {
      memset(&tau_data[0], 0, *tau_size * sizeof(real_T));
    }
  } else {
    ptrdiff_t info_t;
    boolean_T p;
    info_t = LAPACKE_dgeqrf(102, (ptrdiff_t)m, (ptrdiff_t)n, &A_data[0],
                            (ptrdiff_t)A_size[0], &tau_data[0]);
    b_st.site = &df_emlrtRSI;
    minmn = (int32_T)info_t;
    if (minmn != 0) {
      p = true;
      if (minmn != -4) {
        if (minmn == -1010) {
          emlrtErrorWithMessageIdR2018a(&b_st, &p_emlrtRTEI, "MATLAB:nomem",
                                        "MATLAB:nomem", 0);
        } else {
          emlrtErrorWithMessageIdR2018a(&b_st, &o_emlrtRTEI,
                                        "Coder:toolbox:LAPACKCallErrorInfo",
                                        "Coder:toolbox:LAPACKCallErrorInfo", 5,
                                        4, 14, &fname[0], 12, minmn);
        }
      }
    } else {
      p = false;
    }
    if (p) {
      b_st.site = &cf_emlrtRSI;
      if (n > 2147483646) {
        c_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (minmn = 0; minmn < n; minmn++) {
        b_st.site = &bf_emlrtRSI;
        if (m > 2147483646) {
          c_st.site = &db_emlrtRSI;
          check_forloop_overflow_error(&c_st);
        }
        for (i = 0; i < m; i++) {
          A_data[minmn * ma + i] = rtNaN;
        }
      }
      minmn = muIntScalarMin_sint32(m, n);
      b_st.site = &af_emlrtRSI;
      for (i = 0; i < minmn; i++) {
        tau_data[i] = rtNaN;
      }
      minmn++;
      b_st.site = &ye_emlrtRSI;
      if (minmn <= *tau_size) {
        memset(&tau_data[minmn + -1], 0,
               ((*tau_size - minmn) + 1) * sizeof(real_T));
      }
    }
  }
}

/* End of code generation (xgeqrf.c) */
