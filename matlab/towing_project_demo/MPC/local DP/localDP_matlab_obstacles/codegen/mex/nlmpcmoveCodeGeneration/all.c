/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * all.c
 *
 * Code generation for function 'all'
 *
 */

/* Include files */
#include "all.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ib_emlrtRSI =
    {
        13,    /* lineNo */
        "all", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2022a\\toolbox\\eml\\lib\\matlab\\ops\\all.m" /* pathName
                                                                        */
};

static emlrtRSInfo jb_emlrtRSI =
    {
        143,        /* lineNo */
        "allOrAny", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\allOrAny."
        "m" /* pathName */
};

/* Function Definitions */
boolean_T all(const boolean_T x_data[], int32_T x_size)
{
  int32_T ix;
  boolean_T exitg1;
  boolean_T y;
  y = true;
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= x_size)) {
    if (!x_data[ix - 1]) {
      y = false;
      exitg1 = true;
    } else {
      ix++;
    }
  }
  return y;
}

void b_all(const emlrtStack *sp, const boolean_T x[60], boolean_T y[6])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T i;
  int32_T i2;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &ib_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  for (i2 = 0; i2 < 6; i2++) {
    y[i2] = true;
  }
  i2 = 10;
  for (i = 0; i < 6; i++) {
    int32_T a;
    int32_T i1;
    boolean_T exitg1;
    a = i2;
    i1 = i2 - 9;
    i2 += 10;
    b_st.site = &jb_emlrtRSI;
    if ((i1 <= a) && (a > 2147483646)) {
      c_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&c_st);
    }
    exitg1 = false;
    while ((!exitg1) && (i1 <= a)) {
      if (!x[i1 - 1]) {
        y[i] = false;
        exitg1 = true;
      } else {
        i1++;
      }
    }
  }
}

/* End of code generation (all.c) */
