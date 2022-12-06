/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * checkLinearInputs.c
 *
 * Code generation for function 'checkLinearInputs'
 *
 */

/* Include files */
#include "checkLinearInputs.h"
#include "all.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo hb_emlrtRSI = {
    1,                   /* lineNo */
    "checkLinearInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "validate\\checkLinearInputs.p" /* pathName */
};

static emlrtRTEInfo g_emlrtRTEI = {
    1,                   /* lineNo */
    1,                   /* colNo */
    "checkLinearInputs", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "validate\\checkLinearInputs.p" /* pName */
};

static emlrtRTEInfo h_emlrtRTEI = {
    1,             /* lineNo */
    1,             /* colNo */
    "checkBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\shared\\optimlib\\+optim\\+coder\\+"
    "validate\\checkBounds.p" /* pName */
};

/* Function Definitions */
void checkLinearInputs(const emlrtStack *sp, const real_T Aineq_data[],
                       const int32_T Aineq_size[2], const real_T bineq_data[],
                       int32_T bineq_size, const real_T lb[79],
                       const real_T ub[79])
{
  emlrtStack st;
  int32_T b_i;
  int32_T i;
  boolean_T b_tmp_data[29040];
  boolean_T tmp_data[29040];
  boolean_T x_data[79];
  boolean_T exitg1;
  boolean_T guard1 = false;
  boolean_T y;
  st.prev = sp;
  st.tls = sp->tls;
  if ((Aineq_size[0] != 0) && (Aineq_size[1] != 0)) {
    i = Aineq_size[0] * Aineq_size[1];
    for (b_i = 0; b_i < i; b_i++) {
      tmp_data[b_i] = muDoubleScalarIsInf(Aineq_data[b_i]);
    }
    for (b_i = 0; b_i < i; b_i++) {
      b_tmp_data[b_i] = muDoubleScalarIsNaN(Aineq_data[b_i]);
    }
    for (b_i = 0; b_i < i; b_i++) {
      tmp_data[b_i] = ((!tmp_data[b_i]) && (!b_tmp_data[b_i]));
    }
    if (!all(tmp_data, i)) {
      emlrtErrorWithMessageIdR2018a(
          sp, &g_emlrtRTEI, "optimlib_codegen:common:InfNaNComplexDetected",
          "optimlib_codegen:common:InfNaNComplexDetected", 3, 4, 1, "A");
    }
  }
  if (bineq_size != 0) {
    for (b_i = 0; b_i < bineq_size; b_i++) {
      tmp_data[b_i] = muDoubleScalarIsInf(bineq_data[b_i]);
    }
    for (b_i = 0; b_i < bineq_size; b_i++) {
      b_tmp_data[b_i] = muDoubleScalarIsNaN(bineq_data[b_i]);
    }
    for (b_i = 0; b_i < bineq_size; b_i++) {
      tmp_data[b_i] = ((!tmp_data[b_i]) && (!b_tmp_data[b_i]));
    }
    if (!all(tmp_data, bineq_size)) {
      emlrtErrorWithMessageIdR2018a(
          sp, &g_emlrtRTEI, "optimlib_codegen:common:InfNaNComplexDetected",
          "optimlib_codegen:common:InfNaNComplexDetected", 3, 4, 1, "B");
    }
  }
  st.site = &hb_emlrtRSI;
  for (b_i = 0; b_i < 79; b_i++) {
    x_data[b_i] = (lb[b_i] == rtInf);
  }
  y = false;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i <= 78)) {
    if (x_data[i]) {
      y = true;
      exitg1 = true;
    } else {
      i++;
    }
  }
  guard1 = false;
  if (y) {
    guard1 = true;
  } else {
    for (i = 0; i < 79; i++) {
      x_data[i] = muDoubleScalarIsNaN(lb[i]);
    }
    y = false;
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i <= 78)) {
      if (x_data[i]) {
        y = true;
        exitg1 = true;
      } else {
        i++;
      }
    }
    if (y) {
      guard1 = true;
    }
  }
  if (guard1) {
    emlrtErrorWithMessageIdR2018a(
        &st, &h_emlrtRTEI, "optimlib_codegen:common:InfNaNComplexDetectedLB",
        "optimlib_codegen:common:InfNaNComplexDetectedLB", 0);
  }
  for (b_i = 0; b_i < 79; b_i++) {
    x_data[b_i] = (ub[b_i] == rtMinusInf);
  }
  y = false;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i <= 78)) {
    if (x_data[i]) {
      y = true;
      exitg1 = true;
    } else {
      i++;
    }
  }
  guard1 = false;
  if (y) {
    guard1 = true;
  } else {
    for (i = 0; i < 79; i++) {
      x_data[i] = muDoubleScalarIsNaN(ub[i]);
    }
    y = false;
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i <= 78)) {
      if (x_data[i]) {
        y = true;
        exitg1 = true;
      } else {
        i++;
      }
    }
    if (y) {
      guard1 = true;
    }
  }
  if (guard1) {
    emlrtErrorWithMessageIdR2018a(
        &st, &h_emlrtRTEI, "optimlib_codegen:common:InfNaNComplexDetectedUB",
        "optimlib_codegen:common:InfNaNComplexDetectedUB", 0);
  }
  if ((Aineq_size[0] != 0) && (Aineq_size[1] != 0) && (Aineq_size[1] != 79)) {
    emlrtErrorWithMessageIdR2018a(
        sp, &g_emlrtRTEI, "optimlib_codegen:common:WrongNumberOfColumnsInA",
        "optimlib_codegen:common:WrongNumberOfColumnsInA", 2, 12, 79);
  }
  if ((Aineq_size[0] != 0) && (Aineq_size[1] != 0) &&
      (Aineq_size[0] != bineq_size)) {
    emlrtErrorWithMessageIdR2018a(
        sp, &g_emlrtRTEI, "optimlib_codegen:common:AAndBinInconsistent",
        "optimlib_codegen:common:AAndBinInconsistent", 0);
  }
}

/* End of code generation (checkLinearInputs.c) */
