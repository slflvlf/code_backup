/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * indexShapeCheck.c
 *
 * Code generation for function 'indexShapeCheck'
 *
 */

/* Include files */
#include "indexShapeCheck.h"
#include "rt_nonfinite.h"
#include "update_mv.h"

/* Variable Definitions */
static emlrtRSInfo qb_emlrtRSI = { 43, /* lineNo */
  "indexShapeCheck",                   /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\indexShapeCheck.m"/* pathName */
};

static emlrtRTEInfo h_emlrtRTEI = { 121,/* lineNo */
  5,                                   /* colNo */
  "errOrWarnIf",                       /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\indexShapeCheck.m"/* pName */
};

/* Function Definitions */
void indexShapeCheck(const emlrtStack *sp, int32_T matrixSize)
{
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &qb_emlrtRSI;
  if (matrixSize == 1) {
    emlrtErrorWithMessageIdR2018a(&st, &h_emlrtRTEI,
      "Coder:FE:PotentialVectorVector", "Coder:FE:PotentialVectorVector", 0);
  }
}

/* End of code generation (indexShapeCheck.c) */
