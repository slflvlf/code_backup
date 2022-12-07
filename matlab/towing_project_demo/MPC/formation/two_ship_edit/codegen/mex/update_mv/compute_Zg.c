/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compute_Zg.c
 *
 * Code generation for function 'compute_Zg'
 *
 */

/* Include files */
#include "compute_Zg.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "update_mv.h"

/* Variable Definitions */
static emlrtRSInfo gd_emlrtRSI = { 7,  /* lineNo */
  "compute_Zg",                        /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\compute_Zg.m"/* pathName */
};

static emlrtRTEInfo p_emlrtRTEI = { 13,/* lineNo */
  9,                                   /* colNo */
  "sqrt",                              /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\elfun\\sqrt.m"/* pName */
};

/* Function Definitions */
void compute_Zg(const emlrtStack *sp, const struct0_T *sc_past, struct0_T *sc)
{
  real_T sDelta;
  real_T p2;
  real_T p1;
  real_T p_minus;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  *sc = *sc_past;
  st.site = &gd_emlrtRSI;
  sDelta = sc_past->bg * sc_past->bg - 4.0 * sc_past->ag * sc_past->cg;
  if (sDelta < 0.0) {
    emlrtErrorWithMessageIdR2018a(&st, &p_emlrtRTEI,
      "Coder:toolbox:ElFunDomainError", "Coder:toolbox:ElFunDomainError", 3, 4,
      4, "sqrt");
  }

  sDelta = muDoubleScalarSqrt(sDelta);
  if (muDoubleScalarAbs(sc_past->ag) < 2.2204460492503131E-16) {
    p1 = -sc_past->cg / sc_past->bg;
    p2 = -muDoubleScalarSign(sc_past->bg) * rtInf;
  } else {
    p2 = 2.0 * sc_past->ag;
    p1 = (-sc_past->bg - sDelta) / p2;
    p2 = (-sc_past->bg + sDelta) / p2;
  }

  if ((p1 > p2) || (muDoubleScalarIsNaN(p1) && (!muDoubleScalarIsNaN(p2)))) {
    sDelta = p2;
  } else {
    sDelta = p1;
  }

  p_minus = sc_past->pc + sc_past->beta * sDelta;
  if ((p1 < p2) || (muDoubleScalarIsNaN(p1) && (!muDoubleScalarIsNaN(p2)))) {
    p1 = p2;
  }

  sDelta = sc_past->pc + sc_past->beta * p1;
  sc->Z[2] = 0.0;
  sc->Z[3] = 0.0;
  if ((p_minus <= sc_past->pbarmin) && (sc_past->gpbarmin <= 0.0)) {
    sc->ngz = 1.0;
    sc->Z[0] = sc_past->pbarmin;
    sc->Z[1] = sDelta;
  } else if ((p_minus <= sc_past->pbarmin) && (sc_past->gpbarmax <= 0.0)) {
    sc->ngz = 1.0;
    sc->Z[0] = sDelta;
    sc->Z[1] = sc_past->pbarmax;
  } else if ((sDelta >= sc_past->pbarmax) && (sc_past->gpbarmin <= 0.0)) {
    sc->ngz = 1.0;
    sc->Z[0] = sc_past->pbarmin;
    sc->Z[1] = p_minus;
  } else if ((sDelta >= sc_past->pbarmax) && (sc_past->gpbarmax <= 0.0)) {
    sc->ngz = 1.0;
    sc->Z[0] = p_minus;
    sc->Z[1] = sc_past->pbarmax;
  } else if ((p_minus <= sc_past->pbarmax) && (p_minus >= sc_past->pbarmin) &&
             (sDelta <= sc_past->pbarmax) && (sDelta >= sc_past->pbarmin) &&
             (sc_past->ag >= 0.0)) {
    sc->ngz = 1.0;
    sc->Z[0] = p_minus;
    sc->Z[1] = sDelta;
  } else {
    sc->ngz = 2.0;
    sc->Z[0] = sc_past->pbarmin;
    sc->Z[1] = p_minus;
    sc->Z[2] = sDelta;
    sc->Z[3] = sc_past->pbarmax;
  }
}

/* End of code generation (compute_Zg.c) */
