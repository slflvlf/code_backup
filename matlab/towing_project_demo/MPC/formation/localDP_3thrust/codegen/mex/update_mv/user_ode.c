/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * user_ode.c
 *
 * Code generation for function 'user_ode'
 *
 */

/* Include files */
#include "user_ode.h"
#include "indexShapeCheck.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "update_mv.h"
#include "update_mv_data.h"
#include "warning.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo mb_emlrtRSI = { 14, /* lineNo */
  "user_ode",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\nlmpc\\localDP_3thrust\\user_ode.m"/* pathName */
};

static emlrtRSInfo nb_emlrtRSI = { 19, /* lineNo */
  "user_ode",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\nlmpc\\localDP_3thrust\\user_ode.m"/* pathName */
};

static emlrtRSInfo pb_emlrtRSI = { 31, /* lineNo */
  "inv",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\matfun\\inv.m"/* pathName */
};

static emlrtRSInfo qb_emlrtRSI = { 42, /* lineNo */
  "checkcond",                         /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\matfun\\inv.m"/* pathName */
};

static emlrtRSInfo rb_emlrtRSI = { 46, /* lineNo */
  "checkcond",                         /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\matfun\\inv.m"/* pathName */
};

static emlrtMCInfo c_emlrtMCI = { 53,  /* lineNo */
  19,                                  /* colNo */
  "flt2str",                           /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\flt2str.m"/* pName */
};

static emlrtBCInfo p_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  14,                                  /* lineNo */
  35,                                  /* colNo */
  "u",                                 /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\nlmpc\\localDP_3thrust\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRSInfo od_emlrtRSI = { 53, /* lineNo */
  "flt2str",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\flt2str.m"/* pathName */
};

/* Function Declarations */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, char_T y[14]);
static const mxArray *b_sprintf(const emlrtStack *sp, const mxArray *b, const
  mxArray *c, emlrtMCInfo *location);
static void emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *a__output_of_sprintf_, const char_T *identifier, char_T y[14]);
static void v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, char_T ret[14]);

/* Function Definitions */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, char_T y[14])
{
  v_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static const mxArray *b_sprintf(const emlrtStack *sp, const mxArray *b, const
  mxArray *c, emlrtMCInfo *location)
{
  const mxArray *pArrays[2];
  const mxArray *m;
  pArrays[0] = b;
  pArrays[1] = c;
  return emlrtCallMATLABR2012b(sp, 1, &m, 2, pArrays, "sprintf", true, location);
}

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *a__output_of_sprintf_, const char_T *identifier, char_T y[14])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  b_emlrt_marshallIn(sp, emlrtAlias(a__output_of_sprintf_), &thisId, y);
  emlrtDestroyArray(&a__output_of_sprintf_);
}

static void v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, char_T ret[14])
{
  static const int32_T dims[2] = { 1, 14 };

  emlrtCheckBuiltInR2012b(sp, msgId, src, "char", false, 2U, dims);
  emlrtImportCharArrayR2015b(sp, src, &ret[0], 14);
  emlrtDestroyArray(&src);
}

void user_ode(const emlrtStack *sp, const real_T x[6], const real_T u_data[],
              const int32_T u_size[1], const real_T p_ode_M[9], const real_T
              p_ode_D[9], const real_T p_ode_thrust_config[6], real_T xdot[6])
{
  real_T absx11;
  real_T absx21;
  real_T absx31;
  real_T B[9];
  real_T T_tmp;
  int32_T i;
  real_T R_tmp[9];
  int32_T p2;
  int32_T p3;
  int32_T itmp;
  real_T a[9];
  boolean_T exitg1;
  const mxArray *y;
  const mxArray *m;
  static const int32_T iv[2] = { 1, 6 };

  static const char_T rfmt[6] = { '%', '1', '4', '.', '6', 'e' };

  const mxArray *b_y;
  real_T b_B[3];
  const mxArray *m1;
  char_T str[14];
  real_T c_B[3];
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;

  /* ------------------------------------------------------------------------------- */
  /*  pdf_mpc package: Example 1 - Definition of the user_ode map */
  /* ------------------------------------------------------------------------------- */
  absx11 = muDoubleScalarSin(x[2]);
  absx21 = muDoubleScalarCos(x[2]);
  st.site = &mb_emlrtRSI;
  indexShapeCheck(&st, u_size[0]);
  if (4 > u_size[0]) {
    emlrtDynamicBoundsCheckR2012b(4, 1, u_size[0], &p_emlrtBCI, sp);
  }

  if (5 > u_size[0]) {
    emlrtDynamicBoundsCheckR2012b(5, 1, 4, &p_emlrtBCI, sp);
  }

  if (6 > u_size[0]) {
    emlrtDynamicBoundsCheckR2012b(6, 1, 5, &p_emlrtBCI, sp);
  }

  /*  function thruster_configuration is used to obtain the configuration */
  /*  matrix */
  /*  initialization */
  absx31 = muDoubleScalarCos(u_data[3]);
  B[0] = absx31;
  T_tmp = muDoubleScalarSin(u_data[3]);
  B[1] = T_tmp;
  B[2] = p_ode_thrust_config[0] * T_tmp - p_ode_thrust_config[3] * absx31;
  if (*emlrtBreakCheckR2012bFlagVar != 0) {
    emlrtBreakCheckR2012b(sp);
  }

  absx31 = muDoubleScalarCos(u_data[4]);
  B[3] = absx31;
  T_tmp = muDoubleScalarSin(u_data[4]);
  B[4] = T_tmp;
  B[5] = p_ode_thrust_config[1] * T_tmp - p_ode_thrust_config[4] * absx31;
  if (*emlrtBreakCheckR2012bFlagVar != 0) {
    emlrtBreakCheckR2012b(sp);
  }

  absx31 = muDoubleScalarCos(u_data[5]);
  B[6] = absx31;
  T_tmp = muDoubleScalarSin(u_data[5]);
  B[7] = T_tmp;
  B[8] = p_ode_thrust_config[2] * T_tmp - p_ode_thrust_config[5] * absx31;
  if (*emlrtBreakCheckR2012bFlagVar != 0) {
    emlrtBreakCheckR2012b(sp);
  }

  for (i = 0; i < 6; i++) {
    xdot[i] = 0.0;
  }

  R_tmp[0] = absx21;
  R_tmp[3] = -absx11;
  R_tmp[6] = 0.0;
  R_tmp[1] = absx11;
  R_tmp[4] = absx21;
  R_tmp[7] = 0.0;
  R_tmp[2] = 0.0;
  R_tmp[5] = 0.0;
  R_tmp[8] = 1.0;
  for (p2 = 0; p2 < 3; p2++) {
    xdot[p2] = (R_tmp[p2] * x[3] + R_tmp[p2 + 3] * x[4]) + R_tmp[p2 + 6] * x[5];
  }

  st.site = &nb_emlrtRSI;
  indexShapeCheck(&st, u_size[0]);
  st.site = &nb_emlrtRSI;
  memcpy(&R_tmp[0], &p_ode_M[0], 9U * sizeof(real_T));
  i = 0;
  p2 = 3;
  p3 = 6;
  absx11 = muDoubleScalarAbs(p_ode_M[0]);
  absx21 = muDoubleScalarAbs(p_ode_M[1]);
  absx31 = muDoubleScalarAbs(p_ode_M[2]);
  if ((absx21 > absx11) && (absx21 > absx31)) {
    i = 3;
    p2 = 0;
    R_tmp[0] = p_ode_M[1];
    R_tmp[1] = p_ode_M[0];
    R_tmp[3] = p_ode_M[4];
    R_tmp[4] = p_ode_M[3];
    R_tmp[6] = p_ode_M[7];
    R_tmp[7] = p_ode_M[6];
  } else {
    if (absx31 > absx11) {
      i = 6;
      p3 = 0;
      R_tmp[0] = p_ode_M[2];
      R_tmp[2] = p_ode_M[0];
      R_tmp[3] = p_ode_M[5];
      R_tmp[5] = p_ode_M[3];
      R_tmp[6] = p_ode_M[8];
      R_tmp[8] = p_ode_M[6];
    }
  }

  R_tmp[1] /= R_tmp[0];
  R_tmp[2] /= R_tmp[0];
  R_tmp[4] -= R_tmp[1] * R_tmp[3];
  R_tmp[5] -= R_tmp[2] * R_tmp[3];
  R_tmp[7] -= R_tmp[1] * R_tmp[6];
  R_tmp[8] -= R_tmp[2] * R_tmp[6];
  if (muDoubleScalarAbs(R_tmp[5]) > muDoubleScalarAbs(R_tmp[4])) {
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    absx11 = R_tmp[1];
    R_tmp[1] = R_tmp[2];
    R_tmp[2] = absx11;
    absx11 = R_tmp[4];
    R_tmp[4] = R_tmp[5];
    R_tmp[5] = absx11;
    absx11 = R_tmp[7];
    R_tmp[7] = R_tmp[8];
    R_tmp[8] = absx11;
  }

  R_tmp[5] /= R_tmp[4];
  R_tmp[8] -= R_tmp[5] * R_tmp[7];
  absx11 = (R_tmp[5] * R_tmp[1] - R_tmp[2]) / R_tmp[8];
  absx21 = -(R_tmp[1] + R_tmp[7] * absx11) / R_tmp[4];
  a[i] = ((1.0 - R_tmp[3] * absx21) - R_tmp[6] * absx11) / R_tmp[0];
  a[i + 1] = absx21;
  a[i + 2] = absx11;
  absx11 = -R_tmp[5] / R_tmp[8];
  absx21 = (1.0 - R_tmp[7] * absx11) / R_tmp[4];
  a[p2] = -(R_tmp[3] * absx21 + R_tmp[6] * absx11) / R_tmp[0];
  a[p2 + 1] = absx21;
  a[p2 + 2] = absx11;
  absx11 = 1.0 / R_tmp[8];
  absx21 = -R_tmp[7] * absx11 / R_tmp[4];
  a[p3] = -(R_tmp[3] * absx21 + R_tmp[6] * absx11) / R_tmp[0];
  a[p3 + 1] = absx21;
  a[p3 + 2] = absx11;
  b_st.site = &pb_emlrtRSI;
  absx21 = 0.0;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 3)) {
    absx11 = (muDoubleScalarAbs(p_ode_M[3 * i]) + muDoubleScalarAbs(p_ode_M[3 *
               i + 1])) + muDoubleScalarAbs(p_ode_M[3 * i + 2]);
    if (muDoubleScalarIsNaN(absx11)) {
      absx21 = rtNaN;
      exitg1 = true;
    } else {
      if (absx11 > absx21) {
        absx21 = absx11;
      }

      i++;
    }
  }

  absx31 = 0.0;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 3)) {
    absx11 = (muDoubleScalarAbs(a[3 * i]) + muDoubleScalarAbs(a[3 * i + 1])) +
      muDoubleScalarAbs(a[3 * i + 2]);
    if (muDoubleScalarIsNaN(absx11)) {
      absx31 = rtNaN;
      exitg1 = true;
    } else {
      if (absx11 > absx31) {
        absx31 = absx11;
      }

      i++;
    }
  }

  absx11 = 1.0 / (absx21 * absx31);
  if ((absx21 == 0.0) || (absx31 == 0.0) || (absx11 == 0.0)) {
    c_st.site = &qb_emlrtRSI;
    warning(&c_st);
  } else {
    if (muDoubleScalarIsNaN(absx11) || (absx11 < 2.2204460492503131E-16)) {
      c_st.site = &rb_emlrtRSI;
      y = NULL;
      m = emlrtCreateCharArray(2, &iv[0]);
      emlrtInitCharArrayR2013a(&c_st, 6, m, &rfmt[0]);
      emlrtAssign(&y, m);
      b_y = NULL;
      m1 = emlrtCreateDoubleScalar(absx11);
      emlrtAssign(&b_y, m1);
      d_st.site = &od_emlrtRSI;
      emlrt_marshallIn(&d_st, b_sprintf(&d_st, y, b_y, &c_emlrtMCI),
                       "<output of sprintf>", str);
      c_st.site = &rb_emlrtRSI;
      b_warning(&c_st, str);
    }
  }

  for (p2 = 0; p2 < 3; p2++) {
    b_B[p2] = (B[p2] * u_data[0] + B[p2 + 3] * u_data[1]) + B[p2 + 6] * u_data[2];
  }

  R_tmp[0] = 0.0;
  R_tmp[3] = 0.0;
  R_tmp[6] = -p_ode_M[4] * x[4] - p_ode_M[7] * x[5];
  R_tmp[1] = 0.0;
  R_tmp[4] = 0.0;
  R_tmp[7] = p_ode_M[0] * x[3];
  R_tmp[2] = p_ode_M[4] * x[4] + p_ode_M[5] * x[5];
  R_tmp[5] = -p_ode_M[0] * x[3];
  R_tmp[8] = 0.0;
  absx11 = x[3];
  absx21 = x[4];
  absx31 = x[5];
  for (p2 = 0; p2 < 3; p2++) {
    c_B[p2] = (b_B[p2] * 1000.0 - ((R_tmp[p2] * absx11 + R_tmp[p2 + 3] * absx21)
                + R_tmp[p2 + 6] * absx31)) - ((p_ode_D[p2] * absx11 + p_ode_D[p2
      + 3] * absx21) + p_ode_D[p2 + 6] * absx31);
  }

  for (p2 = 0; p2 < 3; p2++) {
    xdot[p2 + 3] = (a[p2] * c_B[0] + a[p2 + 3] * c_B[1]) + a[p2 + 6] * c_B[2];
  }

  /*      return */
  /* ------------------------------------------------------------------------------- */
}

/* End of code generation (user_ode.c) */
