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
#include "update_mv_emxutil.h"
#include "warning.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo mb_emlrtRSI = { 22, /* lineNo */
  "user_ode",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ode.m"/* pathName */
};

static emlrtRSInfo nb_emlrtRSI = { 23, /* lineNo */
  "user_ode",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ode.m"/* pathName */
};

static emlrtRSInfo ob_emlrtRSI = { 34, /* lineNo */
  "user_ode",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ode.m"/* pathName */
};

static emlrtRSInfo qb_emlrtRSI = { 31, /* lineNo */
  "inv",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\matfun\\inv.m"/* pathName */
};

static emlrtRSInfo rb_emlrtRSI = { 42, /* lineNo */
  "checkcond",                         /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\matfun\\inv.m"/* pathName */
};

static emlrtRSInfo sb_emlrtRSI = { 46, /* lineNo */
  "checkcond",                         /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\matfun\\inv.m"/* pathName */
};

static emlrtMCInfo c_emlrtMCI = { 53,  /* lineNo */
  19,                                  /* colNo */
  "flt2str",                           /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\flt2str.m"/* pName */
};

static emlrtRTEInfo h_emlrtRTEI = { 18,/* lineNo */
  13,                                  /* colNo */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ode.m"/* pName */
};

static emlrtBCInfo p_emlrtBCI = { 1,   /* iFirst */
  12,                                  /* iLast */
  20,                                  /* lineNo */
  15,                                  /* colNo */
  "x",                                 /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo j_emlrtDCI = { 14,  /* lineNo */
  5,                                   /* colNo */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ode.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo k_emlrtDCI = { 14,  /* lineNo */
  5,                                   /* colNo */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ode.m",/* pName */
  4                                    /* checkKind */
};

static emlrtBCInfo q_emlrtBCI = { 1,   /* iFirst */
  12,                                  /* iLast */
  19,                                  /* lineNo */
  15,                                  /* colNo */
  "x",                                 /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo r_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  33,                                  /* lineNo */
  9,                                   /* colNo */
  "xdot",                              /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo s_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  34,                                  /* lineNo */
  9,                                   /* colNo */
  "xdot",                              /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo t_emlrtBCI = { 1,   /* iFirst */
  12,                                  /* iLast */
  21,                                  /* lineNo */
  15,                                  /* colNo */
  "x",                                 /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo u_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  22,                                  /* lineNo */
  16,                                  /* colNo */
  "u",                                 /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo v_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  23,                                  /* lineNo */
  16,                                  /* colNo */
  "u",                                 /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo w_emlrtBCI = { 1,   /* iFirst */
  12,                                  /* iLast */
  24,                                  /* lineNo */
  25,                                  /* colNo */
  "u_last",                            /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo x_emlrtBCI = { 1,   /* iFirst */
  12,                                  /* iLast */
  25,                                  /* lineNo */
  25,                                  /* colNo */
  "u_last",                            /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo ib_emlrtRTEI = { 14,/* lineNo */
  5,                                   /* colNo */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ode.m"/* pName */
};

static emlrtRSInfo td_emlrtRSI = { 53, /* lineNo */
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

void user_ode(const emlrtStack *sp, const real_T x[12], const real_T u_data[],
              const int32_T u_size[1], real_T p_ode_Nship, const real_T
              p_ode_u_last[12], const real_T p_ode_M[9], const real_T p_ode_D[9],
              const real_T p_ode_thrust_config[6], emxArray_real_T *xdot)
{
  real_T d;
  int32_T i;
  int32_T p1;
  int32_T u;
  int32_T b_i;
  int32_T b_u;
  real_T absx11;
  real_T absx21;
  int32_T i1;
  real_T absx31;
  real_T t1;
  int32_T i2;
  real_T v_tmp[3];
  int32_T p3;
  real_T v_idx_0;
  int32_T p2;
  real_T v_idx_1;
  real_T v_idx_2;
  real_T df_tmp[3];
  int32_T i3;
  real_T a[3];
  real_T R_tmp;
  real_T t2;
  real_T B1[9];
  real_T n1xinv;
  real_T b_R_tmp[9];
  int32_T itmp;
  real_T b_a[9];
  boolean_T exitg1;
  const mxArray *y;
  const mxArray *m;
  static const int32_T iv[2] = { 1, 6 };

  static const char_T rfmt[6] = { '%', '1', '4', '.', '6', 'e' };

  const mxArray *b_y;
  const mxArray *m1;
  char_T str[14];
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
  /*  ´¬µÄ¸öÊý */
  d = 6.0 * p_ode_Nship;
  if (!(d >= 0.0)) {
    emlrtNonNegativeCheckR2012b(d, &k_emlrtDCI, sp);
  }

  if (d != (int32_T)muDoubleScalarFloor(d)) {
    emlrtIntegerCheckR2012b(d, &j_emlrtDCI, sp);
  }

  i = xdot->size[0];
  xdot->size[0] = (int32_T)d;
  emxEnsureCapacity_real_T(sp, xdot, i, &ib_emlrtRTEI);
  d = 6.0 * p_ode_Nship;
  if (!(d >= 0.0)) {
    emlrtNonNegativeCheckR2012b(d, &k_emlrtDCI, sp);
  }

  if (d != (int32_T)muDoubleScalarFloor(d)) {
    emlrtIntegerCheckR2012b(d, &j_emlrtDCI, sp);
  }

  p1 = (int32_T)d;
  for (i = 0; i < p1; i++) {
    xdot->data[i] = 0.0;
  }

  i = (int32_T)p_ode_Nship;
  emlrtForLoopVectorCheckR2012b(1.0, 1.0, p_ode_Nship, mxDOUBLE_CLASS, (int32_T)
    p_ode_Nship, &h_emlrtRTEI, sp);
  if (0 <= i - 1) {
    u = u_size[0];
    b_u = u_size[0];
    absx11 = muDoubleScalarAbs(p_ode_M[0]);
    absx21 = muDoubleScalarAbs(p_ode_M[1]);
    absx31 = muDoubleScalarAbs(p_ode_M[2]);
  }

  for (b_i = 0; b_i < i; b_i++) {
    d = 6.0 * (((real_T)b_i + 1.0) - 1.0);
    i1 = (int32_T)(d + 3.0);
    if ((i1 < 1) || (i1 > 12)) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, 12, &q_emlrtBCI, sp);
    }

    t1 = x[i1 - 1];
    i2 = (int32_T)(d + 1.0);
    if ((i2 < 1) || (i2 > 12)) {
      emlrtDynamicBoundsCheckR2012b(i2, 1, 12, &p_emlrtBCI, sp);
    }

    v_tmp[0] = d + 4.0;
    v_tmp[1] = d + 5.0;
    v_tmp[2] = d + 6.0;
    p3 = (int32_T)(d + 4.0);
    if ((p3 < 1) || (p3 > 12)) {
      emlrtDynamicBoundsCheckR2012b(p3, 1, 12, &t_emlrtBCI, sp);
    }

    v_idx_0 = x[p3 - 1];
    p2 = (int32_T)(d + 5.0);
    if ((p2 < 1) || (p2 > 12)) {
      emlrtDynamicBoundsCheckR2012b(p2, 1, 12, &t_emlrtBCI, sp);
    }

    v_idx_1 = x[p2 - 1];
    p1 = (int32_T)(d + 6.0);
    if ((p1 < 1) || (p1 > 12)) {
      emlrtDynamicBoundsCheckR2012b(p1, 1, 12, &t_emlrtBCI, sp);
    }

    v_idx_2 = x[p1 - 1];
    st.site = &mb_emlrtRSI;
    indexShapeCheck(&st, u_size[0]);
    df_tmp[0] = d + 1.0;
    df_tmp[1] = d + 2.0;
    df_tmp[2] = d + 3.0;
    if ((i2 < 1) || (i2 > u)) {
      emlrtDynamicBoundsCheckR2012b(i2, 1, u, &u_emlrtBCI, sp);
    }

    i3 = (int32_T)(d + 2.0);
    if ((i3 < 1) || (i3 > u)) {
      emlrtDynamicBoundsCheckR2012b(i3, 1, u, &u_emlrtBCI, sp);
    }

    if ((i1 < 1) || (i1 > u)) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, u, &u_emlrtBCI, sp);
    }

    st.site = &nb_emlrtRSI;
    indexShapeCheck(&st, u_size[0]);
    if (p3 > b_u) {
      emlrtDynamicBoundsCheckR2012b(p3, 1, b_u, &v_emlrtBCI, sp);
    }

    if (p2 > b_u) {
      emlrtDynamicBoundsCheckR2012b(p2, 1, b_u, &v_emlrtBCI, sp);
    }

    if (p1 > b_u) {
      emlrtDynamicBoundsCheckR2012b(p1, 1, b_u, &v_emlrtBCI, sp);
    }

    if (i2 > 12) {
      emlrtDynamicBoundsCheckR2012b(i2, 1, 12, &w_emlrtBCI, sp);
    }

    if (i3 > 12) {
      emlrtDynamicBoundsCheckR2012b(i3, 1, 12, &w_emlrtBCI, sp);
    }

    if (i1 > 12) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, 12, &w_emlrtBCI, sp);
    }

    if ((p3 < 1) || (p3 > 12)) {
      emlrtDynamicBoundsCheckR2012b(p3, 1, 12, &x_emlrtBCI, sp);
    }

    if ((p2 < 1) || (p2 > 12)) {
      emlrtDynamicBoundsCheckR2012b(p2, 1, 12, &x_emlrtBCI, sp);
    }

    if ((p1 < 1) || (p1 > 12)) {
      emlrtDynamicBoundsCheckR2012b(p1, 1, 12, &x_emlrtBCI, sp);
    }

    a[0] = p_ode_u_last[p3 - 1] + u_data[(int32_T)(d + 4.0) - 1];
    a[1] = p_ode_u_last[p2 - 1] + u_data[(int32_T)(d + 5.0) - 1];
    a[2] = p_ode_u_last[p1 - 1] + u_data[(int32_T)(d + 6.0) - 1];
    R_tmp = muDoubleScalarSin(t1);
    t1 = muDoubleScalarCos(t1);

    /*  function thruster_configuration is used to obtain the configuration */
    /*  matrix */
    /*  initialization */
    t2 = muDoubleScalarCos(a[0]);
    B1[0] = t2;
    n1xinv = muDoubleScalarSin(a[0]);
    B1[1] = n1xinv;
    B1[2] = p_ode_thrust_config[0] * n1xinv - p_ode_thrust_config[3] * t2;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }

    t2 = muDoubleScalarCos(a[1]);
    B1[3] = t2;
    n1xinv = muDoubleScalarSin(a[1]);
    B1[4] = n1xinv;
    B1[5] = p_ode_thrust_config[1] * n1xinv - p_ode_thrust_config[4] * t2;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }

    t2 = muDoubleScalarCos(a[2]);
    B1[6] = t2;
    n1xinv = muDoubleScalarSin(a[2]);
    B1[7] = n1xinv;
    B1[8] = p_ode_thrust_config[2] * n1xinv - p_ode_thrust_config[5] * t2;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }

    b_R_tmp[0] = t1;
    b_R_tmp[3] = -R_tmp;
    b_R_tmp[6] = 0.0;
    b_R_tmp[1] = R_tmp;
    b_R_tmp[4] = t1;
    b_R_tmp[7] = 0.0;
    b_R_tmp[2] = 0.0;
    b_R_tmp[5] = 0.0;
    b_R_tmp[8] = 1.0;
    p1 = xdot->size[0];
    for (p3 = 0; p3 < 3; p3++) {
      p2 = (int32_T)df_tmp[p3];
      if ((p2 < 1) || (p2 > p1)) {
        emlrtDynamicBoundsCheckR2012b(p2, 1, p1, &r_emlrtBCI, sp);
      }

      xdot->data[p2 - 1] = 0.0;
      if (p2 > p1) {
        emlrtDynamicBoundsCheckR2012b(p2, 1, p1, &r_emlrtBCI, sp);
      }

      xdot->data[p2 - 1] += b_R_tmp[p3] * v_idx_0;
      if (p2 > p1) {
        emlrtDynamicBoundsCheckR2012b(p2, 1, p1, &r_emlrtBCI, sp);
      }

      xdot->data[p2 - 1] += b_R_tmp[p3 + 3] * v_idx_1;
      if (p2 > p1) {
        emlrtDynamicBoundsCheckR2012b(p2, 1, p1, &r_emlrtBCI, sp);
      }

      xdot->data[p2 - 1] += b_R_tmp[p3 + 6] * v_idx_2;
    }

    st.site = &ob_emlrtRSI;
    memcpy(&b_R_tmp[0], &p_ode_M[0], 9U * sizeof(real_T));
    p1 = 0;
    p2 = 3;
    p3 = 6;
    if ((absx21 > absx11) && (absx21 > absx31)) {
      p1 = 3;
      p2 = 0;
      b_R_tmp[0] = p_ode_M[1];
      b_R_tmp[1] = p_ode_M[0];
      b_R_tmp[3] = p_ode_M[4];
      b_R_tmp[4] = p_ode_M[3];
      b_R_tmp[6] = p_ode_M[7];
      b_R_tmp[7] = p_ode_M[6];
    } else {
      if (absx31 > absx11) {
        p1 = 6;
        p3 = 0;
        b_R_tmp[0] = p_ode_M[2];
        b_R_tmp[2] = p_ode_M[0];
        b_R_tmp[3] = p_ode_M[5];
        b_R_tmp[5] = p_ode_M[3];
        b_R_tmp[6] = p_ode_M[8];
        b_R_tmp[8] = p_ode_M[6];
      }
    }

    b_R_tmp[1] /= b_R_tmp[0];
    b_R_tmp[2] /= b_R_tmp[0];
    b_R_tmp[4] -= b_R_tmp[1] * b_R_tmp[3];
    b_R_tmp[5] -= b_R_tmp[2] * b_R_tmp[3];
    b_R_tmp[7] -= b_R_tmp[1] * b_R_tmp[6];
    b_R_tmp[8] -= b_R_tmp[2] * b_R_tmp[6];
    if (muDoubleScalarAbs(b_R_tmp[5]) > muDoubleScalarAbs(b_R_tmp[4])) {
      itmp = p2;
      p2 = p3;
      p3 = itmp;
      t1 = b_R_tmp[1];
      b_R_tmp[1] = b_R_tmp[2];
      b_R_tmp[2] = t1;
      t1 = b_R_tmp[4];
      b_R_tmp[4] = b_R_tmp[5];
      b_R_tmp[5] = t1;
      t1 = b_R_tmp[7];
      b_R_tmp[7] = b_R_tmp[8];
      b_R_tmp[8] = t1;
    }

    b_R_tmp[5] /= b_R_tmp[4];
    b_R_tmp[8] -= b_R_tmp[5] * b_R_tmp[7];
    t1 = (b_R_tmp[5] * b_R_tmp[1] - b_R_tmp[2]) / b_R_tmp[8];
    t2 = -(b_R_tmp[1] + b_R_tmp[7] * t1) / b_R_tmp[4];
    b_a[p1] = ((1.0 - b_R_tmp[3] * t2) - b_R_tmp[6] * t1) / b_R_tmp[0];
    b_a[p1 + 1] = t2;
    b_a[p1 + 2] = t1;
    t1 = -b_R_tmp[5] / b_R_tmp[8];
    t2 = (1.0 - b_R_tmp[7] * t1) / b_R_tmp[4];
    b_a[p2] = -(b_R_tmp[3] * t2 + b_R_tmp[6] * t1) / b_R_tmp[0];
    b_a[p2 + 1] = t2;
    b_a[p2 + 2] = t1;
    t1 = 1.0 / b_R_tmp[8];
    t2 = -b_R_tmp[7] * t1 / b_R_tmp[4];
    b_a[p3] = -(b_R_tmp[3] * t2 + b_R_tmp[6] * t1) / b_R_tmp[0];
    b_a[p3 + 1] = t2;
    b_a[p3 + 2] = t1;
    b_st.site = &qb_emlrtRSI;
    t2 = 0.0;
    p1 = 0;
    exitg1 = false;
    while ((!exitg1) && (p1 < 3)) {
      t1 = (muDoubleScalarAbs(p_ode_M[3 * p1]) + muDoubleScalarAbs(p_ode_M[3 *
             p1 + 1])) + muDoubleScalarAbs(p_ode_M[3 * p1 + 2]);
      if (muDoubleScalarIsNaN(t1)) {
        t2 = rtNaN;
        exitg1 = true;
      } else {
        if (t1 > t2) {
          t2 = t1;
        }

        p1++;
      }
    }

    n1xinv = 0.0;
    p1 = 0;
    exitg1 = false;
    while ((!exitg1) && (p1 < 3)) {
      t1 = (muDoubleScalarAbs(b_a[3 * p1]) + muDoubleScalarAbs(b_a[3 * p1 + 1]))
        + muDoubleScalarAbs(b_a[3 * p1 + 2]);
      if (muDoubleScalarIsNaN(t1)) {
        n1xinv = rtNaN;
        exitg1 = true;
      } else {
        if (t1 > n1xinv) {
          n1xinv = t1;
        }

        p1++;
      }
    }

    t1 = 1.0 / (t2 * n1xinv);
    if ((t2 == 0.0) || (n1xinv == 0.0) || (t1 == 0.0)) {
      c_st.site = &rb_emlrtRSI;
      warning(&c_st);
    } else {
      if (muDoubleScalarIsNaN(t1) || (t1 < 2.2204460492503131E-16)) {
        c_st.site = &sb_emlrtRSI;
        y = NULL;
        m = emlrtCreateCharArray(2, &iv[0]);
        emlrtInitCharArrayR2013a(&c_st, 6, m, &rfmt[0]);
        emlrtAssign(&y, m);
        b_y = NULL;
        m1 = emlrtCreateDoubleScalar(t1);
        emlrtAssign(&b_y, m1);
        d_st.site = &td_emlrtRSI;
        emlrt_marshallIn(&d_st, b_sprintf(&d_st, y, b_y, &c_emlrtMCI),
                         "<output of sprintf>", str);
        c_st.site = &sb_emlrtRSI;
        b_warning(&c_st, str);
      }
    }

    p1 = i2 - 1;
    t1 = p_ode_u_last[p1] + u_data[p1];
    p1 = i3 - 1;
    t2 = p_ode_u_last[p1] + u_data[p1];
    p1 = i1 - 1;
    n1xinv = p_ode_u_last[p1] + u_data[p1];
    for (i1 = 0; i1 < 3; i1++) {
      a[i1] = (B1[i1] * t1 + B1[i1 + 3] * t2) + B1[i1 + 6] * n1xinv;
    }

    b_R_tmp[0] = 0.0;
    b_R_tmp[3] = 0.0;
    p1 = (int8_T)(d + 5.0) - 1;
    p2 = (int8_T)(d + 6.0) - 1;
    b_R_tmp[6] = -p_ode_M[4] * x[p1] - p_ode_M[7] * x[p2];
    b_R_tmp[1] = 0.0;
    b_R_tmp[4] = 0.0;
    p3 = (int8_T)(d + 4.0) - 1;
    b_R_tmp[7] = p_ode_M[0] * x[p3];
    b_R_tmp[2] = p_ode_M[4] * x[p1] + p_ode_M[5] * x[p2];
    b_R_tmp[5] = -p_ode_M[0] * x[p3];
    b_R_tmp[8] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      df_tmp[i1] = (a[i1] * 1000.0 - ((b_R_tmp[i1] * v_idx_0 + b_R_tmp[i1 + 3] *
        v_idx_1) + b_R_tmp[i1 + 6] * v_idx_2)) - ((p_ode_D[i1] * v_idx_0 +
        p_ode_D[i1 + 3] * v_idx_1) + p_ode_D[i1 + 6] * v_idx_2);
    }

    p1 = xdot->size[0];
    for (i1 = 0; i1 < 3; i1++) {
      i2 = (int32_T)v_tmp[i1];
      if ((i2 < 1) || (i2 > p1)) {
        emlrtDynamicBoundsCheckR2012b(i2, 1, p1, &s_emlrtBCI, sp);
      }

      xdot->data[i2 - 1] = 0.0;
      if (i2 > p1) {
        emlrtDynamicBoundsCheckR2012b(i2, 1, p1, &s_emlrtBCI, sp);
      }

      xdot->data[i2 - 1] += b_a[i1] * df_tmp[0];
      if (i2 > p1) {
        emlrtDynamicBoundsCheckR2012b(i2, 1, p1, &s_emlrtBCI, sp);
      }

      xdot->data[i2 - 1] += b_a[i1 + 3] * df_tmp[1];
      if (i2 > p1) {
        emlrtDynamicBoundsCheckR2012b(i2, 1, p1, &s_emlrtBCI, sp);
      }

      xdot->data[i2 - 1] += b_a[i1 + 6] * df_tmp[2];
    }

    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }

  /*      R1 = rotate_matrix(x(3)); */
  /*      C1 = C_matrix(M, x(4:6)); */
  /*      B1 = thrusters_configuration(u(4:6), thrust_config); */
  /*      xdot(1:3, 1) = R1 * x(4:6); */
  /*      xdot(4:6, 1) = inv(M) * (B1 * u(1:3) * 1e3 - C1 * x(4:6) - D * x(4:6)); */
  /*       */
  /*      ´¬2 */
  /*      R2 = rotate_matrix(x(9)); */
  /*      C2 = C_matrix(M, x(10:12)); */
  /*      B2 = thrusters_configuration(u(10:12), thrust_config); */
  /*      xdot(7:9, 1) = R2 * x(4:6); */
  /*      xdot(10:12, 1) = inv(M) * (B2 * u(7:9) * 1e3 - C2 * x(10:12) - D * x(10:12)); */
  /*      return */
  /* ------------------------------------------------------------------------------- */
}

/* End of code generation (user_ode.c) */
