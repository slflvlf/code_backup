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
static emlrtRSInfo mb_emlrtRSI = { 19, /* lineNo */
  "user_ode",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ode.m"/* pathName */
};

static emlrtRSInfo nb_emlrtRSI = { 21, /* lineNo */
  "user_ode",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ode.m"/* pathName */
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

static emlrtRTEInfo h_emlrtRTEI = { 16,/* lineNo */
  13,                                  /* colNo */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ode.m"/* pName */
};

static emlrtDCInfo j_emlrtDCI = { 14,  /* lineNo */
  5,                                   /* colNo */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ode.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo k_emlrtDCI = { 14,  /* lineNo */
  5,                                   /* colNo */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ode.m",/* pName */
  4                                    /* checkKind */
};

static emlrtBCInfo p_emlrtBCI = { 1,   /* iFirst */
  12,                                  /* iLast */
  17,                                  /* lineNo */
  28,                                  /* colNo */
  "x",                                 /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo q_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  20,                                  /* lineNo */
  9,                                   /* colNo */
  "xdot",                              /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo r_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  21,                                  /* lineNo */
  9,                                   /* colNo */
  "xdot",                              /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo s_emlrtBCI = { 1,   /* iFirst */
  12,                                  /* iLast */
  18,                                  /* lineNo */
  28,                                  /* colNo */
  "x",                                 /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo t_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  19,                                  /* lineNo */
  40,                                  /* colNo */
  "u",                                 /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo u_emlrtBCI = { 1,   /* iFirst */
  12,                                  /* iLast */
  20,                                  /* lineNo */
  49,                                  /* colNo */
  "x",                                 /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo v_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  21,                                  /* lineNo */
  59,                                  /* colNo */
  "u",                                 /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo w_emlrtBCI = { 1,   /* iFirst */
  12,                                  /* iLast */
  21,                                  /* lineNo */
  97,                                  /* colNo */
  "x",                                 /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo x_emlrtBCI = { 1,   /* iFirst */
  12,                                  /* iLast */
  21,                                  /* lineNo */
  128,                                 /* colNo */
  "x",                                 /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo ib_emlrtRTEI = { 14,/* lineNo */
  5,                                   /* colNo */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ode.m"/* pName */
};

static emlrtRSInfo rd_emlrtRSI = { 53, /* lineNo */
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
              const int32_T u_size[1], real_T p_ode_Nship, const real_T p_ode_M
              [9], const real_T p_ode_D[9], const real_T p_ode_thrust_config[6],
              emxArray_real_T *xdot)
{
  real_T unnamed_idx_0;
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
  real_T R_tmp;
  real_T n1x;
  real_T C1_tmp_tmp[3];
  int32_T p2;
  int32_T p3;
  int32_T T_tmp_tmp;
  real_T B1[9];
  real_T t3;
  int32_T b_T_tmp_tmp;
  int32_T c_T_tmp_tmp;
  real_T b_unnamed_idx_0;
  real_T unnamed_idx_1;
  real_T unnamed_idx_2;
  real_T b_R_tmp[9];
  real_T b_B1[3];
  int32_T itmp;
  int32_T i2;
  int32_T i3;
  real_T b_unnamed_idx_2;
  real_T a[9];
  boolean_T exitg1;
  const mxArray *y;
  const mxArray *m;
  static const int32_T iv[2] = { 1, 6 };

  static const char_T rfmt[6] = { '%', '1', '4', '.', '6', 'e' };

  const mxArray *b_y;
  const mxArray *m1;
  char_T str[14];
  real_T c_B1[3];
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
  unnamed_idx_0 = 6.0 * p_ode_Nship;
  if (!(unnamed_idx_0 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(unnamed_idx_0, &k_emlrtDCI, sp);
  }

  if (unnamed_idx_0 != (int32_T)muDoubleScalarFloor(unnamed_idx_0)) {
    emlrtIntegerCheckR2012b(unnamed_idx_0, &j_emlrtDCI, sp);
  }

  i = xdot->size[0];
  xdot->size[0] = (int32_T)unnamed_idx_0;
  emxEnsureCapacity_real_T(sp, xdot, i, &ib_emlrtRTEI);
  unnamed_idx_0 = 6.0 * p_ode_Nship;
  if (!(unnamed_idx_0 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(unnamed_idx_0, &k_emlrtDCI, sp);
  }

  if (unnamed_idx_0 != (int32_T)muDoubleScalarFloor(unnamed_idx_0)) {
    emlrtIntegerCheckR2012b(unnamed_idx_0, &j_emlrtDCI, sp);
  }

  p1 = (int32_T)unnamed_idx_0;
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
    unnamed_idx_0 = 6.0 * (((real_T)b_i + 1.0) - 1.0);
    i1 = (int32_T)(unnamed_idx_0 + 3.0);
    if ((i1 < 1) || (i1 > 12)) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, 12, &p_emlrtBCI, sp);
    }

    t1 = x[i1 - 1];
    R_tmp = muDoubleScalarSin(t1);
    n1x = muDoubleScalarCos(t1);
    C1_tmp_tmp[0] = unnamed_idx_0 + 4.0;
    C1_tmp_tmp[1] = unnamed_idx_0 + 5.0;
    C1_tmp_tmp[2] = unnamed_idx_0 + 6.0;
    p1 = (int32_T)(unnamed_idx_0 + 4.0);
    if ((p1 < 1) || (p1 > 12)) {
      emlrtDynamicBoundsCheckR2012b(p1, 1, 12, &s_emlrtBCI, sp);
    }

    p2 = (int32_T)(unnamed_idx_0 + 5.0);
    if ((p2 < 1) || (p2 > 12)) {
      emlrtDynamicBoundsCheckR2012b(p2, 1, 12, &s_emlrtBCI, sp);
    }

    p3 = (int32_T)(unnamed_idx_0 + 6.0);
    if ((p3 < 1) || (p3 > 12)) {
      emlrtDynamicBoundsCheckR2012b(p3, 1, 12, &s_emlrtBCI, sp);
    }

    st.site = &mb_emlrtRSI;
    indexShapeCheck(&st, u_size[0]);
    if (p1 > u) {
      emlrtDynamicBoundsCheckR2012b(p1, 1, u, &t_emlrtBCI, sp);
    }

    if (p2 > u) {
      emlrtDynamicBoundsCheckR2012b(p2, 1, u, &t_emlrtBCI, sp);
    }

    if (p3 > u) {
      emlrtDynamicBoundsCheckR2012b(p3, 1, u, &t_emlrtBCI, sp);
    }

    /*  function thruster_configuration is used to obtain the configuration */
    /*  matrix */
    /*  initialization */
    T_tmp_tmp = (int8_T)(unnamed_idx_0 + 4.0) - 1;
    t1 = muDoubleScalarCos(u_data[T_tmp_tmp]);
    B1[0] = t1;
    t3 = muDoubleScalarSin(u_data[T_tmp_tmp]);
    B1[1] = t3;
    B1[2] = p_ode_thrust_config[0] * t3 - p_ode_thrust_config[3] * t1;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }

    b_T_tmp_tmp = (int8_T)(unnamed_idx_0 + 5.0) - 1;
    t1 = muDoubleScalarCos(u_data[b_T_tmp_tmp]);
    B1[3] = t1;
    t3 = muDoubleScalarSin(u_data[b_T_tmp_tmp]);
    B1[4] = t3;
    B1[5] = p_ode_thrust_config[1] * t3 - p_ode_thrust_config[4] * t1;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }

    c_T_tmp_tmp = (int8_T)(unnamed_idx_0 + 6.0) - 1;
    t1 = muDoubleScalarCos(u_data[c_T_tmp_tmp]);
    B1[6] = t1;
    t3 = muDoubleScalarSin(u_data[c_T_tmp_tmp]);
    B1[7] = t3;
    B1[8] = p_ode_thrust_config[2] * t3 - p_ode_thrust_config[5] * t1;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }

    if ((p1 < 1) || (p1 > 12)) {
      emlrtDynamicBoundsCheckR2012b(p1, 1, 12, &u_emlrtBCI, sp);
    }

    b_unnamed_idx_0 = x[p1 - 1];
    if ((p2 < 1) || (p2 > 12)) {
      emlrtDynamicBoundsCheckR2012b(p2, 1, 12, &u_emlrtBCI, sp);
    }

    unnamed_idx_1 = x[p2 - 1];
    if ((p3 < 1) || (p3 > 12)) {
      emlrtDynamicBoundsCheckR2012b(p3, 1, 12, &u_emlrtBCI, sp);
    }

    unnamed_idx_2 = x[p3 - 1];
    b_R_tmp[0] = n1x;
    b_R_tmp[3] = -R_tmp;
    b_R_tmp[6] = 0.0;
    b_R_tmp[1] = R_tmp;
    b_R_tmp[4] = n1x;
    b_R_tmp[7] = 0.0;
    b_B1[0] = unnamed_idx_0 + 1.0;
    b_R_tmp[2] = 0.0;
    b_B1[1] = unnamed_idx_0 + 2.0;
    b_R_tmp[5] = 0.0;
    b_B1[2] = unnamed_idx_0 + 3.0;
    b_R_tmp[8] = 1.0;
    itmp = xdot->size[0];
    for (i2 = 0; i2 < 3; i2++) {
      i3 = (int32_T)b_B1[i2];
      if ((i3 < 1) || (i3 > itmp)) {
        emlrtDynamicBoundsCheckR2012b(i3, 1, itmp, &q_emlrtBCI, sp);
      }

      xdot->data[i3 - 1] = 0.0;
      if (i3 > itmp) {
        emlrtDynamicBoundsCheckR2012b(i3, 1, itmp, &q_emlrtBCI, sp);
      }

      xdot->data[i3 - 1] += b_R_tmp[i2] * b_unnamed_idx_0;
      if (i3 > itmp) {
        emlrtDynamicBoundsCheckR2012b(i3, 1, itmp, &q_emlrtBCI, sp);
      }

      xdot->data[i3 - 1] += b_R_tmp[i2 + 3] * unnamed_idx_1;
      if (i3 > itmp) {
        emlrtDynamicBoundsCheckR2012b(i3, 1, itmp, &q_emlrtBCI, sp);
      }

      xdot->data[i3 - 1] += b_R_tmp[i2 + 6] * unnamed_idx_2;
    }

    st.site = &nb_emlrtRSI;
    indexShapeCheck(&st, u_size[0]);
    i2 = (int32_T)(unnamed_idx_0 + 1.0);
    if ((i2 < 1) || (i2 > b_u)) {
      emlrtDynamicBoundsCheckR2012b(i2, 1, b_u, &v_emlrtBCI, sp);
    }

    i3 = (int32_T)(unnamed_idx_0 + 2.0);
    if ((i3 < 1) || (i3 > b_u)) {
      emlrtDynamicBoundsCheckR2012b(i3, 1, b_u, &v_emlrtBCI, sp);
    }

    if ((i1 < 1) || (i1 > b_u)) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, b_u, &v_emlrtBCI, sp);
    }

    if ((p1 < 1) || (p1 > 12)) {
      emlrtDynamicBoundsCheckR2012b(p1, 1, 12, &w_emlrtBCI, sp);
    }

    b_unnamed_idx_0 = x[p1 - 1];
    if ((p2 < 1) || (p2 > 12)) {
      emlrtDynamicBoundsCheckR2012b(p2, 1, 12, &w_emlrtBCI, sp);
    }

    unnamed_idx_1 = x[p2 - 1];
    if ((p3 < 1) || (p3 > 12)) {
      emlrtDynamicBoundsCheckR2012b(p3, 1, 12, &w_emlrtBCI, sp);
    }

    unnamed_idx_2 = x[p3 - 1];
    if ((p1 < 1) || (p1 > 12)) {
      emlrtDynamicBoundsCheckR2012b(p1, 1, 12, &x_emlrtBCI, sp);
    }

    unnamed_idx_0 = x[p1 - 1];
    if ((p2 < 1) || (p2 > 12)) {
      emlrtDynamicBoundsCheckR2012b(p2, 1, 12, &x_emlrtBCI, sp);
    }

    R_tmp = x[p2 - 1];
    if ((p3 < 1) || (p3 > 12)) {
      emlrtDynamicBoundsCheckR2012b(p3, 1, 12, &x_emlrtBCI, sp);
    }

    b_unnamed_idx_2 = x[p3 - 1];
    st.site = &nb_emlrtRSI;
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
    t3 = (b_R_tmp[5] * b_R_tmp[1] - b_R_tmp[2]) / b_R_tmp[8];
    t1 = -(b_R_tmp[1] + b_R_tmp[7] * t3) / b_R_tmp[4];
    a[p1] = ((1.0 - b_R_tmp[3] * t1) - b_R_tmp[6] * t3) / b_R_tmp[0];
    a[p1 + 1] = t1;
    a[p1 + 2] = t3;
    t3 = -b_R_tmp[5] / b_R_tmp[8];
    t1 = (1.0 - b_R_tmp[7] * t3) / b_R_tmp[4];
    a[p2] = -(b_R_tmp[3] * t1 + b_R_tmp[6] * t3) / b_R_tmp[0];
    a[p2 + 1] = t1;
    a[p2 + 2] = t3;
    t3 = 1.0 / b_R_tmp[8];
    t1 = -b_R_tmp[7] * t3 / b_R_tmp[4];
    a[p3] = -(b_R_tmp[3] * t1 + b_R_tmp[6] * t3) / b_R_tmp[0];
    a[p3 + 1] = t1;
    a[p3 + 2] = t3;
    b_st.site = &pb_emlrtRSI;
    n1x = 0.0;
    p1 = 0;
    exitg1 = false;
    while ((!exitg1) && (p1 < 3)) {
      t1 = (muDoubleScalarAbs(p_ode_M[3 * p1]) + muDoubleScalarAbs(p_ode_M[3 *
             p1 + 1])) + muDoubleScalarAbs(p_ode_M[3 * p1 + 2]);
      if (muDoubleScalarIsNaN(t1)) {
        n1x = rtNaN;
        exitg1 = true;
      } else {
        if (t1 > n1x) {
          n1x = t1;
        }

        p1++;
      }
    }

    t3 = 0.0;
    p1 = 0;
    exitg1 = false;
    while ((!exitg1) && (p1 < 3)) {
      t1 = (muDoubleScalarAbs(a[3 * p1]) + muDoubleScalarAbs(a[3 * p1 + 1])) +
        muDoubleScalarAbs(a[3 * p1 + 2]);
      if (muDoubleScalarIsNaN(t1)) {
        t3 = rtNaN;
        exitg1 = true;
      } else {
        if (t1 > t3) {
          t3 = t1;
        }

        p1++;
      }
    }

    t1 = 1.0 / (n1x * t3);
    if ((n1x == 0.0) || (t3 == 0.0) || (t1 == 0.0)) {
      c_st.site = &qb_emlrtRSI;
      warning(&c_st);
    } else {
      if (muDoubleScalarIsNaN(t1) || (t1 < 2.2204460492503131E-16)) {
        c_st.site = &rb_emlrtRSI;
        y = NULL;
        m = emlrtCreateCharArray(2, &iv[0]);
        emlrtInitCharArrayR2013a(&c_st, 6, m, &rfmt[0]);
        emlrtAssign(&y, m);
        b_y = NULL;
        m1 = emlrtCreateDoubleScalar(t1);
        emlrtAssign(&b_y, m1);
        d_st.site = &rd_emlrtRSI;
        emlrt_marshallIn(&d_st, b_sprintf(&d_st, y, b_y, &c_emlrtMCI),
                         "<output of sprintf>", str);
        c_st.site = &rb_emlrtRSI;
        b_warning(&c_st, str);
      }
    }

    t1 = u_data[i2 - 1];
    n1x = u_data[i3 - 1];
    t3 = u_data[i1 - 1];
    for (i1 = 0; i1 < 3; i1++) {
      b_B1[i1] = (B1[i1] * t1 + B1[i1 + 3] * n1x) + B1[i1 + 6] * t3;
    }

    b_R_tmp[0] = 0.0;
    b_R_tmp[3] = 0.0;
    b_R_tmp[6] = -p_ode_M[4] * x[b_T_tmp_tmp] - p_ode_M[7] * x[c_T_tmp_tmp];
    b_R_tmp[1] = 0.0;
    b_R_tmp[4] = 0.0;
    b_R_tmp[7] = p_ode_M[0] * x[T_tmp_tmp];
    b_R_tmp[2] = p_ode_M[4] * x[b_T_tmp_tmp] + p_ode_M[5] * x[c_T_tmp_tmp];
    b_R_tmp[5] = -p_ode_M[0] * x[T_tmp_tmp];
    b_R_tmp[8] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      c_B1[i1] = (b_B1[i1] * 1000.0 - ((b_R_tmp[i1] * b_unnamed_idx_0 +
        b_R_tmp[i1 + 3] * unnamed_idx_1) + b_R_tmp[i1 + 6] * unnamed_idx_2)) -
        ((p_ode_D[i1] * unnamed_idx_0 + p_ode_D[i1 + 3] * R_tmp) + p_ode_D[i1 +
         6] * b_unnamed_idx_2);
    }

    itmp = xdot->size[0];
    for (i1 = 0; i1 < 3; i1++) {
      p1 = (int32_T)C1_tmp_tmp[i1];
      if ((p1 < 1) || (p1 > itmp)) {
        emlrtDynamicBoundsCheckR2012b(p1, 1, itmp, &r_emlrtBCI, sp);
      }

      xdot->data[p1 - 1] = 0.0;
      if (p1 > itmp) {
        emlrtDynamicBoundsCheckR2012b(p1, 1, itmp, &r_emlrtBCI, sp);
      }

      xdot->data[p1 - 1] += a[i1] * c_B1[0];
      if (p1 > itmp) {
        emlrtDynamicBoundsCheckR2012b(p1, 1, itmp, &r_emlrtBCI, sp);
      }

      xdot->data[p1 - 1] += a[i1 + 3] * c_B1[1];
      if (p1 > itmp) {
        emlrtDynamicBoundsCheckR2012b(p1, 1, itmp, &r_emlrtBCI, sp);
      }

      xdot->data[p1 - 1] += a[i1 + 6] * c_B1[2];
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
  /*      % ´¬2 */
  /*      R2 = rotate_matrix(x(9)); */
  /*      C2 = C_matrix(M, x(10:12)); */
  /*      B2 = thrusters_configuration(u(10:12), thrust_config); */
  /*      xdot(7:9, 1) = R2 * x(4:6); */
  /*      xdot(10:12, 1) = inv(M) * (B2 * u(7:9) * 1e3 - C2 * x(10:12) - D * x(10:12)); */
  /*      return */
  /* ------------------------------------------------------------------------------- */
}

/* End of code generation (user_ode.c) */
