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
#include "BBS.h"
#include "BBS_data.h"
#include "BBS_emxutil.h"
#include "BBS_mexutil.h"
#include "indexShapeCheck.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "warning.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo r_emlrtRSI = { 19,  /* lineNo */
  "user_ode",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_4thrust\\user_ode.m"/* pathName */
};

static emlrtRSInfo s_emlrtRSI = { 20,  /* lineNo */
  "user_ode",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_4thrust\\user_ode.m"/* pathName */
};

static emlrtRSInfo t_emlrtRSI = { 30,  /* lineNo */
  "user_ode",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_4thrust\\user_ode.m"/* pathName */
};

static emlrtRSInfo v_emlrtRSI = { 31,  /* lineNo */
  "inv",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\matfun\\inv.m"/* pathName */
};

static emlrtRSInfo w_emlrtRSI = { 42,  /* lineNo */
  "checkcond",                         /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\matfun\\inv.m"/* pathName */
};

static emlrtRSInfo x_emlrtRSI = { 46,  /* lineNo */
  "checkcond",                         /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\matfun\\inv.m"/* pathName */
};

static emlrtMCInfo c_emlrtMCI = { 53,  /* lineNo */
  19,                                  /* colNo */
  "flt2str",                           /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\flt2str.m"/* pName */
};

static emlrtRTEInfo g_emlrtRTEI = { 15,/* lineNo */
  13,                                  /* colNo */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_4thrust\\user_ode.m"/* pName */
};

static emlrtDCInfo f_emlrtDCI = { 13,  /* lineNo */
  5,                                   /* colNo */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_4thrust\\user_ode.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo g_emlrtDCI = { 13,  /* lineNo */
  5,                                   /* colNo */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_4thrust\\user_ode.m",/* pName */
  4                                    /* checkKind */
};

static emlrtBCInfo f_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  29,                                  /* lineNo */
  9,                                   /* colNo */
  "xdot",                              /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_4thrust\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo g_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  30,                                  /* lineNo */
  9,                                   /* colNo */
  "xdot",                              /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_4thrust\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo h_emlrtBCI = { 1,   /* iFirst */
  12,                                  /* iLast */
  17,                                  /* lineNo */
  15,                                  /* colNo */
  "x",                                 /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_4thrust\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo i_emlrtBCI = { 1,   /* iFirst */
  12,                                  /* iLast */
  18,                                  /* lineNo */
  15,                                  /* colNo */
  "x",                                 /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_4thrust\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo j_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  19,                                  /* lineNo */
  15,                                  /* colNo */
  "u",                                 /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_4thrust\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo k_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  20,                                  /* lineNo */
  15,                                  /* colNo */
  "u",                                 /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_4thrust\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo cb_emlrtRTEI = { 13,/* lineNo */
  5,                                   /* colNo */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_4thrust\\user_ode.m"/* pName */
};

static emlrtRSInfo ub_emlrtRSI = { 53, /* lineNo */
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
static void w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, char_T ret[14]);

/* Function Definitions */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, char_T y[14])
{
  w_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
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

static void w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, char_T ret[14])
{
  static const int32_T dims[2] = { 1, 14 };

  emlrtCheckBuiltInR2012b(sp, msgId, src, "char", false, 2U, dims);
  emlrtImportCharArrayR2015b(sp, src, &ret[0], 14);
  emlrtDestroyArray(&src);
}

void user_ode(const emlrtStack *sp, const real_T x[12], const real_T u_data[],
              const int32_T u_size[1], real_T p_ode_Nship, const real_T p_ode_M
              [9], const real_T p_ode_D[9], const real_T p_ode_thrust_config[8],
              emxArray_real_T *xdot)
{
  real_T t1;
  int32_T i;
  int32_T p1;
  int32_T u;
  int32_T b_i;
  int32_T b_u;
  real_T absx11;
  real_T fai_tmp;
  real_T absx21;
  real_T B[3];
  real_T absx31;
  int32_T p3;
  real_T v_tmp[3];
  real_T v_idx_0;
  real_T v_idx_1;
  real_T v_idx_2;
  real_T unnamed_idx_0;
  real_T unnamed_idx_1;
  real_T unnamed_idx_2;
  real_T unnamed_idx_3;
  real_T t2;
  real_T n1xinv;
  real_T T_tmp;
  real_T b_B[12];
  real_T R_tmp[9];
  int32_T p2;
  int32_T itmp;
  real_T a[9];
  boolean_T exitg1;
  const mxArray *y;
  const mxArray *m;
  static const int32_T iv[2] = { 1, 6 };

  static const char_T rfmt[6] = { '%', '1', '4', '.', '6', 'e' };

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
  t1 = 6.0 * p_ode_Nship;
  if (!(t1 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(t1, &g_emlrtDCI, sp);
  }

  if (t1 != (int32_T)muDoubleScalarFloor(t1)) {
    emlrtIntegerCheckR2012b(t1, &f_emlrtDCI, sp);
  }

  i = xdot->size[0];
  xdot->size[0] = (int32_T)t1;
  emxEnsureCapacity_real_T(sp, xdot, i, &cb_emlrtRTEI);
  t1 = 6.0 * p_ode_Nship;
  if (!(t1 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(t1, &g_emlrtDCI, sp);
  }

  if (t1 != (int32_T)muDoubleScalarFloor(t1)) {
    emlrtIntegerCheckR2012b(t1, &f_emlrtDCI, sp);
  }

  p1 = (int32_T)t1;
  for (i = 0; i < p1; i++) {
    xdot->data[i] = 0.0;
  }

  i = (int32_T)p_ode_Nship;
  emlrtForLoopVectorCheckR2012b(1.0, 1.0, p_ode_Nship, mxDOUBLE_CLASS, (int32_T)
    p_ode_Nship, &g_emlrtRTEI, sp);
  if (0 <= i - 1) {
    u = u_size[0];
    b_u = u_size[0];
    absx11 = muDoubleScalarAbs(p_ode_M[0]);
    absx21 = muDoubleScalarAbs(p_ode_M[1]);
    absx31 = muDoubleScalarAbs(p_ode_M[2]);
  }

  for (b_i = 0; b_i < i; b_i++) {
    fai_tmp = 6.0 * (((real_T)b_i + 1.0) - 1.0);
    B[0] = fai_tmp + 1.0;
    B[1] = fai_tmp + 2.0;
    B[2] = fai_tmp + 3.0;
    p3 = (int32_T)(fai_tmp + 1.0);
    if ((p3 < 1) || (p3 > 12)) {
      emlrtDynamicBoundsCheckR2012b(p3, 1, 12, &h_emlrtBCI, sp);
    }

    p3 = (int32_T)(fai_tmp + 2.0);
    if ((p3 < 1) || (p3 > 12)) {
      emlrtDynamicBoundsCheckR2012b(p3, 1, 12, &h_emlrtBCI, sp);
    }

    p3 = (int32_T)(fai_tmp + 3.0);
    if ((p3 < 1) || (p3 > 12)) {
      emlrtDynamicBoundsCheckR2012b(p3, 1, 12, &h_emlrtBCI, sp);
    }

    v_tmp[0] = fai_tmp + 4.0;
    v_tmp[1] = fai_tmp + 5.0;
    v_tmp[2] = fai_tmp + 6.0;
    p3 = (int32_T)(fai_tmp + 4.0);
    if ((p3 < 1) || (p3 > 12)) {
      emlrtDynamicBoundsCheckR2012b(p3, 1, 12, &i_emlrtBCI, sp);
    }

    v_idx_0 = x[p3 - 1];
    p3 = (int32_T)(fai_tmp + 5.0);
    if ((p3 < 1) || (p3 > 12)) {
      emlrtDynamicBoundsCheckR2012b(p3, 1, 12, &i_emlrtBCI, sp);
    }

    v_idx_1 = x[p3 - 1];
    p3 = (int32_T)(fai_tmp + 6.0);
    if ((p3 < 1) || (p3 > 12)) {
      emlrtDynamicBoundsCheckR2012b(p3, 1, 12, &i_emlrtBCI, sp);
    }

    v_idx_2 = x[p3 - 1];
    st.site = &r_emlrtRSI;
    indexShapeCheck(&st, u_size[0]);
    t1 = 8.0 * (((real_T)b_i + 1.0) - 1.0);
    p3 = (int32_T)(t1 + 1.0);
    if ((p3 < 1) || (p3 > u)) {
      emlrtDynamicBoundsCheckR2012b(p3, 1, u, &j_emlrtBCI, sp);
    }

    unnamed_idx_0 = u_data[p3 - 1];
    p3 = (int32_T)(t1 + 2.0);
    if ((p3 < 1) || (p3 > u)) {
      emlrtDynamicBoundsCheckR2012b(p3, 1, u, &j_emlrtBCI, sp);
    }

    unnamed_idx_1 = u_data[p3 - 1];
    p3 = (int32_T)(t1 + 3.0);
    if ((p3 < 1) || (p3 > u)) {
      emlrtDynamicBoundsCheckR2012b(p3, 1, u, &j_emlrtBCI, sp);
    }

    unnamed_idx_2 = u_data[p3 - 1];
    p3 = (int32_T)(t1 + 4.0);
    if ((p3 < 1) || (p3 > u)) {
      emlrtDynamicBoundsCheckR2012b(p3, 1, u, &j_emlrtBCI, sp);
    }

    unnamed_idx_3 = u_data[p3 - 1];
    st.site = &s_emlrtRSI;
    indexShapeCheck(&st, u_size[0]);
    p3 = (int32_T)(t1 + 5.0);
    if ((p3 < 1) || (p3 > b_u)) {
      emlrtDynamicBoundsCheckR2012b(p3, 1, b_u, &k_emlrtBCI, sp);
    }

    p3 = (int32_T)(t1 + 6.0);
    if ((p3 < 1) || (p3 > b_u)) {
      emlrtDynamicBoundsCheckR2012b(p3, 1, b_u, &k_emlrtBCI, sp);
    }

    p3 = (int32_T)(t1 + 7.0);
    if ((p3 < 1) || (p3 > b_u)) {
      emlrtDynamicBoundsCheckR2012b(p3, 1, b_u, &k_emlrtBCI, sp);
    }

    p3 = (int32_T)(t1 + 8.0);
    if ((p3 < 1) || (p3 > b_u)) {
      emlrtDynamicBoundsCheckR2012b(p3, 1, b_u, &k_emlrtBCI, sp);
    }

    t2 = muDoubleScalarSin(fai_tmp + 3.0);
    n1xinv = muDoubleScalarCos(fai_tmp + 3.0);

    /*  function thruster_configuration is used to obtain the configuration */
    /*  matrix */
    /*  initialization */
    p3 = (b_i << 3) + 4;
    T_tmp = muDoubleScalarCos(u_data[p3]);
    b_B[0] = T_tmp;
    t1 = muDoubleScalarSin(u_data[p3]);
    b_B[1] = t1;
    b_B[2] = p_ode_thrust_config[0] * t1 - p_ode_thrust_config[4] * T_tmp;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }

    t1 = u_data[p3 + 1];
    T_tmp = muDoubleScalarCos(t1);
    b_B[3] = T_tmp;
    t1 = muDoubleScalarSin(t1);
    b_B[4] = t1;
    b_B[5] = p_ode_thrust_config[1] * t1 - p_ode_thrust_config[5] * T_tmp;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }

    t1 = u_data[p3 + 2];
    T_tmp = muDoubleScalarCos(t1);
    b_B[6] = T_tmp;
    t1 = muDoubleScalarSin(t1);
    b_B[7] = t1;
    b_B[8] = p_ode_thrust_config[2] * t1 - p_ode_thrust_config[6] * T_tmp;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }

    t1 = u_data[p3 + 3];
    T_tmp = muDoubleScalarCos(t1);
    b_B[9] = T_tmp;
    t1 = muDoubleScalarSin(t1);
    b_B[10] = t1;
    b_B[11] = p_ode_thrust_config[3] * t1 - p_ode_thrust_config[7] * T_tmp;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }

    R_tmp[0] = n1xinv;
    R_tmp[3] = -t2;
    R_tmp[6] = 0.0;
    R_tmp[1] = t2;
    R_tmp[4] = n1xinv;
    R_tmp[7] = 0.0;
    R_tmp[2] = 0.0;
    R_tmp[5] = 0.0;
    R_tmp[8] = 1.0;
    p1 = xdot->size[0];
    for (p3 = 0; p3 < 3; p3++) {
      p2 = (int32_T)B[p3];
      if ((p2 < 1) || (p2 > p1)) {
        emlrtDynamicBoundsCheckR2012b(p2, 1, p1, &f_emlrtBCI, sp);
      }

      xdot->data[p2 - 1] = 0.0;
      if (p2 > p1) {
        emlrtDynamicBoundsCheckR2012b(p2, 1, p1, &f_emlrtBCI, sp);
      }

      xdot->data[p2 - 1] += R_tmp[p3] * v_idx_0;
      if (p2 > p1) {
        emlrtDynamicBoundsCheckR2012b(p2, 1, p1, &f_emlrtBCI, sp);
      }

      xdot->data[p2 - 1] += R_tmp[p3 + 3] * v_idx_1;
      if (p2 > p1) {
        emlrtDynamicBoundsCheckR2012b(p2, 1, p1, &f_emlrtBCI, sp);
      }

      xdot->data[p2 - 1] += R_tmp[p3 + 6] * v_idx_2;
    }

    st.site = &t_emlrtRSI;
    memcpy(&R_tmp[0], &p_ode_M[0], 9U * sizeof(real_T));
    p1 = 0;
    p2 = 3;
    p3 = 6;
    if ((absx21 > absx11) && (absx21 > absx31)) {
      p1 = 3;
      p2 = 0;
      R_tmp[0] = p_ode_M[1];
      R_tmp[1] = p_ode_M[0];
      R_tmp[3] = p_ode_M[4];
      R_tmp[4] = p_ode_M[3];
      R_tmp[6] = p_ode_M[7];
      R_tmp[7] = p_ode_M[6];
    } else {
      if (absx31 > absx11) {
        p1 = 6;
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
      t1 = R_tmp[1];
      R_tmp[1] = R_tmp[2];
      R_tmp[2] = t1;
      t1 = R_tmp[4];
      R_tmp[4] = R_tmp[5];
      R_tmp[5] = t1;
      t1 = R_tmp[7];
      R_tmp[7] = R_tmp[8];
      R_tmp[8] = t1;
    }

    R_tmp[5] /= R_tmp[4];
    R_tmp[8] -= R_tmp[5] * R_tmp[7];
    t1 = (R_tmp[5] * R_tmp[1] - R_tmp[2]) / R_tmp[8];
    t2 = -(R_tmp[1] + R_tmp[7] * t1) / R_tmp[4];
    a[p1] = ((1.0 - R_tmp[3] * t2) - R_tmp[6] * t1) / R_tmp[0];
    a[p1 + 1] = t2;
    a[p1 + 2] = t1;
    t1 = -R_tmp[5] / R_tmp[8];
    t2 = (1.0 - R_tmp[7] * t1) / R_tmp[4];
    a[p2] = -(R_tmp[3] * t2 + R_tmp[6] * t1) / R_tmp[0];
    a[p2 + 1] = t2;
    a[p2 + 2] = t1;
    t1 = 1.0 / R_tmp[8];
    t2 = -R_tmp[7] * t1 / R_tmp[4];
    a[p3] = -(R_tmp[3] * t2 + R_tmp[6] * t1) / R_tmp[0];
    a[p3 + 1] = t2;
    a[p3 + 2] = t1;
    b_st.site = &v_emlrtRSI;
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
      t1 = (muDoubleScalarAbs(a[3 * p1]) + muDoubleScalarAbs(a[3 * p1 + 1])) +
        muDoubleScalarAbs(a[3 * p1 + 2]);
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
      c_st.site = &w_emlrtRSI;
      warning(&c_st);
    } else {
      if (muDoubleScalarIsNaN(t1) || (t1 < 2.2204460492503131E-16)) {
        c_st.site = &x_emlrtRSI;
        y = NULL;
        m = emlrtCreateCharArray(2, &iv[0]);
        emlrtInitCharArrayR2013a(&c_st, 6, m, &rfmt[0]);
        emlrtAssign(&y, m);
        d_st.site = &ub_emlrtRSI;
        emlrt_marshallIn(&d_st, b_sprintf(&d_st, y, emlrt_marshallOut(t1),
          &c_emlrtMCI), "<output of sprintf>", str);
        c_st.site = &x_emlrtRSI;
        b_warning(&c_st, str);
      }
    }

    R_tmp[0] = 0.0;
    R_tmp[3] = 0.0;
    p1 = (int8_T)(fai_tmp + 5.0) - 1;
    p2 = (int8_T)(fai_tmp + 6.0) - 1;
    R_tmp[6] = -p_ode_M[4] * x[p1] - p_ode_M[7] * x[p2];
    R_tmp[1] = 0.0;
    R_tmp[4] = 0.0;
    p3 = (int8_T)(fai_tmp + 4.0) - 1;
    R_tmp[7] = p_ode_M[0] * x[p3];
    R_tmp[2] = p_ode_M[4] * x[p1] + p_ode_M[5] * x[p2];
    R_tmp[5] = -p_ode_M[0] * x[p3];
    R_tmp[8] = 0.0;
    for (p3 = 0; p3 < 3; p3++) {
      B[p3] = ((((b_B[p3] * unnamed_idx_0 + b_B[p3 + 3] * unnamed_idx_1) +
                 b_B[p3 + 6] * unnamed_idx_2) + b_B[p3 + 9] * unnamed_idx_3) *
               1000.0 - ((R_tmp[p3] * v_idx_0 + R_tmp[p3 + 3] * v_idx_1) +
                         R_tmp[p3 + 6] * v_idx_2)) - ((p_ode_D[p3] * v_idx_0 +
        p_ode_D[p3 + 3] * v_idx_1) + p_ode_D[p3 + 6] * v_idx_2);
    }

    p1 = xdot->size[0];
    for (p3 = 0; p3 < 3; p3++) {
      p2 = (int32_T)v_tmp[p3];
      if ((p2 < 1) || (p2 > p1)) {
        emlrtDynamicBoundsCheckR2012b(p2, 1, p1, &g_emlrtBCI, sp);
      }

      xdot->data[p2 - 1] = 0.0;
      if (p2 > p1) {
        emlrtDynamicBoundsCheckR2012b(p2, 1, p1, &g_emlrtBCI, sp);
      }

      xdot->data[p2 - 1] += a[p3] * B[0];
      if (p2 > p1) {
        emlrtDynamicBoundsCheckR2012b(p2, 1, p1, &g_emlrtBCI, sp);
      }

      xdot->data[p2 - 1] += a[p3 + 3] * B[1];
      if (p2 > p1) {
        emlrtDynamicBoundsCheckR2012b(p2, 1, p1, &g_emlrtBCI, sp);
      }

      xdot->data[p2 - 1] += a[p3 + 6] * B[2];
    }

    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }

  /*      return */
  /* ------------------------------------------------------------------------------- */
}

/* End of code generation (user_ode.c) */
