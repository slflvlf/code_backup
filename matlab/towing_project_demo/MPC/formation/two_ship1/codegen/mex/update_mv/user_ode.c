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
#include "inv.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "update_mv.h"
#include "update_mv_data.h"
#include "update_mv_emxutil.h"

/* Variable Definitions */
static emlrtRSInfo mb_emlrtRSI = { 27, /* lineNo */
  "user_ode",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship1\\user_ode.m"/* pathName */
};

static emlrtRSInfo nb_emlrtRSI = { 29, /* lineNo */
  "user_ode",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship1\\user_ode.m"/* pathName */
};

static emlrtRSInfo ob_emlrtRSI = { 34, /* lineNo */
  "user_ode",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship1\\user_ode.m"/* pathName */
};

static emlrtRSInfo pb_emlrtRSI = { 36, /* lineNo */
  "user_ode",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship1\\user_ode.m"/* pathName */
};

static emlrtDCInfo j_emlrtDCI = { 14,  /* lineNo */
  5,                                   /* colNo */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship1\\user_ode.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo k_emlrtDCI = { 14,  /* lineNo */
  5,                                   /* colNo */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship1\\user_ode.m",/* pName */
  4                                    /* checkKind */
};

static emlrtBCInfo p_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  36,                                  /* lineNo */
  5,                                   /* colNo */
  "xdot",                              /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship1\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo q_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  35,                                  /* lineNo */
  5,                                   /* colNo */
  "xdot",                              /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship1\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo r_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  29,                                  /* lineNo */
  5,                                   /* colNo */
  "xdot",                              /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship1\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo s_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  28,                                  /* lineNo */
  5,                                   /* colNo */
  "xdot",                              /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship1\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo t_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  27,                                  /* lineNo */
  36,                                  /* colNo */
  "u",                                 /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship1\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo u_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  34,                                  /* lineNo */
  36,                                  /* colNo */
  "u",                                 /* aName */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship1\\user_ode.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo hb_emlrtRTEI = { 14,/* lineNo */
  5,                                   /* colNo */
  "user_ode",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship1\\user_ode.m"/* pName */
};

/* Function Definitions */
void user_ode(const emlrtStack *sp, const real_T x[12], const real_T u_data[],
              const int32_T u_size[1], real_T p_ode_Nship, const real_T p_ode_M
              [9], const real_T p_ode_D[9], const real_T p_ode_thrust_config[6],
              emxArray_real_T *xdot)
{
  real_T R_tmp;
  int32_T i;
  int32_T loop_ub;
  real_T b_R_tmp;
  real_T T_tmp;
  real_T B1[9];
  real_T b_T_tmp;
  real_T c_R_tmp[9];
  int32_T i1;
  real_T a[9];
  real_T b_B1[3];
  real_T c_B1[3];
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;

  /* ------------------------------------------------------------------------------- */
  /*  pdf_mpc package: Example 1 - Definition of the user_ode map */
  /* ------------------------------------------------------------------------------- */
  /*  ´¬µÄ¸öÊý */
  R_tmp = 6.0 * p_ode_Nship;
  if (!(R_tmp >= 0.0)) {
    emlrtNonNegativeCheckR2012b(R_tmp, &k_emlrtDCI, sp);
  }

  if (R_tmp != (int32_T)muDoubleScalarFloor(R_tmp)) {
    emlrtIntegerCheckR2012b(R_tmp, &j_emlrtDCI, sp);
  }

  i = xdot->size[0];
  xdot->size[0] = (int32_T)R_tmp;
  emxEnsureCapacity_real_T(sp, xdot, i, &hb_emlrtRTEI);
  R_tmp = 6.0 * p_ode_Nship;
  if (!(R_tmp >= 0.0)) {
    emlrtNonNegativeCheckR2012b(R_tmp, &k_emlrtDCI, sp);
  }

  if (R_tmp != (int32_T)muDoubleScalarFloor(R_tmp)) {
    emlrtIntegerCheckR2012b(R_tmp, &j_emlrtDCI, sp);
  }

  loop_ub = (int32_T)R_tmp;
  for (i = 0; i < loop_ub; i++) {
    xdot->data[i] = 0.0;
  }

  /*      for i = 1 : Nship  */
  /*          R1 = rotate_matrix(x(6*(i-1)+3)); */
  /*          C1 = C_matrix(M, x(6*(i-1)+4 : 6*(i-1)+6)); */
  /*          B1 = thrusters_configuration(u(6*(i-1)+4 : 6*(i-1)+6), thrust_config); */
  /*          xdot(6*(i-1)+1 : 6*(i-1)+3, 1) = R1 * x(6*(i-1)+4 : 6*(i-1)+6); */
  /*          xdot(6*(i-1)+4 : 6*(i-1)+6, 1) = inv(M) * (B1 * u(6*(i-1)+1 : 6*(i-1)+3) * 1e3 - C1 * x(6*(i-1)+4 : 6*(i-1)+6) - D * x(6*(i-1)+4 : 6*(i-1)+6)); */
  /*      end */
  R_tmp = muDoubleScalarSin(x[2]);
  b_R_tmp = muDoubleScalarCos(x[2]);
  st.site = &mb_emlrtRSI;
  indexShapeCheck(&st, u_size[0]);
  if (4 > u_size[0]) {
    emlrtDynamicBoundsCheckR2012b(4, 1, u_size[0], &t_emlrtBCI, sp);
  }

  if (5 > u_size[0]) {
    emlrtDynamicBoundsCheckR2012b(5, 1, 4, &t_emlrtBCI, sp);
  }

  if (6 > u_size[0]) {
    emlrtDynamicBoundsCheckR2012b(6, 1, 5, &t_emlrtBCI, sp);
  }

  /*  function thruster_configuration is used to obtain the configuration */
  /*  matrix */
  /*  initialization */
  T_tmp = muDoubleScalarCos(u_data[3]);
  B1[0] = T_tmp;
  b_T_tmp = muDoubleScalarSin(u_data[3]);
  B1[1] = b_T_tmp;
  B1[2] = p_ode_thrust_config[0] * b_T_tmp - p_ode_thrust_config[3] * T_tmp;
  if (*emlrtBreakCheckR2012bFlagVar != 0) {
    emlrtBreakCheckR2012b(sp);
  }

  T_tmp = muDoubleScalarCos(u_data[4]);
  B1[3] = T_tmp;
  b_T_tmp = muDoubleScalarSin(u_data[4]);
  B1[4] = b_T_tmp;
  B1[5] = p_ode_thrust_config[1] * b_T_tmp - p_ode_thrust_config[4] * T_tmp;
  if (*emlrtBreakCheckR2012bFlagVar != 0) {
    emlrtBreakCheckR2012b(sp);
  }

  T_tmp = muDoubleScalarCos(u_data[5]);
  B1[6] = T_tmp;
  b_T_tmp = muDoubleScalarSin(u_data[5]);
  B1[7] = b_T_tmp;
  B1[8] = p_ode_thrust_config[2] * b_T_tmp - p_ode_thrust_config[5] * T_tmp;
  if (*emlrtBreakCheckR2012bFlagVar != 0) {
    emlrtBreakCheckR2012b(sp);
  }

  c_R_tmp[0] = b_R_tmp;
  c_R_tmp[3] = -R_tmp;
  c_R_tmp[6] = 0.0;
  c_R_tmp[1] = R_tmp;
  c_R_tmp[4] = b_R_tmp;
  c_R_tmp[7] = 0.0;
  c_R_tmp[2] = 0.0;
  c_R_tmp[5] = 0.0;
  c_R_tmp[8] = 1.0;
  i = (int32_T)(6.0 * p_ode_Nship);
  for (i1 = 0; i1 < 3; i1++) {
    loop_ub = i1 + 1;
    if (loop_ub > i) {
      emlrtDynamicBoundsCheckR2012b(loop_ub, 1, i, &s_emlrtBCI, sp);
    }

    xdot->data[loop_ub - 1] = (c_R_tmp[i1] * x[3] + c_R_tmp[i1 + 3] * x[4]) +
      c_R_tmp[i1 + 6] * x[5];
  }

  st.site = &nb_emlrtRSI;
  indexShapeCheck(&st, u_size[0]);
  st.site = &nb_emlrtRSI;
  inv(&st, p_ode_M, a);
  for (i = 0; i < 3; i++) {
    b_B1[i] = (B1[i] * u_data[0] + B1[i + 3] * u_data[1]) + B1[i + 6] * u_data[2];
  }

  c_R_tmp[0] = 0.0;
  c_R_tmp[3] = 0.0;
  c_R_tmp[6] = -p_ode_M[4] * x[4] - p_ode_M[7] * x[5];
  c_R_tmp[1] = 0.0;
  c_R_tmp[4] = 0.0;
  c_R_tmp[7] = p_ode_M[0] * x[3];
  c_R_tmp[2] = p_ode_M[4] * x[4] + p_ode_M[5] * x[5];
  c_R_tmp[5] = -p_ode_M[0] * x[3];
  c_R_tmp[8] = 0.0;
  R_tmp = x[3];
  b_R_tmp = x[4];
  T_tmp = x[5];
  for (i = 0; i < 3; i++) {
    c_B1[i] = (b_B1[i] * 1000.0 - ((c_R_tmp[i] * R_tmp + c_R_tmp[i + 3] *
      b_R_tmp) + c_R_tmp[i + 6] * T_tmp)) - ((p_ode_D[i] * R_tmp + p_ode_D[i + 3]
      * b_R_tmp) + p_ode_D[i + 6] * T_tmp);
  }

  loop_ub = xdot->size[0];
  for (i = 0; i < 3; i++) {
    i1 = i + 4;
    if (i1 > loop_ub) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, loop_ub, &r_emlrtBCI, sp);
    }

    xdot->data[i1 - 1] = (a[i] * c_B1[0] + a[i + 3] * c_B1[1]) + a[i + 6] *
      c_B1[2];
  }

  /*  ´¬2 */
  R_tmp = muDoubleScalarSin(x[8]);
  b_R_tmp = muDoubleScalarCos(x[8]);
  st.site = &ob_emlrtRSI;
  indexShapeCheck(&st, u_size[0]);
  if (10 > u_size[0]) {
    emlrtDynamicBoundsCheckR2012b(10, 1, u_size[0], &u_emlrtBCI, sp);
  }

  if (11 > u_size[0]) {
    emlrtDynamicBoundsCheckR2012b(11, 1, 10, &u_emlrtBCI, sp);
  }

  if (12 > u_size[0]) {
    emlrtDynamicBoundsCheckR2012b(12, 1, 11, &u_emlrtBCI, sp);
  }

  /*  function thruster_configuration is used to obtain the configuration */
  /*  matrix */
  /*  initialization */
  T_tmp = muDoubleScalarCos(u_data[9]);
  B1[0] = T_tmp;
  b_T_tmp = muDoubleScalarSin(u_data[9]);
  B1[1] = b_T_tmp;
  B1[2] = p_ode_thrust_config[0] * b_T_tmp - p_ode_thrust_config[3] * T_tmp;
  if (*emlrtBreakCheckR2012bFlagVar != 0) {
    emlrtBreakCheckR2012b(sp);
  }

  T_tmp = muDoubleScalarCos(u_data[10]);
  B1[3] = T_tmp;
  b_T_tmp = muDoubleScalarSin(u_data[10]);
  B1[4] = b_T_tmp;
  B1[5] = p_ode_thrust_config[1] * b_T_tmp - p_ode_thrust_config[4] * T_tmp;
  if (*emlrtBreakCheckR2012bFlagVar != 0) {
    emlrtBreakCheckR2012b(sp);
  }

  T_tmp = muDoubleScalarCos(u_data[11]);
  B1[6] = T_tmp;
  b_T_tmp = muDoubleScalarSin(u_data[11]);
  B1[7] = b_T_tmp;
  B1[8] = p_ode_thrust_config[2] * b_T_tmp - p_ode_thrust_config[5] * T_tmp;
  if (*emlrtBreakCheckR2012bFlagVar != 0) {
    emlrtBreakCheckR2012b(sp);
  }

  c_R_tmp[0] = b_R_tmp;
  c_R_tmp[3] = -R_tmp;
  c_R_tmp[6] = 0.0;
  c_R_tmp[1] = R_tmp;
  c_R_tmp[4] = b_R_tmp;
  c_R_tmp[7] = 0.0;
  c_R_tmp[2] = 0.0;
  c_R_tmp[5] = 0.0;
  c_R_tmp[8] = 1.0;
  loop_ub = xdot->size[0];
  for (i = 0; i < 3; i++) {
    i1 = i + 7;
    if (i1 > loop_ub) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, loop_ub, &q_emlrtBCI, sp);
    }

    xdot->data[i1 - 1] = (c_R_tmp[i] * x[3] + c_R_tmp[i + 3] * x[4]) + c_R_tmp[i
      + 6] * x[5];
  }

  st.site = &pb_emlrtRSI;
  indexShapeCheck(&st, u_size[0]);
  st.site = &pb_emlrtRSI;
  inv(&st, p_ode_M, a);
  for (i = 0; i < 3; i++) {
    b_B1[i] = (B1[i] * u_data[6] + B1[i + 3] * u_data[7]) + B1[i + 6] * u_data[8];
  }

  c_R_tmp[0] = 0.0;
  c_R_tmp[3] = 0.0;
  c_R_tmp[6] = -p_ode_M[4] * x[10] - p_ode_M[7] * x[11];
  c_R_tmp[1] = 0.0;
  c_R_tmp[4] = 0.0;
  c_R_tmp[7] = p_ode_M[0] * x[9];
  c_R_tmp[2] = p_ode_M[4] * x[10] + p_ode_M[5] * x[11];
  c_R_tmp[5] = -p_ode_M[0] * x[9];
  c_R_tmp[8] = 0.0;
  R_tmp = x[9];
  b_R_tmp = x[10];
  T_tmp = x[11];
  for (i = 0; i < 3; i++) {
    c_B1[i] = (b_B1[i] * 1000.0 - ((c_R_tmp[i] * R_tmp + c_R_tmp[i + 3] *
      b_R_tmp) + c_R_tmp[i + 6] * T_tmp)) - ((p_ode_D[i] * R_tmp + p_ode_D[i + 3]
      * b_R_tmp) + p_ode_D[i + 6] * T_tmp);
  }

  loop_ub = xdot->size[0];
  for (i = 0; i < 3; i++) {
    i1 = i + 10;
    if (i1 > loop_ub) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, loop_ub, &p_emlrtBCI, sp);
    }

    xdot->data[i1 - 1] = (a[i] * c_B1[0] + a[i + 3] * c_B1[1]) + a[i + 6] *
      c_B1[2];
  }

  /*      return */
  /* ------------------------------------------------------------------------------- */
}

/* End of code generation (user_ode.c) */
