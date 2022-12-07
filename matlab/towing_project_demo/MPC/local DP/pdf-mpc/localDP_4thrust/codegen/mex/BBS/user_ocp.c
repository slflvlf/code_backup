/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * user_ocp.c
 *
 * Code generation for function 'user_ocp'
 *
 */

/* Include files */
#include "user_ocp.h"
#include "BBS.h"
#include "BBS_data.h"
#include "BBS_emxutil.h"
#include "eml_int_forloop_overflow_check.h"
#include "indexShapeCheck.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtRSInfo x_emlrtRSI = { 20,  /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m"/* pathName */
};

static emlrtRSInfo y_emlrtRSI = { 21,  /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m"/* pathName */
};

static emlrtRSInfo ab_emlrtRSI = { 22, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m"/* pathName */
};

static emlrtRSInfo bb_emlrtRSI = { 25, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m"/* pathName */
};

static emlrtRSInfo cb_emlrtRSI = { 27, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m"/* pathName */
};

static emlrtRSInfo db_emlrtRSI = { 28, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m"/* pathName */
};

static emlrtRSInfo eb_emlrtRSI = { 30, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m"/* pathName */
};

static emlrtRSInfo fb_emlrtRSI = { 14, /* lineNo */
  "max",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\datafun\\max.m"/* pathName */
};

static emlrtRSInfo gb_emlrtRSI = { 44, /* lineNo */
  "minOrMax",                          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax.m"/* pathName */
};

static emlrtRSInfo hb_emlrtRSI = { 79, /* lineNo */
  "maximum",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax.m"/* pathName */
};

static emlrtRSInfo ib_emlrtRSI = { 145,/* lineNo */
  "unaryMinOrMax",                     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo jb_emlrtRSI = { 1019,/* lineNo */
  "maxRealVectorOmitNaN",              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo kb_emlrtRSI = { 932,/* lineNo */
  "minOrMaxRealVector",                /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo lb_emlrtRSI = { 992,/* lineNo */
  "minOrMaxRealVectorKernel",          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo mb_emlrtRSI = { 21, /* lineNo */
  "eml_int_forloop_overflow_check",    /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\eml\\eml_int_forloop_overflow_check.m"/* pathName */
};

static emlrtRSInfo nb_emlrtRSI = { 924,/* lineNo */
  "minOrMaxRealVector",                /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo ob_emlrtRSI = { 975,/* lineNo */
  "findFirst",                         /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo pb_emlrtRSI = { 48, /* lineNo */
  "eml_mtimes_helper",                 /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_helper.m"/* pathName */
};

static emlrtRTEInfo h_emlrtRTEI = { 95,/* lineNo */
  27,                                  /* colNo */
  "unaryMinOrMax",                     /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pName */
};

static emlrtBCInfo g_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  25,                                  /* lineNo */
  57,                                  /* colNo */
  "uu",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo h_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  25,                                  /* lineNo */
  36,                                  /* colNo */
  "uu",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo i_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  24,                                  /* lineNo */
  8,                                   /* colNo */
  "uu",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo j_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  23,                                  /* lineNo */
  12,                                  /* colNo */
  "xx",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo emlrtECI = { -1,    /* nDims */
  18,                                  /* lineNo */
  12,                                  /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m"/* pName */
};

static emlrtBCInfo k_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  18,                                  /* lineNo */
  25,                                  /* colNo */
  "uu",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo l_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  18,                                  /* lineNo */
  15,                                  /* colNo */
  "uu",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo b_emlrtECI = { -1,  /* nDims */
  16,                                  /* lineNo */
  12,                                  /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m"/* pName */
};

static emlrtBCInfo m_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  16,                                  /* lineNo */
  15,                                  /* colNo */
  "uu",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo j_emlrtRTEI = { 14,/* lineNo */
  7,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m"/* pName */
};

static emlrtDCInfo f_emlrtDCI = { 11,  /* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo g_emlrtDCI = { 11,  /* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m",/* pName */
  4                                    /* checkKind */
};

static emlrtDCInfo h_emlrtDCI = { 12,  /* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo n_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  21,                                  /* lineNo */
  5,                                   /* colNo */
  "h1_du",                             /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo o_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  22,                                  /* lineNo */
  5,                                   /* colNo */
  "h2_du",                             /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo p_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  20,                                  /* lineNo */
  11,                                  /* colNo */
  "du",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo q_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  20,                                  /* lineNo */
  27,                                  /* colNo */
  "du",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo l_emlrtRTEI = { 123,/* lineNo */
  23,                                  /* colNo */
  "dynamic_size_checks",               /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_helper.m"/* pName */
};

static emlrtRTEInfo m_emlrtRTEI = { 118,/* lineNo */
  23,                                  /* colNo */
  "dynamic_size_checks",               /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_helper.m"/* pName */
};

static emlrtRTEInfo t_emlrtRTEI = { 11,/* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m"/* pName */
};

static emlrtRTEInfo u_emlrtRTEI = { 12,/* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_4thrust\\user_ocp.m"/* pName */
};

static emlrtRTEInfo v_emlrtRTEI = { 145,/* lineNo */
  38,                                  /* colNo */
  "unaryMinOrMax",                     /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pName */
};

/* Function Declarations */
static void b_dynamic_size_checks(const emlrtStack *sp, const int32_T b_size[2],
  int32_T innerDimB);
static void dynamic_size_checks(const emlrtStack *sp, const int32_T a_size[2],
  int32_T innerDimA);

/* Function Definitions */
static void b_dynamic_size_checks(const emlrtStack *sp, const int32_T b_size[2],
  int32_T innerDimB)
{
  if (8 != innerDimB) {
    if (b_size[1] == 1) {
      emlrtErrorWithMessageIdR2018a(sp, &m_emlrtRTEI,
        "Coder:toolbox:mtimes_noDynamicScalarExpansion",
        "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(sp, &l_emlrtRTEI, "MATLAB:innerdim",
        "MATLAB:innerdim", 0);
    }
  }
}

static void dynamic_size_checks(const emlrtStack *sp, const int32_T a_size[2],
  int32_T innerDimA)
{
  if (innerDimA != 8) {
    if (a_size[1] == 1) {
      emlrtErrorWithMessageIdR2018a(sp, &m_emlrtRTEI,
        "Coder:toolbox:mtimes_noDynamicScalarExpansion",
        "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(sp, &l_emlrtRTEI, "MATLAB:innerdim",
        "MATLAB:innerdim", 0);
    }
  }
}

void user_ocp(const emlrtStack *sp, const emxArray_real_T *xx, const
              emxArray_real_T *uu, const real_T p_ode_u0[8], real_T p_uparam_Np,
              const real_T p_ocp_Q[36], const real_T p_ocp_R[64], const real_T
              p_ocp_Rdu[64], const real_T p_ocp_rd[3], real_T p_ocp_dF_max,
              real_T p_ocp_da_max, real_T *J, real_T *g)
{
  emxArray_real_T *h1_du;
  real_T xd[6];
  int32_T k;
  int32_T tmp_size_idx_0;
  int32_T loop_ub;
  emxArray_real_T *h2_du;
  emxArray_real_T *x;
  int32_T i;
  uint32_T u;
  int32_T b_i;
  int32_T idx;
  real_T du_data[160];
  real_T tmp_data[160];
  real_T du[8];
  boolean_T exitg1;
  real_T d;
  real_T h2;
  real_T varargin_1[8];
  real_T e[6];
  int32_T uu_size[2];
  real_T b_uu;
  real_T c_uu;
  real_T d1;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack i_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  i_st.prev = &h_st;
  i_st.tls = h_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  emxInit_real_T(sp, &h1_du, 1, &t_emlrtRTEI, true);

  /* ------------------------------------------------------------------------------- */
  /*  pdf_mpc package: Example 1 - Definition of the user_ocp map */
  /* ------------------------------------------------------------------------------- */
  *J = 0.0;
  xd[0] = p_ocp_rd[0];
  xd[3] = 0.0;
  xd[1] = p_ocp_rd[1];
  xd[4] = 0.0;
  xd[2] = p_ocp_rd[2];
  xd[5] = 0.0;

  /*  xd=[p_ocp.rd;0;0;0]; */
  if (!(p_uparam_Np >= 0.0)) {
    emlrtNonNegativeCheckR2012b(p_uparam_Np, &g_emlrtDCI, sp);
  }

  k = (int32_T)muDoubleScalarFloor(p_uparam_Np);
  if (p_uparam_Np != k) {
    emlrtIntegerCheckR2012b(p_uparam_Np, &f_emlrtDCI, sp);
  }

  tmp_size_idx_0 = h1_du->size[0];
  loop_ub = (int32_T)p_uparam_Np;
  h1_du->size[0] = loop_ub;
  emxEnsureCapacity_real_T(sp, h1_du, tmp_size_idx_0, &t_emlrtRTEI);
  if (p_uparam_Np != k) {
    emlrtIntegerCheckR2012b(p_uparam_Np, &f_emlrtDCI, sp);
  }

  for (tmp_size_idx_0 = 0; tmp_size_idx_0 < loop_ub; tmp_size_idx_0++) {
    h1_du->data[tmp_size_idx_0] = 0.0;
  }

  emxInit_real_T(sp, &h2_du, 1, &u_emlrtRTEI, true);
  if (p_uparam_Np != k) {
    emlrtIntegerCheckR2012b(p_uparam_Np, &h_emlrtDCI, sp);
  }

  loop_ub = (int32_T)p_uparam_Np;
  tmp_size_idx_0 = h2_du->size[0];
  h2_du->size[0] = (int32_T)p_uparam_Np;
  emxEnsureCapacity_real_T(sp, h2_du, tmp_size_idx_0, &u_emlrtRTEI);
  if ((int32_T)p_uparam_Np != k) {
    emlrtIntegerCheckR2012b(p_uparam_Np, &h_emlrtDCI, sp);
  }

  for (k = 0; k < loop_ub; k++) {
    h2_du->data[k] = 0.0;
  }

  emlrtForLoopVectorCheckR2012b(1.0, 1.0, p_uparam_Np, mxDOUBLE_CLASS, (int32_T)
    p_uparam_Np, &j_emlrtRTEI, sp);
  emxInit_real_T(sp, &x, 1, &v_emlrtRTEI, true);
  for (i = 0; i < loop_ub; i++) {
    u = i + 1U;
    if (u == 1U) {
      if (1 > uu->size[0]) {
        emlrtDynamicBoundsCheckR2012b(1, 1, uu->size[0], &m_emlrtBCI, sp);
      }

      b_i = uu->size[1];
      idx = uu->size[1];
      for (k = 0; k < b_i; k++) {
        du_data[k] = uu->data[uu->size[0] * k];
      }

      if (idx != 8) {
        emlrtSizeEqCheck1DR2012b(idx, 8, &b_emlrtECI, sp);
      }

      for (k = 0; k < 8; k++) {
        du_data[k] -= p_ode_u0[k];
      }
    } else {
      if ((int32_T)u > uu->size[0]) {
        emlrtDynamicBoundsCheckR2012b((int32_T)u, 1, uu->size[0], &l_emlrtBCI,
          sp);
      }

      b_i = uu->size[1];
      idx = uu->size[1];
      for (k = 0; k < b_i; k++) {
        du_data[k] = uu->data[i + uu->size[0] * k];
      }

      if ((i < 1) || (i > uu->size[0])) {
        emlrtDynamicBoundsCheckR2012b(i, 1, uu->size[0], &k_emlrtBCI, sp);
      }

      b_i = uu->size[1];
      tmp_size_idx_0 = uu->size[1];
      for (k = 0; k < b_i; k++) {
        tmp_data[k] = uu->data[(i + uu->size[0] * k) - 1];
      }

      if (idx != tmp_size_idx_0) {
        emlrtSizeEqCheck1DR2012b(idx, tmp_size_idx_0, &emlrtECI, sp);
      }

      for (k = 0; k < idx; k++) {
        du_data[k] -= tmp_data[k];
      }
    }

    st.site = &x_emlrtRSI;
    indexShapeCheck(&st, idx);
    st.site = &x_emlrtRSI;
    indexShapeCheck(&st, idx);
    if (1 > idx) {
      emlrtDynamicBoundsCheckR2012b(1, 1, idx, &p_emlrtBCI, sp);
    }

    du[0] = du_data[0] / p_ocp_dF_max;
    if (2 > idx) {
      emlrtDynamicBoundsCheckR2012b(2, 1, 1, &p_emlrtBCI, sp);
    }

    du[1] = du_data[1] / p_ocp_dF_max;
    if (3 > idx) {
      emlrtDynamicBoundsCheckR2012b(3, 1, 2, &p_emlrtBCI, sp);
    }

    du[2] = du_data[2] / p_ocp_dF_max;
    if (4 > idx) {
      emlrtDynamicBoundsCheckR2012b(4, 1, 3, &p_emlrtBCI, sp);
    }

    du[3] = du_data[3] / p_ocp_dF_max;
    if (5 > idx) {
      emlrtDynamicBoundsCheckR2012b(5, 1, 4, &q_emlrtBCI, sp);
    }

    du[4] = du_data[4] / p_ocp_da_max;
    if (6 > idx) {
      emlrtDynamicBoundsCheckR2012b(6, 1, 5, &q_emlrtBCI, sp);
    }

    du[5] = du_data[5] / p_ocp_da_max;
    if (7 > idx) {
      emlrtDynamicBoundsCheckR2012b(7, 1, 6, &q_emlrtBCI, sp);
    }

    du[6] = du_data[6] / p_ocp_da_max;
    if (8 > idx) {
      emlrtDynamicBoundsCheckR2012b(8, 1, 7, &q_emlrtBCI, sp);
    }

    du[7] = du_data[7] / p_ocp_da_max;
    st.site = &y_emlrtRSI;
    for (b_i = 0; b_i < 8; b_i++) {
      varargin_1[b_i] = du[b_i] - 1.0;
    }

    b_st.site = &fb_emlrtRSI;
    c_st.site = &gb_emlrtRSI;
    d_st.site = &hb_emlrtRSI;
    e_st.site = &ib_emlrtRSI;
    k = x->size[0];
    x->size[0] = 8;
    emxEnsureCapacity_real_T(&e_st, x, k, &v_emlrtRTEI);
    for (k = 0; k < 8; k++) {
      x->data[k] = du[k] - 1.0;
    }

    f_st.site = &jb_emlrtRSI;
    g_st.site = &nb_emlrtRSI;
    if (!muDoubleScalarIsNaN(x->data[0])) {
      idx = 1;
    } else {
      idx = 0;
      h_st.site = &ob_emlrtRSI;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= 8)) {
        if (!muDoubleScalarIsNaN(x->data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      k = (int32_T)(i + 1U);
      if (k > h1_du->size[0]) {
        emlrtDynamicBoundsCheckR2012b(k, 1, h1_du->size[0], &n_emlrtBCI, &f_st);
      }

      h1_du->data[k - 1] = varargin_1[0];
    } else {
      g_st.site = &kb_emlrtRSI;
      h2 = varargin_1[idx - 1];
      tmp_size_idx_0 = idx + 1;
      h_st.site = &lb_emlrtRSI;
      for (k = tmp_size_idx_0; k < 9; k++) {
        d = varargin_1[k - 1];
        if (h2 < d) {
          h2 = d;
        }
      }

      k = (int32_T)(i + 1U);
      if (k > h1_du->size[0]) {
        emlrtDynamicBoundsCheckR2012b(k, 1, h1_du->size[0], &n_emlrtBCI, &f_st);
      }

      h1_du->data[k - 1] = h2;
    }

    st.site = &ab_emlrtRSI;
    for (b_i = 0; b_i < 8; b_i++) {
      varargin_1[b_i] = -du[b_i] - 1.0;
    }

    b_st.site = &fb_emlrtRSI;
    c_st.site = &gb_emlrtRSI;
    d_st.site = &hb_emlrtRSI;
    e_st.site = &ib_emlrtRSI;
    k = x->size[0];
    x->size[0] = 8;
    emxEnsureCapacity_real_T(&e_st, x, k, &v_emlrtRTEI);
    for (k = 0; k < 8; k++) {
      x->data[k] = -du[k] - 1.0;
    }

    f_st.site = &jb_emlrtRSI;
    g_st.site = &nb_emlrtRSI;
    if (!muDoubleScalarIsNaN(x->data[0])) {
      idx = 1;
    } else {
      idx = 0;
      h_st.site = &ob_emlrtRSI;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= 8)) {
        if (!muDoubleScalarIsNaN(x->data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      k = (int32_T)(i + 1U);
      if (k > h2_du->size[0]) {
        emlrtDynamicBoundsCheckR2012b(k, 1, h2_du->size[0], &o_emlrtBCI, &f_st);
      }

      h2_du->data[k - 1] = varargin_1[0];
    } else {
      g_st.site = &kb_emlrtRSI;
      h2 = varargin_1[idx - 1];
      tmp_size_idx_0 = idx + 1;
      h_st.site = &lb_emlrtRSI;
      for (k = tmp_size_idx_0; k < 9; k++) {
        d = varargin_1[k - 1];
        if (h2 < d) {
          h2 = d;
        }
      }

      k = (int32_T)(i + 1U);
      if (k > h2_du->size[0]) {
        emlrtDynamicBoundsCheckR2012b(k, 1, h2_du->size[0], &o_emlrtBCI, &f_st);
      }

      h2_du->data[k - 1] = h2;
    }

    k = (int32_T)(i + 2U);
    if ((k < 1) || (k > xx->size[0])) {
      emlrtDynamicBoundsCheckR2012b(k, 1, xx->size[0], &j_emlrtBCI, sp);
    }

    for (k = 0; k < 6; k++) {
      e[k] = xx->data[(i + xx->size[0] * k) + 1] - xd[k];
    }

    if (1 > uu->size[0]) {
      emlrtDynamicBoundsCheckR2012b(1, 1, uu->size[0], &i_emlrtBCI, sp);
    }

    k = i + 1;
    if (k > uu->size[0]) {
      emlrtDynamicBoundsCheckR2012b(k, 1, uu->size[0], &g_emlrtBCI, sp);
    }

    st.site = &bb_emlrtRSI;
    k = i + 1;
    if (k > uu->size[0]) {
      emlrtDynamicBoundsCheckR2012b(k, 1, uu->size[0], &h_emlrtBCI, &st);
    }

    uu_size[0] = 1;
    uu_size[1] = uu->size[1];
    b_st.site = &pb_emlrtRSI;
    dynamic_size_checks(&b_st, uu_size, uu->size[1]);
    st.site = &bb_emlrtRSI;
    uu_size[0] = 1;
    uu_size[1] = uu->size[1];
    b_st.site = &pb_emlrtRSI;
    b_dynamic_size_checks(&b_st, uu_size, uu->size[1]);
    h2 = 0.0;
    for (k = 0; k < 6; k++) {
      d = 0.0;
      for (tmp_size_idx_0 = 0; tmp_size_idx_0 < 6; tmp_size_idx_0++) {
        d += e[tmp_size_idx_0] * p_ocp_Q[tmp_size_idx_0 + 6 * k];
      }

      h2 += d * e[k];
    }

    b_i = uu->size[1];
    for (k = 0; k < b_i; k++) {
      d = uu->data[i + uu->size[0] * k];
      tmp_data[k] = d;
      du_data[k] = d;
    }

    b_uu = 0.0;
    c_uu = 0.0;
    for (k = 0; k < 8; k++) {
      d = 0.0;
      d1 = 0.0;
      for (tmp_size_idx_0 = 0; tmp_size_idx_0 < 8; tmp_size_idx_0++) {
        b_i = tmp_size_idx_0 + (k << 3);
        d += tmp_data[tmp_size_idx_0] * p_ocp_R[b_i];
        d1 += du[tmp_size_idx_0] * p_ocp_Rdu[b_i];
      }

      b_uu += d * du_data[k];
      c_uu += d1 * du[k];
    }

    *J += (h2 + b_uu) + c_uu;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }

  emxFree_real_T(&x);
  st.site = &cb_emlrtRSI;
  b_st.site = &fb_emlrtRSI;
  c_st.site = &gb_emlrtRSI;
  d_st.site = &hb_emlrtRSI;
  if (h1_du->size[0] < 1) {
    emlrtErrorWithMessageIdR2018a(&d_st, &h_emlrtRTEI,
      "Coder:toolbox:eml_min_or_max_varDimZero",
      "Coder:toolbox:eml_min_or_max_varDimZero", 0);
  }

  e_st.site = &ib_emlrtRSI;
  f_st.site = &jb_emlrtRSI;
  b_i = h1_du->size[0];
  if (h1_du->size[0] <= 2) {
    if (h1_du->size[0] == 1) {
      *g = h1_du->data[0];
    } else if ((h1_du->data[0] < h1_du->data[1]) || (muDoubleScalarIsNaN
                (h1_du->data[0]) && (!muDoubleScalarIsNaN(h1_du->data[1])))) {
      *g = h1_du->data[1];
    } else {
      *g = h1_du->data[0];
    }
  } else {
    g_st.site = &nb_emlrtRSI;
    if (!muDoubleScalarIsNaN(h1_du->data[0])) {
      idx = 1;
    } else {
      idx = 0;
      h_st.site = &ob_emlrtRSI;
      if (h1_du->size[0] > 2147483646) {
        i_st.site = &mb_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }

      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= h1_du->size[0])) {
        if (!muDoubleScalarIsNaN(h1_du->data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      *g = h1_du->data[0];
    } else {
      g_st.site = &kb_emlrtRSI;
      *g = h1_du->data[idx - 1];
      tmp_size_idx_0 = idx + 1;
      h_st.site = &lb_emlrtRSI;
      if ((idx + 1 <= h1_du->size[0]) && (h1_du->size[0] > 2147483646)) {
        i_st.site = &mb_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }

      for (k = tmp_size_idx_0; k <= b_i; k++) {
        d = h1_du->data[k - 1];
        if (*g < d) {
          *g = d;
        }
      }
    }
  }

  emxFree_real_T(&h1_du);
  st.site = &db_emlrtRSI;
  b_st.site = &fb_emlrtRSI;
  c_st.site = &gb_emlrtRSI;
  d_st.site = &hb_emlrtRSI;
  if (h2_du->size[0] < 1) {
    emlrtErrorWithMessageIdR2018a(&d_st, &h_emlrtRTEI,
      "Coder:toolbox:eml_min_or_max_varDimZero",
      "Coder:toolbox:eml_min_or_max_varDimZero", 0);
  }

  e_st.site = &ib_emlrtRSI;
  f_st.site = &jb_emlrtRSI;
  b_i = h2_du->size[0];
  if (h2_du->size[0] <= 2) {
    if (h2_du->size[0] == 1) {
      h2 = h2_du->data[0];
    } else if ((h2_du->data[0] < h2_du->data[1]) || (muDoubleScalarIsNaN
                (h2_du->data[0]) && (!muDoubleScalarIsNaN(h2_du->data[1])))) {
      h2 = h2_du->data[1];
    } else {
      h2 = h2_du->data[0];
    }
  } else {
    g_st.site = &nb_emlrtRSI;
    if (!muDoubleScalarIsNaN(h2_du->data[0])) {
      idx = 1;
    } else {
      idx = 0;
      h_st.site = &ob_emlrtRSI;
      if (h2_du->size[0] > 2147483646) {
        i_st.site = &mb_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }

      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= h2_du->size[0])) {
        if (!muDoubleScalarIsNaN(h2_du->data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      h2 = h2_du->data[0];
    } else {
      g_st.site = &kb_emlrtRSI;
      h2 = h2_du->data[idx - 1];
      tmp_size_idx_0 = idx + 1;
      h_st.site = &lb_emlrtRSI;
      if ((idx + 1 <= h2_du->size[0]) && (h2_du->size[0] > 2147483646)) {
        i_st.site = &mb_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }

      for (k = tmp_size_idx_0; k <= b_i; k++) {
        d = h2_du->data[k - 1];
        if (h2 < d) {
          h2 = d;
        }
      }
    }
  }

  emxFree_real_T(&h2_du);
  st.site = &eb_emlrtRSI;
  b_st.site = &fb_emlrtRSI;
  c_st.site = &gb_emlrtRSI;
  d_st.site = &hb_emlrtRSI;
  e_st.site = &ib_emlrtRSI;
  f_st.site = &jb_emlrtRSI;
  if ((*g < h2) || (muDoubleScalarIsNaN(*g) && (!muDoubleScalarIsNaN(h2)))) {
    *g = h2;
  }

  /*  h1=max(xx(:,3)-p_ocp.theta_max); */
  /*  h2=max(-xx(:,3)-p_ocp.theta_max); */
  /*  h3=max(xx(:,4)-p_ocp.thetap_max); */
  /*  h4=max(-xx(:,4)-p_ocp.thetap_max); */
  /*  g=max([h1;h2;h3;h4]); */
  /*  g = []; */
  /* ------------------------------------------------------------------------------- */
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (user_ocp.c) */
