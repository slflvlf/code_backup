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
#include "eml_int_forloop_overflow_check.h"
#include "mwmathutil.h"
#include "repmat.h"
#include "rt_nonfinite.h"
#include "update_mv.h"
#include "update_mv_data.h"
#include "update_mv_emxutil.h"

/* Variable Definitions */
static emlrtRSInfo tb_emlrtRSI = { 8,  /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m"/* pathName */
};

static emlrtRSInfo ub_emlrtRSI = { 9,  /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m"/* pathName */
};

static emlrtRSInfo vb_emlrtRSI = { 29, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m"/* pathName */
};

static emlrtRSInfo wb_emlrtRSI = { 30, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m"/* pathName */
};

static emlrtRSInfo xb_emlrtRSI = { 40, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m"/* pathName */
};

static emlrtRSInfo yb_emlrtRSI = { 44, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m"/* pathName */
};

static emlrtRSInfo ac_emlrtRSI = { 45, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m"/* pathName */
};

static emlrtRSInfo bc_emlrtRSI = { 46, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m"/* pathName */
};

static emlrtRSInfo fc_emlrtRSI = { 14, /* lineNo */
  "max",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\datafun\\max.m"/* pathName */
};

static emlrtRSInfo gc_emlrtRSI = { 44, /* lineNo */
  "minOrMax",                          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax.m"/* pathName */
};

static emlrtRSInfo hc_emlrtRSI = { 79, /* lineNo */
  "maximum",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax.m"/* pathName */
};

static emlrtRSInfo ic_emlrtRSI = { 145,/* lineNo */
  "unaryMinOrMax",                     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo jc_emlrtRSI = { 1019,/* lineNo */
  "maxRealVectorOmitNaN",              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo kc_emlrtRSI = { 932,/* lineNo */
  "minOrMaxRealVector",                /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo lc_emlrtRSI = { 992,/* lineNo */
  "minOrMaxRealVectorKernel",          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo mc_emlrtRSI = { 924,/* lineNo */
  "minOrMaxRealVector",                /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo nc_emlrtRSI = { 975,/* lineNo */
  "findFirst",                         /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo oc_emlrtRSI = { 48, /* lineNo */
  "eml_mtimes_helper",                 /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_helper.m"/* pathName */
};

static emlrtRTEInfo j_emlrtRTEI = { 95,/* lineNo */
  27,                                  /* colNo */
  "unaryMinOrMax",                     /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pName */
};

static emlrtBCInfo y_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  40,                                  /* lineNo */
  59,                                  /* colNo */
  "uu",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ab_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  40,                                  /* lineNo */
  36,                                  /* colNo */
  "uu",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo bb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  37,                                  /* lineNo */
  12,                                  /* colNo */
  "xx",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo j_emlrtECI = { -1,  /* nDims */
  30,                                  /* lineNo */
  23,                                  /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m"/* pName */
};

static emlrtECInfo k_emlrtECI = { -1,  /* nDims */
  29,                                  /* lineNo */
  23,                                  /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m"/* pName */
};

static emlrtECInfo l_emlrtECI = { -1,  /* nDims */
  25,                                  /* lineNo */
  13,                                  /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m"/* pName */
};

static emlrtECInfo m_emlrtECI = { -1,  /* nDims */
  23,                                  /* lineNo */
  18,                                  /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m"/* pName */
};

static emlrtBCInfo cb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  23,                                  /* lineNo */
  30,                                  /* colNo */
  "uu",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo l_emlrtRTEI = { 20,/* lineNo */
  7,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m"/* pName */
};

static emlrtDCInfo l_emlrtDCI = { 16,  /* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo m_emlrtDCI = { 16,  /* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m",/* pName */
  4                                    /* checkKind */
};

static emlrtDCInfo n_emlrtDCI = { 17,  /* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo o_emlrtDCI = { 21,  /* lineNo */
  5,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo p_emlrtDCI = { 21,  /* lineNo */
  5,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m",/* pName */
  4                                    /* checkKind */
};

static emlrtBCInfo db_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  29,                                  /* lineNo */
  5,                                   /* colNo */
  "h1_du",                             /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo eb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  30,                                  /* lineNo */
  5,                                   /* colNo */
  "h2_du",                             /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo n_emlrtRTEI = { 123,/* lineNo */
  23,                                  /* colNo */
  "dynamic_size_checks",               /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_helper.m"/* pName */
};

static emlrtRTEInfo o_emlrtRTEI = { 118,/* lineNo */
  23,                                  /* colNo */
  "dynamic_size_checks",               /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_helper.m"/* pName */
};

static emlrtRTEInfo jb_emlrtRTEI = { 16,/* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m"/* pName */
};

static emlrtRTEInfo kb_emlrtRTEI = { 17,/* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m"/* pName */
};

static emlrtRTEInfo lb_emlrtRTEI = { 21,/* lineNo */
  5,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m"/* pName */
};

static emlrtRTEInfo mb_emlrtRTEI = { 145,/* lineNo */
  38,                                  /* colNo */
  "unaryMinOrMax",                     /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pName */
};

static emlrtRTEInfo nb_emlrtRTEI = { 8,/* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m"/* pName */
};

static emlrtRTEInfo ob_emlrtRTEI = { 9,/* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship_increment\\user_ocp.m"/* pName */
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
  if (12 != innerDimB) {
    if (b_size[1] == 1) {
      emlrtErrorWithMessageIdR2018a(sp, &o_emlrtRTEI,
        "Coder:toolbox:mtimes_noDynamicScalarExpansion",
        "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(sp, &n_emlrtRTEI, "MATLAB:innerdim",
        "MATLAB:innerdim", 0);
    }
  }
}

static void dynamic_size_checks(const emlrtStack *sp, const int32_T a_size[2],
  int32_T innerDimA)
{
  if (innerDimA != 12) {
    if (a_size[1] == 1) {
      emlrtErrorWithMessageIdR2018a(sp, &o_emlrtRTEI,
        "Coder:toolbox:mtimes_noDynamicScalarExpansion",
        "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(sp, &n_emlrtRTEI, "MATLAB:innerdim",
        "MATLAB:innerdim", 0);
    }
  }
}

void user_ocp(const emlrtStack *sp, const emxArray_real_T *xx, const
              emxArray_real_T *uu, real_T p_ode_Nship, real_T p_uparam_Np, const
              real_T p_ocp_Q[144], const real_T p_ocp_R[144], const real_T
              p_ocp_Rdu[144], const real_T p_ocp_xd_his[240], real_T p_ocp_F_max,
              real_T p_ocp_a_max, const real_T p_ocp_u_last[12], real_T *J,
              real_T *g)
{
  emxArray_real_T *uplus_min;
  emxArray_real_T *uplus_max;
  emxArray_real_T *h1_du;
  real_T b_p_ocp_F_max[6];
  int32_T k;
  int32_T idx;
  int32_T loop_ub;
  emxArray_real_T *h2_du;
  emxArray_real_T *u_plus;
  real_T d;
  int32_T i;
  real_T d1;
  int32_T b_loop_ub;
  int32_T n;
  int32_T j;
  real_T u_new[12];
  real_T uu_data[240];
  boolean_T exitg1;
  real_T e[12];
  real_T h2;
  real_T d2;
  int32_T uu_size[2];
  real_T b_e;
  real_T c_e;
  real_T b_uu_data[240];
  real_T d3;
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
  emxInit_real_T(sp, &uplus_min, 1, &nb_emlrtRTEI, true);
  emxInit_real_T(sp, &uplus_max, 1, &ob_emlrtRTEI, true);
  emxInit_real_T(sp, &h1_du, 1, &jb_emlrtRTEI, true);

  /* ------------------------------------------------------------------------------- */
  /*  pdf_mpc package: Example 1 - Definition of the user_ocp map */
  /* ------------------------------------------------------------------------------- */
  *J = 0.0;
  b_p_ocp_F_max[0] = 0.0;
  b_p_ocp_F_max[1] = 0.0;
  b_p_ocp_F_max[2] = 0.0;
  b_p_ocp_F_max[3] = -p_ocp_a_max;
  b_p_ocp_F_max[4] = -p_ocp_a_max;
  b_p_ocp_F_max[5] = -p_ocp_a_max;
  st.site = &tb_emlrtRSI;
  repmat(&st, b_p_ocp_F_max, p_ode_Nship, uplus_min);
  b_p_ocp_F_max[0] = p_ocp_F_max;
  b_p_ocp_F_max[1] = p_ocp_F_max;
  b_p_ocp_F_max[2] = p_ocp_F_max;
  b_p_ocp_F_max[3] = p_ocp_a_max;
  b_p_ocp_F_max[4] = p_ocp_a_max;
  b_p_ocp_F_max[5] = p_ocp_a_max;
  st.site = &ub_emlrtRSI;
  repmat(&st, b_p_ocp_F_max, p_ode_Nship, uplus_max);

  /*  du_max = repmat([100, 100, 100, 20/180*pi, 20/180*pi, 20/180*pi]', p_ode.Nship, 1); */
  /*  xd=[p_ocp.rd;0;0;0]; */
  if (!(p_uparam_Np >= 0.0)) {
    emlrtNonNegativeCheckR2012b(p_uparam_Np, &m_emlrtDCI, sp);
  }

  k = (int32_T)muDoubleScalarFloor(p_uparam_Np);
  if (p_uparam_Np != k) {
    emlrtIntegerCheckR2012b(p_uparam_Np, &l_emlrtDCI, sp);
  }

  idx = h1_du->size[0];
  loop_ub = (int32_T)p_uparam_Np;
  h1_du->size[0] = loop_ub;
  emxEnsureCapacity_real_T(sp, h1_du, idx, &jb_emlrtRTEI);
  if (p_uparam_Np != k) {
    emlrtIntegerCheckR2012b(p_uparam_Np, &l_emlrtDCI, sp);
  }

  for (idx = 0; idx < loop_ub; idx++) {
    h1_du->data[idx] = 0.0;
  }

  emxInit_real_T(sp, &h2_du, 1, &kb_emlrtRTEI, true);
  if (p_uparam_Np != k) {
    emlrtIntegerCheckR2012b(p_uparam_Np, &n_emlrtDCI, sp);
  }

  loop_ub = (int32_T)p_uparam_Np;
  idx = h2_du->size[0];
  h2_du->size[0] = (int32_T)p_uparam_Np;
  emxEnsureCapacity_real_T(sp, h2_du, idx, &kb_emlrtRTEI);
  if ((int32_T)p_uparam_Np != k) {
    emlrtIntegerCheckR2012b(p_uparam_Np, &n_emlrtDCI, sp);
  }

  for (k = 0; k < loop_ub; k++) {
    h2_du->data[k] = 0.0;
  }

  emlrtForLoopVectorCheckR2012b(1.0, 1.0, p_uparam_Np, mxDOUBLE_CLASS, (int32_T)
    p_uparam_Np, &l_emlrtRTEI, sp);
  emxInit_real_T(sp, &u_plus, 1, &lb_emlrtRTEI, true);
  if (0 <= loop_ub - 1) {
    d = 6.0 * p_ode_Nship;
    d1 = 6.0 * p_ode_Nship;
    b_loop_ub = (int32_T)d1;
  }

  for (i = 0; i < loop_ub; i++) {
    if (!(d >= 0.0)) {
      emlrtNonNegativeCheckR2012b(d, &p_emlrtDCI, sp);
    }

    if (d != (int32_T)muDoubleScalarFloor(d)) {
      emlrtIntegerCheckR2012b(d, &o_emlrtDCI, sp);
    }

    k = u_plus->size[0];
    u_plus->size[0] = (int32_T)d;
    emxEnsureCapacity_real_T(sp, u_plus, k, &lb_emlrtRTEI);
    if (!(d1 >= 0.0)) {
      emlrtNonNegativeCheckR2012b(d1, &p_emlrtDCI, sp);
    }

    if (d1 != (int32_T)muDoubleScalarFloor(d1)) {
      emlrtIntegerCheckR2012b(d1, &o_emlrtDCI, sp);
    }

    for (k = 0; k < b_loop_ub; k++) {
      u_plus->data[k] = 0.0;
    }

    for (j = 0; j <= i; j++) {
      k = (int32_T)(j + 1U);
      if (k > uu->size[0]) {
        emlrtDynamicBoundsCheckR2012b(k, 1, uu->size[0], &cb_emlrtBCI, sp);
      }

      n = uu->size[1];
      idx = uu->size[1];
      for (k = 0; k < n; k++) {
        uu_data[k] = uu->data[j + uu->size[0] * k];
      }

      n = u_plus->size[0];
      if (u_plus->size[0] != idx) {
        emlrtSizeEqCheck1DR2012b(u_plus->size[0], idx, &m_emlrtECI, sp);
      }

      for (k = 0; k < n; k++) {
        u_plus->data[k] += uu_data[k];
      }

      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(sp);
      }
    }

    if (12 != u_plus->size[0]) {
      emlrtSizeEqCheck1DR2012b(12, u_plus->size[0], &l_emlrtECI, sp);
    }

    for (k = 0; k < 12; k++) {
      u_new[k] = p_ocp_u_last[k] + u_plus->data[k];
    }

    if (12 != uplus_max->size[0]) {
      emlrtSizeEqCheck1DR2012b(12, uplus_max->size[0], &k_emlrtECI, sp);
    }

    st.site = &vb_emlrtRSI;
    for (k = 0; k < 12; k++) {
      e[k] = u_new[k] - uplus_max->data[k];
    }

    b_st.site = &fc_emlrtRSI;
    c_st.site = &gc_emlrtRSI;
    d_st.site = &hc_emlrtRSI;
    e_st.site = &ic_emlrtRSI;
    k = u_plus->size[0];
    u_plus->size[0] = 12;
    emxEnsureCapacity_real_T(&e_st, u_plus, k, &mb_emlrtRTEI);
    for (k = 0; k < 12; k++) {
      u_plus->data[k] = u_new[k] - uplus_max->data[k];
    }

    f_st.site = &jc_emlrtRSI;
    g_st.site = &mc_emlrtRSI;
    if (!muDoubleScalarIsNaN(u_plus->data[0])) {
      idx = 1;
    } else {
      idx = 0;
      h_st.site = &nc_emlrtRSI;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= 12)) {
        if (!muDoubleScalarIsNaN(u_plus->data[k - 1])) {
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
        emlrtDynamicBoundsCheckR2012b(k, 1, h1_du->size[0], &db_emlrtBCI, &f_st);
      }

      h1_du->data[k - 1] = e[0];
    } else {
      g_st.site = &kc_emlrtRSI;
      h2 = e[idx - 1];
      j = idx + 1;
      h_st.site = &lc_emlrtRSI;
      for (k = j; k < 13; k++) {
        d2 = e[k - 1];
        if (h2 < d2) {
          h2 = d2;
        }
      }

      k = (int32_T)(i + 1U);
      if (k > h1_du->size[0]) {
        emlrtDynamicBoundsCheckR2012b(k, 1, h1_du->size[0], &db_emlrtBCI, &f_st);
      }

      h1_du->data[k - 1] = h2;
    }

    if (12 != uplus_min->size[0]) {
      emlrtSizeEqCheck1DR2012b(12, uplus_min->size[0], &j_emlrtECI, sp);
    }

    st.site = &wb_emlrtRSI;
    for (k = 0; k < 12; k++) {
      e[k] = -u_new[k] + uplus_min->data[k];
    }

    b_st.site = &fc_emlrtRSI;
    c_st.site = &gc_emlrtRSI;
    d_st.site = &hc_emlrtRSI;
    e_st.site = &ic_emlrtRSI;
    k = u_plus->size[0];
    u_plus->size[0] = 12;
    emxEnsureCapacity_real_T(&e_st, u_plus, k, &mb_emlrtRTEI);
    for (k = 0; k < 12; k++) {
      u_plus->data[k] = -u_new[k] + uplus_min->data[k];
    }

    f_st.site = &jc_emlrtRSI;
    g_st.site = &mc_emlrtRSI;
    if (!muDoubleScalarIsNaN(u_plus->data[0])) {
      idx = 1;
    } else {
      idx = 0;
      h_st.site = &nc_emlrtRSI;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= 12)) {
        if (!muDoubleScalarIsNaN(u_plus->data[k - 1])) {
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
        emlrtDynamicBoundsCheckR2012b(k, 1, h2_du->size[0], &eb_emlrtBCI, &f_st);
      }

      h2_du->data[k - 1] = e[0];
    } else {
      g_st.site = &kc_emlrtRSI;
      h2 = e[idx - 1];
      j = idx + 1;
      h_st.site = &lc_emlrtRSI;
      for (k = j; k < 13; k++) {
        d2 = e[k - 1];
        if (h2 < d2) {
          h2 = d2;
        }
      }

      k = (int32_T)(i + 1U);
      if (k > h2_du->size[0]) {
        emlrtDynamicBoundsCheckR2012b(k, 1, h2_du->size[0], &eb_emlrtBCI, &f_st);
      }

      h2_du->data[k - 1] = h2;
    }

    /*      e = xx(i+1,:)' - xd; */
    /* xd_his(1, :)代表传进来一个点，xd_his(i, :)代表传进来一条轨迹 */
    /*  编队好像传进去一个点稳定性更高一点 */
    k = (int32_T)(i + 2U);
    if ((k < 1) || (k > xx->size[0])) {
      emlrtDynamicBoundsCheckR2012b(k, 1, xx->size[0], &bb_emlrtBCI, sp);
    }

    for (k = 0; k < 12; k++) {
      e[k] = xx->data[(i + xx->size[0] * k) + 1] - p_ocp_xd_his[20 * k];
    }

    k = i + 1;
    if (k > uu->size[0]) {
      emlrtDynamicBoundsCheckR2012b(k, 1, uu->size[0], &y_emlrtBCI, sp);
    }

    st.site = &xb_emlrtRSI;
    k = i + 1;
    if (k > uu->size[0]) {
      emlrtDynamicBoundsCheckR2012b(k, 1, uu->size[0], &ab_emlrtBCI, &st);
    }

    uu_size[0] = 1;
    uu_size[1] = uu->size[1];
    b_st.site = &oc_emlrtRSI;
    dynamic_size_checks(&b_st, uu_size, uu->size[1]);
    st.site = &xb_emlrtRSI;
    uu_size[0] = 1;
    uu_size[1] = uu->size[1];
    b_st.site = &oc_emlrtRSI;
    b_dynamic_size_checks(&b_st, uu_size, uu->size[1]);
    h2 = 0.0;
    for (k = 0; k < 12; k++) {
      d2 = 0.0;
      for (idx = 0; idx < 12; idx++) {
        d2 += e[idx] * p_ocp_Q[idx + 12 * k];
      }

      h2 += d2 * e[k];
    }

    n = uu->size[1];
    for (k = 0; k < n; k++) {
      d2 = uu->data[i + uu->size[0] * k];
      b_uu_data[k] = d2;
      uu_data[k] = d2;
    }

    b_e = 0.0;
    c_e = 0.0;
    for (k = 0; k < 12; k++) {
      d2 = 0.0;
      d3 = 0.0;
      for (idx = 0; idx < 12; idx++) {
        n = idx + 12 * k;
        d2 += b_uu_data[idx] * p_ocp_Rdu[n];
        d3 += u_new[idx] * p_ocp_R[n];
      }

      b_e += d2 * uu_data[k];
      c_e += d3 * u_new[k];
    }

    *J += (h2 + b_e) + c_e;

    /*      J = J + (e' * p_ocp.Q * e + uu(i, :) * p_ocp.R * uu(i, :)'); */
    /*      J = J + e' * p_ocp.Q * e ; */
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }

  emxFree_real_T(&u_plus);
  emxFree_real_T(&uplus_max);
  emxFree_real_T(&uplus_min);
  st.site = &yb_emlrtRSI;
  b_st.site = &fc_emlrtRSI;
  c_st.site = &gc_emlrtRSI;
  d_st.site = &hc_emlrtRSI;
  if (h1_du->size[0] < 1) {
    emlrtErrorWithMessageIdR2018a(&d_st, &j_emlrtRTEI,
      "Coder:toolbox:eml_min_or_max_varDimZero",
      "Coder:toolbox:eml_min_or_max_varDimZero", 0);
  }

  e_st.site = &ic_emlrtRSI;
  f_st.site = &jc_emlrtRSI;
  n = h1_du->size[0];
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
    g_st.site = &mc_emlrtRSI;
    if (!muDoubleScalarIsNaN(h1_du->data[0])) {
      idx = 1;
    } else {
      idx = 0;
      h_st.site = &nc_emlrtRSI;
      if (h1_du->size[0] > 2147483646) {
        i_st.site = &ec_emlrtRSI;
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
      g_st.site = &kc_emlrtRSI;
      *g = h1_du->data[idx - 1];
      j = idx + 1;
      h_st.site = &lc_emlrtRSI;
      if ((idx + 1 <= h1_du->size[0]) && (h1_du->size[0] > 2147483646)) {
        i_st.site = &ec_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }

      for (k = j; k <= n; k++) {
        d = h1_du->data[k - 1];
        if (*g < d) {
          *g = d;
        }
      }
    }
  }

  emxFree_real_T(&h1_du);
  st.site = &ac_emlrtRSI;
  b_st.site = &fc_emlrtRSI;
  c_st.site = &gc_emlrtRSI;
  d_st.site = &hc_emlrtRSI;
  if (h2_du->size[0] < 1) {
    emlrtErrorWithMessageIdR2018a(&d_st, &j_emlrtRTEI,
      "Coder:toolbox:eml_min_or_max_varDimZero",
      "Coder:toolbox:eml_min_or_max_varDimZero", 0);
  }

  e_st.site = &ic_emlrtRSI;
  f_st.site = &jc_emlrtRSI;
  n = h2_du->size[0];
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
    g_st.site = &mc_emlrtRSI;
    if (!muDoubleScalarIsNaN(h2_du->data[0])) {
      idx = 1;
    } else {
      idx = 0;
      h_st.site = &nc_emlrtRSI;
      if (h2_du->size[0] > 2147483646) {
        i_st.site = &ec_emlrtRSI;
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
      g_st.site = &kc_emlrtRSI;
      h2 = h2_du->data[idx - 1];
      j = idx + 1;
      h_st.site = &lc_emlrtRSI;
      if ((idx + 1 <= h2_du->size[0]) && (h2_du->size[0] > 2147483646)) {
        i_st.site = &ec_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }

      for (k = j; k <= n; k++) {
        d = h2_du->data[k - 1];
        if (h2 < d) {
          h2 = d;
        }
      }
    }
  }

  emxFree_real_T(&h2_du);
  st.site = &bc_emlrtRSI;
  b_st.site = &fc_emlrtRSI;
  c_st.site = &gc_emlrtRSI;
  d_st.site = &hc_emlrtRSI;
  e_st.site = &ic_emlrtRSI;
  f_st.site = &jc_emlrtRSI;
  if ((*g < h2) || (muDoubleScalarIsNaN(*g) && (!muDoubleScalarIsNaN(h2)))) {
    *g = h2;
  }

  /*  g = -1;  */
  /* ------------------------------------------------------------------------------- */
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (user_ocp.c) */
