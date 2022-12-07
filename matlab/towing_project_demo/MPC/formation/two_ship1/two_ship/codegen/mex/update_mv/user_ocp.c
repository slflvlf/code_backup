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
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo sb_emlrtRSI = { 8,  /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pathName */
};

static emlrtRSInfo tb_emlrtRSI = { 27, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pathName */
};

static emlrtRSInfo ub_emlrtRSI = { 28, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pathName */
};

static emlrtRSInfo vb_emlrtRSI = { 37, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pathName */
};

static emlrtRSInfo wb_emlrtRSI = { 41, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pathName */
};

static emlrtRSInfo xb_emlrtRSI = { 42, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pathName */
};

static emlrtRSInfo yb_emlrtRSI = { 44, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pathName */
};

static emlrtRSInfo dc_emlrtRSI = { 14, /* lineNo */
  "max",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\datafun\\max.m"/* pathName */
};

static emlrtRSInfo ec_emlrtRSI = { 44, /* lineNo */
  "minOrMax",                          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax.m"/* pathName */
};

static emlrtRSInfo fc_emlrtRSI = { 79, /* lineNo */
  "maximum",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax.m"/* pathName */
};

static emlrtRSInfo gc_emlrtRSI = { 145,/* lineNo */
  "unaryMinOrMax",                     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo hc_emlrtRSI = { 1019,/* lineNo */
  "maxRealVectorOmitNaN",              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo ic_emlrtRSI = { 932,/* lineNo */
  "minOrMaxRealVector",                /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo jc_emlrtRSI = { 924,/* lineNo */
  "minOrMaxRealVector",                /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo kc_emlrtRSI = { 975,/* lineNo */
  "findFirst",                         /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo lc_emlrtRSI = { 992,/* lineNo */
  "minOrMaxRealVectorKernel",          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo mc_emlrtRSI = { 48, /* lineNo */
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
  37,                                  /* lineNo */
  57,                                  /* colNo */
  "uu",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ab_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  37,                                  /* lineNo */
  36,                                  /* colNo */
  "uu",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo bb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  34,                                  /* lineNo */
  12,                                  /* colNo */
  "xx",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo j_emlrtECI = { -1,  /* nDims */
  28,                                  /* lineNo */
  23,                                  /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtECInfo k_emlrtECI = { -1,  /* nDims */
  27,                                  /* lineNo */
  23,                                  /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtECInfo l_emlrtECI = { -1,  /* nDims */
  25,                                  /* lineNo */
  11,                                  /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtECInfo m_emlrtECI = { -1,  /* nDims */
  24,                                  /* lineNo */
  11,                                  /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtECInfo n_emlrtECI = { -1,  /* nDims */
  22,                                  /* lineNo */
  14,                                  /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtBCInfo cb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  22,                                  /* lineNo */
  29,                                  /* colNo */
  "uu",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo db_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  22,                                  /* lineNo */
  17,                                  /* colNo */
  "uu",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo o_emlrtECI = { -1,  /* nDims */
  20,                                  /* lineNo */
  14,                                  /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtBCInfo eb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  20,                                  /* lineNo */
  17,                                  /* colNo */
  "uu",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo l_emlrtRTEI = { 17,/* lineNo */
  7,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtDCInfo l_emlrtDCI = { 15,  /* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo m_emlrtDCI = { 15,  /* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  4                                    /* checkKind */
};

static emlrtDCInfo n_emlrtDCI = { 16,  /* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo fb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  27,                                  /* lineNo */
  5,                                   /* colNo */
  "h1_du",                             /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo gb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  28,                                  /* lineNo */
  5,                                   /* colNo */
  "h2_du",                             /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
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

static emlrtRTEInfo jb_emlrtRTEI = { 15,/* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtRTEInfo kb_emlrtRTEI = { 16,/* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtRTEInfo lb_emlrtRTEI = { 27,/* lineNo */
  23,                                  /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtRTEInfo mb_emlrtRTEI = { 28,/* lineNo */
  23,                                  /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtRTEInfo nb_emlrtRTEI = { 8,/* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

/* Function Declarations */
static void b_dynamic_size_checks(const emlrtStack *sp, const int32_T b_size[2],
  int32_T innerDimB);
static void c_dynamic_size_checks(const emlrtStack *sp, const int32_T a_size[1],
  int32_T innerDimA);
static void d_dynamic_size_checks(const emlrtStack *sp, const int32_T b_size[1],
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

static void c_dynamic_size_checks(const emlrtStack *sp, const int32_T a_size[1],
  int32_T innerDimA)
{
  if (innerDimA != 12) {
    if (a_size[0] == 1) {
      emlrtErrorWithMessageIdR2018a(sp, &o_emlrtRTEI,
        "Coder:toolbox:mtimes_noDynamicScalarExpansion",
        "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(sp, &n_emlrtRTEI, "MATLAB:innerdim",
        "MATLAB:innerdim", 0);
    }
  }
}

static void d_dynamic_size_checks(const emlrtStack *sp, const int32_T b_size[1],
  int32_T innerDimB)
{
  if (12 != innerDimB) {
    if (b_size[0] == 1) {
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
              p_ocp_Rdu[144], const real_T p_ocp_xd_his[240], real_T
              p_ocp_dF_max, real_T p_ocp_da_max, const real_T p_ocp_u_last[12],
              real_T *J, real_T *g)
{
  emxArray_real_T *du_max;
  emxArray_real_T *h1_du;
  real_T b_p_ocp_dF_max[6];
  int32_T a;
  int32_T n;
  int32_T loop_ub;
  emxArray_real_T *h2_du;
  emxArray_real_T *varargin_1;
  int32_T i;
  uint32_T u;
  int32_T du_size[1];
  real_T du_data[240];
  int32_T idx;
  real_T uu_data[240];
  int32_T k;
  boolean_T exitg1;
  real_T d;
  real_T h2;
  real_T e[12];
  int32_T uu_size[2];
  real_T b_e;
  real_T b_uu_data[240];
  real_T c_e;
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
  emxInit_real_T(sp, &du_max, 1, &nb_emlrtRTEI, true);
  emxInit_real_T(sp, &h1_du, 1, &jb_emlrtRTEI, true);

  /* ------------------------------------------------------------------------------- */
  /*  pdf_mpc package: Example 1 - Definition of the user_ocp map */
  /* ------------------------------------------------------------------------------- */
  *J = 0.0;
  b_p_ocp_dF_max[0] = p_ocp_dF_max;
  b_p_ocp_dF_max[1] = p_ocp_dF_max;
  b_p_ocp_dF_max[2] = p_ocp_dF_max;
  b_p_ocp_dF_max[3] = p_ocp_da_max;
  b_p_ocp_dF_max[4] = p_ocp_da_max;
  b_p_ocp_dF_max[5] = p_ocp_da_max;
  st.site = &sb_emlrtRSI;
  repmat(&st, b_p_ocp_dF_max, p_ode_Nship, du_max);

  /*  du_max = repmat([100, 100, 100, 20/180*pi, 20/180*pi, 20/180*pi]', p_ode.Nship, 1); */
  /*  xd=[p_ocp.rd;0;0;0]; */
  if (!(p_uparam_Np >= 0.0)) {
    emlrtNonNegativeCheckR2012b(p_uparam_Np, &m_emlrtDCI, sp);
  }

  a = (int32_T)muDoubleScalarFloor(p_uparam_Np);
  if (p_uparam_Np != a) {
    emlrtIntegerCheckR2012b(p_uparam_Np, &l_emlrtDCI, sp);
  }

  n = h1_du->size[0];
  loop_ub = (int32_T)p_uparam_Np;
  h1_du->size[0] = loop_ub;
  emxEnsureCapacity_real_T(sp, h1_du, n, &jb_emlrtRTEI);
  if (p_uparam_Np != a) {
    emlrtIntegerCheckR2012b(p_uparam_Np, &l_emlrtDCI, sp);
  }

  for (n = 0; n < loop_ub; n++) {
    h1_du->data[n] = 0.0;
  }

  emxInit_real_T(sp, &h2_du, 1, &kb_emlrtRTEI, true);
  if (p_uparam_Np != a) {
    emlrtIntegerCheckR2012b(p_uparam_Np, &n_emlrtDCI, sp);
  }

  loop_ub = (int32_T)p_uparam_Np;
  n = h2_du->size[0];
  h2_du->size[0] = (int32_T)p_uparam_Np;
  emxEnsureCapacity_real_T(sp, h2_du, n, &kb_emlrtRTEI);
  if ((int32_T)p_uparam_Np != a) {
    emlrtIntegerCheckR2012b(p_uparam_Np, &n_emlrtDCI, sp);
  }

  for (a = 0; a < loop_ub; a++) {
    h2_du->data[a] = 0.0;
  }

  emlrtForLoopVectorCheckR2012b(1.0, 1.0, p_uparam_Np, mxDOUBLE_CLASS, (int32_T)
    p_uparam_Np, &l_emlrtRTEI, sp);
  emxInit_real_T(sp, &varargin_1, 1, &lb_emlrtRTEI, true);
  for (i = 0; i < loop_ub; i++) {
    u = i + 1U;
    if (u == 1U) {
      /*          du = uu(1, :)' - p_ode.u0;%这里没有问题，u0 = u_last, p_ode.u0在这里是实时更新的 */
      if (1 > uu->size[0]) {
        emlrtDynamicBoundsCheckR2012b(1, 1, uu->size[0], &eb_emlrtBCI, sp);
      }

      n = uu->size[1];
      du_size[0] = uu->size[1];
      for (a = 0; a < n; a++) {
        du_data[a] = uu->data[uu->size[0] * a];
      }

      if (du_size[0] != 12) {
        emlrtSizeEqCheck1DR2012b(du_size[0], 12, &o_emlrtECI, sp);
      }

      du_size[0] = 12;
      for (a = 0; a < 12; a++) {
        du_data[a] -= p_ocp_u_last[a];
      }
    } else {
      if ((int32_T)u > uu->size[0]) {
        emlrtDynamicBoundsCheckR2012b((int32_T)u, 1, uu->size[0], &db_emlrtBCI,
          sp);
      }

      n = uu->size[1];
      du_size[0] = uu->size[1];
      for (a = 0; a < n; a++) {
        du_data[a] = uu->data[i + uu->size[0] * a];
      }

      if ((i < 1) || (i > uu->size[0])) {
        emlrtDynamicBoundsCheckR2012b(i, 1, uu->size[0], &cb_emlrtBCI, sp);
      }

      n = uu->size[1];
      idx = uu->size[1];
      for (a = 0; a < n; a++) {
        uu_data[a] = uu->data[(i + uu->size[0] * a) - 1];
      }

      n = du_size[0];
      if (du_size[0] != idx) {
        emlrtSizeEqCheck1DR2012b(du_size[0], idx, &n_emlrtECI, sp);
      }

      for (a = 0; a < n; a++) {
        du_data[a] -= uu_data[a];
      }
    }

    n = du_size[0];
    if (du_size[0] != du_max->size[0]) {
      emlrtSizeEqCheck1DR2012b(du_size[0], du_max->size[0], &m_emlrtECI, sp);
    }

    if (du_size[0] != du_max->size[0]) {
      emlrtSizeEqCheck1DR2012b(du_size[0], du_max->size[0], &l_emlrtECI, sp);
    }

    if (du_size[0] != du_max->size[0]) {
      emlrtSizeEqCheck1DR2012b(du_size[0], du_max->size[0], &k_emlrtECI, sp);
    }

    st.site = &tb_emlrtRSI;
    a = varargin_1->size[0];
    varargin_1->size[0] = du_size[0];
    emxEnsureCapacity_real_T(&st, varargin_1, a, &lb_emlrtRTEI);
    for (a = 0; a < n; a++) {
      varargin_1->data[a] = du_data[a] - du_max->data[a];
    }

    b_st.site = &dc_emlrtRSI;
    c_st.site = &ec_emlrtRSI;
    d_st.site = &fc_emlrtRSI;
    if (varargin_1->size[0] < 1) {
      emlrtErrorWithMessageIdR2018a(&d_st, &j_emlrtRTEI,
        "Coder:toolbox:eml_min_or_max_varDimZero",
        "Coder:toolbox:eml_min_or_max_varDimZero", 0);
    }

    e_st.site = &gc_emlrtRSI;
    f_st.site = &hc_emlrtRSI;
    n = varargin_1->size[0];
    if (varargin_1->size[0] <= 2) {
      if (varargin_1->size[0] == 1) {
        a = (int32_T)(i + 1U);
        if (a > h1_du->size[0]) {
          emlrtDynamicBoundsCheckR2012b(a, 1, h1_du->size[0], &fb_emlrtBCI,
            &f_st);
        }

        h1_du->data[a - 1] = varargin_1->data[0];
      } else if ((varargin_1->data[0] < varargin_1->data[1]) ||
                 (muDoubleScalarIsNaN(varargin_1->data[0]) &&
                  (!muDoubleScalarIsNaN(varargin_1->data[1])))) {
        a = (int32_T)(i + 1U);
        if (a > h1_du->size[0]) {
          emlrtDynamicBoundsCheckR2012b(a, 1, h1_du->size[0], &fb_emlrtBCI,
            &f_st);
        }

        h1_du->data[a - 1] = varargin_1->data[1];
      } else {
        a = (int32_T)(i + 1U);
        if (a > h1_du->size[0]) {
          emlrtDynamicBoundsCheckR2012b(a, 1, h1_du->size[0], &fb_emlrtBCI,
            &f_st);
        }

        h1_du->data[a - 1] = varargin_1->data[0];
      }
    } else {
      g_st.site = &jc_emlrtRSI;
      if (!muDoubleScalarIsNaN(varargin_1->data[0])) {
        idx = 1;
      } else {
        idx = 0;
        h_st.site = &kc_emlrtRSI;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k <= varargin_1->size[0])) {
          if (!muDoubleScalarIsNaN(varargin_1->data[k - 1])) {
            idx = k;
            exitg1 = true;
          } else {
            k++;
          }
        }
      }

      if (idx == 0) {
        a = (int32_T)(i + 1U);
        if (a > h1_du->size[0]) {
          emlrtDynamicBoundsCheckR2012b(a, 1, h1_du->size[0], &fb_emlrtBCI,
            &f_st);
        }

        h1_du->data[a - 1] = varargin_1->data[0];
      } else {
        g_st.site = &ic_emlrtRSI;
        h2 = varargin_1->data[idx - 1];
        a = idx + 1;
        h_st.site = &lc_emlrtRSI;
        for (k = a; k <= n; k++) {
          d = varargin_1->data[k - 1];
          if (h2 < d) {
            h2 = d;
          }
        }

        a = (int32_T)(i + 1U);
        if (a > h1_du->size[0]) {
          emlrtDynamicBoundsCheckR2012b(a, 1, h1_du->size[0], &fb_emlrtBCI,
            &f_st);
        }

        h1_du->data[a - 1] = h2;
      }
    }

    n = du_size[0];
    for (a = 0; a < n; a++) {
      uu_data[a] = -du_data[a];
    }

    if (du_size[0] != du_max->size[0]) {
      emlrtSizeEqCheck1DR2012b(du_size[0], du_max->size[0], &j_emlrtECI, sp);
    }

    st.site = &ub_emlrtRSI;
    a = varargin_1->size[0];
    varargin_1->size[0] = du_size[0];
    emxEnsureCapacity_real_T(&st, varargin_1, a, &mb_emlrtRTEI);
    for (a = 0; a < n; a++) {
      varargin_1->data[a] = uu_data[a] - du_max->data[a];
    }

    b_st.site = &dc_emlrtRSI;
    c_st.site = &ec_emlrtRSI;
    d_st.site = &fc_emlrtRSI;
    if (varargin_1->size[0] < 1) {
      emlrtErrorWithMessageIdR2018a(&d_st, &j_emlrtRTEI,
        "Coder:toolbox:eml_min_or_max_varDimZero",
        "Coder:toolbox:eml_min_or_max_varDimZero", 0);
    }

    e_st.site = &gc_emlrtRSI;
    f_st.site = &hc_emlrtRSI;
    n = varargin_1->size[0];
    if (varargin_1->size[0] <= 2) {
      if (varargin_1->size[0] == 1) {
        a = (int32_T)(i + 1U);
        if (a > h2_du->size[0]) {
          emlrtDynamicBoundsCheckR2012b(a, 1, h2_du->size[0], &gb_emlrtBCI,
            &f_st);
        }

        h2_du->data[a - 1] = varargin_1->data[0];
      } else if ((varargin_1->data[0] < varargin_1->data[1]) ||
                 (muDoubleScalarIsNaN(varargin_1->data[0]) &&
                  (!muDoubleScalarIsNaN(varargin_1->data[1])))) {
        a = (int32_T)(i + 1U);
        if (a > h2_du->size[0]) {
          emlrtDynamicBoundsCheckR2012b(a, 1, h2_du->size[0], &gb_emlrtBCI,
            &f_st);
        }

        h2_du->data[a - 1] = varargin_1->data[1];
      } else {
        a = (int32_T)(i + 1U);
        if (a > h2_du->size[0]) {
          emlrtDynamicBoundsCheckR2012b(a, 1, h2_du->size[0], &gb_emlrtBCI,
            &f_st);
        }

        h2_du->data[a - 1] = varargin_1->data[0];
      }
    } else {
      g_st.site = &jc_emlrtRSI;
      if (!muDoubleScalarIsNaN(varargin_1->data[0])) {
        idx = 1;
      } else {
        idx = 0;
        h_st.site = &kc_emlrtRSI;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k <= varargin_1->size[0])) {
          if (!muDoubleScalarIsNaN(varargin_1->data[k - 1])) {
            idx = k;
            exitg1 = true;
          } else {
            k++;
          }
        }
      }

      if (idx == 0) {
        a = (int32_T)(i + 1U);
        if (a > h2_du->size[0]) {
          emlrtDynamicBoundsCheckR2012b(a, 1, h2_du->size[0], &gb_emlrtBCI,
            &f_st);
        }

        h2_du->data[a - 1] = varargin_1->data[0];
      } else {
        g_st.site = &ic_emlrtRSI;
        h2 = varargin_1->data[idx - 1];
        a = idx + 1;
        h_st.site = &lc_emlrtRSI;
        for (k = a; k <= n; k++) {
          d = varargin_1->data[k - 1];
          if (h2 < d) {
            h2 = d;
          }
        }

        a = (int32_T)(i + 1U);
        if (a > h2_du->size[0]) {
          emlrtDynamicBoundsCheckR2012b(a, 1, h2_du->size[0], &gb_emlrtBCI,
            &f_st);
        }

        h2_du->data[a - 1] = h2;
      }
    }

    /*      e = xx(i+1,:)' - xd; */
    /* xd_his(1, :)代表传进来一个点，xd_his(i, :)代表传进来一条轨迹 */
    /*  编队好像传进去一个点稳定性更高一点 */
    a = (int32_T)(i + 2U);
    if ((a < 1) || (a > xx->size[0])) {
      emlrtDynamicBoundsCheckR2012b(a, 1, xx->size[0], &bb_emlrtBCI, sp);
    }

    for (a = 0; a < 12; a++) {
      e[a] = xx->data[(i + xx->size[0] * a) + 1] - p_ocp_xd_his[20 * a];
    }

    a = i + 1;
    if (a > uu->size[0]) {
      emlrtDynamicBoundsCheckR2012b(a, 1, uu->size[0], &y_emlrtBCI, sp);
    }

    st.site = &vb_emlrtRSI;
    a = i + 1;
    if (a > uu->size[0]) {
      emlrtDynamicBoundsCheckR2012b(a, 1, uu->size[0], &ab_emlrtBCI, &st);
    }

    uu_size[0] = 1;
    uu_size[1] = uu->size[1];
    b_st.site = &mc_emlrtRSI;
    dynamic_size_checks(&b_st, uu_size, uu->size[1]);
    st.site = &vb_emlrtRSI;
    uu_size[0] = 1;
    uu_size[1] = uu->size[1];
    b_st.site = &mc_emlrtRSI;
    b_dynamic_size_checks(&b_st, uu_size, uu->size[1]);
    st.site = &vb_emlrtRSI;
    b_st.site = &mc_emlrtRSI;
    c_dynamic_size_checks(&b_st, du_size, du_size[0]);
    st.site = &vb_emlrtRSI;
    b_st.site = &mc_emlrtRSI;
    d_dynamic_size_checks(&b_st, du_size, du_size[0]);
    h2 = 0.0;
    for (a = 0; a < 12; a++) {
      d = 0.0;
      for (n = 0; n < 12; n++) {
        d += e[n] * p_ocp_Q[n + 12 * a];
      }

      h2 += d * e[a];
    }

    n = uu->size[1];
    for (a = 0; a < n; a++) {
      d = uu->data[i + uu->size[0] * a];
      b_uu_data[a] = d;
      uu_data[a] = d;
    }

    b_e = 0.0;
    for (a = 0; a < 12; a++) {
      d = 0.0;
      for (n = 0; n < 12; n++) {
        d += b_uu_data[n] * p_ocp_R[n + 12 * a];
      }

      b_e += d * uu_data[a];
    }

    n = du_size[0];
    if (0 <= n - 1) {
      memcpy(&b_uu_data[0], &du_data[0], n * sizeof(real_T));
    }

    c_e = 0.0;
    for (a = 0; a < 12; a++) {
      d = 0.0;
      for (n = 0; n < 12; n++) {
        d += b_uu_data[n] * p_ocp_Rdu[n + 12 * a];
      }

      c_e += d * du_data[a];
    }

    *J += (h2 + b_e) + c_e;

    /*      J = J + (e' * p_ocp.Q * e + uu(i, :) * p_ocp.R * uu(i, :)'); */
    /*      J = J + e' * p_ocp.Q * e ; */
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }

  emxFree_real_T(&varargin_1);
  emxFree_real_T(&du_max);
  st.site = &wb_emlrtRSI;
  b_st.site = &dc_emlrtRSI;
  c_st.site = &ec_emlrtRSI;
  d_st.site = &fc_emlrtRSI;
  if (h1_du->size[0] < 1) {
    emlrtErrorWithMessageIdR2018a(&d_st, &j_emlrtRTEI,
      "Coder:toolbox:eml_min_or_max_varDimZero",
      "Coder:toolbox:eml_min_or_max_varDimZero", 0);
  }

  e_st.site = &gc_emlrtRSI;
  f_st.site = &hc_emlrtRSI;
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
    g_st.site = &jc_emlrtRSI;
    if (!muDoubleScalarIsNaN(h1_du->data[0])) {
      idx = 1;
    } else {
      idx = 0;
      h_st.site = &kc_emlrtRSI;
      if (h1_du->size[0] > 2147483646) {
        i_st.site = &cc_emlrtRSI;
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
      g_st.site = &ic_emlrtRSI;
      *g = h1_du->data[idx - 1];
      a = idx + 1;
      h_st.site = &lc_emlrtRSI;
      if ((idx + 1 <= h1_du->size[0]) && (h1_du->size[0] > 2147483646)) {
        i_st.site = &cc_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }

      for (k = a; k <= n; k++) {
        d = h1_du->data[k - 1];
        if (*g < d) {
          *g = d;
        }
      }
    }
  }

  emxFree_real_T(&h1_du);
  st.site = &xb_emlrtRSI;
  b_st.site = &dc_emlrtRSI;
  c_st.site = &ec_emlrtRSI;
  d_st.site = &fc_emlrtRSI;
  if (h2_du->size[0] < 1) {
    emlrtErrorWithMessageIdR2018a(&d_st, &j_emlrtRTEI,
      "Coder:toolbox:eml_min_or_max_varDimZero",
      "Coder:toolbox:eml_min_or_max_varDimZero", 0);
  }

  e_st.site = &gc_emlrtRSI;
  f_st.site = &hc_emlrtRSI;
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
    g_st.site = &jc_emlrtRSI;
    if (!muDoubleScalarIsNaN(h2_du->data[0])) {
      idx = 1;
    } else {
      idx = 0;
      h_st.site = &kc_emlrtRSI;
      if (h2_du->size[0] > 2147483646) {
        i_st.site = &cc_emlrtRSI;
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
      g_st.site = &ic_emlrtRSI;
      h2 = h2_du->data[idx - 1];
      a = idx + 1;
      h_st.site = &lc_emlrtRSI;
      if ((idx + 1 <= h2_du->size[0]) && (h2_du->size[0] > 2147483646)) {
        i_st.site = &cc_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }

      for (k = a; k <= n; k++) {
        d = h2_du->data[k - 1];
        if (h2 < d) {
          h2 = d;
        }
      }
    }
  }

  emxFree_real_T(&h2_du);
  st.site = &yb_emlrtRSI;
  b_st.site = &dc_emlrtRSI;
  c_st.site = &ec_emlrtRSI;
  d_st.site = &fc_emlrtRSI;
  e_st.site = &gc_emlrtRSI;
  f_st.site = &hc_emlrtRSI;
  if ((*g < h2) || (muDoubleScalarIsNaN(*g) && (!muDoubleScalarIsNaN(h2)))) {
    *g = h2;
  }

  /*  h1=max(xx(:,3)-p_ocp.theta_max); */
  /*  h2=max(-xx(:,3)-p_ocp.theta_max); */
  /*  h3=max(xx(:,4)-p_ocp.thetap_max); */
  /*  h4=max(-xx(:,4)-p_ocp.thetap_max); */
  /*  g=max([h1;h2;h3;h4]); */
  /*  g = -1;  */
  /* ------------------------------------------------------------------------------- */
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (user_ocp.c) */
