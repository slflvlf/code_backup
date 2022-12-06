/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * simulate_ol.c
 *
 * Code generation for function 'simulate_ol'
 *
 */

/* Include files */
#include "simulate_ol.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "update_mv.h"
#include "update_mv_data.h"
#include "update_mv_emxutil.h"
#include "user_control_profile.h"
#include "user_ode.h"

/* Variable Definitions */
static emlrtRSInfo w_emlrtRSI = { 10,  /* lineNo */
  "simulate_ol",                       /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m"/* pathName */
};

static emlrtRSInfo x_emlrtRSI = { 15,  /* lineNo */
  "simulate_ol",                       /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m"/* pathName */
};

static emlrtRSInfo fb_emlrtRSI = { 6,  /* lineNo */
  "one_step",                          /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pathName */
};

static emlrtRSInfo gb_emlrtRSI = { 8,  /* lineNo */
  "one_step",                          /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pathName */
};

static emlrtRSInfo hb_emlrtRSI = { 9,  /* lineNo */
  "one_step",                          /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pathName */
};

static emlrtRSInfo ib_emlrtRSI = { 12, /* lineNo */
  "one_step",                          /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pathName */
};

static emlrtRSInfo jb_emlrtRSI = { 13, /* lineNo */
  "one_step",                          /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pathName */
};

static emlrtRSInfo kb_emlrtRSI = { 14, /* lineNo */
  "one_step",                          /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pathName */
};

static emlrtRSInfo lb_emlrtRSI = { 15, /* lineNo */
  "one_step",                          /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pathName */
};

static emlrtDCInfo f_emlrtDCI = { 11,  /* lineNo */
  1,                                   /* colNo */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m",/* pName */
  4                                    /* checkKind */
};

static emlrtDCInfo g_emlrtDCI = { 11,  /* lineNo */
  1,                                   /* colNo */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo h_emlrtDCI = { 11,  /* lineNo */
  10,                                  /* colNo */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m",/* pName */
  4                                    /* checkKind */
};

static emlrtDCInfo i_emlrtDCI = { 11,  /* lineNo */
  10,                                  /* colNo */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo l_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  15,                                  /* lineNo */
  8,                                   /* colNo */
  "xx",                                /* aName */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo m_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  15,                                  /* lineNo */
  36,                                  /* colNo */
  "uu",                                /* aName */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo n_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  15,                                  /* lineNo */
  27,                                  /* colNo */
  "xx",                                /* aName */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo o_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  12,                                  /* lineNo */
  4,                                   /* colNo */
  "xx",                                /* aName */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo b_emlrtRTEI = { 14,/* lineNo */
  7,                                   /* colNo */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m"/* pName */
};

static emlrtRTEInfo u_emlrtRTEI = { 11,/* lineNo */
  1,                                   /* colNo */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m"/* pName */
};

static emlrtRTEInfo v_emlrtRTEI = { 13,/* lineNo */
  5,                                   /* colNo */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m"/* pName */
};

static emlrtRTEInfo w_emlrtRTEI = { 13,/* lineNo */
  1,                                   /* colNo */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m"/* pName */
};

static emlrtRTEInfo x_emlrtRTEI = { 7, /* lineNo */
  21,                                  /* colNo */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m"/* pName */
};

/* Function Definitions */
void simulate_ol(const emlrtStack *sp, const real_T p[24], real_T p_ode_tau,
                 real_T p_ode_rk_order, const real_T p_ode_x0[6], const real_T
                 p_ode_M[9], const real_T p_ode_D[9], const real_T
                 p_ode_thrust_config[6], real_T p_uparam_nu, real_T p_uparam_Np,
                 const real_T p_uparam_R[2880], emxArray_real_T *tt,
                 emxArray_real_T *xx, emxArray_real_T *uu)
{
  real_T d;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  emxArray_real_T *r;
  int32_T b_i;
  int32_T u_size[1];
  real_T u_data[120];
  real_T b_xx[6];
  real_T k1[6];
  real_T k2[6];
  real_T k3[6];
  real_T k4[6];
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);

  /* --------------------------------------------------------- */
  /*  Author : Mazen Alamir  */
  /*  CNRS, UNiversity of Grenoble-Alpes */
  /*  Gipsa Lab. Last modification March 2017 */
  /* --------------------------------------------------------- */
  st.site = &w_emlrtRSI;
  user_control_profile(&st, p, p_uparam_nu, p_uparam_Np, p_uparam_R, uu);
  if (!(p_uparam_Np + 1.0 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(p_uparam_Np + 1.0, &h_emlrtDCI, sp);
  }

  d = (int32_T)muDoubleScalarFloor(p_uparam_Np + 1.0);
  if (p_uparam_Np + 1.0 != d) {
    emlrtIntegerCheckR2012b(p_uparam_Np + 1.0, &i_emlrtDCI, sp);
  }

  i = xx->size[0] * xx->size[1];
  i1 = (int32_T)(p_uparam_Np + 1.0);
  xx->size[0] = i1;
  xx->size[1] = 6;
  emxEnsureCapacity_real_T(sp, xx, i, &u_emlrtRTEI);
  if (!(p_uparam_Np + 1.0 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(p_uparam_Np + 1.0, &f_emlrtDCI, sp);
  }

  if (p_uparam_Np + 1.0 != d) {
    emlrtIntegerCheckR2012b(p_uparam_Np + 1.0, &g_emlrtDCI, sp);
  }

  loop_ub = i1 * 6;
  for (i = 0; i < loop_ub; i++) {
    xx->data[i] = 0.0;
  }

  if (1 > i1) {
    emlrtDynamicBoundsCheckR2012b(1, 1, i1, &o_emlrtBCI, sp);
  }

  for (i = 0; i < 6; i++) {
    xx->data[xx->size[0] * i] = p_ode_x0[i];
  }

  emxInit_real_T(sp, &r, 2, &x_emlrtRTEI, true);
  if (muDoubleScalarIsNaN(p_uparam_Np)) {
    i = r->size[0] * r->size[1];
    r->size[0] = 1;
    r->size[1] = 1;
    emxEnsureCapacity_real_T(sp, r, i, &v_emlrtRTEI);
    r->data[0] = rtNaN;
  } else if (p_uparam_Np < 0.0) {
    r->size[0] = 1;
    r->size[1] = 0;
  } else if (muDoubleScalarIsInf(p_uparam_Np) && (0.0 == p_uparam_Np)) {
    i = r->size[0] * r->size[1];
    r->size[0] = 1;
    r->size[1] = 1;
    emxEnsureCapacity_real_T(sp, r, i, &v_emlrtRTEI);
    r->data[0] = rtNaN;
  } else {
    i = r->size[0] * r->size[1];
    r->size[0] = 1;
    loop_ub = (int32_T)muDoubleScalarFloor(p_uparam_Np);
    r->size[1] = loop_ub + 1;
    emxEnsureCapacity_real_T(sp, r, i, &v_emlrtRTEI);
    for (i = 0; i <= loop_ub; i++) {
      r->data[i] = i;
    }
  }

  i = tt->size[0];
  tt->size[0] = r->size[1];
  emxEnsureCapacity_real_T(sp, tt, i, &w_emlrtRTEI);
  loop_ub = r->size[1];
  for (i = 0; i < loop_ub; i++) {
    tt->data[i] = r->data[i] * p_ode_tau;
  }

  emxFree_real_T(&r);
  i = (int32_T)p_uparam_Np;
  emlrtForLoopVectorCheckR2012b(1.0, 1.0, p_uparam_Np, mxDOUBLE_CLASS, (int32_T)
    p_uparam_Np, &b_emlrtRTEI, sp);
  for (b_i = 0; b_i < i; b_i++) {
    st.site = &x_emlrtRSI;
    i1 = b_i + 1;
    if ((i1 < 1) || (i1 > xx->size[0])) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, xx->size[0], &n_emlrtBCI, &st);
    }

    i1 = (int32_T)(b_i + 1U);
    if ((i1 < 1) || (i1 > uu->size[0])) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, uu->size[0], &m_emlrtBCI, &st);
    }

    loop_ub = uu->size[1];
    u_size[0] = uu->size[1];
    for (i1 = 0; i1 < loop_ub; i1++) {
      u_data[i1] = uu->data[b_i + uu->size[0] * i1];
    }

    /* --------------------------------------------------------- */
    if (p_ode_rk_order == 1.0) {
      for (i1 = 0; i1 < 6; i1++) {
        b_xx[i1] = xx->data[b_i + xx->size[0] * i1];
      }

      b_st.site = &fb_emlrtRSI;
      user_ode(&b_st, b_xx, u_data, u_size, p_ode_M, p_ode_D,
               p_ode_thrust_config, k1);
      for (i1 = 0; i1 < 6; i1++) {
        k1[i1] = xx->data[b_i + xx->size[0] * i1] + p_ode_tau * k1[i1];
      }
    } else if (p_ode_rk_order == 2.0) {
      for (i1 = 0; i1 < 6; i1++) {
        b_xx[i1] = xx->data[b_i + xx->size[0] * i1];
      }

      b_st.site = &gb_emlrtRSI;
      user_ode(&b_st, b_xx, u_data, u_size, p_ode_M, p_ode_D,
               p_ode_thrust_config, k1);
      for (i1 = 0; i1 < 6; i1++) {
        k1[i1] = xx->data[b_i + xx->size[0] * i1] + 0.5 * k1[i1] * p_ode_tau;
      }

      b_st.site = &hb_emlrtRSI;
      user_ode(&b_st, k1, u_data, u_size, p_ode_M, p_ode_D, p_ode_thrust_config,
               k2);
      for (i1 = 0; i1 < 6; i1++) {
        k1[i1] = xx->data[b_i + xx->size[0] * i1] + k2[i1] * p_ode_tau;
      }
    } else {
      for (i1 = 0; i1 < 6; i1++) {
        b_xx[i1] = xx->data[b_i + xx->size[0] * i1];
      }

      b_st.site = &ib_emlrtRSI;
      user_ode(&b_st, b_xx, u_data, u_size, p_ode_M, p_ode_D,
               p_ode_thrust_config, k1);
      for (i1 = 0; i1 < 6; i1++) {
        b_xx[i1] = xx->data[b_i + xx->size[0] * i1] + 0.5 * k1[i1] * p_ode_tau;
      }

      b_st.site = &jb_emlrtRSI;
      user_ode(&b_st, b_xx, u_data, u_size, p_ode_M, p_ode_D,
               p_ode_thrust_config, k2);
      for (i1 = 0; i1 < 6; i1++) {
        b_xx[i1] = xx->data[b_i + xx->size[0] * i1] + 0.5 * k2[i1] * p_ode_tau;
      }

      b_st.site = &kb_emlrtRSI;
      user_ode(&b_st, b_xx, u_data, u_size, p_ode_M, p_ode_D,
               p_ode_thrust_config, k3);
      for (i1 = 0; i1 < 6; i1++) {
        b_xx[i1] = xx->data[b_i + xx->size[0] * i1] + k3[i1] * p_ode_tau;
      }

      b_st.site = &lb_emlrtRSI;
      user_ode(&b_st, b_xx, u_data, u_size, p_ode_M, p_ode_D,
               p_ode_thrust_config, k4);
      for (i1 = 0; i1 < 6; i1++) {
        k1[i1] = xx->data[b_i + xx->size[0] * i1] + p_ode_tau * ((k1[i1] + 2.0 *
          (k2[i1] + k3[i1])) + k4[i1]) / 6.0;
      }
    }

    i1 = (int32_T)(b_i + 2U);
    if ((i1 < 1) || (i1 > xx->size[0])) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, xx->size[0], &l_emlrtBCI, sp);
    }

    for (i1 = 0; i1 < 6; i1++) {
      xx->data[(b_i + xx->size[0] * i1) + 1] = k1[i1];
    }

    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (simulate_ol.c) */
