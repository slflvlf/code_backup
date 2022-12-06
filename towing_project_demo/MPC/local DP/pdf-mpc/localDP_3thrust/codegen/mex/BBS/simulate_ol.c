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
#include "BBS.h"
#include "BBS_data.h"
#include "BBS_emxutil.h"
#include "assertValidSizeArg.h"
#include "blas.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "user_ode.h"

/* Variable Definitions */
static emlrtRSInfo c_emlrtRSI = { 10,  /* lineNo */
  "simulate_ol",                       /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m"/* pathName */
};

static emlrtRSInfo d_emlrtRSI = { 15,  /* lineNo */
  "simulate_ol",                       /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m"/* pathName */
};

static emlrtRSInfo e_emlrtRSI = { 5,   /* lineNo */
  "user_control_profile",              /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\local DP\\localDP_3thrust\\user_control_profile.m"/* pathName */
};

static emlrtRSInfo f_emlrtRSI = { 79,  /* lineNo */
  "eml_mtimes_helper",                 /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_helper.m"/* pathName */
};

static emlrtRSInfo i_emlrtRSI = { 29,  /* lineNo */
  "reshapeSizeChecks",                 /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\reshapeSizeChecks.m"/* pathName */
};

static emlrtRSInfo j_emlrtRSI = { 109, /* lineNo */
  "computeDimsData",                   /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\reshapeSizeChecks.m"/* pathName */
};

static emlrtRSInfo k_emlrtRSI = { 6,   /* lineNo */
  "one_step",                          /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pathName */
};

static emlrtRSInfo l_emlrtRSI = { 8,   /* lineNo */
  "one_step",                          /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pathName */
};

static emlrtRSInfo m_emlrtRSI = { 9,   /* lineNo */
  "one_step",                          /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pathName */
};

static emlrtRSInfo n_emlrtRSI = { 12,  /* lineNo */
  "one_step",                          /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pathName */
};

static emlrtRSInfo o_emlrtRSI = { 13,  /* lineNo */
  "one_step",                          /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pathName */
};

static emlrtRSInfo p_emlrtRSI = { 14,  /* lineNo */
  "one_step",                          /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pathName */
};

static emlrtRSInfo q_emlrtRSI = { 15,  /* lineNo */
  "one_step",                          /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pathName */
};

static emlrtDCInfo b_emlrtDCI = { 11,  /* lineNo */
  1,                                   /* colNo */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m",/* pName */
  4                                    /* checkKind */
};

static emlrtDCInfo c_emlrtDCI = { 11,  /* lineNo */
  1,                                   /* colNo */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo d_emlrtDCI = { 11,  /* lineNo */
  10,                                  /* colNo */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m",/* pName */
  4                                    /* checkKind */
};

static emlrtDCInfo e_emlrtDCI = { 11,  /* lineNo */
  10,                                  /* colNo */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m",/* pName */
  1                                    /* checkKind */
};

static emlrtRTEInfo emlrtRTEI = { 59,  /* lineNo */
  23,                                  /* colNo */
  "reshapeSizeChecks",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\reshapeSizeChecks.m"/* pName */
};

static emlrtRTEInfo b_emlrtRTEI = { 57,/* lineNo */
  23,                                  /* colNo */
  "reshapeSizeChecks",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\reshapeSizeChecks.m"/* pName */
};

static emlrtRTEInfo c_emlrtRTEI = { 52,/* lineNo */
  13,                                  /* colNo */
  "reshapeSizeChecks",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\reshapeSizeChecks.m"/* pName */
};

static emlrtBCInfo b_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  15,                                  /* lineNo */
  8,                                   /* colNo */
  "xx",                                /* aName */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo c_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  15,                                  /* lineNo */
  36,                                  /* colNo */
  "uu",                                /* aName */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  15,                                  /* lineNo */
  27,                                  /* colNo */
  "xx",                                /* aName */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo e_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  12,                                  /* lineNo */
  4,                                   /* colNo */
  "xx",                                /* aName */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo d_emlrtRTEI = { 14,/* lineNo */
  7,                                   /* colNo */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m"/* pName */
};

static emlrtRTEInfo o_emlrtRTEI = { 10,/* lineNo */
  1,                                   /* colNo */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m"/* pName */
};

static emlrtRTEInfo p_emlrtRTEI = { 11,/* lineNo */
  1,                                   /* colNo */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m"/* pName */
};

static emlrtRTEInfo q_emlrtRTEI = { 13,/* lineNo */
  5,                                   /* colNo */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m"/* pName */
};

static emlrtRTEInfo r_emlrtRTEI = { 13,/* lineNo */
  1,                                   /* colNo */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m"/* pName */
};

static emlrtRTEInfo s_emlrtRTEI = { 7, /* lineNo */
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
  char_T TRANSB1;
  char_T TRANSA1;
  real_T alpha1;
  real_T beta1;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  real_T u_data[120];
  int32_T loop_ub;
  int32_T b_loop_ub;
  int32_T i;
  int32_T b_i;
  emxArray_real_T *r;
  int32_T u_size[1];
  real_T b_xx[6];
  real_T k1[6];
  real_T k2[6];
  real_T k3[6];
  real_T k4[6];
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
  emlrtHeapReferenceStackEnterFcnR2012b(sp);

  /* --------------------------------------------------------- */
  /*  Author : Mazen Alamir  */
  /*  CNRS, UNiversity of Grenoble-Alpes */
  /*  Gipsa Lab. Last modification March 2017 */
  /* --------------------------------------------------------- */
  st.site = &c_emlrtRSI;

  /* ------------------------------------------------------------------------------- */
  /* pdf_mpc package: Example 1 - Definition of the user_control_profile map */
  /* ------------------------------------------------------------------------------- */
  b_st.site = &e_emlrtRSI;
  c_st.site = &f_emlrtRSI;
  TRANSB1 = 'N';
  TRANSA1 = 'N';
  alpha1 = 1.0;
  beta1 = 0.0;
  m_t = (ptrdiff_t)120;
  n_t = (ptrdiff_t)1;
  k_t = (ptrdiff_t)24;
  lda_t = (ptrdiff_t)120;
  ldb_t = (ptrdiff_t)24;
  ldc_t = (ptrdiff_t)120;
  dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &p_uparam_R[0], &lda_t,
        &p[0], &ldb_t, &beta1, &u_data[0], &ldc_t);
  b_st.site = &e_emlrtRSI;
  c_st.site = &i_emlrtRSI;
  d_st.site = &j_emlrtRSI;
  assertValidSizeArg(&d_st, p_uparam_nu);
  d_st.site = &j_emlrtRSI;
  assertValidSizeArg(&d_st, p_uparam_Np);
  loop_ub = (int32_T)p_uparam_nu;
  if (loop_ub > 120) {
    emlrtErrorWithMessageIdR2018a(&b_st, &c_emlrtRTEI,
      "Coder:toolbox:reshape_emptyReshapeLimit",
      "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }

  b_loop_ub = (int32_T)p_uparam_Np;
  if (b_loop_ub > 120) {
    emlrtErrorWithMessageIdR2018a(&b_st, &c_emlrtRTEI,
      "Coder:toolbox:reshape_emptyReshapeLimit",
      "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }

  if ((loop_ub < 0) || (b_loop_ub < 0)) {
    emlrtErrorWithMessageIdR2018a(&b_st, &b_emlrtRTEI,
      "MATLAB:checkDimCommon:nonnegativeSize",
      "MATLAB:checkDimCommon:nonnegativeSize", 0);
  }

  if (loop_ub * b_loop_ub != 120) {
    emlrtErrorWithMessageIdR2018a(&b_st, &emlrtRTEI,
      "Coder:MATLAB:getReshapeDims_notSameNumel",
      "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
  }

  /*      u_profile=reshape(p_uparam.p, p_uparam.Np,p_uparam.nu); */
  i = uu->size[0] * uu->size[1];
  uu->size[0] = b_loop_ub;
  uu->size[1] = loop_ub;
  emxEnsureCapacity_real_T(&st, uu, i, &o_emlrtRTEI);
  for (i = 0; i < loop_ub; i++) {
    for (b_i = 0; b_i < b_loop_ub; b_i++) {
      uu->data[b_i + uu->size[0] * i] = u_data[i + loop_ub * b_i];
    }
  }

  /*      u_profile=reshape(p_uparam.R*p, p_uparam.Np, p_uparam.nu); */
  /*      u_profile=reshape(p_uparam.p, p_uparam.Np,p_uparam.nu); */
  /* ------------------------------------------------------------------------------- */
  if (!(p_uparam_Np + 1.0 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(p_uparam_Np + 1.0, &d_emlrtDCI, sp);
  }

  alpha1 = (int32_T)muDoubleScalarFloor(p_uparam_Np + 1.0);
  if (p_uparam_Np + 1.0 != alpha1) {
    emlrtIntegerCheckR2012b(p_uparam_Np + 1.0, &e_emlrtDCI, sp);
  }

  i = xx->size[0] * xx->size[1];
  b_i = (int32_T)(p_uparam_Np + 1.0);
  xx->size[0] = b_i;
  xx->size[1] = 6;
  emxEnsureCapacity_real_T(sp, xx, i, &p_emlrtRTEI);
  if (!(p_uparam_Np + 1.0 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(p_uparam_Np + 1.0, &b_emlrtDCI, sp);
  }

  if (p_uparam_Np + 1.0 != alpha1) {
    emlrtIntegerCheckR2012b(p_uparam_Np + 1.0, &c_emlrtDCI, sp);
  }

  loop_ub = b_i * 6;
  for (i = 0; i < loop_ub; i++) {
    xx->data[i] = 0.0;
  }

  if (1 > b_i) {
    emlrtDynamicBoundsCheckR2012b(1, 1, b_i, &e_emlrtBCI, sp);
  }

  for (i = 0; i < 6; i++) {
    xx->data[xx->size[0] * i] = p_ode_x0[i];
  }

  emxInit_real_T(sp, &r, 2, &s_emlrtRTEI, true);
  if (muDoubleScalarIsNaN(p_uparam_Np)) {
    i = r->size[0] * r->size[1];
    r->size[0] = 1;
    r->size[1] = 1;
    emxEnsureCapacity_real_T(sp, r, i, &q_emlrtRTEI);
    r->data[0] = rtNaN;
  } else if (p_uparam_Np < 0.0) {
    r->size[0] = 1;
    r->size[1] = 0;
  } else if (muDoubleScalarIsInf(p_uparam_Np) && (0.0 == p_uparam_Np)) {
    i = r->size[0] * r->size[1];
    r->size[0] = 1;
    r->size[1] = 1;
    emxEnsureCapacity_real_T(sp, r, i, &q_emlrtRTEI);
    r->data[0] = rtNaN;
  } else {
    i = r->size[0] * r->size[1];
    r->size[0] = 1;
    loop_ub = (int32_T)muDoubleScalarFloor(p_uparam_Np);
    r->size[1] = loop_ub + 1;
    emxEnsureCapacity_real_T(sp, r, i, &q_emlrtRTEI);
    for (i = 0; i <= loop_ub; i++) {
      r->data[i] = i;
    }
  }

  i = tt->size[0];
  tt->size[0] = r->size[1];
  emxEnsureCapacity_real_T(sp, tt, i, &r_emlrtRTEI);
  loop_ub = r->size[1];
  for (i = 0; i < loop_ub; i++) {
    tt->data[i] = r->data[i] * p_ode_tau;
  }

  emxFree_real_T(&r);
  emlrtForLoopVectorCheckR2012b(1.0, 1.0, p_uparam_Np, mxDOUBLE_CLASS, (int32_T)
    p_uparam_Np, &d_emlrtRTEI, sp);
  for (b_i = 0; b_i < b_loop_ub; b_i++) {
    st.site = &d_emlrtRSI;
    i = b_i + 1;
    if ((i < 1) || (i > xx->size[0])) {
      emlrtDynamicBoundsCheckR2012b(i, 1, xx->size[0], &d_emlrtBCI, &st);
    }

    i = (int32_T)(b_i + 1U);
    if ((i < 1) || (i > uu->size[0])) {
      emlrtDynamicBoundsCheckR2012b(i, 1, uu->size[0], &c_emlrtBCI, &st);
    }

    loop_ub = uu->size[1];
    u_size[0] = uu->size[1];
    for (i = 0; i < loop_ub; i++) {
      u_data[i] = uu->data[b_i + uu->size[0] * i];
    }

    /* --------------------------------------------------------- */
    if (p_ode_rk_order == 1.0) {
      for (i = 0; i < 6; i++) {
        b_xx[i] = xx->data[b_i + xx->size[0] * i];
      }

      b_st.site = &k_emlrtRSI;
      user_ode(&b_st, b_xx, u_data, u_size, p_ode_M, p_ode_D,
               p_ode_thrust_config, k1);
      for (i = 0; i < 6; i++) {
        k1[i] = xx->data[b_i + xx->size[0] * i] + p_ode_tau * k1[i];
      }
    } else if (p_ode_rk_order == 2.0) {
      for (i = 0; i < 6; i++) {
        b_xx[i] = xx->data[b_i + xx->size[0] * i];
      }

      b_st.site = &l_emlrtRSI;
      user_ode(&b_st, b_xx, u_data, u_size, p_ode_M, p_ode_D,
               p_ode_thrust_config, k1);
      for (i = 0; i < 6; i++) {
        k1[i] = xx->data[b_i + xx->size[0] * i] + 0.5 * k1[i] * p_ode_tau;
      }

      b_st.site = &m_emlrtRSI;
      user_ode(&b_st, k1, u_data, u_size, p_ode_M, p_ode_D, p_ode_thrust_config,
               k2);
      for (i = 0; i < 6; i++) {
        k1[i] = xx->data[b_i + xx->size[0] * i] + k2[i] * p_ode_tau;
      }
    } else {
      for (i = 0; i < 6; i++) {
        b_xx[i] = xx->data[b_i + xx->size[0] * i];
      }

      b_st.site = &n_emlrtRSI;
      user_ode(&b_st, b_xx, u_data, u_size, p_ode_M, p_ode_D,
               p_ode_thrust_config, k1);
      for (i = 0; i < 6; i++) {
        b_xx[i] = xx->data[b_i + xx->size[0] * i] + 0.5 * k1[i] * p_ode_tau;
      }

      b_st.site = &o_emlrtRSI;
      user_ode(&b_st, b_xx, u_data, u_size, p_ode_M, p_ode_D,
               p_ode_thrust_config, k2);
      for (i = 0; i < 6; i++) {
        b_xx[i] = xx->data[b_i + xx->size[0] * i] + 0.5 * k2[i] * p_ode_tau;
      }

      b_st.site = &p_emlrtRSI;
      user_ode(&b_st, b_xx, u_data, u_size, p_ode_M, p_ode_D,
               p_ode_thrust_config, k3);
      for (i = 0; i < 6; i++) {
        b_xx[i] = xx->data[b_i + xx->size[0] * i] + k3[i] * p_ode_tau;
      }

      b_st.site = &q_emlrtRSI;
      user_ode(&b_st, b_xx, u_data, u_size, p_ode_M, p_ode_D,
               p_ode_thrust_config, k4);
      for (i = 0; i < 6; i++) {
        k1[i] = xx->data[b_i + xx->size[0] * i] + p_ode_tau * ((k1[i] + 2.0 *
          (k2[i] + k3[i])) + k4[i]) / 6.0;
      }
    }

    i = (int32_T)(b_i + 2U);
    if ((i < 1) || (i > xx->size[0])) {
      emlrtDynamicBoundsCheckR2012b(i, 1, xx->size[0], &b_emlrtBCI, sp);
    }

    for (i = 0; i < 6; i++) {
      xx->data[(b_i + xx->size[0] * i) + 1] = k1[i];
    }

    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (simulate_ol.c) */
