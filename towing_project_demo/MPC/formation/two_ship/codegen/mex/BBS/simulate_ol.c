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

static emlrtRSInfo e_emlrtRSI = { 8,   /* lineNo */
  "user_control_profile",              /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_control_profile.m"/* pathName */
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

static emlrtECInfo emlrtECI = { -1,    /* nDims */
  16,                                  /* lineNo */
  15,                                  /* colNo */
  "one_step",                          /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pName */
};

static emlrtECInfo b_emlrtECI = { -1,  /* nDims */
  16,                                  /* lineNo */
  20,                                  /* colNo */
  "one_step",                          /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pName */
};

static emlrtECInfo c_emlrtECI = { -1,  /* nDims */
  16,                                  /* lineNo */
  26,                                  /* colNo */
  "one_step",                          /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pName */
};

static emlrtECInfo d_emlrtECI = { -1,  /* nDims */
  15,                                  /* lineNo */
  21,                                  /* colNo */
  "one_step",                          /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pName */
};

static emlrtECInfo e_emlrtECI = { -1,  /* nDims */
  14,                                  /* lineNo */
  21,                                  /* colNo */
  "one_step",                          /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pName */
};

static emlrtECInfo f_emlrtECI = { -1,  /* nDims */
  13,                                  /* lineNo */
  21,                                  /* colNo */
  "one_step",                          /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pName */
};

static emlrtECInfo g_emlrtECI = { -1,  /* nDims */
  10,                                  /* lineNo */
  15,                                  /* colNo */
  "one_step",                          /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pName */
};

static emlrtECInfo h_emlrtECI = { -1,  /* nDims */
  9,                                   /* lineNo */
  21,                                  /* colNo */
  "one_step",                          /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pName */
};

static emlrtECInfo i_emlrtECI = { -1,  /* nDims */
  6,                                   /* lineNo */
  15,                                  /* colNo */
  "one_step",                          /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pName */
};

static emlrtRTEInfo c_emlrtRTEI = { 59,/* lineNo */
  23,                                  /* colNo */
  "reshapeSizeChecks",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\reshapeSizeChecks.m"/* pName */
};

static emlrtRTEInfo d_emlrtRTEI = { 57,/* lineNo */
  23,                                  /* colNo */
  "reshapeSizeChecks",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\reshapeSizeChecks.m"/* pName */
};

static emlrtRTEInfo e_emlrtRTEI = { 52,/* lineNo */
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

static emlrtRTEInfo f_emlrtRTEI = { 14,/* lineNo */
  7,                                   /* colNo */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m"/* pName */
};

static emlrtRTEInfo q_emlrtRTEI = { 10,/* lineNo */
  1,                                   /* colNo */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m"/* pName */
};

static emlrtRTEInfo r_emlrtRTEI = { 11,/* lineNo */
  1,                                   /* colNo */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m"/* pName */
};

static emlrtRTEInfo s_emlrtRTEI = { 13,/* lineNo */
  5,                                   /* colNo */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m"/* pName */
};

static emlrtRTEInfo t_emlrtRTEI = { 13,/* lineNo */
  1,                                   /* colNo */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m"/* pName */
};

static emlrtRTEInfo u_emlrtRTEI = { 13,/* lineNo */
  23,                                  /* colNo */
  "one_step",                          /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pName */
};

static emlrtRTEInfo v_emlrtRTEI = { 14,/* lineNo */
  23,                                  /* colNo */
  "one_step",                          /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pName */
};

static emlrtRTEInfo w_emlrtRTEI = { 15,/* lineNo */
  23,                                  /* colNo */
  "one_step",                          /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pName */
};

static emlrtRTEInfo x_emlrtRTEI = { 7, /* lineNo */
  21,                                  /* colNo */
  "simulate_ol",                       /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\simulate_ol.m"/* pName */
};

static emlrtRTEInfo y_emlrtRTEI = { 8, /* lineNo */
  9,                                   /* colNo */
  "one_step",                          /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pName */
};

static emlrtRTEInfo ab_emlrtRTEI = { 9,/* lineNo */
  9,                                   /* colNo */
  "one_step",                          /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pName */
};

static emlrtRTEInfo bb_emlrtRTEI = { 14,/* lineNo */
  9,                                   /* colNo */
  "one_step",                          /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pName */
};

static emlrtRTEInfo cb_emlrtRTEI = { 15,/* lineNo */
  9,                                   /* colNo */
  "one_step",                          /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\one_step.m"/* pName */
};

/* Function Definitions */
void simulate_ol(const emlrtStack *sp, const real_T p[48], real_T p_ode_Nship,
                 real_T p_ode_tau, real_T p_ode_rk_order, const real_T p_ode_x0
                 [12], const real_T p_ode_M[9], const real_T p_ode_D[9], const
                 real_T p_ode_thrust_config[6], real_T p_uparam_nu, real_T
                 p_uparam_Np, const real_T p_uparam_R[11520], emxArray_real_T
                 *tt, emxArray_real_T *xx, emxArray_real_T *uu)
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
  real_T u_data[240];
  int32_T loop_ub;
  int32_T b_loop_ub;
  int32_T i;
  int32_T i1;
  emxArray_real_T *r;
  int32_T b_i;
  emxArray_real_T *k1;
  int32_T c_loop_ub;
  emxArray_real_T *k2;
  emxArray_real_T *k3;
  emxArray_real_T *k4;
  int32_T u_size[1];
  real_T xplus[12];
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
  /*  u_profile是控制序列， Np行，nu列 */
  /*  一般这个子程序不需要改 */
  /* ------------------------------------------------------------------------------- */
  b_st.site = &e_emlrtRSI;
  c_st.site = &f_emlrtRSI;
  TRANSB1 = 'N';
  TRANSA1 = 'N';
  alpha1 = 1.0;
  beta1 = 0.0;
  m_t = (ptrdiff_t)240;
  n_t = (ptrdiff_t)1;
  k_t = (ptrdiff_t)48;
  lda_t = (ptrdiff_t)240;
  ldb_t = (ptrdiff_t)48;
  ldc_t = (ptrdiff_t)240;
  dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &p_uparam_R[0], &lda_t,
        &p[0], &ldb_t, &beta1, &u_data[0], &ldc_t);
  b_st.site = &e_emlrtRSI;
  c_st.site = &i_emlrtRSI;
  d_st.site = &j_emlrtRSI;
  assertValidSizeArg(&d_st, p_uparam_nu);
  d_st.site = &j_emlrtRSI;
  assertValidSizeArg(&d_st, p_uparam_Np);
  loop_ub = (int32_T)p_uparam_nu;
  if (loop_ub > 240) {
    emlrtErrorWithMessageIdR2018a(&b_st, &e_emlrtRTEI,
      "Coder:toolbox:reshape_emptyReshapeLimit",
      "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }

  b_loop_ub = (int32_T)p_uparam_Np;
  if (b_loop_ub > 240) {
    emlrtErrorWithMessageIdR2018a(&b_st, &e_emlrtRTEI,
      "Coder:toolbox:reshape_emptyReshapeLimit",
      "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }

  if ((loop_ub < 0) || (b_loop_ub < 0)) {
    emlrtErrorWithMessageIdR2018a(&b_st, &d_emlrtRTEI,
      "MATLAB:checkDimCommon:nonnegativeSize",
      "MATLAB:checkDimCommon:nonnegativeSize", 0);
  }

  if (loop_ub * b_loop_ub != 240) {
    emlrtErrorWithMessageIdR2018a(&b_st, &c_emlrtRTEI,
      "Coder:MATLAB:getReshapeDims_notSameNumel",
      "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
  }

  i = uu->size[0] * uu->size[1];
  uu->size[0] = b_loop_ub;
  uu->size[1] = loop_ub;
  emxEnsureCapacity_real_T(&st, uu, i, &q_emlrtRTEI);
  for (i = 0; i < loop_ub; i++) {
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      uu->data[i1 + uu->size[0] * i] = u_data[i + loop_ub * i1];
    }
  }

  /* ------------------------------------------------------------------------------- */
  if (!(p_uparam_Np + 1.0 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(p_uparam_Np + 1.0, &d_emlrtDCI, sp);
  }

  alpha1 = (int32_T)muDoubleScalarFloor(p_uparam_Np + 1.0);
  if (p_uparam_Np + 1.0 != alpha1) {
    emlrtIntegerCheckR2012b(p_uparam_Np + 1.0, &e_emlrtDCI, sp);
  }

  i = xx->size[0] * xx->size[1];
  i1 = (int32_T)(p_uparam_Np + 1.0);
  xx->size[0] = i1;
  xx->size[1] = 12;
  emxEnsureCapacity_real_T(sp, xx, i, &r_emlrtRTEI);
  if (!(p_uparam_Np + 1.0 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(p_uparam_Np + 1.0, &b_emlrtDCI, sp);
  }

  if (p_uparam_Np + 1.0 != alpha1) {
    emlrtIntegerCheckR2012b(p_uparam_Np + 1.0, &c_emlrtDCI, sp);
  }

  loop_ub = i1 * 12;
  for (i = 0; i < loop_ub; i++) {
    xx->data[i] = 0.0;
  }

  if (1 > i1) {
    emlrtDynamicBoundsCheckR2012b(1, 1, i1, &e_emlrtBCI, sp);
  }

  for (i = 0; i < 12; i++) {
    xx->data[xx->size[0] * i] = p_ode_x0[i];
  }

  emxInit_real_T(sp, &r, 2, &x_emlrtRTEI, true);
  if (muDoubleScalarIsNaN(p_uparam_Np)) {
    i = r->size[0] * r->size[1];
    r->size[0] = 1;
    r->size[1] = 1;
    emxEnsureCapacity_real_T(sp, r, i, &s_emlrtRTEI);
    r->data[0] = rtNaN;
  } else if (p_uparam_Np < 0.0) {
    r->size[0] = 1;
    r->size[1] = 0;
  } else if (muDoubleScalarIsInf(p_uparam_Np) && (0.0 == p_uparam_Np)) {
    i = r->size[0] * r->size[1];
    r->size[0] = 1;
    r->size[1] = 1;
    emxEnsureCapacity_real_T(sp, r, i, &s_emlrtRTEI);
    r->data[0] = rtNaN;
  } else {
    i = r->size[0] * r->size[1];
    r->size[0] = 1;
    loop_ub = (int32_T)muDoubleScalarFloor(p_uparam_Np);
    r->size[1] = loop_ub + 1;
    emxEnsureCapacity_real_T(sp, r, i, &s_emlrtRTEI);
    for (i = 0; i <= loop_ub; i++) {
      r->data[i] = i;
    }
  }

  i = tt->size[0];
  tt->size[0] = r->size[1];
  emxEnsureCapacity_real_T(sp, tt, i, &t_emlrtRTEI);
  loop_ub = r->size[1];
  for (i = 0; i < loop_ub; i++) {
    tt->data[i] = r->data[i] * p_ode_tau;
  }

  emxFree_real_T(&r);
  emlrtForLoopVectorCheckR2012b(1.0, 1.0, p_uparam_Np, mxDOUBLE_CLASS, (int32_T)
    p_uparam_Np, &f_emlrtRTEI, sp);
  if (0 <= b_loop_ub - 1) {
    b_i = uu->size[1];
    c_loop_ub = uu->size[1];
  }

  emxInit_real_T(sp, &k1, 1, &y_emlrtRTEI, true);
  emxInit_real_T(sp, &k2, 1, &ab_emlrtRTEI, true);
  emxInit_real_T(sp, &k3, 1, &bb_emlrtRTEI, true);
  emxInit_real_T(sp, &k4, 1, &cb_emlrtRTEI, true);
  if (0 <= b_loop_ub - 1) {
    u_size[0] = b_i;
  }

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

    for (i = 0; i < c_loop_ub; i++) {
      u_data[i] = uu->data[b_i + uu->size[0] * i];
    }

    /* --------------------------------------------------------- */
    if (p_ode_rk_order == 1.0) {
      for (i = 0; i < 12; i++) {
        xplus[i] = xx->data[b_i + xx->size[0] * i];
      }

      b_st.site = &k_emlrtRSI;
      user_ode(&b_st, xplus, u_data, u_size, p_ode_Nship, p_ode_M, p_ode_D,
               p_ode_thrust_config, k4);
      loop_ub = k4->size[0];
      for (i = 0; i < loop_ub; i++) {
        k4->data[i] *= p_ode_tau;
      }

      if (12 != k4->size[0]) {
        emlrtSizeEqCheck1DR2012b(12, k4->size[0], &i_emlrtECI, &st);
      }

      for (i = 0; i < 12; i++) {
        xplus[i] = xx->data[b_i + xx->size[0] * i] + k4->data[i];
      }
    } else if (p_ode_rk_order == 2.0) {
      for (i = 0; i < 12; i++) {
        xplus[i] = xx->data[b_i + xx->size[0] * i];
      }

      b_st.site = &l_emlrtRSI;
      user_ode(&b_st, xplus, u_data, u_size, p_ode_Nship, p_ode_M, p_ode_D,
               p_ode_thrust_config, k1);
      loop_ub = k1->size[0];
      for (i = 0; i < loop_ub; i++) {
        k1->data[i] = 0.5 * k1->data[i] * p_ode_tau;
      }

      if (12 != k1->size[0]) {
        emlrtSizeEqCheck1DR2012b(12, k1->size[0], &h_emlrtECI, &st);
      }

      for (i = 0; i < 12; i++) {
        xplus[i] = xx->data[b_i + xx->size[0] * i] + k1->data[i];
      }

      b_st.site = &m_emlrtRSI;
      user_ode(&b_st, xplus, u_data, u_size, p_ode_Nship, p_ode_M, p_ode_D,
               p_ode_thrust_config, k2);
      loop_ub = k2->size[0];
      for (i = 0; i < loop_ub; i++) {
        k2->data[i] *= p_ode_tau;
      }

      if (12 != k2->size[0]) {
        emlrtSizeEqCheck1DR2012b(12, k2->size[0], &g_emlrtECI, &st);
      }

      for (i = 0; i < 12; i++) {
        xplus[i] = xx->data[b_i + xx->size[0] * i] + k2->data[i];
      }
    } else {
      for (i = 0; i < 12; i++) {
        xplus[i] = xx->data[b_i + xx->size[0] * i];
      }

      b_st.site = &n_emlrtRSI;
      user_ode(&b_st, xplus, u_data, u_size, p_ode_Nship, p_ode_M, p_ode_D,
               p_ode_thrust_config, k1);
      i = k4->size[0];
      k4->size[0] = k1->size[0];
      emxEnsureCapacity_real_T(&st, k4, i, &u_emlrtRTEI);
      loop_ub = k1->size[0];
      for (i = 0; i < loop_ub; i++) {
        k4->data[i] = 0.5 * k1->data[i] * p_ode_tau;
      }

      if (12 != k4->size[0]) {
        emlrtSizeEqCheck1DR2012b(12, k4->size[0], &f_emlrtECI, &st);
      }

      for (i = 0; i < 12; i++) {
        xplus[i] = xx->data[b_i + xx->size[0] * i] + k4->data[i];
      }

      b_st.site = &o_emlrtRSI;
      user_ode(&b_st, xplus, u_data, u_size, p_ode_Nship, p_ode_M, p_ode_D,
               p_ode_thrust_config, k2);
      i = k4->size[0];
      k4->size[0] = k2->size[0];
      emxEnsureCapacity_real_T(&st, k4, i, &v_emlrtRTEI);
      loop_ub = k2->size[0];
      for (i = 0; i < loop_ub; i++) {
        k4->data[i] = 0.5 * k2->data[i] * p_ode_tau;
      }

      if (12 != k4->size[0]) {
        emlrtSizeEqCheck1DR2012b(12, k4->size[0], &e_emlrtECI, &st);
      }

      for (i = 0; i < 12; i++) {
        xplus[i] = xx->data[b_i + xx->size[0] * i] + k4->data[i];
      }

      b_st.site = &p_emlrtRSI;
      user_ode(&b_st, xplus, u_data, u_size, p_ode_Nship, p_ode_M, p_ode_D,
               p_ode_thrust_config, k3);
      i = k4->size[0];
      k4->size[0] = k3->size[0];
      emxEnsureCapacity_real_T(&st, k4, i, &w_emlrtRTEI);
      loop_ub = k3->size[0];
      for (i = 0; i < loop_ub; i++) {
        k4->data[i] = k3->data[i] * p_ode_tau;
      }

      if (12 != k4->size[0]) {
        emlrtSizeEqCheck1DR2012b(12, k4->size[0], &d_emlrtECI, &st);
      }

      for (i = 0; i < 12; i++) {
        xplus[i] = xx->data[b_i + xx->size[0] * i] + k4->data[i];
      }

      b_st.site = &q_emlrtRSI;
      user_ode(&b_st, xplus, u_data, u_size, p_ode_Nship, p_ode_M, p_ode_D,
               p_ode_thrust_config, k4);
      if (k2->size[0] != k3->size[0]) {
        emlrtSizeEqCheck1DR2012b(k2->size[0], k3->size[0], &c_emlrtECI, &st);
      }

      loop_ub = k2->size[0];
      for (i = 0; i < loop_ub; i++) {
        k2->data[i] = 2.0 * (k2->data[i] + k3->data[i]);
      }

      if (k1->size[0] != k2->size[0]) {
        emlrtSizeEqCheck1DR2012b(k1->size[0], k2->size[0], &b_emlrtECI, &st);
      }

      if (k1->size[0] != k4->size[0]) {
        emlrtSizeEqCheck1DR2012b(k1->size[0], k4->size[0], &b_emlrtECI, &st);
      }

      loop_ub = k1->size[0];
      for (i = 0; i < loop_ub; i++) {
        k1->data[i] = p_ode_tau * ((k1->data[i] + k2->data[i]) + k4->data[i]) /
          6.0;
      }

      if (12 != k1->size[0]) {
        emlrtSizeEqCheck1DR2012b(12, k1->size[0], &emlrtECI, &st);
      }

      for (i = 0; i < 12; i++) {
        xplus[i] = xx->data[b_i + xx->size[0] * i] + k1->data[i];
      }
    }

    i = (int32_T)(b_i + 2U);
    if ((i < 1) || (i > xx->size[0])) {
      emlrtDynamicBoundsCheckR2012b(i, 1, xx->size[0], &b_emlrtBCI, sp);
    }

    for (i = 0; i < 12; i++) {
      xx->data[(b_i + xx->size[0] * i) + 1] = xplus[i];
    }

    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }

  emxFree_real_T(&k4);
  emxFree_real_T(&k3);
  emxFree_real_T(&k2);
  emxFree_real_T(&k1);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (simulate_ol.c) */
