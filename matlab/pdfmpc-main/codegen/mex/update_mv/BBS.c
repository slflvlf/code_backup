/*
 * BBS.c
 *
 * Code generation for function 'BBS'
 *
 */

/* Include files */
#include "BBS.h"
#include "rt_nonfinite.h"
#include "update_mv_data.h"
#include "update_mv_emxutil.h"
#include "update_mv_types.h"
#include "user_control_profile.h"
#include "user_ocp.h"
#include "user_ode.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo u_emlrtRSI = {
    10,                                             /* lineNo */
    "BBS",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\BBS.m" /* pathName */
};

static emlrtRSInfo v_emlrtRSI = {
    11,                                             /* lineNo */
    "BBS",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\BBS.m" /* pathName */
};

static emlrtRSInfo w_emlrtRSI = {
    10,                                                     /* lineNo */
    "simulate_ol",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\simulate_ol.m" /* pathName */
};

static emlrtRSInfo x_emlrtRSI = {
    15,                                                     /* lineNo */
    "simulate_ol",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\simulate_ol.m" /* pathName */
};

static emlrtRSInfo cb_emlrtRSI = {
    6,                                                   /* lineNo */
    "one_step",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\one_step.m" /* pathName */
};

static emlrtRSInfo db_emlrtRSI = {
    8,                                                   /* lineNo */
    "one_step",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\one_step.m" /* pathName */
};

static emlrtRSInfo eb_emlrtRSI = {
    9,                                                   /* lineNo */
    "one_step",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\one_step.m" /* pathName */
};

static emlrtRSInfo fb_emlrtRSI = {
    12,                                                  /* lineNo */
    "one_step",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\one_step.m" /* pathName */
};

static emlrtRSInfo gb_emlrtRSI = {
    13,                                                  /* lineNo */
    "one_step",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\one_step.m" /* pathName */
};

static emlrtRSInfo hb_emlrtRSI = {
    14,                                                  /* lineNo */
    "one_step",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\one_step.m" /* pathName */
};

static emlrtRSInfo ib_emlrtRSI = {
    15,                                                  /* lineNo */
    "one_step",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\one_step.m" /* pathName */
};

static emlrtDCInfo d_emlrtDCI = {
    9,                                               /* lineNo */
    27,                                              /* colNo */
    "BBS",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\BBS.m", /* pName */
    1                                                /* checkKind */
};

static emlrtBCInfo k_emlrtBCI = {
    1,                                               /* iFirst */
    4,                                               /* iLast */
    9,                                               /* lineNo */
    27,                                              /* colNo */
    "p",                                             /* aName */
    "BBS",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\BBS.m", /* pName */
    3                                                /* checkKind */
};

static emlrtRTEInfo b_emlrtRTEI = {
    14,                                                     /* lineNo */
    7,                                                      /* colNo */
    "simulate_ol",                                          /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\simulate_ol.m" /* pName */
};

static emlrtBCInfo l_emlrtBCI = {
    -1,                                                      /* iFirst */
    -1,                                                      /* iLast */
    12,                                                      /* lineNo */
    4,                                                       /* colNo */
    "xx",                                                    /* aName */
    "simulate_ol",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\simulate_ol.m", /* pName */
    0                                                        /* checkKind */
};

static emlrtBCInfo m_emlrtBCI = {
    -1,                                                      /* iFirst */
    -1,                                                      /* iLast */
    15,                                                      /* lineNo */
    27,                                                      /* colNo */
    "xx",                                                    /* aName */
    "simulate_ol",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\simulate_ol.m", /* pName */
    0                                                        /* checkKind */
};

static emlrtBCInfo n_emlrtBCI = {
    -1,                                                      /* iFirst */
    -1,                                                      /* iLast */
    15,                                                      /* lineNo */
    36,                                                      /* colNo */
    "uu",                                                    /* aName */
    "simulate_ol",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\simulate_ol.m", /* pName */
    0                                                        /* checkKind */
};

static emlrtBCInfo o_emlrtBCI = {
    -1,                                                      /* iFirst */
    -1,                                                      /* iLast */
    15,                                                      /* lineNo */
    8,                                                       /* colNo */
    "xx",                                                    /* aName */
    "simulate_ol",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\simulate_ol.m", /* pName */
    0                                                        /* checkKind */
};

static emlrtDCInfo e_emlrtDCI = {
    11,                                                      /* lineNo */
    10,                                                      /* colNo */
    "simulate_ol",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\simulate_ol.m", /* pName */
    1                                                        /* checkKind */
};

static emlrtDCInfo f_emlrtDCI = {
    11,                                                      /* lineNo */
    10,                                                      /* colNo */
    "simulate_ol",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\simulate_ol.m", /* pName */
    4                                                        /* checkKind */
};

static emlrtDCInfo g_emlrtDCI = {
    11,                                                      /* lineNo */
    1,                                                       /* colNo */
    "simulate_ol",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\simulate_ol.m", /* pName */
    1                                                        /* checkKind */
};

static emlrtDCInfo h_emlrtDCI = {
    11,                                                      /* lineNo */
    1,                                                       /* colNo */
    "simulate_ol",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\simulate_ol.m", /* pName */
    4                                                        /* checkKind */
};

static emlrtRTEInfo p_emlrtRTEI = {
    11,                                                     /* lineNo */
    1,                                                      /* colNo */
    "simulate_ol",                                          /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\simulate_ol.m" /* pName */
};

static emlrtRTEInfo q_emlrtRTEI = {
    6,                                              /* lineNo */
    16,                                             /* colNo */
    "BBS",                                          /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\BBS.m" /* pName */
};

/* Function Definitions */
void BBS(const emlrtStack *sp, real_T eta, const real_T param_p[4],
         real_T param_ell, real_T param_uparam_nu, real_T param_uparam_Np,
         const real_T param_uparam_R[80], real_T param_ode_tau,
         real_T param_ode_rk_order, real_T param_ode_u0,
         const real_T param_ode_w[3], const struct4_T *param_ocp,
         const real_T param_x0[4], real_T *J, real_T *g)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  emxArray_real_T *xx;
  real_T uu_data[400];
  real_T p[4];
  real_T d;
  real_T *xx_data;
  int32_T uu_size[2];
  int32_T b_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtCTX)sp);
  /* --------------------------------------------------------- */
  /*  Author : Mazen Alamir */
  /*  CNRS, UNiversity of Grenoble-Alpes */
  /*  Gipsa Lab. Last modification March 2017 */
  /* --------------------------------------------------------- */
  p[0] = param_p[0];
  p[1] = param_p[1];
  p[2] = param_p[2];
  p[3] = param_p[3];
  if (param_ell != (int32_T)muDoubleScalarFloor(param_ell)) {
    emlrtIntegerCheckR2012b(param_ell, &d_emlrtDCI, (emlrtCTX)sp);
  }
  if (((int32_T)param_ell < 1) || ((int32_T)param_ell > 4)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)param_ell, 1, 4, &k_emlrtBCI,
                                  (emlrtCTX)sp);
  }
  emxInit_real_T(sp, &xx, 2, &q_emlrtRTEI);
  p[(int32_T)param_ell - 1] = eta;
  st.site = &u_emlrtRSI;
  /* --------------------------------------------------------- */
  /*  Author : Mazen Alamir  */
  /*  CNRS, UNiversity of Grenoble-Alpes */
  /*  Gipsa Lab. Last modification March 2017 */
  /* --------------------------------------------------------- */
  b_st.site = &w_emlrtRSI;
  user_control_profile(&b_st, p, param_uparam_nu, param_uparam_Np,
                       param_uparam_R, uu_data, uu_size);
  if (!(param_uparam_Np + 1.0 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(param_uparam_Np + 1.0, &f_emlrtDCI, &st);
  }
  d = (int32_T)muDoubleScalarFloor(param_uparam_Np + 1.0);
  if (param_uparam_Np + 1.0 != d) {
    emlrtIntegerCheckR2012b(param_uparam_Np + 1.0, &e_emlrtDCI, &st);
  }
  i = xx->size[0] * xx->size[1];
  xx->size[0] = (int32_T)(param_uparam_Np + 1.0);
  xx->size[1] = 4;
  emxEnsureCapacity_real_T(&st, xx, i, &p_emlrtRTEI);
  xx_data = xx->data;
  if (!(param_uparam_Np + 1.0 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(param_uparam_Np + 1.0, &h_emlrtDCI, &st);
  }
  if (param_uparam_Np + 1.0 != d) {
    emlrtIntegerCheckR2012b(param_uparam_Np + 1.0, &g_emlrtDCI, &st);
  }
  loop_ub = (int32_T)(param_uparam_Np + 1.0) << 2;
  for (i = 0; i < loop_ub; i++) {
    xx_data[i] = 0.0;
  }
  if ((int32_T)(param_uparam_Np + 1.0) < 1) {
    emlrtDynamicBoundsCheckR2012b(1, 1, (int32_T)(param_uparam_Np + 1.0),
                                  &l_emlrtBCI, &st);
  }
  xx_data[0] = param_x0[0];
  xx_data[xx->size[0]] = param_x0[1];
  xx_data[xx->size[0] * 2] = param_x0[2];
  xx_data[xx->size[0] * 3] = param_x0[3];
  i = (int32_T)param_uparam_Np;
  emlrtForLoopVectorCheckR2021a(1.0, 1.0, param_uparam_Np, mxDOUBLE_CLASS,
                                (int32_T)param_uparam_Np, &b_emlrtRTEI, &st);
  if (i - 1 >= 0) {
    b_loop_ub = uu_size[1];
  }
  for (loop_ub = 0; loop_ub < i; loop_ub++) {
    real_T u_data[20];
    b_st.site = &x_emlrtRSI;
    if ((loop_ub + 1 < 1) || (loop_ub + 1 > xx->size[0])) {
      emlrtDynamicBoundsCheckR2012b(loop_ub + 1, 1, xx->size[0], &m_emlrtBCI,
                                    &b_st);
    }
    if (((int32_T)(loop_ub + 1U) < 1) ||
        ((int32_T)(loop_ub + 1U) > uu_size[0])) {
      emlrtDynamicBoundsCheckR2012b((int32_T)(loop_ub + 1U), 1, uu_size[0],
                                    &n_emlrtBCI, &b_st);
    }
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      u_data[i1] = uu_data[loop_ub + uu_size[0] * i1];
    }
    /* --------------------------------------------------------- */
    if (param_ode_rk_order == 1.0) {
      real_T b_xx[4];
      b_xx[0] = xx_data[loop_ub];
      b_xx[1] = xx_data[loop_ub + xx->size[0]];
      b_xx[2] = xx_data[loop_ub + xx->size[0] * 2];
      b_xx[3] = xx_data[loop_ub + xx->size[0] * 3];
      c_st.site = &cb_emlrtRSI;
      user_ode(&c_st, b_xx, u_data, uu_size[1], param_ode_w, p);
      p[0] = xx_data[loop_ub] + param_ode_tau * p[0];
      p[1] = xx_data[loop_ub + xx->size[0]] + param_ode_tau * p[1];
      p[2] = xx_data[loop_ub + xx->size[0] * 2] + param_ode_tau * p[2];
      p[3] = xx_data[loop_ub + xx->size[0] * 3] + param_ode_tau * p[3];
    } else if (param_ode_rk_order == 2.0) {
      real_T b_xx[4];
      real_T k2[4];
      b_xx[0] = xx_data[loop_ub];
      b_xx[1] = xx_data[loop_ub + xx->size[0]];
      b_xx[2] = xx_data[loop_ub + xx->size[0] * 2];
      b_xx[3] = xx_data[loop_ub + xx->size[0] * 3];
      c_st.site = &db_emlrtRSI;
      user_ode(&c_st, b_xx, u_data, uu_size[1], param_ode_w, p);
      p[0] = xx_data[loop_ub] + 0.5 * p[0] * param_ode_tau;
      p[1] = xx_data[loop_ub + xx->size[0]] + 0.5 * p[1] * param_ode_tau;
      p[2] = xx_data[loop_ub + xx->size[0] * 2] + 0.5 * p[2] * param_ode_tau;
      p[3] = xx_data[loop_ub + xx->size[0] * 3] + 0.5 * p[3] * param_ode_tau;
      c_st.site = &eb_emlrtRSI;
      user_ode(&c_st, p, u_data, uu_size[1], param_ode_w, k2);
      p[0] = xx_data[loop_ub] + k2[0] * param_ode_tau;
      p[1] = xx_data[loop_ub + xx->size[0]] + k2[1] * param_ode_tau;
      p[2] = xx_data[loop_ub + xx->size[0] * 2] + k2[2] * param_ode_tau;
      p[3] = xx_data[loop_ub + xx->size[0] * 3] + k2[3] * param_ode_tau;
    } else {
      real_T b_xx[4];
      real_T k2[4];
      real_T k3[4];
      real_T k4[4];
      b_xx[0] = xx_data[loop_ub];
      b_xx[1] = xx_data[loop_ub + xx->size[0]];
      b_xx[2] = xx_data[loop_ub + xx->size[0] * 2];
      b_xx[3] = xx_data[loop_ub + xx->size[0] * 3];
      c_st.site = &fb_emlrtRSI;
      user_ode(&c_st, b_xx, u_data, uu_size[1], param_ode_w, p);
      b_xx[0] = xx_data[loop_ub] + 0.5 * p[0] * param_ode_tau;
      b_xx[1] = xx_data[loop_ub + xx->size[0]] + 0.5 * p[1] * param_ode_tau;
      b_xx[2] = xx_data[loop_ub + xx->size[0] * 2] + 0.5 * p[2] * param_ode_tau;
      b_xx[3] = xx_data[loop_ub + xx->size[0] * 3] + 0.5 * p[3] * param_ode_tau;
      c_st.site = &gb_emlrtRSI;
      user_ode(&c_st, b_xx, u_data, uu_size[1], param_ode_w, k2);
      b_xx[0] = xx_data[loop_ub] + 0.5 * k2[0] * param_ode_tau;
      b_xx[1] = xx_data[loop_ub + xx->size[0]] + 0.5 * k2[1] * param_ode_tau;
      b_xx[2] =
          xx_data[loop_ub + xx->size[0] * 2] + 0.5 * k2[2] * param_ode_tau;
      b_xx[3] =
          xx_data[loop_ub + xx->size[0] * 3] + 0.5 * k2[3] * param_ode_tau;
      c_st.site = &hb_emlrtRSI;
      user_ode(&c_st, b_xx, u_data, uu_size[1], param_ode_w, k3);
      b_xx[0] = xx_data[loop_ub] + k3[0] * param_ode_tau;
      b_xx[1] = xx_data[loop_ub + xx->size[0]] + k3[1] * param_ode_tau;
      b_xx[2] = xx_data[loop_ub + xx->size[0] * 2] + k3[2] * param_ode_tau;
      b_xx[3] = xx_data[loop_ub + xx->size[0] * 3] + k3[3] * param_ode_tau;
      c_st.site = &ib_emlrtRSI;
      user_ode(&c_st, b_xx, u_data, uu_size[1], param_ode_w, k4);
      p[0] = xx_data[loop_ub] +
             param_ode_tau * ((p[0] + 2.0 * (k2[0] + k3[0])) + k4[0]) / 6.0;
      p[1] = xx_data[loop_ub + xx->size[0]] +
             param_ode_tau * ((p[1] + 2.0 * (k2[1] + k3[1])) + k4[1]) / 6.0;
      p[2] = xx_data[loop_ub + xx->size[0] * 2] +
             param_ode_tau * ((p[2] + 2.0 * (k2[2] + k3[2])) + k4[2]) / 6.0;
      p[3] = xx_data[loop_ub + xx->size[0] * 3] +
             param_ode_tau * ((p[3] + 2.0 * (k2[3] + k3[3])) + k4[3]) / 6.0;
    }
    if (((int32_T)(loop_ub + 2U) < 1) ||
        ((int32_T)(loop_ub + 2U) > xx->size[0])) {
      emlrtDynamicBoundsCheckR2012b((int32_T)(loop_ub + 2U), 1, xx->size[0],
                                    &o_emlrtBCI, &st);
    }
    xx_data[loop_ub + 1] = p[0];
    xx_data[(loop_ub + xx->size[0]) + 1] = p[1];
    xx_data[(loop_ub + xx->size[0] * 2) + 1] = p[2];
    xx_data[(loop_ub + xx->size[0] * 3) + 1] = p[3];
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(&st);
    }
  }
  st.site = &v_emlrtRSI;
  user_ocp(&st, xx, uu_data, uu_size, param_ode_u0, param_uparam_Np,
           param_ocp->Q, param_ocp->R, param_ocp->M, param_ocp->rd,
           param_ocp->theta_max, param_ocp->thetap_max, J, g);
  emxFree_real_T(sp, &xx);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtCTX)sp);
}

/* End of code generation (BBS.c) */
