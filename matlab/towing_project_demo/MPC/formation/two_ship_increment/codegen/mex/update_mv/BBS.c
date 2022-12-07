/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * BBS.c
 *
 * Code generation for function 'BBS'
 *
 */

/* Include files */
#include "BBS.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "simulate_ol.h"
#include "update_mv.h"
#include "update_mv_emxutil.h"
#include "user_ocp.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo u_emlrtRSI = { 10,  /* lineNo */
  "BBS",                               /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\BBS.m"/* pathName */
};

static emlrtRSInfo v_emlrtRSI = { 11,  /* lineNo */
  "BBS",                               /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\BBS.m"/* pathName */
};

static emlrtDCInfo e_emlrtDCI = { 9,   /* lineNo */
  27,                                  /* colNo */
  "BBS",                               /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\BBS.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo k_emlrtBCI = { 1,   /* iFirst */
  48,                                  /* iLast */
  9,                                   /* lineNo */
  27,                                  /* colNo */
  "p",                                 /* aName */
  "BBS",                               /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\BBS.m",/* pName */
  3                                    /* checkKind */
};

static emlrtRTEInfo u_emlrtRTEI = { 6, /* lineNo */
  16,                                  /* colNo */
  "BBS",                               /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\BBS.m"/* pName */
};

/* Function Definitions */
void BBS(const emlrtStack *sp, real_T eta, const real_T param_p[48], real_T
         param_ell, real_T param_uparam_nu, real_T param_uparam_Np, const real_T
         param_uparam_R[11520], real_T param_ode_Nship, real_T param_ode_tau,
         real_T param_ode_rk_order, const real_T param_ode_u_last[12], const
         real_T param_ode_M[9], const real_T param_ode_D[9], const real_T
         param_ode_thrust_config[6], const struct4_T *param_ocp, const real_T
         param_x0[12], real_T *J, real_T *g)
{
  real_T p[48];
  int32_T i;
  emxArray_real_T *unusedU0;
  emxArray_real_T *xx;
  emxArray_real_T *uu;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);

  /* --------------------------------------------------------- */
  /*  Author : Mazen Alamir */
  /*  CNRS, UNiversity of Grenoble-Alpes */
  /*  Gipsa Lab. Last modification March 2017 */
  /* --------------------------------------------------------- */
  memcpy(&p[0], &param_p[0], 48U * sizeof(real_T));
  if (param_ell != (int32_T)muDoubleScalarFloor(param_ell)) {
    emlrtIntegerCheckR2012b(param_ell, &e_emlrtDCI, sp);
  }

  i = (int32_T)param_ell;
  if ((i < 1) || (i > 48)) {
    emlrtDynamicBoundsCheckR2012b(i, 1, 48, &k_emlrtBCI, sp);
  }

  emxInit_real_T(sp, &unusedU0, 1, &u_emlrtRTEI, true);
  emxInit_real_T(sp, &xx, 2, &u_emlrtRTEI, true);
  emxInit_real_T(sp, &uu, 2, &u_emlrtRTEI, true);
  p[i - 1] = eta;
  st.site = &u_emlrtRSI;
  simulate_ol(&st, p, param_ode_Nship, param_ode_tau, param_ode_rk_order,
              param_x0, param_ode_u_last, param_ode_M, param_ode_D,
              param_ode_thrust_config, param_uparam_nu, param_uparam_Np,
              param_uparam_R, unusedU0, xx, uu);
  st.site = &v_emlrtRSI;
  user_ocp(&st, xx, uu, param_ode_Nship, param_uparam_Np, param_ocp->Q,
           param_ocp->R, param_ocp->Rdu, param_ocp->xd_his, param_ocp->F_max,
           param_ocp->a_max, param_ocp->u_last, J, g);

  /* --------------------------------------------------------- */
  emxFree_real_T(&uu);
  emxFree_real_T(&xx);
  emxFree_real_T(&unusedU0);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (BBS.c) */
