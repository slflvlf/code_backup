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
#include "BBS_emxutil.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "simulate_ol.h"
#include "user_ocp.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = { 10,    /* lineNo */
  "BBS",                               /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\BBS.m"/* pathName */
};

static emlrtRSInfo b_emlrtRSI = { 11,  /* lineNo */
  "BBS",                               /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\BBS.m"/* pathName */
};

static emlrtDCInfo emlrtDCI = { 9,     /* lineNo */
  27,                                  /* colNo */
  "BBS",                               /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\BBS.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo emlrtBCI = { 1,     /* iFirst */
  48,                                  /* iLast */
  9,                                   /* lineNo */
  27,                                  /* colNo */
  "p",                                 /* aName */
  "BBS",                               /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\BBS.m",/* pName */
  3                                    /* checkKind */
};

static emlrtRTEInfo o_emlrtRTEI = { 6, /* lineNo */
  16,                                  /* colNo */
  "BBS",                               /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\BBS.m"/* pName */
};

/* Function Definitions */
void BBS(const emlrtStack *sp, real_T eta, struct0_T *param, real_T *J, real_T
         *g)
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
  memcpy(&param->ode.x0[0], &param->x0[0], 12U * sizeof(real_T));
  memcpy(&p[0], &param->p[0], 48U * sizeof(real_T));
  if (param->ell != (int32_T)muDoubleScalarFloor(param->ell)) {
    emlrtIntegerCheckR2012b(param->ell, &emlrtDCI, sp);
  }

  i = (int32_T)param->ell;
  if ((i < 1) || (i > 48)) {
    emlrtDynamicBoundsCheckR2012b(i, 1, 48, &emlrtBCI, sp);
  }

  emxInit_real_T(sp, &unusedU0, 1, &o_emlrtRTEI, true);
  emxInit_real_T(sp, &xx, 2, &o_emlrtRTEI, true);
  emxInit_real_T(sp, &uu, 2, &o_emlrtRTEI, true);
  p[i - 1] = eta;
  st.site = &emlrtRSI;
  simulate_ol(&st, p, param->ode.Nship, param->ode.tau, param->ode.rk_order,
              param->ode.x0, param->ode.M, param->ode.D,
              param->ode.thrust_config, param->uparam.nu, param->uparam.Np,
              param->uparam.R, unusedU0, xx, uu);
  st.site = &b_emlrtRSI;
  user_ocp(&st, xx, uu, param->ode.Nship, param->uparam.Np, param->ocp.Q,
           param->ocp.R, param->ocp.Rdu, param->ocp.xd_his, param->ocp.dF_max,
           param->ocp.da_max, param->ocp.u_last, J, g);

  /* --------------------------------------------------------- */
  emxFree_real_T(&uu);
  emxFree_real_T(&xx);
  emxFree_real_T(&unusedU0);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (BBS.c) */
