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
  "C:\\Users\\jiang\\Desktop\\nlmpc\\pdfmpc\\BBS.m"/* pathName */
};

static emlrtRSInfo b_emlrtRSI = { 11,  /* lineNo */
  "BBS",                               /* fcnName */
  "C:\\Users\\jiang\\Desktop\\nlmpc\\pdfmpc\\BBS.m"/* pathName */
};

static emlrtDCInfo emlrtDCI = { 9,     /* lineNo */
  27,                                  /* colNo */
  "BBS",                               /* fName */
  "C:\\Users\\jiang\\Desktop\\nlmpc\\pdfmpc\\BBS.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo emlrtBCI = { 1,     /* iFirst */
  32,                                  /* iLast */
  9,                                   /* lineNo */
  27,                                  /* colNo */
  "p",                                 /* aName */
  "BBS",                               /* fName */
  "C:\\Users\\jiang\\Desktop\\nlmpc\\pdfmpc\\BBS.m",/* pName */
  3                                    /* checkKind */
};

static emlrtRTEInfo n_emlrtRTEI = { 6, /* lineNo */
  16,                                  /* colNo */
  "BBS",                               /* fName */
  "C:\\Users\\jiang\\Desktop\\nlmpc\\pdfmpc\\BBS.m"/* pName */
};

/* Function Definitions */
void BBS(const emlrtStack *sp, real_T eta, struct0_T *param, real_T *J, real_T
         *g)
{
  int32_T i;
  real_T p[32];
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
  for (i = 0; i < 6; i++) {
    param->ode.x0[i] = param->x0[i];
  }

  memcpy(&p[0], &param->p[0], 32U * sizeof(real_T));
  if (param->ell != (int32_T)muDoubleScalarFloor(param->ell)) {
    emlrtIntegerCheckR2012b(param->ell, &emlrtDCI, sp);
  }

  i = (int32_T)param->ell;
  if ((i < 1) || (i > 32)) {
    emlrtDynamicBoundsCheckR2012b(i, 1, 32, &emlrtBCI, sp);
  }

  emxInit_real_T(sp, &unusedU0, 1, &n_emlrtRTEI, true);
  emxInit_real_T(sp, &xx, 2, &n_emlrtRTEI, true);
  emxInit_real_T(sp, &uu, 2, &n_emlrtRTEI, true);
  p[i - 1] = eta;
  st.site = &emlrtRSI;
  simulate_ol(&st, p, param->ode.tau, param->ode.rk_order, param->ode.x0,
              param->ode.M, param->ode.D, param->ode.thrust_config,
              param->uparam.nu, param->uparam.Np, param->uparam.R, unusedU0, xx,
              uu);
  st.site = &b_emlrtRSI;
  user_ocp(&st, xx, uu, param->ode.u0, param->uparam.Np, param->ocp.Q,
           param->ocp.R, param->ocp.Rdu, param->ocp.rd, param->ocp.dF_max,
           param->ocp.da_max, J, g);

  /* --------------------------------------------------------- */
  emxFree_real_T(&uu);
  emxFree_real_T(&xx);
  emxFree_real_T(&unusedU0);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (BBS.c) */
