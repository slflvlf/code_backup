/*
 * BBS.c
 *
 * Code generation for function 'BBS'
 *
 */

/* Include files */
#include "BBS.h"
#include "BBS_data.h"
#include "BBS_emxutil.h"
#include "BBS_types.h"
#include "rt_nonfinite.h"
#include "user_ocp.h"
#include "user_ode.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = {
    10,                                             /* lineNo */
    "BBS",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\BBS.m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    11,                                             /* lineNo */
    "BBS",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\BBS.m" /* pathName */
};

static emlrtRSInfo c_emlrtRSI = {
    10,                                                     /* lineNo */
    "simulate_ol",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\simulate_ol.m" /* pathName */
};

static emlrtRSInfo d_emlrtRSI = {
    15,                                                     /* lineNo */
    "simulate_ol",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\simulate_ol.m" /* pathName */
};

static emlrtRSInfo e_emlrtRSI = {
    5,                      /* lineNo */
    "user_control_profile", /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\user_control_profile.m" /* pathName
                                                                      */
};

static emlrtRSInfo f_emlrtRSI = {
    29,                  /* lineNo */
    "reshapeSizeChecks", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pathName */
};

static emlrtRSInfo g_emlrtRSI = {
    109,               /* lineNo */
    "computeDimsData", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pathName */
};

static emlrtRSInfo h_emlrtRSI = {
    6,                                                   /* lineNo */
    "one_step",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\one_step.m" /* pathName */
};

static emlrtRSInfo i_emlrtRSI = {
    8,                                                   /* lineNo */
    "one_step",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\one_step.m" /* pathName */
};

static emlrtRSInfo j_emlrtRSI = {
    9,                                                   /* lineNo */
    "one_step",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\one_step.m" /* pathName */
};

static emlrtRSInfo k_emlrtRSI = {
    12,                                                  /* lineNo */
    "one_step",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\one_step.m" /* pathName */
};

static emlrtRSInfo l_emlrtRSI = {
    13,                                                  /* lineNo */
    "one_step",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\one_step.m" /* pathName */
};

static emlrtRSInfo m_emlrtRSI = {
    14,                                                  /* lineNo */
    "one_step",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\one_step.m" /* pathName */
};

static emlrtRSInfo n_emlrtRSI = {
    15,                                                  /* lineNo */
    "one_step",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\one_step.m" /* pathName */
};

static emlrtDCInfo emlrtDCI = {
    9,                                               /* lineNo */
    27,                                              /* colNo */
    "BBS",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\BBS.m", /* pName */
    1                                                /* checkKind */
};

static emlrtBCInfo emlrtBCI = {
    1,                                               /* iFirst */
    4,                                               /* iLast */
    9,                                               /* lineNo */
    27,                                              /* colNo */
    "p",                                             /* aName */
    "BBS",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\BBS.m", /* pName */
    3                                                /* checkKind */
};

static emlrtRTEInfo emlrtRTEI = {
    14,                                                     /* lineNo */
    7,                                                      /* colNo */
    "simulate_ol",                                          /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\simulate_ol.m" /* pName */
};

static emlrtBCInfo b_emlrtBCI = {
    -1,                                                      /* iFirst */
    -1,                                                      /* iLast */
    12,                                                      /* lineNo */
    4,                                                       /* colNo */
    "xx",                                                    /* aName */
    "simulate_ol",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\simulate_ol.m", /* pName */
    0                                                        /* checkKind */
};

static emlrtBCInfo c_emlrtBCI = {
    -1,                                                      /* iFirst */
    -1,                                                      /* iLast */
    15,                                                      /* lineNo */
    27,                                                      /* colNo */
    "xx",                                                    /* aName */
    "simulate_ol",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\simulate_ol.m", /* pName */
    0                                                        /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = {
    -1,                                                      /* iFirst */
    -1,                                                      /* iLast */
    15,                                                      /* lineNo */
    36,                                                      /* colNo */
    "uu",                                                    /* aName */
    "simulate_ol",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\simulate_ol.m", /* pName */
    0                                                        /* checkKind */
};

static emlrtBCInfo e_emlrtBCI = {
    -1,                                                      /* iFirst */
    -1,                                                      /* iLast */
    15,                                                      /* lineNo */
    8,                                                       /* colNo */
    "xx",                                                    /* aName */
    "simulate_ol",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\simulate_ol.m", /* pName */
    0                                                        /* checkKind */
};

static emlrtRTEInfo b_emlrtRTEI = {
    52,                  /* lineNo */
    13,                  /* colNo */
    "reshapeSizeChecks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pName */
};

static emlrtRTEInfo c_emlrtRTEI = {
    57,                  /* lineNo */
    23,                  /* colNo */
    "reshapeSizeChecks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pName */
};

static emlrtRTEInfo d_emlrtRTEI = {
    59,                  /* lineNo */
    23,                  /* colNo */
    "reshapeSizeChecks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pName */
};

static emlrtDCInfo b_emlrtDCI = {
    11,                                                      /* lineNo */
    10,                                                      /* colNo */
    "simulate_ol",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\simulate_ol.m", /* pName */
    1                                                        /* checkKind */
};

static emlrtDCInfo c_emlrtDCI = {
    11,                                                      /* lineNo */
    10,                                                      /* colNo */
    "simulate_ol",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\simulate_ol.m", /* pName */
    4                                                        /* checkKind */
};

static emlrtDCInfo d_emlrtDCI = {
    11,                                                      /* lineNo */
    1,                                                       /* colNo */
    "simulate_ol",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\simulate_ol.m", /* pName */
    1                                                        /* checkKind */
};

static emlrtDCInfo e_emlrtDCI = {
    11,                                                      /* lineNo */
    1,                                                       /* colNo */
    "simulate_ol",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\simulate_ol.m", /* pName */
    4                                                        /* checkKind */
};

static emlrtRTEInfo e_emlrtRTEI = {
    58,                   /* lineNo */
    23,                   /* colNo */
    "assertValidSizeArg", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\assertValidSizeArg.m" /* pName */
};

static emlrtRTEInfo f_emlrtRTEI = {
    64,                   /* lineNo */
    15,                   /* colNo */
    "assertValidSizeArg", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\assertValidSizeArg.m" /* pName */
};

static emlrtRTEInfo k_emlrtRTEI = {
    11,                                                     /* lineNo */
    1,                                                      /* colNo */
    "simulate_ol",                                          /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\simulate_ol.m" /* pName */
};

static emlrtRTEInfo l_emlrtRTEI = {
    6,                                              /* lineNo */
    16,                                             /* colNo */
    "BBS",                                          /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\BBS.m" /* pName */
};

/* Function Definitions */
void BBS(const emlrtStack *sp, real_T eta, struct0_T *param, real_T *J,
         real_T *g)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  emxArray_real_T *xx;
  real_T dv[20];
  real_T b_param;
  real_T *xx_data;
  int32_T varargin_1[2];
  int32_T b_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T u_size;
  boolean_T out;
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
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtCTX)sp);
  /* --------------------------------------------------------- */
  /*  Author : Mazen Alamir */
  /*  CNRS, UNiversity of Grenoble-Alpes */
  /*  Gipsa Lab. Last modification March 2017 */
  /* --------------------------------------------------------- */
  param->ode.x0[0] = param->x0[0];
  param->ode.x0[1] = param->x0[1];
  param->ode.x0[2] = param->x0[2];
  param->ode.x0[3] = param->x0[3];
  if (param->ell != (int32_T)muDoubleScalarFloor(param->ell)) {
    emlrtIntegerCheckR2012b(param->ell, &emlrtDCI, (emlrtCTX)sp);
  }
  if (((int32_T)param->ell < 1) || ((int32_T)param->ell > 4)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)param->ell, 1, 4, &emlrtBCI,
                                  (emlrtCTX)sp);
  }
  param->p[(int32_T)param->ell - 1] = eta;
  st.site = &emlrtRSI;
  /* --------------------------------------------------------- */
  /*  Author : Mazen Alamir  */
  /*  CNRS, UNiversity of Grenoble-Alpes */
  /*  Gipsa Lab. Last modification March 2017 */
  /* --------------------------------------------------------- */
  b_st.site = &c_emlrtRSI;
  /* -------------------------------------------------------------------------------
   */
  /* pdf_mpc package: Example 1 - Definition of the user_control_profile map */
  /* -------------------------------------------------------------------------------
   */
  for (i = 0; i < 20; i++) {
    dv[i] = ((param->uparam.R[i] * param->p[0] +
              param->uparam.R[i + 20] * param->p[1]) +
             param->uparam.R[i + 40] * param->p[2]) +
            param->uparam.R[i + 60] * param->p[3];
  }
  c_st.site = &e_emlrtRSI;
  d_st.site = &f_emlrtRSI;
  e_st.site = &g_emlrtRSI;
  if ((param->uparam.Np != muDoubleScalarFloor(param->uparam.Np)) ||
      muDoubleScalarIsInf(param->uparam.Np) ||
      (param->uparam.Np < -2.147483648E+9) ||
      (param->uparam.Np > 2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(
        &e_st, &e_emlrtRTEI, "Coder:MATLAB:NonIntegerInput",
        "Coder:MATLAB:NonIntegerInput", 4, 12, MIN_int32_T, 12, MAX_int32_T);
  }
  if (param->uparam.Np <= 0.0) {
    b_param = 0.0;
  } else {
    b_param = param->uparam.Np;
  }
  if (!(b_param <= 2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(&e_st, &f_emlrtRTEI, "Coder:MATLAB:pmaxsize",
                                  "Coder:MATLAB:pmaxsize", 0);
  }
  e_st.site = &g_emlrtRSI;
  if ((param->uparam.nu != muDoubleScalarFloor(param->uparam.nu)) ||
      muDoubleScalarIsInf(param->uparam.nu) ||
      (param->uparam.nu < -2.147483648E+9) ||
      (param->uparam.nu > 2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(
        &e_st, &e_emlrtRTEI, "Coder:MATLAB:NonIntegerInput",
        "Coder:MATLAB:NonIntegerInput", 4, 12, MIN_int32_T, 12, MAX_int32_T);
  }
  if (param->uparam.nu <= 0.0) {
    b_param = 0.0;
  } else {
    b_param = param->uparam.nu;
  }
  if (!(b_param <= 2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(&e_st, &f_emlrtRTEI, "Coder:MATLAB:pmaxsize",
                                  "Coder:MATLAB:pmaxsize", 0);
  }
  if ((int32_T)param->uparam.Np > 20) {
    emlrtErrorWithMessageIdR2018a(&c_st, &b_emlrtRTEI,
                                  "Coder:toolbox:reshape_emptyReshapeLimit",
                                  "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }
  if ((int32_T)param->uparam.nu > 20) {
    emlrtErrorWithMessageIdR2018a(&c_st, &b_emlrtRTEI,
                                  "Coder:toolbox:reshape_emptyReshapeLimit",
                                  "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }
  out = ((int32_T)param->uparam.Np >= 0);
  if ((!out) || ((int32_T)param->uparam.nu < 0)) {
    out = false;
  }
  if (!out) {
    emlrtErrorWithMessageIdR2018a(&c_st, &c_emlrtRTEI,
                                  "MATLAB:checkDimCommon:nonnegativeSize",
                                  "MATLAB:checkDimCommon:nonnegativeSize", 0);
  }
  if ((int32_T)param->uparam.Np * (int32_T)param->uparam.nu != 20) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &d_emlrtRTEI, "Coder:MATLAB:getReshapeDims_notSameNumel",
        "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
  }
  emxInit_real_T(&c_st, &xx, 2, &l_emlrtRTEI);
  if (!(param->uparam.Np + 1.0 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(param->uparam.Np + 1.0, &c_emlrtDCI, &st);
  }
  b_param = (int32_T)muDoubleScalarFloor(param->uparam.Np + 1.0);
  if (param->uparam.Np + 1.0 != b_param) {
    emlrtIntegerCheckR2012b(param->uparam.Np + 1.0, &b_emlrtDCI, &st);
  }
  i = xx->size[0] * xx->size[1];
  xx->size[0] = (int32_T)(param->uparam.Np + 1.0);
  xx->size[1] = 4;
  emxEnsureCapacity_real_T(&st, xx, i, &k_emlrtRTEI);
  xx_data = xx->data;
  if (!(param->uparam.Np + 1.0 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(param->uparam.Np + 1.0, &e_emlrtDCI, &st);
  }
  if (param->uparam.Np + 1.0 != b_param) {
    emlrtIntegerCheckR2012b(param->uparam.Np + 1.0, &d_emlrtDCI, &st);
  }
  loop_ub = (int32_T)(param->uparam.Np + 1.0) << 2;
  for (i = 0; i < loop_ub; i++) {
    xx_data[i] = 0.0;
  }
  if ((int32_T)(param->uparam.Np + 1.0) < 1) {
    emlrtDynamicBoundsCheckR2012b(1, 1, (int32_T)(param->uparam.Np + 1.0),
                                  &b_emlrtBCI, &st);
  }
  xx_data[0] = param->ode.x0[0];
  xx_data[xx->size[0]] = param->ode.x0[1];
  xx_data[xx->size[0] * 2] = param->ode.x0[2];
  xx_data[xx->size[0] * 3] = param->ode.x0[3];
  i = (int32_T)param->uparam.Np;
  emlrtForLoopVectorCheckR2021a(1.0, 1.0, param->uparam.Np, mxDOUBLE_CLASS,
                                (int32_T)param->uparam.Np, &emlrtRTEI, &st);
  if ((int32_T)param->uparam.Np - 1 >= 0) {
    varargin_1[0] = (int32_T)param->uparam.Np;
    u_size = (int32_T)param->uparam.nu;
    b_loop_ub = (int32_T)param->uparam.nu;
  }
  for (loop_ub = 0; loop_ub < i; loop_ub++) {
    real_T u_data[20];
    b_st.site = &d_emlrtRSI;
    if ((loop_ub + 1 < 1) || (loop_ub + 1 > xx->size[0])) {
      emlrtDynamicBoundsCheckR2012b(loop_ub + 1, 1, xx->size[0], &c_emlrtBCI,
                                    &b_st);
    }
    if (((int32_T)(loop_ub + 1U) < 1) ||
        ((int32_T)(loop_ub + 1U) > (int32_T)param->uparam.Np)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)(loop_ub + 1U), 1,
                                    (int32_T)param->uparam.Np, &d_emlrtBCI,
                                    &b_st);
    }
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      u_data[i1] = dv[loop_ub + varargin_1[0] * i1];
    }
    /* --------------------------------------------------------- */
    if (param->ode.rk_order == 1.0) {
      real_T b_xx[4];
      b_xx[0] = xx_data[loop_ub];
      b_xx[1] = xx_data[loop_ub + xx->size[0]];
      b_xx[2] = xx_data[loop_ub + xx->size[0] * 2];
      b_xx[3] = xx_data[loop_ub + xx->size[0] * 3];
      c_st.site = &h_emlrtRSI;
      user_ode(&c_st, b_xx, u_data, u_size, param->ode.w, param->p);
      param->p[0] = xx_data[loop_ub] + param->ode.tau * param->p[0];
      param->p[1] =
          xx_data[loop_ub + xx->size[0]] + param->ode.tau * param->p[1];
      param->p[2] =
          xx_data[loop_ub + xx->size[0] * 2] + param->ode.tau * param->p[2];
      param->p[3] =
          xx_data[loop_ub + xx->size[0] * 3] + param->ode.tau * param->p[3];
    } else if (param->ode.rk_order == 2.0) {
      real_T b_xx[4];
      real_T k2[4];
      b_xx[0] = xx_data[loop_ub];
      b_xx[1] = xx_data[loop_ub + xx->size[0]];
      b_xx[2] = xx_data[loop_ub + xx->size[0] * 2];
      b_xx[3] = xx_data[loop_ub + xx->size[0] * 3];
      c_st.site = &i_emlrtRSI;
      user_ode(&c_st, b_xx, u_data, u_size, param->ode.w, param->p);
      b_xx[0] = xx_data[loop_ub] + 0.5 * param->p[0] * param->ode.tau;
      b_xx[1] =
          xx_data[loop_ub + xx->size[0]] + 0.5 * param->p[1] * param->ode.tau;
      b_xx[2] = xx_data[loop_ub + xx->size[0] * 2] +
                0.5 * param->p[2] * param->ode.tau;
      b_xx[3] = xx_data[loop_ub + xx->size[0] * 3] +
                0.5 * param->p[3] * param->ode.tau;
      c_st.site = &j_emlrtRSI;
      user_ode(&c_st, b_xx, u_data, u_size, param->ode.w, k2);
      param->p[0] = xx_data[loop_ub] + k2[0] * param->ode.tau;
      param->p[1] = xx_data[loop_ub + xx->size[0]] + k2[1] * param->ode.tau;
      param->p[2] = xx_data[loop_ub + xx->size[0] * 2] + k2[2] * param->ode.tau;
      param->p[3] = xx_data[loop_ub + xx->size[0] * 3] + k2[3] * param->ode.tau;
    } else {
      real_T b_xx[4];
      real_T k2[4];
      real_T k3[4];
      real_T k4[4];
      b_xx[0] = xx_data[loop_ub];
      b_xx[1] = xx_data[loop_ub + xx->size[0]];
      b_xx[2] = xx_data[loop_ub + xx->size[0] * 2];
      b_xx[3] = xx_data[loop_ub + xx->size[0] * 3];
      c_st.site = &k_emlrtRSI;
      user_ode(&c_st, b_xx, u_data, u_size, param->ode.w, param->p);
      b_xx[0] = xx_data[loop_ub] + 0.5 * param->p[0] * param->ode.tau;
      b_xx[1] =
          xx_data[loop_ub + xx->size[0]] + 0.5 * param->p[1] * param->ode.tau;
      b_xx[2] = xx_data[loop_ub + xx->size[0] * 2] +
                0.5 * param->p[2] * param->ode.tau;
      b_xx[3] = xx_data[loop_ub + xx->size[0] * 3] +
                0.5 * param->p[3] * param->ode.tau;
      c_st.site = &l_emlrtRSI;
      user_ode(&c_st, b_xx, u_data, u_size, param->ode.w, k2);
      b_xx[0] = xx_data[loop_ub] + 0.5 * k2[0] * param->ode.tau;
      b_xx[1] = xx_data[loop_ub + xx->size[0]] + 0.5 * k2[1] * param->ode.tau;
      b_xx[2] =
          xx_data[loop_ub + xx->size[0] * 2] + 0.5 * k2[2] * param->ode.tau;
      b_xx[3] =
          xx_data[loop_ub + xx->size[0] * 3] + 0.5 * k2[3] * param->ode.tau;
      c_st.site = &m_emlrtRSI;
      user_ode(&c_st, b_xx, u_data, u_size, param->ode.w, k3);
      b_xx[0] = xx_data[loop_ub] + k3[0] * param->ode.tau;
      b_xx[1] = xx_data[loop_ub + xx->size[0]] + k3[1] * param->ode.tau;
      b_xx[2] = xx_data[loop_ub + xx->size[0] * 2] + k3[2] * param->ode.tau;
      b_xx[3] = xx_data[loop_ub + xx->size[0] * 3] + k3[3] * param->ode.tau;
      c_st.site = &n_emlrtRSI;
      user_ode(&c_st, b_xx, u_data, u_size, param->ode.w, k4);
      param->p[0] = xx_data[loop_ub] +
                    param->ode.tau *
                        ((param->p[0] + 2.0 * (k2[0] + k3[0])) + k4[0]) / 6.0;
      param->p[1] = xx_data[loop_ub + xx->size[0]] +
                    param->ode.tau *
                        ((param->p[1] + 2.0 * (k2[1] + k3[1])) + k4[1]) / 6.0;
      param->p[2] = xx_data[loop_ub + xx->size[0] * 2] +
                    param->ode.tau *
                        ((param->p[2] + 2.0 * (k2[2] + k3[2])) + k4[2]) / 6.0;
      param->p[3] = xx_data[loop_ub + xx->size[0] * 3] +
                    param->ode.tau *
                        ((param->p[3] + 2.0 * (k2[3] + k3[3])) + k4[3]) / 6.0;
    }
    if (((int32_T)(loop_ub + 2U) < 1) ||
        ((int32_T)(loop_ub + 2U) > xx->size[0])) {
      emlrtDynamicBoundsCheckR2012b((int32_T)(loop_ub + 2U), 1, xx->size[0],
                                    &e_emlrtBCI, &st);
    }
    xx_data[loop_ub + 1] = param->p[0];
    xx_data[(loop_ub + xx->size[0]) + 1] = param->p[1];
    xx_data[(loop_ub + xx->size[0] * 2) + 1] = param->p[2];
    xx_data[(loop_ub + xx->size[0] * 3) + 1] = param->p[3];
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(&st);
    }
  }
  varargin_1[0] = (int32_T)param->uparam.Np;
  varargin_1[1] = (int32_T)param->uparam.nu;
  st.site = &b_emlrtRSI;
  user_ocp(&st, xx, dv, varargin_1, param->ode.u0, param->uparam.Np,
           param->ocp.Q, param->ocp.R, param->ocp.M, param->ocp.rd,
           param->ocp.theta_max, param->ocp.thetap_max, J, g);
  emxFree_real_T(sp, &xx);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtCTX)sp);
}

/* End of code generation (BBS.c) */
