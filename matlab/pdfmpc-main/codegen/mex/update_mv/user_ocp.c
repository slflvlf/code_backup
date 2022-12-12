/*
 * user_ocp.c
 *
 * Code generation for function 'user_ocp'
 *
 */

/* Include files */
#include "user_ocp.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "update_mv_data.h"
#include "update_mv_emxutil.h"
#include "update_mv_types.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo lb_emlrtRSI = {
    44,       /* lineNo */
    "mpower", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\lib\\matlab\\matfun\\mpower.m" /* pathName
                                                                          */
};

static emlrtRSInfo mb_emlrtRSI =
    {
        71,      /* lineNo */
        "power", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2022a\\toolbox\\eml\\lib\\matlab\\ops\\power.m" /* pathName
                                                                          */
};

static emlrtRSInfo nb_emlrtRSI = {
    13,                                                  /* lineNo */
    "user_ocp",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\user_ocp.m" /* pathName */
};

static emlrtRSInfo ob_emlrtRSI = {
    15,                                                  /* lineNo */
    "user_ocp",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\user_ocp.m" /* pathName */
};

static emlrtRSInfo pb_emlrtRSI = {
    16,                                                  /* lineNo */
    "user_ocp",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\user_ocp.m" /* pathName */
};

static emlrtRSInfo qb_emlrtRSI = {
    17,                                                  /* lineNo */
    "user_ocp",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\user_ocp.m" /* pathName */
};

static emlrtRSInfo rb_emlrtRSI = {
    18,                                                  /* lineNo */
    "user_ocp",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\user_ocp.m" /* pathName */
};

static emlrtRSInfo sb_emlrtRSI = {
    19,                                                  /* lineNo */
    "user_ocp",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\user_ocp.m" /* pathName */
};

static emlrtRSInfo tb_emlrtRSI = {
    15,    /* lineNo */
    "max", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\lib\\matlab\\datafun\\max.m" /* pathName
                                                                        */
};

static emlrtRSInfo ub_emlrtRSI =
    {
        44,         /* lineNo */
        "minOrMax", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax."
        "m" /* pathName */
};

static emlrtRSInfo vb_emlrtRSI =
    {
        79,        /* lineNo */
        "maximum", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax."
        "m" /* pathName */
};

static emlrtRSInfo wb_emlrtRSI = {
    191,             /* lineNo */
    "unaryMinOrMax", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo xb_emlrtRSI = {
    902,                    /* lineNo */
    "maxRealVectorOmitNaN", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo yb_emlrtRSI = {
    72,                      /* lineNo */
    "vectorMinOrMaxInPlace", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\vectorMinOrMaxInPlace.m" /* pathName */
};

static emlrtRSInfo ac_emlrtRSI = {
    64,                      /* lineNo */
    "vectorMinOrMaxInPlace", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\vectorMinOrMaxInPlace.m" /* pathName */
};

static emlrtRSInfo bc_emlrtRSI = {
    113,         /* lineNo */
    "findFirst", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\vectorMinOrMaxInPlace.m" /* pathName */
};

static emlrtRSInfo cc_emlrtRSI = {
    20,                               /* lineNo */
    "eml_int_forloop_overflow_check", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\lib\\matlab\\eml\\eml_int_forloop_"
    "overflow_check.m" /* pathName */
};

static emlrtRSInfo dc_emlrtRSI = {
    130,                        /* lineNo */
    "minOrMaxRealVectorKernel", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\vectorMinOrMaxInPlace.m" /* pathName */
};

static emlrtRTEInfo h_emlrtRTEI = {
    6,                                                   /* lineNo */
    7,                                                   /* colNo */
    "user_ocp",                                          /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\user_ocp.m" /* pName */
};

static emlrtBCInfo p_emlrtBCI = {
    -1,                                                   /* iFirst */
    -1,                                                   /* iLast */
    8,                                                    /* lineNo */
    15,                                                   /* colNo */
    "uu",                                                 /* aName */
    "user_ocp",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\user_ocp.m", /* pName */
    0                                                     /* checkKind */
};

static emlrtBCInfo q_emlrtBCI = {
    -1,                                                   /* iFirst */
    -1,                                                   /* iLast */
    12,                                                   /* lineNo */
    10,                                                   /* colNo */
    "xx",                                                 /* aName */
    "user_ocp",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\user_ocp.m", /* pName */
    0                                                     /* checkKind */
};

static emlrtBCInfo r_emlrtBCI = {
    -1,                                                   /* iFirst */
    -1,                                                   /* iLast */
    13,                                                   /* lineNo */
    34,                                                   /* colNo */
    "uu",                                                 /* aName */
    "user_ocp",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\user_ocp.m", /* pName */
    0                                                     /* checkKind */
};

static emlrtRTEInfo j_emlrtRTEI = {
    135,             /* lineNo */
    27,              /* colNo */
    "unaryMinOrMax", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\unaryMinOrMax.m" /* pName */
};

static emlrtBCInfo s_emlrtBCI = {
    -1,                                                   /* iFirst */
    -1,                                                   /* iLast */
    10,                                                   /* lineNo */
    15,                                                   /* colNo */
    "uu",                                                 /* aName */
    "user_ocp",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\user_ocp.m", /* pName */
    0                                                     /* checkKind */
};

static emlrtBCInfo t_emlrtBCI = {
    -1,                                                   /* iFirst */
    -1,                                                   /* iLast */
    10,                                                   /* lineNo */
    21,                                                   /* colNo */
    "uu",                                                 /* aName */
    "user_ocp",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\user_ocp.m", /* pName */
    0                                                     /* checkKind */
};

static emlrtRTEInfo r_emlrtRTEI = {
    15,                                                  /* lineNo */
    8,                                                   /* colNo */
    "user_ocp",                                          /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\user_ocp.m" /* pName */
};

static emlrtRTEInfo s_emlrtRTEI = {
    16,                                                  /* lineNo */
    8,                                                   /* colNo */
    "user_ocp",                                          /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\user_ocp.m" /* pName */
};

static emlrtRTEInfo t_emlrtRTEI = {
    17,                                                  /* lineNo */
    8,                                                   /* colNo */
    "user_ocp",                                          /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\user_ocp.m" /* pName */
};

static emlrtRTEInfo u_emlrtRTEI = {
    18,                                                  /* lineNo */
    8,                                                   /* colNo */
    "user_ocp",                                          /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\user_ocp.m" /* pName */
};

/* Function Definitions */
void user_ocp(const emlrtStack *sp, const emxArray_real_T *xx,
              const real_T uu_data[], const int32_T uu_size[2], real_T p_ode_u0,
              real_T p_uparam_Np, const real_T p_ocp_Q[16], real_T p_ocp_R,
              real_T p_ocp_M, real_T p_ocp_rd, real_T p_ocp_theta_max,
              real_T p_ocp_thetap_max, real_T *J, real_T *g)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack i_st;
  emlrtStack st;
  emxArray_real_T *varargin_1;
  real_T xd[4];
  const real_T *xx_data;
  real_T a;
  real_T h2;
  real_T h3;
  real_T h4;
  real_T *varargin_1_data;
  int32_T b_i;
  int32_T i;
  int32_T idx;
  int32_T k;
  int32_T last;
  boolean_T exitg1;
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
  xx_data = xx->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtCTX)sp);
  /* -------------------------------------------------------------------------------
   */
  /*  pdf_mpc package: Example 1 - Definition of the user_ocp map */
  /* -------------------------------------------------------------------------------
   */
  *J = 0.0;
  i = (int32_T)p_uparam_Np;
  emlrtForLoopVectorCheckR2021a(1.0, 1.0, p_uparam_Np, mxDOUBLE_CLASS,
                                (int32_T)p_uparam_Np, &h_emlrtRTEI,
                                (emlrtCTX)sp);
  for (b_i = 0; b_i < i; b_i++) {
    real_T e[4];
    if (b_i + 1U == 1U) {
      k = uu_size[0];
      last = uu_size[1];
      idx = uu_size[0] * uu_size[1];
      if (idx < 1) {
        emlrtDynamicBoundsCheckR2012b(1, 1, idx, &p_emlrtBCI, (emlrtCTX)sp);
      }
      a = uu_data[0] - p_ode_u0;
    } else {
      k = uu_size[0];
      last = uu_size[1];
      idx = uu_size[0] * uu_size[1];
      if (((int32_T)(b_i + 1U) < 1) || ((int32_T)(b_i + 1U) > idx)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)(b_i + 1U), 1, idx, &s_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      if ((b_i < 1) || (b_i > idx)) {
        emlrtDynamicBoundsCheckR2012b(b_i, 1, idx, &t_emlrtBCI, (emlrtCTX)sp);
      }
      a = uu_data[b_i] - uu_data[b_i - 1];
    }
    if (((int32_T)(b_i + 2U) < 1) || ((int32_T)(b_i + 2U) > xx->size[0])) {
      emlrtDynamicBoundsCheckR2012b((int32_T)(b_i + 2U), 1, xx->size[0],
                                    &q_emlrtBCI, (emlrtCTX)sp);
    }
    e[0] = xx_data[b_i + 1] - p_ocp_rd;
    e[1] = xx_data[(b_i + xx->size[0]) + 1];
    e[2] = xx_data[(b_i + xx->size[0] * 2) + 1];
    e[3] = xx_data[(b_i + xx->size[0] * 3) + 1];
    st.site = &nb_emlrtRSI;
    k *= last;
    if ((b_i + 1 < 1) || (b_i + 1 > k)) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, k, &r_emlrtBCI, &st);
    }
    b_st.site = &lb_emlrtRSI;
    c_st.site = &mb_emlrtRSI;
    st.site = &nb_emlrtRSI;
    b_st.site = &lb_emlrtRSI;
    c_st.site = &mb_emlrtRSI;
    h2 = 0.0;
    for (k = 0; k < 4; k++) {
      last = k << 2;
      h2 += (((e[0] * p_ocp_Q[last] + e[1] * p_ocp_Q[last + 1]) +
              e[2] * p_ocp_Q[last + 2]) +
             e[3] * p_ocp_Q[last + 3]) *
            e[k];
    }
    *J += (h2 + p_ocp_R * (uu_data[b_i] * uu_data[b_i])) + p_ocp_M * (a * a);
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtCTX)sp);
    }
  }
  emxInit_real_T(sp, &varargin_1, 1, &r_emlrtRTEI);
  st.site = &ob_emlrtRSI;
  last = xx->size[0];
  i = varargin_1->size[0];
  varargin_1->size[0] = xx->size[0];
  emxEnsureCapacity_real_T(&st, varargin_1, i, &r_emlrtRTEI);
  varargin_1_data = varargin_1->data;
  for (i = 0; i < last; i++) {
    varargin_1_data[i] = xx_data[i + xx->size[0] * 2] - p_ocp_theta_max;
  }
  b_st.site = &tb_emlrtRSI;
  c_st.site = &ub_emlrtRSI;
  d_st.site = &vb_emlrtRSI;
  if (varargin_1->size[0] < 1) {
    emlrtErrorWithMessageIdR2018a(&d_st, &j_emlrtRTEI,
                                  "Coder:toolbox:eml_min_or_max_varDimZero",
                                  "Coder:toolbox:eml_min_or_max_varDimZero", 0);
  }
  e_st.site = &wb_emlrtRSI;
  f_st.site = &xb_emlrtRSI;
  last = varargin_1->size[0];
  if (varargin_1->size[0] <= 2) {
    if (varargin_1->size[0] == 1) {
      *g = varargin_1_data[0];
    } else if ((varargin_1_data[0] < varargin_1_data[1]) ||
               (muDoubleScalarIsNaN(varargin_1_data[0]) &&
                (!muDoubleScalarIsNaN(varargin_1_data[1])))) {
      *g = varargin_1_data[1];
    } else {
      *g = varargin_1_data[0];
    }
  } else {
    g_st.site = &ac_emlrtRSI;
    if (!muDoubleScalarIsNaN(varargin_1_data[0])) {
      idx = 1;
    } else {
      idx = 0;
      h_st.site = &bc_emlrtRSI;
      if (varargin_1->size[0] > 2147483646) {
        i_st.site = &cc_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= last)) {
        if (!muDoubleScalarIsNaN(varargin_1_data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (idx == 0) {
      *g = varargin_1_data[0];
    } else {
      g_st.site = &yb_emlrtRSI;
      *g = varargin_1_data[idx - 1];
      b_i = idx + 1;
      h_st.site = &dc_emlrtRSI;
      if ((idx + 1 <= varargin_1->size[0]) &&
          (varargin_1->size[0] > 2147483646)) {
        i_st.site = &cc_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }
      for (k = b_i; k <= last; k++) {
        a = varargin_1_data[k - 1];
        if (*g < a) {
          *g = a;
        }
      }
    }
  }
  st.site = &pb_emlrtRSI;
  last = xx->size[0];
  i = varargin_1->size[0];
  varargin_1->size[0] = xx->size[0];
  emxEnsureCapacity_real_T(&st, varargin_1, i, &s_emlrtRTEI);
  varargin_1_data = varargin_1->data;
  for (i = 0; i < last; i++) {
    varargin_1_data[i] = -xx_data[i + xx->size[0] * 2] - p_ocp_theta_max;
  }
  b_st.site = &tb_emlrtRSI;
  c_st.site = &ub_emlrtRSI;
  d_st.site = &vb_emlrtRSI;
  if (varargin_1->size[0] < 1) {
    emlrtErrorWithMessageIdR2018a(&d_st, &j_emlrtRTEI,
                                  "Coder:toolbox:eml_min_or_max_varDimZero",
                                  "Coder:toolbox:eml_min_or_max_varDimZero", 0);
  }
  e_st.site = &wb_emlrtRSI;
  f_st.site = &xb_emlrtRSI;
  last = varargin_1->size[0];
  if (varargin_1->size[0] <= 2) {
    if (varargin_1->size[0] == 1) {
      h2 = varargin_1_data[0];
    } else if ((varargin_1_data[0] < varargin_1_data[1]) ||
               (muDoubleScalarIsNaN(varargin_1_data[0]) &&
                (!muDoubleScalarIsNaN(varargin_1_data[1])))) {
      h2 = varargin_1_data[1];
    } else {
      h2 = varargin_1_data[0];
    }
  } else {
    g_st.site = &ac_emlrtRSI;
    if (!muDoubleScalarIsNaN(varargin_1_data[0])) {
      idx = 1;
    } else {
      idx = 0;
      h_st.site = &bc_emlrtRSI;
      if (varargin_1->size[0] > 2147483646) {
        i_st.site = &cc_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= last)) {
        if (!muDoubleScalarIsNaN(varargin_1_data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (idx == 0) {
      h2 = varargin_1_data[0];
    } else {
      g_st.site = &yb_emlrtRSI;
      h2 = varargin_1_data[idx - 1];
      b_i = idx + 1;
      h_st.site = &dc_emlrtRSI;
      if ((idx + 1 <= varargin_1->size[0]) &&
          (varargin_1->size[0] > 2147483646)) {
        i_st.site = &cc_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }
      for (k = b_i; k <= last; k++) {
        a = varargin_1_data[k - 1];
        if (h2 < a) {
          h2 = a;
        }
      }
    }
  }
  st.site = &qb_emlrtRSI;
  last = xx->size[0];
  i = varargin_1->size[0];
  varargin_1->size[0] = xx->size[0];
  emxEnsureCapacity_real_T(&st, varargin_1, i, &t_emlrtRTEI);
  varargin_1_data = varargin_1->data;
  for (i = 0; i < last; i++) {
    varargin_1_data[i] = xx_data[i + xx->size[0] * 3] - p_ocp_thetap_max;
  }
  b_st.site = &tb_emlrtRSI;
  c_st.site = &ub_emlrtRSI;
  d_st.site = &vb_emlrtRSI;
  if (varargin_1->size[0] < 1) {
    emlrtErrorWithMessageIdR2018a(&d_st, &j_emlrtRTEI,
                                  "Coder:toolbox:eml_min_or_max_varDimZero",
                                  "Coder:toolbox:eml_min_or_max_varDimZero", 0);
  }
  e_st.site = &wb_emlrtRSI;
  f_st.site = &xb_emlrtRSI;
  last = varargin_1->size[0];
  if (varargin_1->size[0] <= 2) {
    if (varargin_1->size[0] == 1) {
      h3 = varargin_1_data[0];
    } else if ((varargin_1_data[0] < varargin_1_data[1]) ||
               (muDoubleScalarIsNaN(varargin_1_data[0]) &&
                (!muDoubleScalarIsNaN(varargin_1_data[1])))) {
      h3 = varargin_1_data[1];
    } else {
      h3 = varargin_1_data[0];
    }
  } else {
    g_st.site = &ac_emlrtRSI;
    if (!muDoubleScalarIsNaN(varargin_1_data[0])) {
      idx = 1;
    } else {
      idx = 0;
      h_st.site = &bc_emlrtRSI;
      if (varargin_1->size[0] > 2147483646) {
        i_st.site = &cc_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= last)) {
        if (!muDoubleScalarIsNaN(varargin_1_data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (idx == 0) {
      h3 = varargin_1_data[0];
    } else {
      g_st.site = &yb_emlrtRSI;
      h3 = varargin_1_data[idx - 1];
      b_i = idx + 1;
      h_st.site = &dc_emlrtRSI;
      if ((idx + 1 <= varargin_1->size[0]) &&
          (varargin_1->size[0] > 2147483646)) {
        i_st.site = &cc_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }
      for (k = b_i; k <= last; k++) {
        a = varargin_1_data[k - 1];
        if (h3 < a) {
          h3 = a;
        }
      }
    }
  }
  st.site = &rb_emlrtRSI;
  last = xx->size[0];
  i = varargin_1->size[0];
  varargin_1->size[0] = xx->size[0];
  emxEnsureCapacity_real_T(&st, varargin_1, i, &u_emlrtRTEI);
  varargin_1_data = varargin_1->data;
  for (i = 0; i < last; i++) {
    varargin_1_data[i] = -xx_data[i + xx->size[0] * 3] - p_ocp_thetap_max;
  }
  b_st.site = &tb_emlrtRSI;
  c_st.site = &ub_emlrtRSI;
  d_st.site = &vb_emlrtRSI;
  if (varargin_1->size[0] < 1) {
    emlrtErrorWithMessageIdR2018a(&d_st, &j_emlrtRTEI,
                                  "Coder:toolbox:eml_min_or_max_varDimZero",
                                  "Coder:toolbox:eml_min_or_max_varDimZero", 0);
  }
  e_st.site = &wb_emlrtRSI;
  f_st.site = &xb_emlrtRSI;
  last = varargin_1->size[0];
  if (varargin_1->size[0] <= 2) {
    if (varargin_1->size[0] == 1) {
      h4 = varargin_1_data[0];
    } else if ((varargin_1_data[0] < varargin_1_data[1]) ||
               (muDoubleScalarIsNaN(varargin_1_data[0]) &&
                (!muDoubleScalarIsNaN(varargin_1_data[1])))) {
      h4 = varargin_1_data[1];
    } else {
      h4 = varargin_1_data[0];
    }
  } else {
    g_st.site = &ac_emlrtRSI;
    if (!muDoubleScalarIsNaN(varargin_1_data[0])) {
      idx = 1;
    } else {
      idx = 0;
      h_st.site = &bc_emlrtRSI;
      if (varargin_1->size[0] > 2147483646) {
        i_st.site = &cc_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= last)) {
        if (!muDoubleScalarIsNaN(varargin_1_data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (idx == 0) {
      h4 = varargin_1_data[0];
    } else {
      g_st.site = &yb_emlrtRSI;
      h4 = varargin_1_data[idx - 1];
      b_i = idx + 1;
      h_st.site = &dc_emlrtRSI;
      if ((idx + 1 <= varargin_1->size[0]) &&
          (varargin_1->size[0] > 2147483646)) {
        i_st.site = &cc_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }
      for (k = b_i; k <= last; k++) {
        a = varargin_1_data[k - 1];
        if (h4 < a) {
          h4 = a;
        }
      }
    }
  }
  emxFree_real_T(&f_st, &varargin_1);
  st.site = &sb_emlrtRSI;
  xd[0] = *g;
  xd[1] = h2;
  xd[2] = h3;
  xd[3] = h4;
  b_st.site = &tb_emlrtRSI;
  c_st.site = &ub_emlrtRSI;
  d_st.site = &vb_emlrtRSI;
  e_st.site = &wb_emlrtRSI;
  f_st.site = &xb_emlrtRSI;
  if (!muDoubleScalarIsNaN(*g)) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 5)) {
      if (!muDoubleScalarIsNaN(xd[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (idx != 0) {
    g_st.site = &yb_emlrtRSI;
    *g = xd[idx - 1];
    i = idx + 1;
    h_st.site = &dc_emlrtRSI;
    for (k = i; k < 5; k++) {
      a = xd[k - 1];
      if (*g < a) {
        *g = a;
      }
    }
  }
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtCTX)sp);
}

/* End of code generation (user_ocp.c) */
