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
#include "BBS.h"
#include "BBS_data.h"
#include "BBS_emxutil.h"
#include "eml_int_forloop_overflow_check.h"
#include "indexShapeCheck.h"
#include "mwmathutil.h"
#include "repmat.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtRTEInfo emlrtRTEI = { 24,  /* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtRTEInfo b_emlrtRTEI = { 25,/* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtRSInfo ab_emlrtRSI = { 8,  /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pathName */
};

static emlrtRSInfo bb_emlrtRSI = { 33, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pathName */
};

static emlrtRSInfo cb_emlrtRSI = { 34, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pathName */
};

static emlrtRSInfo db_emlrtRSI = { 36, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pathName */
};

static emlrtRSInfo eb_emlrtRSI = { 37, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pathName */
};

static emlrtRSInfo fb_emlrtRSI = { 38, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pathName */
};

static emlrtRSInfo gb_emlrtRSI = { 39, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pathName */
};

static emlrtRSInfo hb_emlrtRSI = { 41, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pathName */
};

static emlrtRSInfo ib_emlrtRSI = { 42, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pathName */
};

static emlrtRSInfo jb_emlrtRSI = { 53, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pathName */
};

static emlrtRSInfo kb_emlrtRSI = { 60, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pathName */
};

static emlrtRSInfo lb_emlrtRSI = { 61, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pathName */
};

static emlrtRSInfo mb_emlrtRSI = { 62, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pathName */
};

static emlrtRSInfo nb_emlrtRSI = { 63, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pathName */
};

static emlrtRSInfo ob_emlrtRSI = { 64, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pathName */
};

static emlrtRSInfo pb_emlrtRSI = { 65, /* lineNo */
  "user_ocp",                          /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pathName */
};

static emlrtRSInfo tb_emlrtRSI = { 14, /* lineNo */
  "max",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\datafun\\max.m"/* pathName */
};

static emlrtRSInfo ub_emlrtRSI = { 44, /* lineNo */
  "minOrMax",                          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax.m"/* pathName */
};

static emlrtRSInfo vb_emlrtRSI = { 79, /* lineNo */
  "maximum",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax.m"/* pathName */
};

static emlrtRSInfo wb_emlrtRSI = { 145,/* lineNo */
  "unaryMinOrMax",                     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo xb_emlrtRSI = { 1019,/* lineNo */
  "maxRealVectorOmitNaN",              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo yb_emlrtRSI = { 932,/* lineNo */
  "minOrMaxRealVector",                /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo ac_emlrtRSI = { 992,/* lineNo */
  "minOrMaxRealVectorKernel",          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo bc_emlrtRSI = { 924,/* lineNo */
  "minOrMaxRealVector",                /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo cc_emlrtRSI = { 975,/* lineNo */
  "findFirst",                         /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo dc_emlrtRSI = { 48, /* lineNo */
  "eml_mtimes_helper",                 /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_helper.m"/* pathName */
};

static emlrtRTEInfo j_emlrtRTEI = { 95,/* lineNo */
  27,                                  /* colNo */
  "unaryMinOrMax",                     /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pName */
};

static emlrtBCInfo l_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  53,                                  /* lineNo */
  57,                                  /* colNo */
  "uu",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo m_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  53,                                  /* lineNo */
  36,                                  /* colNo */
  "uu",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo n_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  49,                                  /* lineNo */
  12,                                  /* colNo */
  "xx",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo j_emlrtECI = { -1,  /* nDims */
  42,                                  /* lineNo */
  23,                                  /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtECInfo k_emlrtECI = { -1,  /* nDims */
  41,                                  /* lineNo */
  23,                                  /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtECInfo l_emlrtECI = { -1,  /* nDims */
  31,                                  /* lineNo */
  14,                                  /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtBCInfo o_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  31,                                  /* lineNo */
  29,                                  /* colNo */
  "uu",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo p_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  31,                                  /* lineNo */
  17,                                  /* colNo */
  "uu",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo m_emlrtECI = { -1,  /* nDims */
  28,                                  /* lineNo */
  14,                                  /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtBCInfo q_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  28,                                  /* lineNo */
  17,                                  /* colNo */
  "uu",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo l_emlrtRTEI = { 26,/* lineNo */
  7,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtDCInfo h_emlrtDCI = { 16,  /* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo i_emlrtDCI = { 16,  /* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  4                                    /* checkKind */
};

static emlrtDCInfo j_emlrtDCI = { 17,  /* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo k_emlrtDCI = { 19,  /* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo l_emlrtDCI = { 20,  /* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo m_emlrtDCI = { 21,  /* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo n_emlrtDCI = { 22,  /* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo r_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  36,                                  /* lineNo */
  5,                                   /* colNo */
  "hff1_du",                           /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo s_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  37,                                  /* lineNo */
  5,                                   /* colNo */
  "hff2_du",                           /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo t_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  38,                                  /* lineNo */
  5,                                   /* colNo */
  "haa1_du",                           /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo u_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  39,                                  /* lineNo */
  5,                                   /* colNo */
  "haa2_du",                           /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo v_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  41,                                  /* lineNo */
  5,                                   /* colNo */
  "h1_du",                             /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo w_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  42,                                  /* lineNo */
  5,                                   /* colNo */
  "h2_du",                             /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo x_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  33,                                  /* lineNo */
  13,                                  /* colNo */
  "du",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo y_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  33,                                  /* lineNo */
  22,                                  /* colNo */
  "du",                                /* aName */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ab_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  34,                                  /* lineNo */
  22,                                  /* colNo */
  "du",                                /* aName */
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

static emlrtRTEInfo eb_emlrtRTEI = { 16,/* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtRTEInfo fb_emlrtRTEI = { 17,/* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtRTEInfo gb_emlrtRTEI = { 19,/* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtRTEInfo hb_emlrtRTEI = { 20,/* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtRTEInfo ib_emlrtRTEI = { 21,/* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtRTEInfo jb_emlrtRTEI = { 22,/* lineNo */
  1,                                   /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtRTEInfo kb_emlrtRTEI = { 145,/* lineNo */
  38,                                  /* colNo */
  "unaryMinOrMax",                     /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pName */
};

static emlrtRTEInfo lb_emlrtRTEI = { 41,/* lineNo */
  23,                                  /* colNo */
  "user_ocp",                          /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship\\user_ocp.m"/* pName */
};

static emlrtRTEInfo mb_emlrtRTEI = { 42,/* lineNo */
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
static const mxArray *b_emlrt_marshallOut(const emxArray_real_T *u);
static const mxArray *c_emlrt_marshallOut(const emxArray_real_T *u);
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

static const mxArray *b_emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *y;
  int32_T iv[2];
  const mxArray *m;
  real_T *pData;
  int32_T i;
  int32_T b_i;
  int32_T c_i;
  y = NULL;
  iv[0] = u->size[0];
  iv[1] = u->size[1];
  m = emlrtCreateNumericArray(2, &iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (b_i = 0; b_i < u->size[1]; b_i++) {
    for (c_i = 0; c_i < u->size[0]; c_i++) {
      pData[i] = u->data[c_i + u->size[0] * b_i];
      i++;
    }
  }

  emlrtAssign(&y, m);
  return y;
}

static const mxArray *c_emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *y;
  int32_T iv[2];
  const mxArray *m;
  real_T *pData;
  int32_T i;
  int32_T b_i;
  int32_T c_i;
  y = NULL;
  iv[0] = u->size[0];
  iv[1] = u->size[1];
  m = emlrtCreateNumericArray(2, &iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (b_i = 0; b_i < 12; b_i++) {
    for (c_i = 0; c_i < u->size[0]; c_i++) {
      pData[i] = u->data[c_i + u->size[0] * b_i];
      i++;
    }
  }

  emlrtAssign(&y, m);
  return y;
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
              emxArray_real_T *uu, real_T p_ode_Nship, const real_T p_ode_u0[12],
              real_T p_uparam_Np, const real_T p_ocp_Q[144], const real_T
              p_ocp_R[144], const real_T p_ocp_xd_his[240], real_T p_ocp_dF_max,
              real_T p_ocp_da_max, real_T *J, real_T *g)
{
  emxArray_real_T *du_max;
  real_T duff[6];
  int32_T i;
  real_T dff_max[6];
  real_T daa_max[6];
  int32_T n;
  emxArray_real_T *h1_du;
  int32_T b_i;
  int32_T loop_ub;
  emxArray_real_T *h2_du;
  emxArray_real_T *hff1_du;
  emxArray_real_T *hff2_du;
  emxArray_real_T *haa1_du;
  emxArray_real_T *haa2_du;
  emxArray_real_T *x;
  uint32_T u;
  int32_T idx;
  int32_T du_size_idx_0;
  real_T du_data[240];
  real_T tmp_data[240];
  real_T h3;
  int32_T k;
  boolean_T exitg1;
  real_T d;
  real_T duaa[6];
  real_T h4;
  real_T varargin_1[6];
  real_T h6;
  real_T e[12];
  int32_T uu_size[2];
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

  /* ------------------------------------------------------------------------------- */
  /*  pdf_mpc package: Example 1 - Definition of the user_ocp map */
  /* ------------------------------------------------------------------------------- */
  *J = 0.0;
  duff[0] = p_ocp_dF_max;
  duff[1] = p_ocp_dF_max;
  duff[2] = p_ocp_dF_max;
  duff[3] = p_ocp_da_max;
  duff[4] = p_ocp_da_max;
  duff[5] = p_ocp_da_max;
  st.site = &ab_emlrtRSI;
  repmat(&st, duff, p_ode_Nship, du_max);

  /*  du_max = repmat([100, 100, 100, 20/180*pi, 20/180*pi, 20/180*pi]', p_ode.Nship, 1); */
  /*  xd=[p_ocp.rd;0;0;0]; */
  for (i = 0; i < 6; i++) {
    dff_max[i] = p_ocp_dF_max;
    daa_max[i] = p_ocp_da_max;
  }

  if (!(p_uparam_Np >= 0.0)) {
    emlrtNonNegativeCheckR2012b(p_uparam_Np, &i_emlrtDCI, sp);
  }

  n = (int32_T)muDoubleScalarFloor(p_uparam_Np);
  if (p_uparam_Np != n) {
    emlrtIntegerCheckR2012b(p_uparam_Np, &h_emlrtDCI, sp);
  }

  emxInit_real_T(sp, &h1_du, 1, &eb_emlrtRTEI, true);
  b_i = h1_du->size[0];
  loop_ub = (int32_T)p_uparam_Np;
  h1_du->size[0] = loop_ub;
  emxEnsureCapacity_real_T(sp, h1_du, b_i, &eb_emlrtRTEI);
  if (p_uparam_Np != n) {
    emlrtIntegerCheckR2012b(p_uparam_Np, &h_emlrtDCI, sp);
  }

  for (b_i = 0; b_i < loop_ub; b_i++) {
    h1_du->data[b_i] = 0.0;
  }

  if (p_uparam_Np != n) {
    emlrtIntegerCheckR2012b(p_uparam_Np, &j_emlrtDCI, sp);
  }

  emxInit_real_T(sp, &h2_du, 1, &fb_emlrtRTEI, true);
  loop_ub = (int32_T)p_uparam_Np;
  b_i = h2_du->size[0];
  h2_du->size[0] = (int32_T)p_uparam_Np;
  emxEnsureCapacity_real_T(sp, h2_du, b_i, &fb_emlrtRTEI);
  if ((int32_T)p_uparam_Np != n) {
    emlrtIntegerCheckR2012b(p_uparam_Np, &j_emlrtDCI, sp);
  }

  for (b_i = 0; b_i < loop_ub; b_i++) {
    h2_du->data[b_i] = 0.0;
  }

  emxInit_real_T(sp, &hff1_du, 1, &gb_emlrtRTEI, true);
  if ((int32_T)p_uparam_Np != n) {
    emlrtIntegerCheckR2012b(p_uparam_Np, &k_emlrtDCI, sp);
  }

  b_i = hff1_du->size[0];
  hff1_du->size[0] = (int32_T)p_uparam_Np;
  emxEnsureCapacity_real_T(sp, hff1_du, b_i, &gb_emlrtRTEI);
  if ((int32_T)p_uparam_Np != n) {
    emlrtIntegerCheckR2012b(p_uparam_Np, &k_emlrtDCI, sp);
  }

  for (b_i = 0; b_i < loop_ub; b_i++) {
    hff1_du->data[b_i] = 0.0;
  }

  emxInit_real_T(sp, &hff2_du, 1, &hb_emlrtRTEI, true);
  if ((int32_T)p_uparam_Np != n) {
    emlrtIntegerCheckR2012b(p_uparam_Np, &l_emlrtDCI, sp);
  }

  b_i = hff2_du->size[0];
  hff2_du->size[0] = (int32_T)p_uparam_Np;
  emxEnsureCapacity_real_T(sp, hff2_du, b_i, &hb_emlrtRTEI);
  if ((int32_T)p_uparam_Np != n) {
    emlrtIntegerCheckR2012b(p_uparam_Np, &l_emlrtDCI, sp);
  }

  for (b_i = 0; b_i < loop_ub; b_i++) {
    hff2_du->data[b_i] = 0.0;
  }

  emxInit_real_T(sp, &haa1_du, 1, &ib_emlrtRTEI, true);
  if ((int32_T)p_uparam_Np != n) {
    emlrtIntegerCheckR2012b(p_uparam_Np, &m_emlrtDCI, sp);
  }

  b_i = haa1_du->size[0];
  haa1_du->size[0] = (int32_T)p_uparam_Np;
  emxEnsureCapacity_real_T(sp, haa1_du, b_i, &ib_emlrtRTEI);
  if ((int32_T)p_uparam_Np != n) {
    emlrtIntegerCheckR2012b(p_uparam_Np, &m_emlrtDCI, sp);
  }

  for (b_i = 0; b_i < loop_ub; b_i++) {
    haa1_du->data[b_i] = 0.0;
  }

  emxInit_real_T(sp, &haa2_du, 1, &jb_emlrtRTEI, true);
  if ((int32_T)p_uparam_Np != n) {
    emlrtIntegerCheckR2012b(p_uparam_Np, &n_emlrtDCI, sp);
  }

  b_i = haa2_du->size[0];
  haa2_du->size[0] = (int32_T)p_uparam_Np;
  emxEnsureCapacity_real_T(sp, haa2_du, b_i, &jb_emlrtRTEI);
  if ((int32_T)p_uparam_Np != n) {
    emlrtIntegerCheckR2012b(p_uparam_Np, &n_emlrtDCI, sp);
  }

  for (n = 0; n < loop_ub; n++) {
    haa2_du->data[n] = 0.0;
  }

  emlrtDisplayR2012b(b_emlrt_marshallOut(uu), "uu", &emlrtRTEI, sp);
  emlrtDisplayR2012b(c_emlrt_marshallOut(xx), "xx", &b_emlrtRTEI, sp);
  emlrtForLoopVectorCheckR2012b(1.0, 1.0, p_uparam_Np, mxDOUBLE_CLASS, (int32_T)
    p_uparam_Np, &l_emlrtRTEI, sp);
  emxInit_real_T(sp, &x, 1, &kb_emlrtRTEI, true);
  for (i = 0; i < loop_ub; i++) {
    u = i + 1U;
    if (u == 1U) {
      if (1 > uu->size[0]) {
        emlrtDynamicBoundsCheckR2012b(1, 1, uu->size[0], &q_emlrtBCI, sp);
      }

      idx = uu->size[1];
      du_size_idx_0 = uu->size[1];
      for (n = 0; n < idx; n++) {
        du_data[n] = uu->data[uu->size[0] * n];
      }

      if (du_size_idx_0 != 12) {
        emlrtSizeEqCheck1DR2012b(du_size_idx_0, 12, &m_emlrtECI, sp);
      }

      for (n = 0; n < 12; n++) {
        du_data[n] -= p_ode_u0[n];
      }

      /* 这里没有问题，u0 = u_last, p_ode.u0在这里是实时更新的 */
      /*          du = uu(1, :)' - u_last; */
    } else {
      if ((int32_T)u > uu->size[0]) {
        emlrtDynamicBoundsCheckR2012b((int32_T)u, 1, uu->size[0], &p_emlrtBCI,
          sp);
      }

      idx = uu->size[1];
      du_size_idx_0 = uu->size[1];
      for (n = 0; n < idx; n++) {
        du_data[n] = uu->data[i + uu->size[0] * n];
      }

      if ((i < 1) || (i > uu->size[0])) {
        emlrtDynamicBoundsCheckR2012b(i, 1, uu->size[0], &o_emlrtBCI, sp);
      }

      idx = uu->size[1];
      b_i = uu->size[1];
      for (n = 0; n < idx; n++) {
        tmp_data[n] = uu->data[(i + uu->size[0] * n) - 1];
      }

      if (du_size_idx_0 != b_i) {
        emlrtSizeEqCheck1DR2012b(du_size_idx_0, b_i, &l_emlrtECI, sp);
      }

      for (n = 0; n < du_size_idx_0; n++) {
        du_data[n] -= tmp_data[n];
      }
    }

    st.site = &bb_emlrtRSI;
    indexShapeCheck(&st, du_size_idx_0);
    st.site = &bb_emlrtRSI;
    indexShapeCheck(&st, du_size_idx_0);
    if (1 > du_size_idx_0) {
      emlrtDynamicBoundsCheckR2012b(1, 1, du_size_idx_0, &x_emlrtBCI, sp);
    }

    duff[0] = du_data[0];
    if (2 > du_size_idx_0) {
      emlrtDynamicBoundsCheckR2012b(2, 1, 1, &x_emlrtBCI, sp);
    }

    duff[1] = du_data[1];
    if (3 > du_size_idx_0) {
      emlrtDynamicBoundsCheckR2012b(3, 1, 2, &x_emlrtBCI, sp);
    }

    duff[2] = du_data[2];
    if (7 > du_size_idx_0) {
      emlrtDynamicBoundsCheckR2012b(7, 1, du_size_idx_0, &y_emlrtBCI, sp);
    }

    duff[3] = du_data[6];
    if (8 > du_size_idx_0) {
      emlrtDynamicBoundsCheckR2012b(8, 1, 7, &y_emlrtBCI, sp);
    }

    duff[4] = du_data[7];
    if (9 > du_size_idx_0) {
      emlrtDynamicBoundsCheckR2012b(9, 1, 8, &y_emlrtBCI, sp);
    }

    duff[5] = du_data[8];
    st.site = &cb_emlrtRSI;
    indexShapeCheck(&st, du_size_idx_0);
    st.site = &cb_emlrtRSI;
    indexShapeCheck(&st, du_size_idx_0);
    duaa[0] = du_data[3];
    duaa[1] = du_data[4];
    duaa[2] = du_data[5];
    if (10 > du_size_idx_0) {
      emlrtDynamicBoundsCheckR2012b(10, 1, 9, &ab_emlrtBCI, sp);
    }

    duaa[3] = du_data[9];
    if (11 > du_size_idx_0) {
      emlrtDynamicBoundsCheckR2012b(11, 1, 10, &ab_emlrtBCI, sp);
    }

    duaa[4] = du_data[10];
    if (12 > du_size_idx_0) {
      emlrtDynamicBoundsCheckR2012b(12, 1, 11, &ab_emlrtBCI, sp);
    }

    duaa[5] = du_data[11];
    st.site = &db_emlrtRSI;
    for (b_i = 0; b_i < 6; b_i++) {
      varargin_1[b_i] = duff[b_i] - dff_max[b_i];
    }

    b_st.site = &tb_emlrtRSI;
    c_st.site = &ub_emlrtRSI;
    d_st.site = &vb_emlrtRSI;
    e_st.site = &wb_emlrtRSI;
    n = x->size[0];
    x->size[0] = 6;
    emxEnsureCapacity_real_T(&e_st, x, n, &kb_emlrtRTEI);
    for (n = 0; n < 6; n++) {
      x->data[n] = duff[n] - dff_max[n];
    }

    f_st.site = &xb_emlrtRSI;
    g_st.site = &bc_emlrtRSI;
    if (!muDoubleScalarIsNaN(x->data[0])) {
      idx = 1;
    } else {
      idx = 0;
      h_st.site = &cc_emlrtRSI;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= 6)) {
        if (!muDoubleScalarIsNaN(x->data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      n = (int32_T)(i + 1U);
      if (n > hff1_du->size[0]) {
        emlrtDynamicBoundsCheckR2012b(n, 1, hff1_du->size[0], &r_emlrtBCI, &f_st);
      }

      hff1_du->data[n - 1] = varargin_1[0];
    } else {
      g_st.site = &yb_emlrtRSI;
      h3 = varargin_1[idx - 1];
      b_i = idx + 1;
      h_st.site = &ac_emlrtRSI;
      for (k = b_i; k < 7; k++) {
        d = varargin_1[k - 1];
        if (h3 < d) {
          h3 = d;
        }
      }

      n = (int32_T)(i + 1U);
      if (n > hff1_du->size[0]) {
        emlrtDynamicBoundsCheckR2012b(n, 1, hff1_du->size[0], &r_emlrtBCI, &f_st);
      }

      hff1_du->data[n - 1] = h3;
    }

    st.site = &eb_emlrtRSI;
    for (b_i = 0; b_i < 6; b_i++) {
      varargin_1[b_i] = -duff[b_i] - dff_max[b_i];
    }

    b_st.site = &tb_emlrtRSI;
    c_st.site = &ub_emlrtRSI;
    d_st.site = &vb_emlrtRSI;
    e_st.site = &wb_emlrtRSI;
    n = x->size[0];
    x->size[0] = 6;
    emxEnsureCapacity_real_T(&e_st, x, n, &kb_emlrtRTEI);
    for (n = 0; n < 6; n++) {
      x->data[n] = -duff[n] - dff_max[n];
    }

    f_st.site = &xb_emlrtRSI;
    g_st.site = &bc_emlrtRSI;
    if (!muDoubleScalarIsNaN(x->data[0])) {
      idx = 1;
    } else {
      idx = 0;
      h_st.site = &cc_emlrtRSI;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= 6)) {
        if (!muDoubleScalarIsNaN(x->data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      n = (int32_T)(i + 1U);
      if (n > hff2_du->size[0]) {
        emlrtDynamicBoundsCheckR2012b(n, 1, hff2_du->size[0], &s_emlrtBCI, &f_st);
      }

      hff2_du->data[n - 1] = varargin_1[0];
    } else {
      g_st.site = &yb_emlrtRSI;
      h3 = varargin_1[idx - 1];
      b_i = idx + 1;
      h_st.site = &ac_emlrtRSI;
      for (k = b_i; k < 7; k++) {
        d = varargin_1[k - 1];
        if (h3 < d) {
          h3 = d;
        }
      }

      n = (int32_T)(i + 1U);
      if (n > hff2_du->size[0]) {
        emlrtDynamicBoundsCheckR2012b(n, 1, hff2_du->size[0], &s_emlrtBCI, &f_st);
      }

      hff2_du->data[n - 1] = h3;
    }

    st.site = &fb_emlrtRSI;
    for (b_i = 0; b_i < 6; b_i++) {
      varargin_1[b_i] = duaa[b_i] - daa_max[b_i];
    }

    b_st.site = &tb_emlrtRSI;
    c_st.site = &ub_emlrtRSI;
    d_st.site = &vb_emlrtRSI;
    e_st.site = &wb_emlrtRSI;
    n = x->size[0];
    x->size[0] = 6;
    emxEnsureCapacity_real_T(&e_st, x, n, &kb_emlrtRTEI);
    for (n = 0; n < 6; n++) {
      x->data[n] = duaa[n] - daa_max[n];
    }

    f_st.site = &xb_emlrtRSI;
    g_st.site = &bc_emlrtRSI;
    if (!muDoubleScalarIsNaN(x->data[0])) {
      idx = 1;
    } else {
      idx = 0;
      h_st.site = &cc_emlrtRSI;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= 6)) {
        if (!muDoubleScalarIsNaN(x->data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      n = (int32_T)(i + 1U);
      if (n > haa1_du->size[0]) {
        emlrtDynamicBoundsCheckR2012b(n, 1, haa1_du->size[0], &t_emlrtBCI, &f_st);
      }

      haa1_du->data[n - 1] = varargin_1[0];
    } else {
      g_st.site = &yb_emlrtRSI;
      h3 = varargin_1[idx - 1];
      b_i = idx + 1;
      h_st.site = &ac_emlrtRSI;
      for (k = b_i; k < 7; k++) {
        d = varargin_1[k - 1];
        if (h3 < d) {
          h3 = d;
        }
      }

      n = (int32_T)(i + 1U);
      if (n > haa1_du->size[0]) {
        emlrtDynamicBoundsCheckR2012b(n, 1, haa1_du->size[0], &t_emlrtBCI, &f_st);
      }

      haa1_du->data[n - 1] = h3;
    }

    st.site = &gb_emlrtRSI;
    for (b_i = 0; b_i < 6; b_i++) {
      varargin_1[b_i] = -duaa[b_i] - daa_max[b_i];
    }

    b_st.site = &tb_emlrtRSI;
    c_st.site = &ub_emlrtRSI;
    d_st.site = &vb_emlrtRSI;
    e_st.site = &wb_emlrtRSI;
    n = x->size[0];
    x->size[0] = 6;
    emxEnsureCapacity_real_T(&e_st, x, n, &kb_emlrtRTEI);
    for (n = 0; n < 6; n++) {
      x->data[n] = -duaa[n] - daa_max[n];
    }

    f_st.site = &xb_emlrtRSI;
    g_st.site = &bc_emlrtRSI;
    if (!muDoubleScalarIsNaN(x->data[0])) {
      idx = 1;
    } else {
      idx = 0;
      h_st.site = &cc_emlrtRSI;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= 6)) {
        if (!muDoubleScalarIsNaN(x->data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      n = (int32_T)(i + 1U);
      if (n > haa2_du->size[0]) {
        emlrtDynamicBoundsCheckR2012b(n, 1, haa2_du->size[0], &u_emlrtBCI, &f_st);
      }

      haa2_du->data[n - 1] = varargin_1[0];
    } else {
      g_st.site = &yb_emlrtRSI;
      h3 = varargin_1[idx - 1];
      b_i = idx + 1;
      h_st.site = &ac_emlrtRSI;
      for (k = b_i; k < 7; k++) {
        d = varargin_1[k - 1];
        if (h3 < d) {
          h3 = d;
        }
      }

      n = (int32_T)(i + 1U);
      if (n > haa2_du->size[0]) {
        emlrtDynamicBoundsCheckR2012b(n, 1, haa2_du->size[0], &u_emlrtBCI, &f_st);
      }

      haa2_du->data[n - 1] = h3;
    }

    if (du_size_idx_0 != du_max->size[0]) {
      emlrtSizeEqCheck1DR2012b(du_size_idx_0, du_max->size[0], &k_emlrtECI, sp);
    }

    st.site = &hb_emlrtRSI;
    n = x->size[0];
    x->size[0] = du_size_idx_0;
    emxEnsureCapacity_real_T(&st, x, n, &lb_emlrtRTEI);
    for (n = 0; n < du_size_idx_0; n++) {
      x->data[n] = du_data[n] - du_max->data[n];
    }

    b_st.site = &tb_emlrtRSI;
    c_st.site = &ub_emlrtRSI;
    d_st.site = &vb_emlrtRSI;
    if (x->size[0] < 1) {
      emlrtErrorWithMessageIdR2018a(&d_st, &j_emlrtRTEI,
        "Coder:toolbox:eml_min_or_max_varDimZero",
        "Coder:toolbox:eml_min_or_max_varDimZero", 0);
    }

    e_st.site = &wb_emlrtRSI;
    f_st.site = &xb_emlrtRSI;
    n = x->size[0];
    if (x->size[0] <= 2) {
      if (x->size[0] == 1) {
        n = (int32_T)(i + 1U);
        if (n > h1_du->size[0]) {
          emlrtDynamicBoundsCheckR2012b(n, 1, h1_du->size[0], &v_emlrtBCI, &f_st);
        }

        h1_du->data[n - 1] = x->data[0];
      } else if ((x->data[0] < x->data[1]) || (muDoubleScalarIsNaN(x->data[0]) &&
                  (!muDoubleScalarIsNaN(x->data[1])))) {
        n = (int32_T)(i + 1U);
        if (n > h1_du->size[0]) {
          emlrtDynamicBoundsCheckR2012b(n, 1, h1_du->size[0], &v_emlrtBCI, &f_st);
        }

        h1_du->data[n - 1] = x->data[1];
      } else {
        n = (int32_T)(i + 1U);
        if (n > h1_du->size[0]) {
          emlrtDynamicBoundsCheckR2012b(n, 1, h1_du->size[0], &v_emlrtBCI, &f_st);
        }

        h1_du->data[n - 1] = x->data[0];
      }
    } else {
      g_st.site = &bc_emlrtRSI;
      if (!muDoubleScalarIsNaN(x->data[0])) {
        idx = 1;
      } else {
        idx = 0;
        h_st.site = &cc_emlrtRSI;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k <= x->size[0])) {
          if (!muDoubleScalarIsNaN(x->data[k - 1])) {
            idx = k;
            exitg1 = true;
          } else {
            k++;
          }
        }
      }

      if (idx == 0) {
        n = (int32_T)(i + 1U);
        if (n > h1_du->size[0]) {
          emlrtDynamicBoundsCheckR2012b(n, 1, h1_du->size[0], &v_emlrtBCI, &f_st);
        }

        h1_du->data[n - 1] = x->data[0];
      } else {
        g_st.site = &yb_emlrtRSI;
        h3 = x->data[idx - 1];
        b_i = idx + 1;
        h_st.site = &ac_emlrtRSI;
        for (k = b_i; k <= n; k++) {
          d = x->data[k - 1];
          if (h3 < d) {
            h3 = d;
          }
        }

        n = (int32_T)(i + 1U);
        if (n > h1_du->size[0]) {
          emlrtDynamicBoundsCheckR2012b(n, 1, h1_du->size[0], &v_emlrtBCI, &f_st);
        }

        h1_du->data[n - 1] = h3;
      }
    }

    for (n = 0; n < du_size_idx_0; n++) {
      du_data[n] = -du_data[n];
    }

    if (du_size_idx_0 != du_max->size[0]) {
      emlrtSizeEqCheck1DR2012b(du_size_idx_0, du_max->size[0], &j_emlrtECI, sp);
    }

    st.site = &ib_emlrtRSI;
    n = x->size[0];
    x->size[0] = du_size_idx_0;
    emxEnsureCapacity_real_T(&st, x, n, &mb_emlrtRTEI);
    for (n = 0; n < du_size_idx_0; n++) {
      x->data[n] = du_data[n] - du_max->data[n];
    }

    b_st.site = &tb_emlrtRSI;
    c_st.site = &ub_emlrtRSI;
    d_st.site = &vb_emlrtRSI;
    if (x->size[0] < 1) {
      emlrtErrorWithMessageIdR2018a(&d_st, &j_emlrtRTEI,
        "Coder:toolbox:eml_min_or_max_varDimZero",
        "Coder:toolbox:eml_min_or_max_varDimZero", 0);
    }

    e_st.site = &wb_emlrtRSI;
    f_st.site = &xb_emlrtRSI;
    n = x->size[0];
    if (x->size[0] <= 2) {
      if (x->size[0] == 1) {
        n = (int32_T)(i + 1U);
        if (n > h2_du->size[0]) {
          emlrtDynamicBoundsCheckR2012b(n, 1, h2_du->size[0], &w_emlrtBCI, &f_st);
        }

        h2_du->data[n - 1] = x->data[0];
      } else if ((x->data[0] < x->data[1]) || (muDoubleScalarIsNaN(x->data[0]) &&
                  (!muDoubleScalarIsNaN(x->data[1])))) {
        n = (int32_T)(i + 1U);
        if (n > h2_du->size[0]) {
          emlrtDynamicBoundsCheckR2012b(n, 1, h2_du->size[0], &w_emlrtBCI, &f_st);
        }

        h2_du->data[n - 1] = x->data[1];
      } else {
        n = (int32_T)(i + 1U);
        if (n > h2_du->size[0]) {
          emlrtDynamicBoundsCheckR2012b(n, 1, h2_du->size[0], &w_emlrtBCI, &f_st);
        }

        h2_du->data[n - 1] = x->data[0];
      }
    } else {
      g_st.site = &bc_emlrtRSI;
      if (!muDoubleScalarIsNaN(x->data[0])) {
        idx = 1;
      } else {
        idx = 0;
        h_st.site = &cc_emlrtRSI;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k <= x->size[0])) {
          if (!muDoubleScalarIsNaN(x->data[k - 1])) {
            idx = k;
            exitg1 = true;
          } else {
            k++;
          }
        }
      }

      if (idx == 0) {
        n = (int32_T)(i + 1U);
        if (n > h2_du->size[0]) {
          emlrtDynamicBoundsCheckR2012b(n, 1, h2_du->size[0], &w_emlrtBCI, &f_st);
        }

        h2_du->data[n - 1] = x->data[0];
      } else {
        g_st.site = &yb_emlrtRSI;
        h3 = x->data[idx - 1];
        b_i = idx + 1;
        h_st.site = &ac_emlrtRSI;
        for (k = b_i; k <= n; k++) {
          d = x->data[k - 1];
          if (h3 < d) {
            h3 = d;
          }
        }

        n = (int32_T)(i + 1U);
        if (n > h2_du->size[0]) {
          emlrtDynamicBoundsCheckR2012b(n, 1, h2_du->size[0], &w_emlrtBCI, &f_st);
        }

        h2_du->data[n - 1] = h3;
      }
    }

    /*      e = xx(i+1,:)' - xd; */
    /* xd_his(1, :)代表传进来一个点，xd_his(i, :)代表传进来一条轨迹 */
    /*  编队好像传进去一个点稳定性更高一点 */
    n = (int32_T)(i + 2U);
    if ((n < 1) || (n > xx->size[0])) {
      emlrtDynamicBoundsCheckR2012b(n, 1, xx->size[0], &n_emlrtBCI, sp);
    }

    for (n = 0; n < 12; n++) {
      e[n] = xx->data[(i + xx->size[0] * n) + 1] - p_ocp_xd_his[20 * n];
    }

    /*      J = J + (e' * p_ocp.Q * e + uu(i, :) * p_ocp.R * uu(i, :)' + du' * p_ocp.Rdu * du); */
    n = i + 1;
    if (n > uu->size[0]) {
      emlrtDynamicBoundsCheckR2012b(n, 1, uu->size[0], &l_emlrtBCI, sp);
    }

    st.site = &jb_emlrtRSI;
    n = i + 1;
    if (n > uu->size[0]) {
      emlrtDynamicBoundsCheckR2012b(n, 1, uu->size[0], &m_emlrtBCI, &st);
    }

    uu_size[0] = 1;
    uu_size[1] = uu->size[1];
    b_st.site = &dc_emlrtRSI;
    dynamic_size_checks(&b_st, uu_size, uu->size[1]);
    st.site = &jb_emlrtRSI;
    uu_size[0] = 1;
    uu_size[1] = uu->size[1];
    b_st.site = &dc_emlrtRSI;
    b_dynamic_size_checks(&b_st, uu_size, uu->size[1]);
    h3 = 0.0;
    for (n = 0; n < 12; n++) {
      d = 0.0;
      for (b_i = 0; b_i < 12; b_i++) {
        d += e[b_i] * p_ocp_Q[b_i + 12 * n];
      }

      h3 += d * e[n];
    }

    idx = uu->size[1];
    for (n = 0; n < idx; n++) {
      d = uu->data[i + uu->size[0] * n];
      tmp_data[n] = d;
      du_data[n] = d;
    }

    h4 = 0.0;
    for (n = 0; n < 12; n++) {
      d = 0.0;
      for (b_i = 0; b_i < 12; b_i++) {
        d += tmp_data[b_i] * p_ocp_R[b_i + 12 * n];
      }

      h4 += d * du_data[n];
    }

    *J += h3 + h4;

    /*      J = J + e' * p_ocp.Q * e ; */
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }

  emxFree_real_T(&h2_du);
  emxFree_real_T(&h1_du);
  emxFree_real_T(&du_max);

  /*  h1 = max(h1_du); */
  /*  h2 = max(h2_du); */
  /*  g = max([h1; h2]); */
  st.site = &kb_emlrtRSI;
  b_st.site = &tb_emlrtRSI;
  c_st.site = &ub_emlrtRSI;
  d_st.site = &vb_emlrtRSI;
  if (hff1_du->size[0] < 1) {
    emlrtErrorWithMessageIdR2018a(&d_st, &j_emlrtRTEI,
      "Coder:toolbox:eml_min_or_max_varDimZero",
      "Coder:toolbox:eml_min_or_max_varDimZero", 0);
  }

  e_st.site = &wb_emlrtRSI;
  f_st.site = &xb_emlrtRSI;
  n = hff1_du->size[0];
  if (hff1_du->size[0] <= 2) {
    if (hff1_du->size[0] == 1) {
      h3 = hff1_du->data[0];
    } else if ((hff1_du->data[0] < hff1_du->data[1]) || (muDoubleScalarIsNaN
                (hff1_du->data[0]) && (!muDoubleScalarIsNaN(hff1_du->data[1]))))
    {
      h3 = hff1_du->data[1];
    } else {
      h3 = hff1_du->data[0];
    }
  } else {
    g_st.site = &bc_emlrtRSI;
    if (!muDoubleScalarIsNaN(hff1_du->data[0])) {
      idx = 1;
    } else {
      idx = 0;
      h_st.site = &cc_emlrtRSI;
      if (hff1_du->size[0] > 2147483646) {
        i_st.site = &sb_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }

      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= hff1_du->size[0])) {
        if (!muDoubleScalarIsNaN(hff1_du->data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      h3 = hff1_du->data[0];
    } else {
      g_st.site = &yb_emlrtRSI;
      h3 = hff1_du->data[idx - 1];
      b_i = idx + 1;
      h_st.site = &ac_emlrtRSI;
      if ((idx + 1 <= hff1_du->size[0]) && (hff1_du->size[0] > 2147483646)) {
        i_st.site = &sb_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }

      for (k = b_i; k <= n; k++) {
        d = hff1_du->data[k - 1];
        if (h3 < d) {
          h3 = d;
        }
      }
    }
  }

  emxFree_real_T(&hff1_du);
  st.site = &lb_emlrtRSI;
  b_st.site = &tb_emlrtRSI;
  c_st.site = &ub_emlrtRSI;
  d_st.site = &vb_emlrtRSI;
  if (hff2_du->size[0] < 1) {
    emlrtErrorWithMessageIdR2018a(&d_st, &j_emlrtRTEI,
      "Coder:toolbox:eml_min_or_max_varDimZero",
      "Coder:toolbox:eml_min_or_max_varDimZero", 0);
  }

  e_st.site = &wb_emlrtRSI;
  f_st.site = &xb_emlrtRSI;
  n = hff2_du->size[0];
  if (hff2_du->size[0] <= 2) {
    if (hff2_du->size[0] == 1) {
      h4 = hff2_du->data[0];
    } else if ((hff2_du->data[0] < hff2_du->data[1]) || (muDoubleScalarIsNaN
                (hff2_du->data[0]) && (!muDoubleScalarIsNaN(hff2_du->data[1]))))
    {
      h4 = hff2_du->data[1];
    } else {
      h4 = hff2_du->data[0];
    }
  } else {
    g_st.site = &bc_emlrtRSI;
    if (!muDoubleScalarIsNaN(hff2_du->data[0])) {
      idx = 1;
    } else {
      idx = 0;
      h_st.site = &cc_emlrtRSI;
      if (hff2_du->size[0] > 2147483646) {
        i_st.site = &sb_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }

      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= hff2_du->size[0])) {
        if (!muDoubleScalarIsNaN(hff2_du->data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      h4 = hff2_du->data[0];
    } else {
      g_st.site = &yb_emlrtRSI;
      h4 = hff2_du->data[idx - 1];
      b_i = idx + 1;
      h_st.site = &ac_emlrtRSI;
      if ((idx + 1 <= hff2_du->size[0]) && (hff2_du->size[0] > 2147483646)) {
        i_st.site = &sb_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }

      for (k = b_i; k <= n; k++) {
        d = hff2_du->data[k - 1];
        if (h4 < d) {
          h4 = d;
        }
      }
    }
  }

  emxFree_real_T(&hff2_du);
  st.site = &mb_emlrtRSI;
  b_st.site = &tb_emlrtRSI;
  c_st.site = &ub_emlrtRSI;
  d_st.site = &vb_emlrtRSI;
  if (haa1_du->size[0] < 1) {
    emlrtErrorWithMessageIdR2018a(&d_st, &j_emlrtRTEI,
      "Coder:toolbox:eml_min_or_max_varDimZero",
      "Coder:toolbox:eml_min_or_max_varDimZero", 0);
  }

  e_st.site = &wb_emlrtRSI;
  f_st.site = &xb_emlrtRSI;
  n = haa1_du->size[0];
  if (haa1_du->size[0] <= 2) {
    if (haa1_du->size[0] == 1) {
      *g = haa1_du->data[0];
    } else if ((haa1_du->data[0] < haa1_du->data[1]) || (muDoubleScalarIsNaN
                (haa1_du->data[0]) && (!muDoubleScalarIsNaN(haa1_du->data[1]))))
    {
      *g = haa1_du->data[1];
    } else {
      *g = haa1_du->data[0];
    }
  } else {
    g_st.site = &bc_emlrtRSI;
    if (!muDoubleScalarIsNaN(haa1_du->data[0])) {
      idx = 1;
    } else {
      idx = 0;
      h_st.site = &cc_emlrtRSI;
      if (haa1_du->size[0] > 2147483646) {
        i_st.site = &sb_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }

      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= haa1_du->size[0])) {
        if (!muDoubleScalarIsNaN(haa1_du->data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      *g = haa1_du->data[0];
    } else {
      g_st.site = &yb_emlrtRSI;
      *g = haa1_du->data[idx - 1];
      b_i = idx + 1;
      h_st.site = &ac_emlrtRSI;
      if ((idx + 1 <= haa1_du->size[0]) && (haa1_du->size[0] > 2147483646)) {
        i_st.site = &sb_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }

      for (k = b_i; k <= n; k++) {
        d = haa1_du->data[k - 1];
        if (*g < d) {
          *g = d;
        }
      }
    }
  }

  emxFree_real_T(&haa1_du);
  st.site = &nb_emlrtRSI;
  b_st.site = &tb_emlrtRSI;
  c_st.site = &ub_emlrtRSI;
  d_st.site = &vb_emlrtRSI;
  if (haa2_du->size[0] < 1) {
    emlrtErrorWithMessageIdR2018a(&d_st, &j_emlrtRTEI,
      "Coder:toolbox:eml_min_or_max_varDimZero",
      "Coder:toolbox:eml_min_or_max_varDimZero", 0);
  }

  e_st.site = &wb_emlrtRSI;
  f_st.site = &xb_emlrtRSI;
  n = haa2_du->size[0];
  if (haa2_du->size[0] <= 2) {
    if (haa2_du->size[0] == 1) {
      h6 = haa2_du->data[0];
    } else if ((haa2_du->data[0] < haa2_du->data[1]) || (muDoubleScalarIsNaN
                (haa2_du->data[0]) && (!muDoubleScalarIsNaN(haa2_du->data[1]))))
    {
      h6 = haa2_du->data[1];
    } else {
      h6 = haa2_du->data[0];
    }
  } else {
    g_st.site = &bc_emlrtRSI;
    if (!muDoubleScalarIsNaN(haa2_du->data[0])) {
      idx = 1;
    } else {
      idx = 0;
      h_st.site = &cc_emlrtRSI;
      if (haa2_du->size[0] > 2147483646) {
        i_st.site = &sb_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }

      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= haa2_du->size[0])) {
        if (!muDoubleScalarIsNaN(haa2_du->data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      h6 = haa2_du->data[0];
    } else {
      g_st.site = &yb_emlrtRSI;
      h6 = haa2_du->data[idx - 1];
      b_i = idx + 1;
      h_st.site = &ac_emlrtRSI;
      if ((idx + 1 <= haa2_du->size[0]) && (haa2_du->size[0] > 2147483646)) {
        i_st.site = &sb_emlrtRSI;
        check_forloop_overflow_error(&i_st);
      }

      for (k = b_i; k <= n; k++) {
        d = haa2_du->data[k - 1];
        if (h6 < d) {
          h6 = d;
        }
      }
    }
  }

  emxFree_real_T(&haa2_du);
  st.site = &ob_emlrtRSI;
  b_st.site = &tb_emlrtRSI;
  c_st.site = &ub_emlrtRSI;
  d_st.site = &vb_emlrtRSI;
  e_st.site = &wb_emlrtRSI;
  n = x->size[0];
  x->size[0] = 4;
  emxEnsureCapacity_real_T(&e_st, x, n, &kb_emlrtRTEI);
  x->data[0] = h3;
  x->data[1] = h4;
  x->data[2] = *g;
  x->data[3] = h6;
  f_st.site = &xb_emlrtRSI;
  g_st.site = &bc_emlrtRSI;
  if (!muDoubleScalarIsNaN(x->data[0])) {
    idx = 1;
  } else {
    idx = 0;
    h_st.site = &cc_emlrtRSI;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k <= 4)) {
      if (!muDoubleScalarIsNaN(x->data[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  emxFree_real_T(&x);
  if (idx != 0) {
    g_st.site = &yb_emlrtRSI;
    h_st.site = &ac_emlrtRSI;
  }

  st.site = &pb_emlrtRSI;
  b_st.site = &tb_emlrtRSI;
  c_st.site = &ub_emlrtRSI;
  d_st.site = &vb_emlrtRSI;
  e_st.site = &wb_emlrtRSI;
  f_st.site = &xb_emlrtRSI;
  if ((*g < h6) || (muDoubleScalarIsNaN(*g) && (!muDoubleScalarIsNaN(h6)))) {
    *g = h6;
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
