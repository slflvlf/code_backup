/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * user_control_profile.c
 *
 * Code generation for function 'user_control_profile'
 *
 */

/* Include files */
#include "user_control_profile.h"
#include "assertValidSizeArg.h"
#include "blas.h"
#include "rt_nonfinite.h"
#include "update_mv.h"
#include "update_mv_emxutil.h"

/* Variable Definitions */
static emlrtRSInfo y_emlrtRSI = { 8,   /* lineNo */
  "user_control_profile",              /* fcnName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship1\\user_control_profile.m"/* pathName */
};

static emlrtRSInfo ab_emlrtRSI = { 79, /* lineNo */
  "eml_mtimes_helper",                 /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_helper.m"/* pathName */
};

static emlrtRSInfo db_emlrtRSI = { 29, /* lineNo */
  "reshapeSizeChecks",                 /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\reshapeSizeChecks.m"/* pathName */
};

static emlrtRSInfo eb_emlrtRSI = { 109,/* lineNo */
  "computeDimsData",                   /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\reshapeSizeChecks.m"/* pathName */
};

static emlrtRTEInfo c_emlrtRTEI = { 52,/* lineNo */
  13,                                  /* colNo */
  "reshapeSizeChecks",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\reshapeSizeChecks.m"/* pName */
};

static emlrtRTEInfo d_emlrtRTEI = { 57,/* lineNo */
  23,                                  /* colNo */
  "reshapeSizeChecks",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\reshapeSizeChecks.m"/* pName */
};

static emlrtRTEInfo e_emlrtRTEI = { 59,/* lineNo */
  23,                                  /* colNo */
  "reshapeSizeChecks",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\reshapeSizeChecks.m"/* pName */
};

static emlrtRTEInfo gb_emlrtRTEI = { 11,/* lineNo */
  5,                                   /* colNo */
  "user_control_profile",              /* fName */
  "C:\\Users\\jiang\\Desktop\\towing_project_demo\\MPC\\formation\\two_ship1\\user_control_profile.m"/* pName */
};

/* Function Definitions */
void user_control_profile(const emlrtStack *sp, const real_T p[48], real_T
  p_uparam_nu, real_T p_uparam_Np, const real_T p_uparam_R[11520],
  emxArray_real_T *u_profile)
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
  real_T dv[240];
  int32_T loop_ub;
  int32_T b_loop_ub;
  int32_T i;
  int32_T i1;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;

  /* ------------------------------------------------------------------------------- */
  /* pdf_mpc package: Example 1 - Definition of the user_control_profile map */
  /*  u_profile是控制序列， Np行，nu列 */
  /*  一般这个子程序不需要改 */
  /* ------------------------------------------------------------------------------- */
  st.site = &y_emlrtRSI;
  b_st.site = &ab_emlrtRSI;
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
        &p[0], &ldb_t, &beta1, &dv[0], &ldc_t);
  st.site = &y_emlrtRSI;
  b_st.site = &db_emlrtRSI;
  c_st.site = &eb_emlrtRSI;
  assertValidSizeArg(&c_st, p_uparam_nu);
  c_st.site = &eb_emlrtRSI;
  assertValidSizeArg(&c_st, p_uparam_Np);
  loop_ub = (int32_T)p_uparam_nu;
  if (loop_ub > 240) {
    emlrtErrorWithMessageIdR2018a(&st, &c_emlrtRTEI,
      "Coder:toolbox:reshape_emptyReshapeLimit",
      "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }

  b_loop_ub = (int32_T)p_uparam_Np;
  if (b_loop_ub > 240) {
    emlrtErrorWithMessageIdR2018a(&st, &c_emlrtRTEI,
      "Coder:toolbox:reshape_emptyReshapeLimit",
      "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }

  if ((loop_ub < 0) || (b_loop_ub < 0)) {
    emlrtErrorWithMessageIdR2018a(&st, &d_emlrtRTEI,
      "MATLAB:checkDimCommon:nonnegativeSize",
      "MATLAB:checkDimCommon:nonnegativeSize", 0);
  }

  if (loop_ub * b_loop_ub != 240) {
    emlrtErrorWithMessageIdR2018a(&st, &e_emlrtRTEI,
      "Coder:MATLAB:getReshapeDims_notSameNumel",
      "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
  }

  i = u_profile->size[0] * u_profile->size[1];
  u_profile->size[0] = b_loop_ub;
  u_profile->size[1] = loop_ub;
  emxEnsureCapacity_real_T(sp, u_profile, i, &gb_emlrtRTEI);
  for (i = 0; i < loop_ub; i++) {
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      u_profile->data[i1 + u_profile->size[0] * i] = dv[i + loop_ub * i1];
    }
  }

  /* ------------------------------------------------------------------------------- */
}

/* End of code generation (user_control_profile.c) */
