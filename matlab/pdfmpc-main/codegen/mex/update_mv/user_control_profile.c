/*
 * user_control_profile.c
 *
 * Code generation for function 'user_control_profile'
 *
 */

/* Include files */
#include "user_control_profile.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo y_emlrtRSI = {
    5,                      /* lineNo */
    "user_control_profile", /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\user_control_profile.m" /* pathName
                                                                      */
};

static emlrtRSInfo ab_emlrtRSI = {
    29,                  /* lineNo */
    "reshapeSizeChecks", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pathName */
};

static emlrtRSInfo bb_emlrtRSI = {
    109,               /* lineNo */
    "computeDimsData", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pathName */
};

static emlrtRTEInfo c_emlrtRTEI = {
    52,                  /* lineNo */
    13,                  /* colNo */
    "reshapeSizeChecks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pName */
};

static emlrtRTEInfo d_emlrtRTEI = {
    57,                  /* lineNo */
    23,                  /* colNo */
    "reshapeSizeChecks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pName */
};

static emlrtRTEInfo e_emlrtRTEI = {
    59,                  /* lineNo */
    23,                  /* colNo */
    "reshapeSizeChecks", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\reshapeSizeChecks.m" /* pName */
};

static emlrtRTEInfo f_emlrtRTEI = {
    58,                   /* lineNo */
    23,                   /* colNo */
    "assertValidSizeArg", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\assertValidSizeArg.m" /* pName */
};

static emlrtRTEInfo g_emlrtRTEI = {
    64,                   /* lineNo */
    15,                   /* colNo */
    "assertValidSizeArg", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\assertValidSizeArg.m" /* pName */
};

/* Function Definitions */
void user_control_profile(const emlrtStack *sp, const real_T p[4],
                          real_T p_uparam_nu, real_T p_uparam_Np,
                          const real_T p_uparam_R[80], real_T u_profile_data[],
                          int32_T u_profile_size[2])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T b_p_uparam_R[20];
  real_T b_p_uparam_Np;
  real_T d;
  real_T d1;
  real_T d2;
  int32_T i;
  boolean_T out;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  /* -------------------------------------------------------------------------------
   */
  /* pdf_mpc package: Example 1 - Definition of the user_control_profile map */
  /* -------------------------------------------------------------------------------
   */
  st.site = &y_emlrtRSI;
  b_st.site = &ab_emlrtRSI;
  c_st.site = &bb_emlrtRSI;
  if ((p_uparam_Np != muDoubleScalarFloor(p_uparam_Np)) ||
      muDoubleScalarIsInf(p_uparam_Np) || (p_uparam_Np < -2.147483648E+9) ||
      (p_uparam_Np > 2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &f_emlrtRTEI, "Coder:MATLAB:NonIntegerInput",
        "Coder:MATLAB:NonIntegerInput", 4, 12, MIN_int32_T, 12, MAX_int32_T);
  }
  if (p_uparam_Np <= 0.0) {
    b_p_uparam_Np = 0.0;
  } else {
    b_p_uparam_Np = p_uparam_Np;
  }
  if (!(b_p_uparam_Np <= 2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(&c_st, &g_emlrtRTEI, "Coder:MATLAB:pmaxsize",
                                  "Coder:MATLAB:pmaxsize", 0);
  }
  c_st.site = &bb_emlrtRSI;
  if ((p_uparam_nu != muDoubleScalarFloor(p_uparam_nu)) ||
      muDoubleScalarIsInf(p_uparam_nu) || (p_uparam_nu < -2.147483648E+9) ||
      (p_uparam_nu > 2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &f_emlrtRTEI, "Coder:MATLAB:NonIntegerInput",
        "Coder:MATLAB:NonIntegerInput", 4, 12, MIN_int32_T, 12, MAX_int32_T);
  }
  if (p_uparam_nu <= 0.0) {
    b_p_uparam_Np = 0.0;
  } else {
    b_p_uparam_Np = p_uparam_nu;
  }
  if (!(b_p_uparam_Np <= 2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(&c_st, &g_emlrtRTEI, "Coder:MATLAB:pmaxsize",
                                  "Coder:MATLAB:pmaxsize", 0);
  }
  if ((int32_T)p_uparam_Np > 20) {
    emlrtErrorWithMessageIdR2018a(&st, &c_emlrtRTEI,
                                  "Coder:toolbox:reshape_emptyReshapeLimit",
                                  "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }
  if ((int32_T)p_uparam_nu > 20) {
    emlrtErrorWithMessageIdR2018a(&st, &c_emlrtRTEI,
                                  "Coder:toolbox:reshape_emptyReshapeLimit",
                                  "Coder:toolbox:reshape_emptyReshapeLimit", 0);
  }
  out = ((int32_T)p_uparam_Np >= 0);
  if ((!out) || ((int32_T)p_uparam_nu < 0)) {
    out = false;
  }
  if (!out) {
    emlrtErrorWithMessageIdR2018a(&st, &d_emlrtRTEI,
                                  "MATLAB:checkDimCommon:nonnegativeSize",
                                  "MATLAB:checkDimCommon:nonnegativeSize", 0);
  }
  if ((int32_T)p_uparam_Np * (int32_T)p_uparam_nu != 20) {
    emlrtErrorWithMessageIdR2018a(
        &st, &e_emlrtRTEI, "Coder:MATLAB:getReshapeDims_notSameNumel",
        "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
  }
  b_p_uparam_Np = p[0];
  d = p[1];
  d1 = p[2];
  d2 = p[3];
  for (i = 0; i < 20; i++) {
    b_p_uparam_R[i] =
        ((p_uparam_R[i] * b_p_uparam_Np + p_uparam_R[i + 20] * d) +
         p_uparam_R[i + 40] * d1) +
        p_uparam_R[i + 60] * d2;
  }
  u_profile_size[0] = (int32_T)p_uparam_Np;
  u_profile_size[1] = (int32_T)p_uparam_nu;
  memcpy(&u_profile_data[0], &b_p_uparam_R[0], 20U * sizeof(real_T));
}

/* End of code generation (user_control_profile.c) */
