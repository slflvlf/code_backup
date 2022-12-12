/*
 * user_ode.c
 *
 * Code generation for function 'user_ode'
 *
 */

/* Include files */
#include "user_ode.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtECInfo emlrtECI = {
    -1,                                                  /* nDims */
    15,                                                  /* lineNo */
    1,                                                   /* colNo */
    "user_ode",                                          /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\user_ode.m" /* pName */
};

static emlrtECInfo b_emlrtECI = {
    -1,                                                  /* nDims */
    17,                                                  /* lineNo */
    1,                                                   /* colNo */
    "user_ode",                                          /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\user_ode.m" /* pName */
};

/* Function Definitions */
void user_ode(const emlrtStack *sp, const real_T x[4], const real_T u_data[],
              int32_T u_size, const real_T p_ode_w[3], real_T xdot[4])
{
  real_T tmp_data[20];
  real_T b_m;
  real_T c;
  real_T c_m;
  real_T d_m;
  real_T frot_r;
  real_T m;
  real_T m_tmp;
  real_T s;
  int32_T i;
  /* -------------------------------------------------------------------------------
   */
  /*  pdf_mpc package: Example 1 - Definition of the user_ode map */
  /* -------------------------------------------------------------------------------
   */
  /*  x=(r,rp,theta,thetap) */
  m = 200.0 * (p_ode_w[0] + 1.0);
  c = muDoubleScalarCos(x[2]);
  s = muDoubleScalarSin(x[2]);
  xdot[0] = x[1];
  b_m = m * 0.81 * c * s;
  m_tmp = x[3] * x[3];
  c_m = m * 100.0 * s * m_tmp;
  frot_r = 10.0 * (p_ode_w[2] + 1.0) * x[1];
  d_m = m * (1.0 - c * c) + 1500.0;
  for (i = 0; i < u_size; i++) {
    tmp_data[i] = (((u_data[i] + b_m) + c_m) - frot_r) / d_m;
  }
  if (u_size != 1) {
    emlrtSubAssignSizeCheck1dR2017a(1, u_size, &emlrtECI, (emlrtCTX)sp);
  }
  xdot[1] = tmp_data[0];
  xdot[2] = x[3];
  b_m = m * 100.0 * m_tmp * c * s;
  m_tmp = (1500.0 - m) * 0.81 * s;
  c_m = 100000.0 * (p_ode_w[1] + 1.0) * x[3];
  m = (m * (s * s) + 1500.0) * 100.0;
  for (i = 0; i < u_size; i++) {
    tmp_data[i] = (((-u_data[i] * c - b_m) - m_tmp) - c_m) / m;
  }
  if (u_size != 1) {
    emlrtSubAssignSizeCheck1dR2017a(1, u_size, &b_emlrtECI, (emlrtCTX)sp);
  }
  xdot[3] = tmp_data[0];
}

/* End of code generation (user_ode.c) */
