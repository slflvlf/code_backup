/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * nlmpcmoveCodeGeneration.c
 *
 * Code generation for function 'nlmpcmoveCodeGeneration'
 *
 */

/* Include files */
#include "nlmpcmoveCodeGeneration.h"
#include "fmincon.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_internal_types.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "znlmpc_confun.h"
#include "znlmpc_generateRuntimeData.h"
#include "znlmpc_getUBounds.h"
#include "znlmpc_getXUe.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = {
    56,                        /* lineNo */
    "nlmpcmoveCodeGeneration", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpc\\nlmpcmoveCodeGeneration.m" /* pathName
                                                                           */
};

static emlrtRSInfo b_emlrtRSI = {
    58,                        /* lineNo */
    "nlmpcmoveCodeGeneration", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpc\\nlmpcmoveCodeGeneration.m" /* pathName
                                                                           */
};

static emlrtRSInfo c_emlrtRSI = {
    101,                       /* lineNo */
    "nlmpcmoveCodeGeneration", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpc\\nlmpcmoveCodeGeneration.m" /* pathName
                                                                           */
};

static emlrtRSInfo d_emlrtRSI = {
    122,                       /* lineNo */
    "nlmpcmoveCodeGeneration", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpc\\nlmpcmoveCodeGeneration.m" /* pathName
                                                                           */
};

static emlrtRSInfo e_emlrtRSI = {
    142,                       /* lineNo */
    "nlmpcmoveCodeGeneration", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpc\\nlmpcmoveCodeGeneration.m" /* pathName
                                                                           */
};

static emlrtRSInfo f_emlrtRSI = {
    149,                       /* lineNo */
    "nlmpcmoveCodeGeneration", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpc\\nlmpcmoveCodeGeneration.m" /* pathName
                                                                           */
};

static emlrtRSInfo rb_emlrtRSI =
    {
        84,              /* lineNo */
        "znlmpc_confun", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_confun.m" /* pathName
                                                                          */
};

/* Function Definitions */
void c_nlmpcmoveCodeGeneration_anonF(
    c_nlmpcmoveCodeGenerationStackD *SD, const emlrtStack *sp,
    const real_T runtimedata_x[24], const real_T runtimedata_OutputMin[192],
    const real_T runtimedata_OutputMax[192], const real_T z[265],
    real_T varargout_1_data[], int32_T varargout_1_size[2],
    real_T varargout_2[192], real_T varargout_3_data[],
    int32_T varargout_3_size[2], real_T varargout_4[50880])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  real_T c_data[384];
  real_T U[216];
  real_T X[216];
  real_T e;
  int32_T Jc_size[2];
  int32_T c_size[2];
  int32_T b_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int16_T input_sizes_idx_0;
  int16_T sizes_idx_1;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &mb_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  b_st.site = &nb_emlrtRSI;
  znlmpc_getXUe(z, runtimedata_x, X, U, &e);
  b_st.site = &ob_emlrtRSI;
  stateEvolution(SD, &b_st, X, U, varargout_2, varargout_4);
  b_st.site = &pb_emlrtRSI;
  outputBounds(SD, &b_st, runtimedata_OutputMin, runtimedata_OutputMax, X, e,
               c_data, c_size, SD->f8.Jc_data, Jc_size);
  b_st.site = &qb_emlrtRSI;
  c_st.site = &eb_emlrtRSI;
  sizes_idx_1 = (int16_T)((c_size[0] != 0) && (c_size[1] != 0));
  d_st.site = &fb_emlrtRSI;
  if ((c_size[1] != sizes_idx_1) && ((c_size[0] != 0) && (c_size[1] != 0))) {
    emlrtErrorWithMessageIdR2018a(&d_st, &d_emlrtRTEI,
                                  "MATLAB:catenate:matrixDimensionMismatch",
                                  "MATLAB:catenate:matrixDimensionMismatch", 0);
  }
  if ((sizes_idx_1 == 0) || ((c_size[0] != 0) && (c_size[1] != 0))) {
    input_sizes_idx_0 = (int16_T)c_size[0];
  } else {
    input_sizes_idx_0 = 0;
  }
  varargout_1_size[0] = input_sizes_idx_0;
  varargout_1_size[1] = sizes_idx_1;
  loop_ub = sizes_idx_1;
  for (i = 0; i < loop_ub; i++) {
    b_loop_ub = input_sizes_idx_0;
    if (b_loop_ub - 1 >= 0) {
      memcpy(&varargout_1_data[0], &c_data[0], b_loop_ub * sizeof(real_T));
    }
  }
  b_st.site = &rb_emlrtRSI;
  c_st.site = &eb_emlrtRSI;
  if ((Jc_size[0] != 0) && (Jc_size[1] != 0)) {
    sizes_idx_1 = (int16_T)Jc_size[0];
  } else {
    sizes_idx_1 = 0;
  }
  d_st.site = &fb_emlrtRSI;
  if ((Jc_size[0] != sizes_idx_1) && ((Jc_size[0] != 0) && (Jc_size[1] != 0))) {
    emlrtErrorWithMessageIdR2018a(&d_st, &d_emlrtRTEI,
                                  "MATLAB:catenate:matrixDimensionMismatch",
                                  "MATLAB:catenate:matrixDimensionMismatch", 0);
  }
  if ((sizes_idx_1 == 0) || ((Jc_size[0] != 0) && (Jc_size[1] != 0))) {
    input_sizes_idx_0 = (int16_T)Jc_size[1];
  } else {
    input_sizes_idx_0 = 0;
  }
  varargout_3_size[0] = sizes_idx_1;
  varargout_3_size[1] = input_sizes_idx_0;
  loop_ub = input_sizes_idx_0;
  for (i = 0; i < loop_ub; i++) {
    b_loop_ub = sizes_idx_1;
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      varargout_3_data[i1 + sizes_idx_1 * i] =
          SD->f8.Jc_data[i1 + sizes_idx_1 * i];
    }
  }
}

void nlmpcmoveCodeGeneration(c_nlmpcmoveCodeGenerationStackD *SD,
                             const emlrtStack *sp, const real_T x[24],
                             const real_T lastMV[24], struct1_T *onlinedata,
                             real_T mv[24], struct2_T *info)
{
  static const int8_T iv[8] = {2, 3, 4, 5, 6, 7, 8, 8};
  c_struct_T Out;
  emlrtStack b_st;
  emlrtStack st;
  k_struct_T CostFcn_workspace_userdata;
  l_struct_T runtimedata;
  real_T B_data[768];
  real_T d_runtimedata[265];
  real_T e_runtimedata[265];
  real_T z[265];
  real_T z0[265];
  real_T b_runtimedata[192];
  real_T c_runtimedata[192];
  real_T ExitFlag;
  real_T e;
  int32_T A_size[2];
  int32_T i;
  int32_T i1;
  int32_T k;
  boolean_T exitg1;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  b_st.site = &g_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 24)) {
    if ((!muDoubleScalarIsInf(x[k])) && (!muDoubleScalarIsNaN(x[k]))) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &b_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:nlmpcmoveCodeGeneration:expectedFinite", 3, 4, 3, "\"x\"");
  }
  st.site = &b_emlrtRSI;
  b_st.site = &g_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 24)) {
    if ((!muDoubleScalarIsInf(lastMV[k])) &&
        (!muDoubleScalarIsNaN(lastMV[k]))) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &b_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:nlmpcmoveCodeGeneration:expectedFinite", 3, 4, 8, "\"lastMV\"");
  }
  st.site = &c_emlrtRSI;
  znlmpc_generateRuntimeData(&st, x, lastMV, onlinedata->ref,
                             onlinedata->MVTarget, onlinedata->X0,
                             onlinedata->MV0, onlinedata->Slack0, &runtimedata,
                             &CostFcn_workspace_userdata, z0);
  st.site = &d_emlrtRSI;
  znlmpc_getUBounds(SD, &st, &runtimedata, SD->f12.A_data, A_size, B_data, &k);
  for (i = 0; i < 8; i++) {
    for (i1 = 0; i1 < 24; i1++) {
      int32_T b_runtimedata_tmp;
      int32_T runtimedata_tmp;
      runtimedata_tmp = i + (i1 << 3);
      b_runtimedata_tmp = i1 + 24 * i;
      b_runtimedata[b_runtimedata_tmp] = runtimedata.StateMin[runtimedata_tmp];
      c_runtimedata[b_runtimedata_tmp] = runtimedata.StateMax[runtimedata_tmp];
    }
  }
  memcpy(&d_runtimedata[0], &b_runtimedata[0], 192U * sizeof(real_T));
  for (i = 0; i < 72; i++) {
    d_runtimedata[i + 192] = rtMinusInf;
  }
  d_runtimedata[264] = 0.0;
  memcpy(&e_runtimedata[0], &c_runtimedata[0], 192U * sizeof(real_T));
  for (i = 0; i < 72; i++) {
    e_runtimedata[i + 192] = rtInf;
  }
  e_runtimedata[264] = rtInf;
  st.site = &e_emlrtRSI;
  fmincon(SD, &st, &runtimedata, &CostFcn_workspace_userdata, z0,
          SD->f12.A_data, A_size, B_data, k, d_runtimedata, e_runtimedata,
          &runtimedata, &CostFcn_workspace_userdata, z, &info->Cost, &ExitFlag,
          &Out);
  if ((ExitFlag == 0.0) && (Out.constrviolation > 1.0E-6)) {
    ExitFlag = -2.0;
  }
  st.site = &f_emlrtRSI;
  znlmpc_getXUe(z, x, info->Xopt, info->MVopt, &e);
  if (ExitFlag > 0.0) {
    for (i = 0; i < 24; i++) {
      mv[i] = info->MVopt[9 * i];
    }
  } else {
    memcpy(&mv[0], &lastMV[0], 24U * sizeof(real_T));
  }
  info->ExitFlag = ExitFlag;
  info->Iterations = Out.iterations;
  for (i = 0; i < 24; i++) {
    for (i1 = 0; i1 < 8; i1++) {
      k = i1 + (i << 3);
      onlinedata->MV0[k] = info->MVopt[(i1 + 9 * i) + 1];
      onlinedata->X0[k] = info->Xopt[iv[i1] + 9 * i];
    }
  }
  onlinedata->Slack0 = muDoubleScalarMax(0.0, e);
  memcpy(&info->Yopt[0], &info->Xopt[0], 216U * sizeof(real_T));
  for (i = 0; i < 9; i++) {
    info->Topt[i] = i;
  }
  info->Slack = e;
}

/* End of code generation (nlmpcmoveCodeGeneration.c) */
