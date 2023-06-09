/*
 * znlmpc_generateRuntimeData.c
 *
 * Code generation for function 'znlmpc_generateRuntimeData'
 *
 */

/* Include files */
#include "znlmpc_generateRuntimeData.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_internal_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo h_emlrtRSI = {
    20,                           /* lineNo */
    "znlmpc_generateRuntimeData", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_generateRuntimeData."
    "m" /* pathName */
};

static emlrtRSInfo i_emlrtRSI = {
    30,                           /* lineNo */
    "znlmpc_generateRuntimeData", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_generateRuntimeData."
    "m" /* pathName */
};

static emlrtRSInfo j_emlrtRSI = {
    72,                           /* lineNo */
    "znlmpc_generateRuntimeData", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_generateRuntimeData."
    "m" /* pathName */
};

static emlrtRSInfo k_emlrtRSI = {
    74,                           /* lineNo */
    "znlmpc_generateRuntimeData", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_generateRuntimeData."
    "m" /* pathName */
};

static emlrtRSInfo l_emlrtRSI = {
    76,                           /* lineNo */
    "znlmpc_generateRuntimeData", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_generateRuntimeData."
    "m" /* pathName */
};

static emlrtRSInfo m_emlrtRSI = {
    78,                           /* lineNo */
    "znlmpc_generateRuntimeData", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_generateRuntimeData."
    "m" /* pathName */
};

static emlrtRSInfo n_emlrtRSI = {
    147,           /* lineNo */
    "getInitialX", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_generateRuntimeData."
    "m" /* pathName */
};

static emlrtRSInfo o_emlrtRSI = {
    156,            /* lineNo */
    "getInitialMV", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_generateRuntimeData."
    "m" /* pathName */
};

static emlrtRSInfo p_emlrtRSI = {
    165,               /* lineNo */
    "getInitialSlack", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_generateRuntimeData."
    "m" /* pathName */
};

static emlrtRSInfo q_emlrtRSI = {
    36,                    /* lineNo */
    "znlmpc_setDecisions", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_setDecisions.m" /* pathName
                                                                            */
};

static emlrtRTEInfo emlrtRTEI = {
    14,               /* lineNo */
    37,               /* colNo */
    "validatenonnan", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatenonnan.m" /* pName */
};

static emlrtRTEInfo c_emlrtRTEI = {
    14,                    /* lineNo */
    37,                    /* colNo */
    "validatenonnegative", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatenonnegative.m" /* pName */
};

/* Function Definitions */
void znlmpc_generateRuntimeData(const emlrtStack *sp, const real_T x[6],
                                const real_T lastMV[8], const real_T ref0[6],
                                const real_T MVTarget0[8], const real_T X0[60],
                                const real_T MV0[80], real_T Slack0,
                                l_struct_T *runtimedata, k_struct_T *userdata,
                                real_T z0[85])
{
  static const real_T a[1920] = {
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0};
  static const real_T dv1[80] = {0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 -0.78539816339744828,
                                 -0.78539816339744828,
                                 -0.78539816339744828,
                                 -0.78539816339744828,
                                 -0.78539816339744828,
                                 -0.78539816339744828,
                                 -0.78539816339744828,
                                 -0.78539816339744828,
                                 -0.78539816339744828,
                                 -0.78539816339744828,
                                 2.3561944901923448,
                                 2.3561944901923448,
                                 2.3561944901923448,
                                 2.3561944901923448,
                                 2.3561944901923448,
                                 2.3561944901923448,
                                 2.3561944901923448,
                                 2.3561944901923448,
                                 2.3561944901923448,
                                 2.3561944901923448,
                                 2.3561944901923448,
                                 2.3561944901923448,
                                 2.3561944901923448,
                                 2.3561944901923448,
                                 2.3561944901923448,
                                 2.3561944901923448,
                                 2.3561944901923448,
                                 2.3561944901923448,
                                 2.3561944901923448,
                                 2.3561944901923448,
                                 -0.78539816339744828,
                                 -0.78539816339744828,
                                 -0.78539816339744828,
                                 -0.78539816339744828,
                                 -0.78539816339744828,
                                 -0.78539816339744828,
                                 -0.78539816339744828,
                                 -0.78539816339744828,
                                 -0.78539816339744828,
                                 -0.78539816339744828};
  static const real_T dv2[80] = {800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 800.0,
                                 0.78539816339744828,
                                 0.78539816339744828,
                                 0.78539816339744828,
                                 0.78539816339744828,
                                 0.78539816339744828,
                                 0.78539816339744828,
                                 0.78539816339744828,
                                 0.78539816339744828,
                                 0.78539816339744828,
                                 0.78539816339744828,
                                 3.9269908169872414,
                                 3.9269908169872414,
                                 3.9269908169872414,
                                 3.9269908169872414,
                                 3.9269908169872414,
                                 3.9269908169872414,
                                 3.9269908169872414,
                                 3.9269908169872414,
                                 3.9269908169872414,
                                 3.9269908169872414,
                                 3.9269908169872414,
                                 3.9269908169872414,
                                 3.9269908169872414,
                                 3.9269908169872414,
                                 3.9269908169872414,
                                 3.9269908169872414,
                                 3.9269908169872414,
                                 3.9269908169872414,
                                 3.9269908169872414,
                                 3.9269908169872414,
                                 0.78539816339744828,
                                 0.78539816339744828,
                                 0.78539816339744828,
                                 0.78539816339744828,
                                 0.78539816339744828,
                                 0.78539816339744828,
                                 0.78539816339744828,
                                 0.78539816339744828,
                                 0.78539816339744828,
                                 0.78539816339744828};
  static const real_T dv3[80] = {-100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -100.0,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295,
                                 -0.17453292519943295};
  static const real_T dv4[80] = {100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 100.0,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295,
                                 0.17453292519943295};
  static const real_T b_dv[60] = {
      12500.0, 12500.0, 12500.0, 12500.0, 12500.0, 12500.0, 12500.0, 12500.0,
      12500.0, 12500.0, 12500.0, 12500.0, 12500.0, 12500.0, 12500.0, 12500.0,
      12500.0, 12500.0, 12500.0, 12500.0, 12500.0, 12500.0, 12500.0, 12500.0,
      12500.0, 12500.0, 12500.0, 12500.0, 12500.0, 12500.0, 0.5,     0.5,
      0.5,     0.5,     0.5,     0.5,     0.5,     0.5,     0.5,     0.5,
      0.5,     0.5,     0.5,     0.5,     0.5,     0.5,     0.5,     0.5,
      0.5,     0.5,     0.5,     0.5,     0.5,     0.5,     0.5,     0.5,
      0.5,     0.5,     0.5,     0.5};
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T b_MV0[80];
  real_T b_X0[60];
  real_T uz[24];
  real_T alpha1;
  real_T beta1;
  int32_T b_i;
  int32_T i;
  char_T TRANSA1;
  char_T TRANSB1;
  boolean_T exitg1;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &h_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  b_st.site = &g_emlrtRSI;
  p = true;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 6)) {
    if (!muDoubleScalarIsNaN(ref0[i])) {
      i++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedNonNaN",
        "MATLAB:expectedNonNaN", 3, 4, 5, "\"ref\"");
  }
  b_st.site = &g_emlrtRSI;
  p = true;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 6)) {
    if ((!muDoubleScalarIsInf(ref0[i])) && (!muDoubleScalarIsNaN(ref0[i]))) {
      i++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &b_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:expectedFinite", 3, 4, 5, "\"ref\"");
  }
  for (i = 0; i < 6; i++) {
    userdata->References[10 * i] = ref0[i];
    for (b_i = 0; b_i < 9; b_i++) {
      userdata->References[(b_i + 10 * i) + 1] = ref0[i];
    }
  }
  st.site = &i_emlrtRSI;
  b_st.site = &g_emlrtRSI;
  p = true;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 8)) {
    if (!muDoubleScalarIsNaN(MVTarget0[i])) {
      i++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedNonNaN",
        "MATLAB:expectedNonNaN", 3, 4, 10, "\"MVTarget\"");
  }
  b_st.site = &g_emlrtRSI;
  p = true;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 8)) {
    if ((!muDoubleScalarIsInf(MVTarget0[i])) &&
        (!muDoubleScalarIsNaN(MVTarget0[i]))) {
      i++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &b_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:expectedFinite", 3, 4, 10, "\"MVTarget\"");
  }
  for (i = 0; i < 8; i++) {
    userdata->MVTarget[10 * i] = MVTarget0[i];
    for (b_i = 0; b_i < 9; b_i++) {
      userdata->MVTarget[(b_i + 10 * i) + 1] = MVTarget0[i];
    }
  }
  st.site = &j_emlrtRSI;
  b_st.site = &n_emlrtRSI;
  c_st.site = &g_emlrtRSI;
  p = true;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 60)) {
    if ((!muDoubleScalarIsInf(X0[i])) && (!muDoubleScalarIsNaN(X0[i]))) {
      i++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &b_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:expectedFinite", 3, 4, 4, "\"X0\"");
  }
  c_st.site = &g_emlrtRSI;
  p = true;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 60)) {
    if (!muDoubleScalarIsNaN(X0[i])) {
      i++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedNonNaN",
        "MATLAB:expectedNonNaN", 3, 4, 4, "\"X0\"");
  }
  st.site = &k_emlrtRSI;
  b_st.site = &o_emlrtRSI;
  c_st.site = &g_emlrtRSI;
  p = true;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 80)) {
    if ((!muDoubleScalarIsInf(MV0[i])) && (!muDoubleScalarIsNaN(MV0[i]))) {
      i++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &b_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:expectedFinite", 3, 4, 5, "\"MV0\"");
  }
  c_st.site = &g_emlrtRSI;
  p = true;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 80)) {
    if (!muDoubleScalarIsNaN(MV0[i])) {
      i++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedNonNaN",
        "MATLAB:expectedNonNaN", 3, 4, 5, "\"MV0\"");
  }
  st.site = &l_emlrtRSI;
  b_st.site = &p_emlrtRSI;
  c_st.site = &g_emlrtRSI;
  if (muDoubleScalarIsInf(Slack0) || muDoubleScalarIsNaN(Slack0)) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &b_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:expectedFinite", 3, 4, 8, "\"Slack0\"");
  }
  c_st.site = &g_emlrtRSI;
  if (Slack0 < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &c_emlrtRTEI,
        "Coder:toolbox:ValidateattributesexpectedNonnegative",
        "MATLAB:expectedNonnegative", 3, 4, 8, "\"Slack0\"");
  }
  st.site = &m_emlrtRSI;
  b_st.site = &q_emlrtRSI;
  for (i = 0; i < 10; i++) {
    for (b_i = 0; b_i < 8; b_i++) {
      b_MV0[b_i + (i << 3)] = MV0[i + 10 * b_i];
    }
  }
  TRANSB1 = 'N';
  TRANSA1 = 'N';
  alpha1 = 1.0;
  beta1 = 0.0;
  m_t = (ptrdiff_t)24;
  n_t = (ptrdiff_t)1;
  k_t = (ptrdiff_t)80;
  lda_t = (ptrdiff_t)24;
  ldb_t = (ptrdiff_t)80;
  ldc_t = (ptrdiff_t)24;
  dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &a[0], &lda_t, &b_MV0[0],
        &ldb_t, &beta1, &uz[0], &ldc_t);
  for (i = 0; i < 10; i++) {
    for (b_i = 0; b_i < 6; b_i++) {
      b_X0[b_i + 6 * i] = X0[i + 10 * b_i];
    }
  }
  memcpy(&z0[0], &b_X0[0], 60U * sizeof(real_T));
  memcpy(&z0[60], &uz[0], 24U * sizeof(real_T));
  z0[84] = Slack0;
  userdata->Ts = 1.0;
  for (i = 0; i < 6; i++) {
    userdata->CurrentStates[i] = x[i];
  }
  userdata->PredictionHorizon = 10.0;
  userdata->NumOfStates = 6.0;
  userdata->NumOfOutputs = 6.0;
  userdata->NumOfInputs = 8.0;
  for (i = 0; i < 8; i++) {
    userdata->LastMV[i] = lastMV[i];
    userdata->MVIndex[i] = (real_T)i + 1.0;
  }
  for (i = 0; i < 6; i++) {
    runtimedata->x[i] = x[i];
  }
  memcpy(&runtimedata->lastMV[0], &lastMV[0], 8U * sizeof(real_T));
  memcpy(&runtimedata->ref[0], &userdata->References[0], 60U * sizeof(real_T));
  memcpy(&runtimedata->OutputWeights[0], &b_dv[0], 60U * sizeof(real_T));
  memset(&runtimedata->MVWeights[0], 0, 80U * sizeof(real_T));
  memset(&runtimedata->MVRateWeights[0], 0, 80U * sizeof(real_T));
  runtimedata->ECRWeight = 100000.0;
  for (i = 0; i < 60; i++) {
    runtimedata->OutputMin[i] = rtMinusInf;
    runtimedata->OutputMax[i] = rtInf;
    runtimedata->StateMin[i] = rtMinusInf;
    runtimedata->StateMax[i] = rtInf;
  }
  memcpy(&runtimedata->MVMin[0], &dv1[0], 80U * sizeof(real_T));
  memcpy(&runtimedata->MVMax[0], &dv2[0], 80U * sizeof(real_T));
  memcpy(&runtimedata->MVRateMin[0], &dv3[0], 80U * sizeof(real_T));
  memcpy(&runtimedata->MVRateMax[0], &dv4[0], 80U * sizeof(real_T));
  memcpy(&runtimedata->MVScaledTarget[0], &userdata->MVTarget[0],
         80U * sizeof(real_T));
}

/* End of code generation (znlmpc_generateRuntimeData.c) */
