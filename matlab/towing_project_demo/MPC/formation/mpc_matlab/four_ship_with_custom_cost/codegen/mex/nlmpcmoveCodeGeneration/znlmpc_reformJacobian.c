/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * znlmpc_reformJacobian.c
 *
 * Code generation for function 'znlmpc_reformJacobian'
 *
 */

/* Include files */
#include "znlmpc_reformJacobian.h"
#include "mtimes.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo
    vb_emlrtRSI =
        {
            25,                      /* lineNo */
            "znlmpc_reformJacobian", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_"
            "reformJacobian.m" /* pathName */
};

/* Function Definitions */
void znlmpc_reformJacobian(c_nlmpcmoveCodeGenerationStackD *SD,
                           const emlrtStack *sp, const real_T Jx_data[],
                           const int32_T Jx_size[3], const real_T Jmv_data[],
                           const int32_T Jmv_size[3], const real_T Je_data[],
                           int32_T Je_size, real_T Jc_data[],
                           int32_T Jc_size[2])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T b_Je_data[384];
  int32_T b_y_size;
  int32_T i;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  if (Jx_size[0] == 0) {
    Jc_size[0] = 0;
    Jc_size[1] = 0;
  } else {
    int32_T Jx[2];
    int32_T y_size[2];
    int32_T maxdimlen;
    int32_T nx;
    int32_T varargin_1_size_idx_1;
    st.site = &vb_emlrtRSI;
    nx = (Jx_size[0] * 24) << 3;
    b_st.site = &bb_emlrtRSI;
    maxdimlen = Jx_size[0];
    if (Jx_size[0] < 24) {
      maxdimlen = 24;
    }
    if (Jx_size[0] > muIntScalarMax_sint32(nx, maxdimlen)) {
      emlrtErrorWithMessageIdR2018a(
          &st, &f_emlrtRTEI, "Coder:toolbox:reshape_emptyReshapeLimit",
          "Coder:toolbox:reshape_emptyReshapeLimit", 0);
    }
    if (Jx_size[0] * 192 != nx) {
      emlrtErrorWithMessageIdR2018a(
          &st, &e_emlrtRTEI, "Coder:MATLAB:getReshapeDims_notSameNumel",
          "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
    }
    st.site = &vb_emlrtRSI;
    nx = (Jmv_size[0] * 24) << 3;
    b_st.site = &bb_emlrtRSI;
    maxdimlen = Jmv_size[0];
    if (Jmv_size[0] < 24) {
      maxdimlen = 24;
    }
    maxdimlen = muIntScalarMax_sint32(nx, maxdimlen);
    if (Jx_size[0] > maxdimlen) {
      emlrtErrorWithMessageIdR2018a(
          &st, &f_emlrtRTEI, "Coder:toolbox:reshape_emptyReshapeLimit",
          "Coder:toolbox:reshape_emptyReshapeLimit", 0);
    }
    if (maxdimlen < 192) {
      emlrtErrorWithMessageIdR2018a(
          &st, &f_emlrtRTEI, "Coder:toolbox:reshape_emptyReshapeLimit",
          "Coder:toolbox:reshape_emptyReshapeLimit", 0);
    }
    if (Jx_size[0] * 192 != nx) {
      emlrtErrorWithMessageIdR2018a(
          &st, &e_emlrtRTEI, "Coder:MATLAB:getReshapeDims_notSameNumel",
          "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
    }
    st.site = &vb_emlrtRSI;
    Jx[0] = Jx_size[0];
    Jx[1] = 192;
    b_st.site = &r_emlrtRSI;
    mtimes(Jmv_data, Jx, SD->u1.f1.y_data, y_size);
    st.site = &vb_emlrtRSI;
    Jx[0] = Jx_size[0];
    varargin_1_size_idx_1 = Jx_size[0];
    maxdimlen = Jx_size[0];
    for (b_y_size = 0; b_y_size < maxdimlen; b_y_size++) {
      for (i = 0; i < 192; i++) {
        SD->u1.f1.varargin_1_data[i + 192 * b_y_size] =
            Jx_data[b_y_size + Jx[0] * i];
      }
    }
    maxdimlen = y_size[0];
    for (b_y_size = 0; b_y_size < maxdimlen; b_y_size++) {
      for (i = 0; i < 72; i++) {
        SD->u1.f1.varargin_2_data[i + 72 * b_y_size] =
            SD->u1.f1.y_data[b_y_size + y_size[0] * i];
      }
    }
    b_st.site = &eb_emlrtRSI;
    c_st.site = &fb_emlrtRSI;
    if ((y_size[0] != Jx_size[0]) && (y_size[0] != 0)) {
      emlrtErrorWithMessageIdR2018a(
          &c_st, &d_emlrtRTEI, "MATLAB:catenate:matrixDimensionMismatch",
          "MATLAB:catenate:matrixDimensionMismatch", 0);
    }
    if ((Je_size != Jx_size[0]) && (Je_size != 0)) {
      emlrtErrorWithMessageIdR2018a(
          &c_st, &d_emlrtRTEI, "MATLAB:catenate:matrixDimensionMismatch",
          "MATLAB:catenate:matrixDimensionMismatch", 0);
    }
    if (y_size[0] != 0) {
      maxdimlen = 72;
    } else {
      maxdimlen = 0;
    }
    if (Je_size - 1 >= 0) {
      memcpy(&b_Je_data[0], &Je_data[0], Je_size * sizeof(real_T));
    }
    nx = (Je_size != 0);
    if (y_size[0] != 0) {
      b_y_size = 72;
    } else {
      b_y_size = 0;
    }
    Jc_size[0] = (b_y_size + (Je_size != 0)) + 192;
    Jc_size[1] = Jx_size[0];
    for (b_y_size = 0; b_y_size < varargin_1_size_idx_1; b_y_size++) {
      for (i = 0; i < 192; i++) {
        Jc_data[i + Jc_size[0] * b_y_size] =
            SD->u1.f1.varargin_1_data[i + 192 * b_y_size];
      }
    }
    for (b_y_size = 0; b_y_size < varargin_1_size_idx_1; b_y_size++) {
      for (i = 0; i < maxdimlen; i++) {
        Jc_data[(i + Jc_size[0] * b_y_size) + 192] =
            SD->u1.f1.varargin_2_data[i + maxdimlen * b_y_size];
      }
    }
    for (b_y_size = 0; b_y_size < varargin_1_size_idx_1; b_y_size++) {
      if (nx - 1 >= 0) {
        Jc_data[(maxdimlen + Jc_size[0] * b_y_size) + 192] =
            b_Je_data[nx * b_y_size];
      }
    }
  }
}

/* End of code generation (znlmpc_reformJacobian.c) */
