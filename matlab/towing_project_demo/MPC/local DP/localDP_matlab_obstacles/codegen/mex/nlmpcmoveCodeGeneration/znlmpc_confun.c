/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * znlmpc_confun.c
 *
 * Code generation for function 'znlmpc_confun'
 *
 */

/* Include files */
#include "znlmpc_confun.h"
#include "all.h"
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

static emlrtRSInfo xb_emlrtRSI =
    {
        184,            /* lineNo */
        "outputBounds", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_confun.m" /* pathName
                                                                          */
};

static emlrtBCInfo h_emlrtBCI = {
    1,              /* iFirst */
    120,            /* iLast */
    169,            /* lineNo */
    17,             /* colNo */
    "",             /* aName */
    "outputBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_confun.m", /* pName
                                                                       */
    3 /* checkKind */
};

static emlrtBCInfo i_emlrtBCI = {
    1,              /* iFirst */
    120,            /* iLast */
    172,            /* lineNo */
    13,             /* colNo */
    "",             /* aName */
    "outputBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_confun.m", /* pName
                                                                       */
    3 /* checkKind */
};

/* Function Definitions */
void outputBounds(c_nlmpcmoveCodeGenerationStackD *SD, const emlrtStack *sp,
                  const real_T runtimedata_OutputMin[60],
                  const real_T runtimedata_OutputMax[60], const real_T X[66],
                  real_T e, real_T c_data[], int32_T c_size[2],
                  real_T Jc_data[], int32_T Jc_size[2])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  real_T b_c_data[120];
  int32_T tmp_size[2];
  int32_T b_i;
  int32_T i;
  int32_T i1;
  int32_T k;
  int8_T Jx[7200];
  int8_T Je[120];
  int8_T b_tmp_data[120];
  int8_T tmp_data[120];
  boolean_T bv[60];
  boolean_T x[6];
  boolean_T exitg1;
  boolean_T guard1 = false;
  boolean_T y;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  for (i = 0; i < 60; i++) {
    bv[i] = muDoubleScalarIsInf(runtimedata_OutputMin[i]);
  }
  st.site = &wb_emlrtRSI;
  b_all(&st, bv, x);
  y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 6)) {
    if (!x[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  guard1 = false;
  if (y) {
    for (i = 0; i < 60; i++) {
      bv[i] = muDoubleScalarIsInf(runtimedata_OutputMax[i]);
    }
    st.site = &wb_emlrtRSI;
    b_all(&st, bv, x);
    y = true;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 6)) {
      if (!x[k]) {
        y = false;
        exitg1 = true;
      } else {
        k++;
      }
    }
    if (y) {
      c_size[0] = 0;
      c_size[1] = 0;
      Jc_size[0] = 0;
      Jc_size[1] = 0;
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1) {
    real_T c[120];
    int32_T trueCount;
    uint8_T ic[6];
    boolean_T icf[120];
    for (b_i = 0; b_i < 120; b_i++) {
      c[b_i] = 0.0;
      icf[b_i] = true;
    }
    memset(&Jx[0], 0, 7200U * sizeof(int8_T));
    memset(&Je[0], 0, 120U * sizeof(int8_T));
    for (i = 0; i < 6; i++) {
      ic[i] = (uint8_T)(i + 1U);
    }
    for (b_i = 0; b_i < 10; b_i++) {
      real_T d;
      uint8_T u;
      boolean_T bv1[6];
      for (i = 0; i < 6; i++) {
        d = runtimedata_OutputMin[b_i + 10 * i];
        x[i] = muDoubleScalarIsInf(d);
        bv1[i] = muDoubleScalarIsNaN(d);
      }
      for (i = 0; i < 6; i++) {
        u = ic[i];
        if ((u < 1) || (u > 120)) {
          emlrtDynamicBoundsCheckR2012b(u, 1, 120, &d_emlrtBCI, (emlrtCTX)sp);
        }
        icf[ic[i] - 1] = ((!x[i]) && (!bv1[i]));
      }
      for (i = 0; i < 6; i++) {
        d = runtimedata_OutputMax[b_i + 10 * i];
        x[i] = muDoubleScalarIsInf(d);
        bv1[i] = muDoubleScalarIsNaN(d);
      }
      for (i = 0; i < 6; i++) {
        i1 = ic[i] + 6;
        if (i1 > 120) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, 120, &e_emlrtBCI, (emlrtCTX)sp);
        }
        icf[i1 - 1] = ((!x[i]) && (!bv1[i]));
      }
      for (i = 0; i < 6; i++) {
        i1 = (int8_T)ic[i];
        if ((i1 < 1) || (i1 > 120)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, 120, &f_emlrtBCI, (emlrtCTX)sp);
        }
      }
      for (i = 0; i < 6; i++) {
        i1 = (int8_T)((int8_T)ic[i] + 6);
        if ((i1 < 1) || (i1 > 120)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, 120, &f_emlrtBCI, (emlrtCTX)sp);
        }
      }
      y = false;
      k = 0;
      exitg1 = false;
      while ((!exitg1) && (k <= 11)) {
        int32_T b_ic[12];
        for (i = 0; i < 6; i++) {
          u = ic[i];
          b_ic[i] = u - 1;
          b_ic[i + 6] = u + 5;
        }
        if (icf[b_ic[k]]) {
          y = true;
          exitg1 = true;
        } else {
          k++;
        }
      }
      if (y) {
        int8_T Ck[36];
        int8_T val[36];
        for (i = 0; i < 36; i++) {
          Ck[i] = 0;
        }
        for (k = 0; k < 6; k++) {
          Ck[k + 6 * k] = 1;
          c[ic[k] - 1] =
              (runtimedata_OutputMin[b_i + 10 * k] - e) - X[(b_i + 11 * k) + 1];
        }
        for (i = 0; i < 6; i++) {
          i1 = ic[i] + 6;
          if (i1 > 120) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, 120, &g_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          c[i1 - 1] =
              (X[(b_i + 11 * i) + 1] - runtimedata_OutputMax[b_i + 10 * i]) - e;
        }
        for (i = 0; i < 36; i++) {
          val[i] = (int8_T)-Ck[i];
        }
        for (k = 0; k < 6; k++) {
          for (i = 0; i < 6; i++) {
            Jx[((ic[i] + 120 * k) + 720 * b_i) - 1] = val[i + 6 * k];
          }
        }
        for (k = 0; k < 6; k++) {
          for (i = 0; i < 6; i++) {
            i1 = ic[i] + 6;
            if (i1 > 120) {
              emlrtDynamicBoundsCheckR2012b(i1, 1, 120, &h_emlrtBCI,
                                            (emlrtCTX)sp);
            }
            Jx[((i1 + 120 * k) + 720 * b_i) - 1] = Ck[i + 6 * k];
          }
        }
        for (i = 0; i < 6; i++) {
          Je[ic[i] - 1] = -1;
        }
        for (i = 0; i < 6; i++) {
          i1 = ic[i] + 6;
          if (i1 > 120) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, 120, &i_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          Je[i1 - 1] = -1;
        }
      }
      for (i = 0; i < 6; i++) {
        ic[i] = (uint8_T)(ic[i] + 12U);
      }
    }
    trueCount = 0;
    k = 0;
    for (b_i = 0; b_i < 120; b_i++) {
      if (icf[b_i]) {
        trueCount++;
        tmp_data[k] = (int8_T)(b_i + 1);
        k++;
      }
    }
    for (i = 0; i < trueCount; i++) {
      b_c_data[i] = c[tmp_data[i] - 1];
    }
    c_size[0] = trueCount;
    c_size[1] = 1;
    if (trueCount - 1 >= 0) {
      memcpy(&c_data[0], &b_c_data[0], trueCount * sizeof(real_T));
    }
    trueCount = 0;
    k = 0;
    for (b_i = 0; b_i < 120; b_i++) {
      if (icf[b_i]) {
        trueCount++;
        b_tmp_data[k] = (int8_T)(b_i + 1);
        k++;
      }
    }
    st.site = &xb_emlrtRSI;
    if (trueCount == 0) {
      Jc_size[0] = 0;
      Jc_size[1] = 0;
    } else {
      real_T varargin_2_data[2160];
      int32_T y_size[2];
      int8_T Jx_data[7200];
      int8_T input_sizes_idx_0;
      b_st.site = &vb_emlrtRSI;
      k = trueCount * 6 * 10;
      c_st.site = &bb_emlrtRSI;
      b_i = trueCount;
      if (trueCount < 6) {
        b_i = 6;
      }
      if (b_i < 10) {
        b_i = 10;
      }
      if (trueCount > muIntScalarMax_sint32(k, b_i)) {
        emlrtErrorWithMessageIdR2018a(
            &b_st, &l_emlrtRTEI, "Coder:toolbox:reshape_emptyReshapeLimit",
            "Coder:toolbox:reshape_emptyReshapeLimit", 0);
      }
      if (trueCount * 60 != k) {
        emlrtErrorWithMessageIdR2018a(
            &b_st, &m_emlrtRTEI, "Coder:MATLAB:getReshapeDims_notSameNumel",
            "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
      }
      b_st.site = &vb_emlrtRSI;
      c_st.site = &bb_emlrtRSI;
      b_i = trueCount;
      if (trueCount < 6) {
        b_i = 6;
      }
      if (b_i < 10) {
        b_i = 10;
      }
      if (trueCount > muIntScalarMax_sint32(k, b_i)) {
        emlrtErrorWithMessageIdR2018a(
            &b_st, &l_emlrtRTEI, "Coder:toolbox:reshape_emptyReshapeLimit",
            "Coder:toolbox:reshape_emptyReshapeLimit", 0);
      }
      if (trueCount * 60 != k) {
        emlrtErrorWithMessageIdR2018a(
            &b_st, &m_emlrtRTEI, "Coder:MATLAB:getReshapeDims_notSameNumel",
            "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
      }
      b_st.site = &vb_emlrtRSI;
      tmp_size[0] = trueCount;
      tmp_size[1] = 60;
      k = trueCount * 60;
      memset(&SD->u1.f1.tmp_data[0], 0, k * sizeof(real_T));
      c_st.site = &r_emlrtRSI;
      mtimes(SD->u1.f1.tmp_data, tmp_size, SD->u1.f1.y_data, y_size);
      b_st.site = &vb_emlrtRSI;
      for (i = 0; i < 10; i++) {
        for (i1 = 0; i1 < 6; i1++) {
          for (b_i = 0; b_i < trueCount; b_i++) {
            Jx_data[(b_i + trueCount * i1) + trueCount * 6 * i] =
                Jx[((b_tmp_data[b_i] + 120 * i1) + 720 * i) - 1];
          }
        }
      }
      for (i = 0; i < trueCount; i++) {
        for (i1 = 0; i1 < 60; i1++) {
          Jx[i1 + 60 * i] = Jx_data[i + trueCount * i1];
        }
      }
      k = y_size[0];
      for (i = 0; i < k; i++) {
        for (i1 = 0; i1 < 18; i1++) {
          varargin_2_data[i1 + 18 * i] = SD->u1.f1.y_data[i + y_size[0] * i1];
        }
      }
      c_st.site = &eb_emlrtRSI;
      d_st.site = &fb_emlrtRSI;
      if ((y_size[0] != trueCount) && (y_size[0] != 0)) {
        emlrtErrorWithMessageIdR2018a(
            &d_st, &k_emlrtRTEI, "MATLAB:catenate:matrixDimensionMismatch",
            "MATLAB:catenate:matrixDimensionMismatch", 0);
      }
      if ((trueCount == 0) || (y_size[0] != 0)) {
        input_sizes_idx_0 = 18;
      } else {
        input_sizes_idx_0 = 0;
      }
      k = input_sizes_idx_0;
      for (i = 0; i < trueCount; i++) {
        tmp_data[i] = Je[b_tmp_data[i] - 1];
      }
      Jc_size[0] = input_sizes_idx_0 + 61;
      Jc_size[1] = trueCount;
      for (i = 0; i < trueCount; i++) {
        for (i1 = 0; i1 < 60; i1++) {
          Jc_data[i1 + (input_sizes_idx_0 + 61) * i] = Jx[i1 + 60 * i];
        }
      }
      for (i = 0; i < trueCount; i++) {
        for (i1 = 0; i1 < k; i1++) {
          Jc_data[(i1 + (input_sizes_idx_0 + 61) * i) + 60] =
              varargin_2_data[i1 + input_sizes_idx_0 * i];
        }
      }
      for (i = 0; i < trueCount; i++) {
        Jc_data[(input_sizes_idx_0 + (input_sizes_idx_0 + 61) * i) + 60] =
            tmp_data[i];
      }
    }
  }
}

/* End of code generation (znlmpc_confun.c) */
