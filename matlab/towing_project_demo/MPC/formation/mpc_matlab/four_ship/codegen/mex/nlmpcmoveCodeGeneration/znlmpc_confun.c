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
#include "FourShipStateFcn.h"
#include "FourShipStateJacobianFcn.h"
#include "all.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "znlmpc_reformJacobian.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo xb_emlrtRSI =
    {
        184,            /* lineNo */
        "outputBounds", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_confun.m" /* pathName
                                                                          */
};

static emlrtBCInfo y_emlrtBCI = {
    1,              /* iFirst */
    384,            /* iLast */
    169,            /* lineNo */
    17,             /* colNo */
    "",             /* aName */
    "outputBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_confun.m", /* pName
                                                                       */
    3 /* checkKind */
};

static emlrtBCInfo ab_emlrtBCI = {
    1,              /* iFirst */
    384,            /* iLast */
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
                  const real_T runtimedata_OutputMin[192],
                  const real_T runtimedata_OutputMax[192], const real_T X[216],
                  real_T e, real_T c_data[], int32_T c_size[2],
                  real_T Jc_data[], int32_T Jc_size[2])
{
  emlrtStack st;
  real_T b_c_data[384];
  int32_T Jx_size[3];
  int32_T tmp_size[3];
  int32_T b_i;
  int32_T i;
  int32_T i2;
  int32_T k;
  int16_T b_tmp_data[384];
  int16_T tmp_data[384];
  int8_T Jx[73728];
  int8_T Ck[576];
  int8_T Je[384];
  boolean_T bv[192];
  boolean_T x[24];
  boolean_T exitg1;
  boolean_T guard1 = false;
  boolean_T y;
  st.prev = sp;
  st.tls = sp->tls;
  for (i = 0; i < 192; i++) {
    bv[i] = muDoubleScalarIsInf(runtimedata_OutputMin[i]);
  }
  st.site = &wb_emlrtRSI;
  b_all(&st, bv, x);
  y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 24)) {
    if (!x[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  guard1 = false;
  if (y) {
    for (i = 0; i < 192; i++) {
      bv[i] = muDoubleScalarIsInf(runtimedata_OutputMax[i]);
    }
    st.site = &wb_emlrtRSI;
    b_all(&st, bv, x);
    y = true;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 24)) {
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
    real_T c[384];
    int32_T trueCount;
    int16_T ic[24];
    boolean_T icf[384];
    for (b_i = 0; b_i < 384; b_i++) {
      c[b_i] = 0.0;
      icf[b_i] = true;
    }
    memset(&Jx[0], 0, 73728U * sizeof(int8_T));
    memset(&Je[0], 0, 384U * sizeof(int8_T));
    for (i = 0; i < 24; i++) {
      ic[i] = (int16_T)(i + 1);
    }
    for (b_i = 0; b_i < 8; b_i++) {
      real_T d;
      int16_T i1;
      boolean_T bv1[24];
      for (i = 0; i < 24; i++) {
        d = runtimedata_OutputMin[b_i + (i << 3)];
        x[i] = muDoubleScalarIsInf(d);
        bv1[i] = muDoubleScalarIsNaN(d);
      }
      for (i = 0; i < 24; i++) {
        i1 = ic[i];
        if ((i1 < 1) || (i1 > 384)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, 384, &u_emlrtBCI, (emlrtCTX)sp);
        }
        icf[ic[i] - 1] = ((!x[i]) && (!bv1[i]));
      }
      for (i = 0; i < 24; i++) {
        d = runtimedata_OutputMax[b_i + (i << 3)];
        x[i] = muDoubleScalarIsInf(d);
        bv1[i] = muDoubleScalarIsNaN(d);
      }
      for (i = 0; i < 24; i++) {
        i2 = ic[i] + 24;
        if ((i2 < 1) || (i2 > 384)) {
          emlrtDynamicBoundsCheckR2012b(i2, 1, 384, &v_emlrtBCI, (emlrtCTX)sp);
        }
        icf[i2 - 1] = ((!x[i]) && (!bv1[i]));
      }
      for (i = 0; i < 24; i++) {
        i2 = ic[i];
        if ((i2 < 1) || (i2 > 384)) {
          emlrtDynamicBoundsCheckR2012b(i2, 1, 384, &w_emlrtBCI, (emlrtCTX)sp);
        }
      }
      for (i = 0; i < 24; i++) {
        i2 = (int16_T)(ic[i] + 24);
        if ((i2 < 1) || (i2 > 384)) {
          emlrtDynamicBoundsCheckR2012b(i2, 1, 384, &w_emlrtBCI, (emlrtCTX)sp);
        }
      }
      y = false;
      k = 0;
      exitg1 = false;
      while ((!exitg1) && (k <= 47)) {
        int32_T b_ic[48];
        for (i = 0; i < 24; i++) {
          i1 = ic[i];
          b_ic[i] = i1 - 1;
          b_ic[i + 24] = i1 + 23;
        }
        if (icf[b_ic[k]]) {
          y = true;
          exitg1 = true;
        } else {
          k++;
        }
      }
      if (y) {
        int8_T val[576];
        memset(&Ck[0], 0, 576U * sizeof(int8_T));
        for (k = 0; k < 24; k++) {
          Ck[k + 24 * k] = 1;
          c[ic[k] - 1] = (runtimedata_OutputMin[b_i + (k << 3)] - e) -
                         X[(b_i + 9 * k) + 1];
        }
        for (i = 0; i < 24; i++) {
          i2 = ic[i] + 24;
          if ((i2 < 1) || (i2 > 384)) {
            emlrtDynamicBoundsCheckR2012b(i2, 1, 384, &x_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          c[i2 - 1] =
              (X[(b_i + 9 * i) + 1] - runtimedata_OutputMax[b_i + (i << 3)]) -
              e;
        }
        for (i = 0; i < 576; i++) {
          val[i] = (int8_T)-Ck[i];
        }
        for (k = 0; k < 24; k++) {
          for (i = 0; i < 24; i++) {
            Jx[((ic[i] + 384 * k) + 9216 * b_i) - 1] = val[i + 24 * k];
          }
        }
        for (k = 0; k < 24; k++) {
          for (i = 0; i < 24; i++) {
            i2 = ic[i] + 24;
            if ((i2 < 1) || (i2 > 384)) {
              emlrtDynamicBoundsCheckR2012b(i2, 1, 384, &y_emlrtBCI,
                                            (emlrtCTX)sp);
            }
            Jx[((i2 + 384 * k) + 9216 * b_i) - 1] = Ck[i + 24 * k];
          }
        }
        for (i = 0; i < 24; i++) {
          Je[ic[i] - 1] = -1;
        }
        for (i = 0; i < 24; i++) {
          i2 = ic[i] + 24;
          if ((i2 < 1) || (i2 > 384)) {
            emlrtDynamicBoundsCheckR2012b(i2, 1, 384, &ab_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          Je[i2 - 1] = -1;
        }
      }
      for (i = 0; i < 24; i++) {
        ic[i] = (int16_T)(ic[i] + 48);
      }
    }
    trueCount = 0;
    k = 0;
    for (b_i = 0; b_i < 384; b_i++) {
      if (icf[b_i]) {
        trueCount++;
        tmp_data[k] = (int16_T)(b_i + 1);
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
    for (b_i = 0; b_i < 384; b_i++) {
      if (icf[b_i]) {
        trueCount++;
        b_tmp_data[k] = (int16_T)(b_i + 1);
        k++;
      }
    }
    Jx_size[0] = trueCount;
    Jx_size[1] = 24;
    Jx_size[2] = 8;
    tmp_size[0] = trueCount;
    tmp_size[1] = 24;
    tmp_size[2] = 8;
    for (i = 0; i < 8; i++) {
      for (i2 = 0; i2 < 24; i2++) {
        for (k = 0; k < trueCount; k++) {
          SD->u2.f6.Jx_data[(k + trueCount * i2) + trueCount * 24 * i] =
              Jx[((b_tmp_data[k] + 384 * i2) + 9216 * i) - 1];
          SD->u2.f6.tmp_data[(k + trueCount * i2) + trueCount * 24 * i] = 0.0;
        }
      }
    }
    for (i = 0; i < trueCount; i++) {
      b_c_data[i] = Je[b_tmp_data[i] - 1];
    }
    st.site = &xb_emlrtRSI;
    znlmpc_reformJacobian(SD, &st, SD->u2.f6.Jx_data, Jx_size,
                          SD->u2.f6.tmp_data, tmp_size, b_c_data, trueCount,
                          Jc_data, Jc_size);
  }
}

void stateEvolution(c_nlmpcmoveCodeGenerationStackD *SD, const emlrtStack *sp,
                    const real_T X[216], const real_T U[216], real_T c[192],
                    real_T J[50880])
{
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T b_U[216];
  real_T b_X[216];
  real_T alpha1;
  real_T beta1;
  int32_T U_tmp;
  int32_T b_U_tmp;
  int32_T b_i;
  int32_T i;
  int32_T i1;
  char_T TRANSA1;
  char_T TRANSB1;
  uint8_T ic[24];
  memset(&SD->u1.f2.Jx[0], 0, 36864U * sizeof(real_T));
  memset(&SD->u1.f2.Jmv[0], 0, 36864U * sizeof(real_T));
  memset(&c[0], 0, 192U * sizeof(real_T));
  for (i = 0; i < 24; i++) {
    ic[i] = (uint8_T)(i + 1U);
  }
  for (i = 0; i < 9; i++) {
    for (i1 = 0; i1 < 24; i1++) {
      U_tmp = i + 9 * i1;
      b_U_tmp = i1 + 24 * i;
      b_U[b_U_tmp] = U[U_tmp];
      b_X[b_U_tmp] = X[U_tmp];
    }
  }
  for (b_i = 0; b_i < 8; b_i++) {
    real_T Ak[576];
    real_T Ak1[576];
    real_T Bk1[576];
    real_T val[576];
    real_T b_dv[24];
    real_T dv1[24];
    FourShipStateJacobianFcn(*(real_T(*)[24]) & b_X[24 * b_i],
                             *(real_T(*)[24]) & b_U[24 * b_i], Ak, val);
    FourShipStateJacobianFcn(*(real_T(*)[24]) & b_X[24 * (b_i + 1)],
                             *(real_T(*)[24]) & b_U[24 * b_i], Ak1, Bk1);
    FourShipStateFcn(*(real_T(*)[24]) & b_X[24 * b_i],
                     *(real_T(*)[24]) & b_U[24 * b_i], b_dv);
    i = 24 * (b_i + 1);
    FourShipStateFcn(*(real_T(*)[24]) & b_X[i],
                     *(real_T(*)[24]) & b_U[24 * b_i], dv1);
    for (i1 = 0; i1 < 24; i1++) {
      uint8_T u;
      u = ic[i1];
      if ((u < 1) || (u > 192)) {
        emlrtDynamicBoundsCheckR2012b(u, 1, 192, &t_emlrtBCI, (emlrtCTX)sp);
      }
      c[ic[i1] - 1] =
          (b_X[i1 + 24 * b_i] + 0.5 * (b_dv[i1] + dv1[i1])) - b_X[i1 + i];
    }
    if (b_i + 1 > 1) {
      for (b_U_tmp = 0; b_U_tmp < 24; b_U_tmp++) {
        for (i = 0; i < 24; i++) {
          SD->u1.f2.Jx[((ic[i] + 192 * b_U_tmp) + 4608 * (b_i - 1)) - 1] =
              0.5 * Ak[i + 24 * b_U_tmp];
        }
        U_tmp = ((ic[b_U_tmp] + 192 * b_U_tmp) + 4608 * (b_i - 1)) - 1;
        SD->u1.f2.Jx[U_tmp]++;
      }
    }
    for (b_U_tmp = 0; b_U_tmp < 24; b_U_tmp++) {
      for (i = 0; i < 24; i++) {
        SD->u1.f2.Jx[((ic[i] + 192 * b_U_tmp) + 4608 * b_i) - 1] =
            0.5 * Ak1[i + 24 * b_U_tmp];
      }
      U_tmp = ((ic[b_U_tmp] + 192 * b_U_tmp) + 4608 * b_i) - 1;
      SD->u1.f2.Jx[U_tmp]--;
    }
    for (i = 0; i < 576; i++) {
      val[i] = 0.5 * (val[i] + Bk1[i]);
    }
    for (b_U_tmp = 0; b_U_tmp < 24; b_U_tmp++) {
      for (i = 0; i < 24; i++) {
        SD->u1.f2.Jmv[((ic[i] + 192 * b_U_tmp) + 4608 * b_i) - 1] =
            val[i + 24 * b_U_tmp];
      }
    }
    for (i = 0; i < 24; i++) {
      ic[i] = (uint8_T)(ic[i] + 24U);
    }
  }
  TRANSB1 = 'N';
  TRANSA1 = 'N';
  alpha1 = 1.0;
  beta1 = 0.0;
  m_t = (ptrdiff_t)192;
  n_t = (ptrdiff_t)72;
  k_t = (ptrdiff_t)192;
  lda_t = (ptrdiff_t)192;
  ldb_t = (ptrdiff_t)192;
  ldc_t = (ptrdiff_t)192;
  dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &SD->u1.f2.Jmv[0],
        &lda_t, &dv[0], &ldb_t, &beta1, &SD->u1.f2.y[0], &ldc_t);
  for (i = 0; i < 192; i++) {
    for (i1 = 0; i1 < 192; i1++) {
      J[i1 + 265 * i] = SD->u1.f2.Jx[i + 192 * i1];
    }
    for (i1 = 0; i1 < 72; i1++) {
      J[(i1 + 265 * i) + 192] = SD->u1.f2.y[i + 192 * i1];
    }
    J[265 * i + 264] = 0.0;
  }
}

/* End of code generation (znlmpc_confun.c) */
