/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * evalObjAndConstrAndDerivatives.c
 *
 * Code generation for function 'evalObjAndConstrAndDerivatives'
 *
 */

/* Include files */
#include "evalObjAndConstrAndDerivatives.h"
#include "checkVectorNonFinite.h"
#include "computeConstraintsAndUserJacobian_.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "xcopy.h"
#include "znlmpc_getXUe.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo bc_emlrtRSI = {
    1,                                  /* lineNo */
    "computeObjectiveAndUserGradient_", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+utils\\+"
    "ObjNonlinEvaluator\\computeObjectiveAndUserGradient_.p" /* pathName */
};

static emlrtRSInfo ec_emlrtRSI =
    {
        28,              /* lineNo */
        "znlmpc_objfun", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_objfun.m" /* pathName
                                                                          */
};

static emlrtRSInfo lc_emlrtRSI = {
    1,                                /* lineNo */
    "evalObjAndConstrAndDerivatives", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+utils\\+"
    "ObjNonlinEvaluator\\evalObjAndConstrAndDerivatives.p" /* pathName */
};

static emlrtBCInfo he_emlrtBCI = {
    1,                    /* iFirst */
    192,                  /* iLast */
    141,                  /* lineNo */
    13,                   /* colNo */
    "",                   /* aName */
    "quadraticObjective", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_objfun.m", /* pName
                                                                       */
    3 /* checkKind */
};

static emlrtBCInfo ie_emlrtBCI = {
    1,                    /* iFirst */
    192,                  /* iLast */
    172,                  /* lineNo */
    13,                   /* colNo */
    "",                   /* aName */
    "quadraticObjective", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_objfun.m", /* pName
                                                                       */
    3 /* checkKind */
};

static emlrtBCInfo je_emlrtBCI = {
    1,                    /* iFirst */
    192,                  /* iLast */
    193,                  /* lineNo */
    35,                   /* colNo */
    "",                   /* aName */
    "quadraticObjective", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_objfun.m", /* pName
                                                                       */
    0 /* checkKind */
};

/* Function Definitions */
void evalObjAndConstrAndDerivatives(
    c_nlmpcmoveCodeGenerationStackD *SD, const emlrtStack *sp,
    const real_T c_obj_objfun_workspace_runtimed[24],
    const real_T d_obj_objfun_workspace_runtimed[24],
    const real_T e_obj_objfun_workspace_runtimed[192],
    const real_T f_obj_objfun_workspace_runtimed[192],
    const real_T g_obj_objfun_workspace_runtimed[192],
    const real_T h_obj_objfun_workspace_runtimed[192],
    const real_T i_obj_objfun_workspace_runtimed[192],
    const real_T c_obj_nonlcon_workspace_runtime[24],
    const real_T d_obj_nonlcon_workspace_runtime[192],
    const real_T e_obj_nonlcon_workspace_runtime[192], int32_T obj_mCineq,
    const real_T x[265], real_T grad_workspace_data[],
    const int32_T *grad_workspace_size, real_T Cineq_workspace_data[],
    const int32_T *Cineq_workspace_size, int32_T ineq0,
    real_T Ceq_workspace[192], real_T JacIneqTrans_workspace_data[],
    const int32_T *JacIneqTrans_workspace_size, int32_T iJI_col, int32_T ldJI,
    real_T JacEqTrans_workspace_data[],
    const int32_T *JacEqTrans_workspace_size, int32_T ldJE, real_T *fval,
    int32_T *status)
{
  static const real_T a[576] = {
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T b_gfX[265];
  real_T U[216];
  real_T X[216];
  real_T b_U[216];
  real_T gfU[192];
  real_T gfX[192];
  real_T y[72];
  real_T duk[24];
  real_T wtYerr[24];
  real_T alpha1;
  real_T beta1;
  real_T e;
  real_T fs;
  int32_T b_i;
  int32_T c_i;
  int32_T i;
  char_T TRANSA1;
  char_T TRANSB1;
  uint8_T iu[24];
  uint8_T ix[24];
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &lc_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  b_st.site = &bc_emlrtRSI;
  c_st.site = &lb_emlrtRSI;
  d_st.site = &cc_emlrtRSI;
  znlmpc_getXUe(x, c_obj_objfun_workspace_runtimed, U, b_U, &e);
  e_st.site = &ec_emlrtRSI;
  memset(&gfX[0], 0, 192U * sizeof(real_T));
  memset(&gfU[0], 0, 192U * sizeof(real_T));
  fs = 0.0;
  for (i = 0; i < 24; i++) {
    ix[i] = (uint8_T)(i + 1U);
    iu[i] = (uint8_T)(i + 1U);
  }
  for (i = 0; i < 9; i++) {
    for (b_i = 0; b_i < 24; b_i++) {
      X[b_i + 24 * i] = U[i + 9 * b_i];
    }
  }
  for (i = 0; i < 9; i++) {
    for (b_i = 0; b_i < 24; b_i++) {
      U[b_i + 24 * i] = b_U[i + 9 * b_i];
    }
  }
  TRANSB1 = 'N';
  TRANSA1 = 'T';
  alpha1 = 1.0;
  beta1 = 0.0;
  m_t = (ptrdiff_t)24;
  n_t = (ptrdiff_t)1;
  k_t = (ptrdiff_t)24;
  lda_t = (ptrdiff_t)24;
  ldb_t = (ptrdiff_t)24;
  ldc_t = (ptrdiff_t)24;
  for (c_i = 0; c_i < 8; c_i++) {
    real_T b_gfU[24];
    real_T b_wtYerr;
    real_T d;
    real_T d1;
    uint8_T u;
    b_wtYerr = 0.0;
    for (i = 0; i < 24; i++) {
      b_i = c_i + (i << 3);
      d = f_obj_objfun_workspace_runtimed[b_i];
      d1 = d * (X[i + 24 * (c_i + 1)] - e_obj_objfun_workspace_runtimed[b_i]);
      b_wtYerr += d1 * d1;
      d1 *= d;
      wtYerr[i] = d1;
    }
    fs += b_wtYerr;
    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &a[0], &lda_t,
          &wtYerr[0], &ldb_t, &beta1, &duk[0], &ldc_t);
    for (i = 0; i < 24; i++) {
      wtYerr[i] = gfX[ix[i] - 1] + duk[i];
    }
    for (i = 0; i < 24; i++) {
      u = ix[i];
      if ((u < 1) || (u > 192)) {
        emlrtDynamicBoundsCheckR2012b(u, 1, 192, &he_emlrtBCI, &e_st);
      }
      gfX[ix[i] - 1] = wtYerr[i];
    }
    for (i = 0; i < 24; i++) {
      ix[i] = (uint8_T)(ix[i] + 24U);
      wtYerr[i] = U[i + 24 * c_i];
    }
    if (c_i + 1 == 1) {
      for (i = 0; i < 24; i++) {
        duk[i] = wtYerr[i] - d_obj_objfun_workspace_runtimed[i];
      }
    } else {
      for (i = 0; i < 24; i++) {
        duk[i] = wtYerr[i] - U[i + 24 * (c_i - 1)];
      }
    }
    b_wtYerr = 0.0;
    for (i = 0; i < 24; i++) {
      b_i = c_i + (i << 3);
      d = g_obj_objfun_workspace_runtimed[b_i];
      d1 = d * (wtYerr[i] - i_obj_objfun_workspace_runtimed[b_i]);
      wtYerr[i] = d1;
      b_wtYerr += d1 * d1;
      b_gfU[i] = gfU[iu[i] - 1] + d * d1;
    }
    fs += b_wtYerr;
    for (i = 0; i < 24; i++) {
      u = iu[i];
      if ((u < 1) || (u > 192)) {
        emlrtDynamicBoundsCheckR2012b(u, 1, 192, &ie_emlrtBCI, &e_st);
      }
      gfU[iu[i] - 1] = b_gfU[i];
    }
    b_wtYerr = 0.0;
    for (i = 0; i < 24; i++) {
      d = h_obj_objfun_workspace_runtimed[c_i + (i << 3)];
      d1 = d * duk[i];
      b_wtYerr += d1 * d1;
      d1 *= d;
      duk[i] = d1;
    }
    fs += b_wtYerr;
    for (i = 0; i < 24; i++) {
      wtYerr[i] = gfU[iu[i] - 1] + duk[i];
    }
    for (i = 0; i < 24; i++) {
      gfU[iu[i] - 1] = wtYerr[i];
    }
    if (c_i + 1 > 1) {
      uint8_T gfU_tmp[24];
      for (i = 0; i < 24; i++) {
        b_i = (uint8_T)(iu[i] - 24);
        if ((b_i < 1) || (b_i > 192)) {
          emlrtDynamicBoundsCheckR2012b(b_i, 1, 192, &je_emlrtBCI, &e_st);
        }
        gfU_tmp[i] = (uint8_T)b_i;
      }
      for (i = 0; i < 24; i++) {
        wtYerr[i] = gfU[gfU_tmp[i] - 1] - duk[i];
      }
      for (i = 0; i < 24; i++) {
        gfU[gfU_tmp[i] - 1] = wtYerr[i];
      }
    }
    for (i = 0; i < 24; i++) {
      iu[i] = (uint8_T)(iu[i] + 24U);
    }
  }
  *fval = fs + 100000.0 * e * e;
  for (i = 0; i < 192; i++) {
    gfX[i] *= 2.0;
    gfU[i] *= 2.0;
  }
  TRANSB1 = 'N';
  TRANSA1 = 'T';
  alpha1 = 1.0;
  beta1 = 0.0;
  m_t = (ptrdiff_t)72;
  n_t = (ptrdiff_t)1;
  k_t = (ptrdiff_t)192;
  lda_t = (ptrdiff_t)192;
  ldb_t = (ptrdiff_t)192;
  ldc_t = (ptrdiff_t)72;
  dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &dv[0], &lda_t, &gfU[0],
        &ldb_t, &beta1, &y[0], &ldc_t);
  memcpy(&b_gfX[0], &gfX[0], 192U * sizeof(real_T));
  memcpy(&b_gfX[192], &y[0], 72U * sizeof(real_T));
  b_gfX[264] = 200000.0 * e;
  b_st.site = &bc_emlrtRSI;
  xcopy(&b_st, 265, b_gfX, grad_workspace_data);
  if (muDoubleScalarIsInf(*fval) || muDoubleScalarIsNaN(*fval)) {
    if (muDoubleScalarIsNaN(*fval)) {
      *status = -3;
    } else if (*fval < 0.0) {
      *status = -1;
    } else {
      *status = -2;
    }
  } else {
    b_st.site = &bc_emlrtRSI;
    *status = checkVectorNonFinite(&b_st, 265, grad_workspace_data,
                                   *grad_workspace_size, 1);
  }
  if (*status == 1) {
    st.site = &lc_emlrtRSI;
    *status = c_computeConstraintsAndUserJaco(
        SD, &st, c_obj_nonlcon_workspace_runtime,
        d_obj_nonlcon_workspace_runtime, e_obj_nonlcon_workspace_runtime,
        obj_mCineq, x, Cineq_workspace_data, Cineq_workspace_size, ineq0,
        Ceq_workspace, JacIneqTrans_workspace_data, JacIneqTrans_workspace_size,
        iJI_col, ldJI, JacEqTrans_workspace_data, JacEqTrans_workspace_size,
        ldJE);
  }
}

/* End of code generation (evalObjAndConstrAndDerivatives.c) */
