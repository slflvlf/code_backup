/*
 * evalObjAndConstrAndDerivatives.c
 *
 * Code generation for function 'evalObjAndConstrAndDerivatives'
 *
 */

/* Include files */
#include "evalObjAndConstrAndDerivatives.h"
#include "checkMatrixNonFinite.h"
#include "checkVectorNonFinite.h"
#include "nlmpcmoveCodeGeneration.h"
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

static emlrtRSInfo kc_emlrtRSI = {
    1,                                /* lineNo */
    "evalObjAndConstrAndDerivatives", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+utils\\+"
    "ObjNonlinEvaluator\\evalObjAndConstrAndDerivatives.p" /* pathName */
};

static emlrtRSInfo lc_emlrtRSI = {
    1,                                    /* lineNo */
    "computeConstraintsAndUserJacobian_", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+utils\\+"
    "ObjNonlinEvaluator\\computeConstraintsAndUserJacobian_.p" /* pathName */
};

static emlrtRSInfo nc_emlrtRSI = {
    55,           /* lineNo */
    "xcopy_blas", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+blas\\xcopy."
    "m" /* pathName */
};

static emlrtBCInfo he_emlrtBCI = {
    1,                    /* iFirst */
    60,                   /* iLast */
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
    80,                   /* iLast */
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
    80,                   /* iLast */
    193,                  /* lineNo */
    35,                   /* colNo */
    "",                   /* aName */
    "quadraticObjective", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_objfun.m", /* pName
                                                                       */
    0 /* checkKind */
};

static emlrtBCInfo ke_emlrtBCI = {
    -1,                                   /* iFirst */
    -1,                                   /* iLast */
    1,                                    /* lineNo */
    1,                                    /* colNo */
    "",                                   /* aName */
    "computeConstraintsAndUserJacobian_", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+utils\\+"
    "ObjNonlinEvaluator\\computeConstraintsAndUserJacobian_.p", /* pName */
    0                                                           /* checkKind */
};

/* Function Definitions */
void evalObjAndConstrAndDerivatives(
    c_nlmpcmoveCodeGenerationStackD *SD, const emlrtStack *sp,
    const real_T c_obj_objfun_workspace_runtimed[6],
    const real_T d_obj_objfun_workspace_runtimed[8],
    const real_T e_obj_objfun_workspace_runtimed[60],
    const real_T f_obj_objfun_workspace_runtimed[60],
    const real_T g_obj_objfun_workspace_runtimed[80],
    const real_T h_obj_objfun_workspace_runtimed[80],
    const real_T i_obj_objfun_workspace_runtimed[80],
    const real_T c_obj_nonlcon_workspace_runtime[6],
    const real_T d_obj_nonlcon_workspace_runtime[60],
    const real_T e_obj_nonlcon_workspace_runtime[60], int32_T obj_mCineq,
    const real_T x[85], real_T grad_workspace_data[],
    const int32_T *grad_workspace_size, real_T Cineq_workspace_data[],
    const int32_T *Cineq_workspace_size, int32_T ineq0,
    real_T Ceq_workspace[60], real_T JacIneqTrans_workspace_data[],
    const int32_T *JacIneqTrans_workspace_size, int32_T iJI_col, int32_T ldJI,
    real_T JacEqTrans_workspace_data[],
    const int32_T *JacEqTrans_workspace_size, int32_T ldJE, real_T *fval,
    int32_T *status)
{
  static const int8_T iv[36] = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
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
  real_T varargout_1_data[120];
  real_T U[88];
  real_T b_U[88];
  real_T c_gfX[85];
  real_T gfU[80];
  real_T X[66];
  real_T b_X[66];
  real_T gfX[60];
  real_T y[24];
  real_T umvk[8];
  real_T alpha1;
  real_T beta1;
  real_T e;
  int32_T b_i;
  int32_T c_i;
  int32_T i;
  int32_T i1;
  int32_T idx_col;
  int32_T idx_row;
  char_T TRANSA1;
  char_T TRANSB1;
  int8_T iu[8];
  int8_T ix[6];
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &kc_emlrtRSI;
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
  znlmpc_getXUe(x, c_obj_objfun_workspace_runtimed, X, U, &e);
  e_st.site = &ec_emlrtRSI;
  memset(&gfX[0], 0, 60U * sizeof(real_T));
  memset(&gfU[0], 0, 80U * sizeof(real_T));
  beta1 = 0.0;
  for (i = 0; i < 6; i++) {
    ix[i] = (int8_T)(i + 1);
  }
  for (i = 0; i < 8; i++) {
    iu[i] = (int8_T)(i + 1);
  }
  for (i = 0; i < 11; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      b_X[i1 + 6 * i] = X[i + 11 * i1];
    }
    for (i1 = 0; i1 < 8; i1++) {
      b_U[i1 + (i << 3)] = U[i + 11 * i1];
    }
  }
  for (b_i = 0; b_i < 10; b_i++) {
    real_T b_gfU[8];
    real_T duk[8];
    real_T b_gfX[6];
    real_T wtYerr[6];
    real_T d;
    real_T d1;
    int8_T i2;
    alpha1 = 0.0;
    for (i = 0; i < 6; i++) {
      i1 = b_i + 10 * i;
      d = f_obj_objfun_workspace_runtimed[i1];
      d1 = d * (b_X[i + 6 * (b_i + 1)] - e_obj_objfun_workspace_runtimed[i1]);
      alpha1 += d1 * d1;
      d1 *= d;
      wtYerr[i] = d1;
    }
    beta1 += alpha1;
    for (i = 0; i < 6; i++) {
      d = 0.0;
      for (i1 = 0; i1 < 6; i1++) {
        d += (real_T)iv[i + 6 * i1] * wtYerr[i1];
      }
      b_gfX[i] = gfX[ix[i] - 1] + d;
    }
    for (i = 0; i < 6; i++) {
      i2 = ix[i];
      if ((i2 < 1) || (i2 > 60)) {
        emlrtDynamicBoundsCheckR2012b(i2, 1, 60, &he_emlrtBCI, &e_st);
      }
      gfX[ix[i] - 1] = b_gfX[i];
    }
    for (i = 0; i < 6; i++) {
      ix[i] = (int8_T)(ix[i] + 6);
    }
    memcpy(&umvk[0], &b_U[b_i * 8], 8U * sizeof(real_T));
    if (b_i + 1 == 1) {
      for (c_i = 0; c_i < 8; c_i++) {
        duk[c_i] = umvk[c_i] - d_obj_objfun_workspace_runtimed[c_i];
      }
    } else {
      for (i = 0; i < 8; i++) {
        duk[i] = umvk[i] - b_U[i + ((b_i - 1) << 3)];
      }
    }
    alpha1 = 0.0;
    for (i = 0; i < 8; i++) {
      i1 = b_i + 10 * i;
      d = g_obj_objfun_workspace_runtimed[i1];
      d1 = d * (umvk[i] - i_obj_objfun_workspace_runtimed[i1]);
      umvk[i] = d1;
      alpha1 += d1 * d1;
      b_gfU[i] = gfU[iu[i] - 1] + d * d1;
    }
    beta1 += alpha1;
    for (i = 0; i < 8; i++) {
      i2 = iu[i];
      if ((i2 < 1) || (i2 > 80)) {
        emlrtDynamicBoundsCheckR2012b(i2, 1, 80, &ie_emlrtBCI, &e_st);
      }
      gfU[iu[i] - 1] = b_gfU[i];
    }
    alpha1 = 0.0;
    for (i = 0; i < 8; i++) {
      d = h_obj_objfun_workspace_runtimed[b_i + 10 * i];
      d1 = d * duk[i];
      alpha1 += d1 * d1;
      d1 *= d;
      duk[i] = d1;
    }
    beta1 += alpha1;
    for (i = 0; i < 8; i++) {
      umvk[i] = gfU[iu[i] - 1] + duk[i];
    }
    for (i = 0; i < 8; i++) {
      gfU[iu[i] - 1] = umvk[i];
    }
    if (b_i + 1 > 1) {
      int8_T gfU_tmp[8];
      for (i = 0; i < 8; i++) {
        i1 = (int8_T)(iu[i] - 8);
        if ((i1 < 1) || (i1 > 80)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, 80, &je_emlrtBCI, &e_st);
        }
        gfU_tmp[i] = (int8_T)i1;
      }
      for (i = 0; i < 8; i++) {
        umvk[i] = gfU[gfU_tmp[i] - 1] - duk[i];
      }
      for (i = 0; i < 8; i++) {
        gfU[gfU_tmp[i] - 1] = umvk[i];
      }
    }
    for (i = 0; i < 8; i++) {
      iu[i] = (int8_T)(iu[i] + 8);
    }
  }
  *fval = beta1 + 100000.0 * e * e;
  for (i = 0; i < 60; i++) {
    gfX[i] *= 2.0;
  }
  for (i = 0; i < 80; i++) {
    gfU[i] *= 2.0;
  }
  TRANSB1 = 'N';
  TRANSA1 = 'T';
  alpha1 = 1.0;
  beta1 = 0.0;
  m_t = (ptrdiff_t)24;
  n_t = (ptrdiff_t)1;
  k_t = (ptrdiff_t)80;
  lda_t = (ptrdiff_t)80;
  ldb_t = (ptrdiff_t)80;
  ldc_t = (ptrdiff_t)24;
  dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &dv[0], &lda_t, &gfU[0],
        &ldb_t, &beta1, &y[0], &ldc_t);
  memcpy(&c_gfX[0], &gfX[0], 60U * sizeof(real_T));
  memcpy(&c_gfX[60], &y[0], 24U * sizeof(real_T));
  c_gfX[84] = 200000.0 * e;
  b_st.site = &bc_emlrtRSI;
  xcopy(&b_st, 85, c_gfX, grad_workspace_data);
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
    *status = checkVectorNonFinite(&b_st, 85, grad_workspace_data,
                                   *grad_workspace_size, 1);
  }
  if (*status == 1) {
    st.site = &kc_emlrtRSI;
    if (obj_mCineq > 0) {
      int32_T varargout_1_size[2];
      int32_T varargout_3_size[2];
      b_st.site = &lc_emlrtRSI;
      c_st.site = &lb_emlrtRSI;
      c_nlmpcmoveCodeGeneration_anonF(
          SD, &c_st, c_obj_nonlcon_workspace_runtime,
          d_obj_nonlcon_workspace_runtime, e_obj_nonlcon_workspace_runtime, x,
          varargout_1_data, varargout_1_size, gfX, SD->u3.f5.varargout_3_data,
          varargout_3_size, SD->u3.f5.varargout_4);
      b_st.site = &lc_emlrtRSI;
      c_st.site = &mc_emlrtRSI;
      d_st.site = &nc_emlrtRSI;
      n_t = (ptrdiff_t)obj_mCineq;
      m_t = (ptrdiff_t)1;
      k_t = (ptrdiff_t)1;
      dcopy(&n_t, &varargout_1_data[0], &m_t, &Cineq_workspace_data[ineq0 - 1],
            &k_t);
      b_st.site = &lc_emlrtRSI;
      c_st.site = &gc_emlrtRSI;
      d_st.site = &hc_emlrtRSI;
      memcpy(&Ceq_workspace[0], &gfX[0], 60U * sizeof(real_T));
      i = varargout_3_size[0];
      b_st.site = &lc_emlrtRSI;
      for (idx_row = 0; idx_row < i; idx_row++) {
        i1 = varargout_3_size[1];
        b_st.site = &lc_emlrtRSI;
        for (idx_col = 0; idx_col < i1; idx_col++) {
          if (idx_row + 1 > varargout_3_size[0]) {
            emlrtDynamicBoundsCheckR2012b(idx_row + 1, 1, varargout_3_size[0],
                                          &ke_emlrtBCI, &st);
          }
          if (idx_col + 1 > varargout_3_size[1]) {
            emlrtDynamicBoundsCheckR2012b(idx_col + 1, 1, varargout_3_size[1],
                                          &ke_emlrtBCI, &st);
          }
          c_i = *JacIneqTrans_workspace_size;
          b_i = (idx_row + ldJI * ((iJI_col + idx_col) - 1)) + 1;
          if ((b_i < 1) || (b_i > c_i)) {
            emlrtDynamicBoundsCheckR2012b(b_i, 1, c_i, &ke_emlrtBCI, &st);
          }
          JacIneqTrans_workspace_data[b_i - 1] =
              SD->u3.f5
                  .varargout_3_data[idx_row + varargout_3_size[0] * idx_col];
        }
      }
      i = *JacEqTrans_workspace_size;
      for (idx_row = 0; idx_row < 85; idx_row++) {
        for (idx_col = 0; idx_col < 60; idx_col++) {
          i1 = (idx_row + ldJE * idx_col) + 1;
          if ((i1 < 1) || (i1 > i)) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, i, &ke_emlrtBCI, &st);
          }
          JacEqTrans_workspace_data[i1 - 1] =
              SD->u3.f5.varargout_4[idx_row + 85 * idx_col];
        }
      }
    } else {
      int32_T varargout_1_size[2];
      int32_T varargout_3_size[2];
      b_st.site = &lc_emlrtRSI;
      c_st.site = &lb_emlrtRSI;
      c_nlmpcmoveCodeGeneration_anonF(
          SD, &c_st, c_obj_nonlcon_workspace_runtime,
          d_obj_nonlcon_workspace_runtime, e_obj_nonlcon_workspace_runtime, x,
          varargout_1_data, varargout_1_size, gfX, SD->u3.f5.varargout_3_data,
          varargout_3_size, SD->u3.f5.varargout_4);
      b_st.site = &lc_emlrtRSI;
      c_st.site = &gc_emlrtRSI;
      d_st.site = &hc_emlrtRSI;
      memcpy(&Ceq_workspace[0], &gfX[0], 60U * sizeof(real_T));
      i = *JacEqTrans_workspace_size;
      for (idx_row = 0; idx_row < 85; idx_row++) {
        for (idx_col = 0; idx_col < 60; idx_col++) {
          i1 = (idx_row + ldJE * idx_col) + 1;
          if ((i1 < 1) || (i1 > i)) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, i, &ke_emlrtBCI, &st);
          }
          JacEqTrans_workspace_data[i1 - 1] =
              SD->u3.f5.varargout_4[idx_row + 85 * idx_col];
        }
      }
    }
    b_st.site = &lc_emlrtRSI;
    *status = checkVectorNonFinite(&b_st, obj_mCineq, Cineq_workspace_data,
                                   *Cineq_workspace_size, ineq0);
    if (*status == 1) {
      b_st.site = &lc_emlrtRSI;
      *status = b_checkVectorNonFinite(&b_st, Ceq_workspace);
      if (*status == 1) {
        b_st.site = &lc_emlrtRSI;
        *status =
            checkMatrixNonFinite(&b_st, obj_mCineq, JacIneqTrans_workspace_data,
                                 *JacIneqTrans_workspace_size, iJI_col, ldJI);
        if (*status == 1) {
          b_st.site = &lc_emlrtRSI;
          *status = checkMatrixNonFinite(&b_st, 60, JacEqTrans_workspace_data,
                                         *JacEqTrans_workspace_size, 1, ldJE);
        }
      }
    }
  }
}

/* End of code generation (evalObjAndConstrAndDerivatives.c) */
