/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * evalObjAndConstr.c
 *
 * Code generation for function 'evalObjAndConstr'
 *
 */

/* Include files */
#include "evalObjAndConstr.h"
#include "FourShipStateFcn.h"
#include "all.h"
#include "checkVectorNonFinite.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "rt_nonfinite.h"
#include "znlmpc_getXUe.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo dc_emlrtRSI =
    {
        19,              /* lineNo */
        "znlmpc_objfun", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_objfun.m" /* pathName
                                                                          */
};

static emlrtRSInfo xh_emlrtRSI = {
    1,                  /* lineNo */
    "evalObjAndConstr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+utils\\+"
    "ObjNonlinEvaluator\\evalObjAndConstr.p" /* pathName */
};

static emlrtRSInfo yh_emlrtRSI = {
    1,                   /* lineNo */
    "computeObjective_", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+utils\\+"
    "ObjNonlinEvaluator\\computeObjective_.p" /* pathName */
};

static emlrtRSInfo ai_emlrtRSI = {
    1,                     /* lineNo */
    "computeConstraints_", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+utils\\+"
    "ObjNonlinEvaluator\\computeConstraints_.p" /* pathName */
};

static emlrtECInfo emlrtECI = {
    -1,                    /* nDims */
    1,                     /* lineNo */
    1,                     /* colNo */
    "computeConstraints_", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+utils\\+"
    "ObjNonlinEvaluator\\computeConstraints_.p" /* pName */
};

static emlrtBCInfo oe_emlrtBCI = {
    -1,                    /* iFirst */
    -1,                    /* iLast */
    1,                     /* lineNo */
    1,                     /* colNo */
    "",                    /* aName */
    "computeConstraints_", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+utils\\+"
    "ObjNonlinEvaluator\\computeConstraints_.p", /* pName */
    0                                            /* checkKind */
};

/* Function Definitions */
void evalObjAndConstr(const emlrtStack *sp,
                      const real_T c_obj_objfun_workspace_runtimed[24],
                      const real_T d_obj_objfun_workspace_runtimed[24],
                      const real_T e_obj_objfun_workspace_runtimed[192],
                      const real_T f_obj_objfun_workspace_runtimed[192],
                      const real_T g_obj_objfun_workspace_runtimed[192],
                      const real_T h_obj_objfun_workspace_runtimed[192],
                      const real_T i_obj_objfun_workspace_runtimed[192],
                      const real_T c_obj_nonlcon_workspace_runtime[24],
                      const real_T d_obj_nonlcon_workspace_runtime[192],
                      const real_T e_obj_nonlcon_workspace_runtime[192],
                      int32_T obj_mCineq, const real_T x[265],
                      real_T Cineq_workspace_data[],
                      const int32_T *Cineq_workspace_size, int32_T ineq0,
                      real_T Ceq_workspace[192], real_T *fval, int32_T *status)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack st;
  real_T b_c_data[384];
  real_T c[384];
  real_T c_data[384];
  real_T U[216];
  real_T X[216];
  real_T b_U[216];
  real_T b_X[216];
  real_T reshapes_f1[192];
  real_T b_wtYerr[24];
  real_T duk[24];
  real_T d;
  real_T e;
  real_T fs;
  int32_T X_tmp;
  int32_T b_i;
  int32_T c_i;
  int32_T i;
  int32_T i1;
  int16_T tmp_data[384];
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &xh_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  b_st.site = &yh_emlrtRSI;
  c_st.site = &lb_emlrtRSI;
  d_st.site = &cc_emlrtRSI;
  e_st.site = &dc_emlrtRSI;
  znlmpc_getXUe(x, c_obj_objfun_workspace_runtimed, X, U, &e);
  fs = 0.0;
  for (i = 0; i < 9; i++) {
    for (i1 = 0; i1 < 24; i1++) {
      c_i = i + 9 * i1;
      X_tmp = i1 + 24 * i;
      b_X[X_tmp] = X[c_i];
      b_U[X_tmp] = U[c_i];
    }
  }
  for (b_i = 0; b_i < 8; b_i++) {
    real_T b_duk;
    real_T wtYerr;
    wtYerr = 0.0;
    for (i = 0; i < 24; i++) {
      i1 = b_i + (i << 3);
      d = f_obj_objfun_workspace_runtimed[i1] *
          (b_X[i + 24 * (b_i + 1)] - e_obj_objfun_workspace_runtimed[i1]);
      wtYerr += d * d;
      b_wtYerr[i] = b_U[i + 24 * b_i];
    }
    fs += wtYerr;
    if (b_i + 1 == 1) {
      for (c_i = 0; c_i < 24; c_i++) {
        duk[c_i] = b_wtYerr[c_i] - d_obj_objfun_workspace_runtimed[c_i];
      }
    } else {
      for (i = 0; i < 24; i++) {
        duk[i] = b_wtYerr[i] - b_U[i + 24 * (b_i - 1)];
      }
    }
    wtYerr = 0.0;
    b_duk = 0.0;
    for (i = 0; i < 24; i++) {
      i1 = b_i + (i << 3);
      d = g_obj_objfun_workspace_runtimed[i1] *
          (b_wtYerr[i] - i_obj_objfun_workspace_runtimed[i1]);
      b_wtYerr[i] = d;
      wtYerr += d * d;
      d = h_obj_objfun_workspace_runtimed[i1] * duk[i];
      duk[i] = d;
      b_duk += d * d;
    }
    fs += wtYerr;
    fs += b_duk;
  }
  *fval = fs + 100000.0 * e * e;
  *status = 1;
  if (muDoubleScalarIsInf(*fval) || muDoubleScalarIsNaN(*fval)) {
    if (muDoubleScalarIsNaN(*fval)) {
      *status = -3;
    } else if (*fval < 0.0) {
      *status = -1;
    } else {
      *status = -2;
    }
  }
  if (*status == 1) {
    int32_T ineqEnd;
    int16_T input_sizes_idx_0;
    int8_T sizes_idx_1;
    boolean_T bv[192];
    boolean_T b_x[24];
    boolean_T exitg1;
    boolean_T guard1 = false;
    boolean_T y;
    st.site = &xh_emlrtRSI;
    ineqEnd = (ineq0 + obj_mCineq) - 1;
    b_st.site = &ai_emlrtRSI;
    c_st.site = &lb_emlrtRSI;
    d_st.site = &mb_emlrtRSI;
    e_st.site = &nb_emlrtRSI;
    znlmpc_getXUe(x, c_obj_nonlcon_workspace_runtime, X, U, &e);
    e_st.site = &ob_emlrtRSI;
    memset(&reshapes_f1[0], 0, 192U * sizeof(real_T));
    for (i = 0; i < 24; i++) {
      b_wtYerr[i] = (real_T)i + 1.0;
    }
    for (i = 0; i < 9; i++) {
      for (i1 = 0; i1 < 24; i1++) {
        c_i = i + 9 * i1;
        X_tmp = i1 + 24 * i;
        b_U[X_tmp] = U[c_i];
        b_X[X_tmp] = X[c_i];
      }
    }
    for (b_i = 0; b_i < 8; b_i++) {
      real_T b_dv[24];
      FourShipStateFcn(*(real_T(*)[24]) & b_X[24 * b_i],
                       *(real_T(*)[24]) & b_U[24 * b_i], duk);
      i = 24 * (b_i + 1);
      FourShipStateFcn(*(real_T(*)[24]) & b_X[i],
                       *(real_T(*)[24]) & b_U[24 * b_i], b_dv);
      for (i1 = 0; i1 < 24; i1++) {
        d = b_wtYerr[i1];
        if (((int32_T)d < 1) || ((int32_T)d > 192)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, 192, &t_emlrtBCI, &e_st);
        }
        reshapes_f1[(int32_T)d - 1] =
            (b_X[i1 + 24 * b_i] + 0.5 * (duk[i1] + b_dv[i1])) - b_X[i1 + i];
      }
      for (i = 0; i < 24; i++) {
        b_wtYerr[i] += 24.0;
      }
    }
    e_st.site = &pb_emlrtRSI;
    for (i = 0; i < 192; i++) {
      bv[i] = muDoubleScalarIsInf(d_obj_nonlcon_workspace_runtime[i]);
    }
    f_st.site = &wb_emlrtRSI;
    b_all(&f_st, bv, b_x);
    y = true;
    c_i = 0;
    exitg1 = false;
    while ((!exitg1) && (c_i < 24)) {
      if (!b_x[c_i]) {
        y = false;
        exitg1 = true;
      } else {
        c_i++;
      }
    }
    guard1 = false;
    if (y) {
      for (i = 0; i < 192; i++) {
        bv[i] = muDoubleScalarIsInf(e_obj_nonlcon_workspace_runtime[i]);
      }
      f_st.site = &wb_emlrtRSI;
      b_all(&f_st, bv, b_x);
      y = true;
      c_i = 0;
      exitg1 = false;
      while ((!exitg1) && (c_i < 24)) {
        if (!b_x[c_i]) {
          y = false;
          exitg1 = true;
        } else {
          c_i++;
        }
      }
      if (y) {
        X_tmp = 0;
        c_i = 0;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      boolean_T icf[384];
      for (b_i = 0; b_i < 384; b_i++) {
        c[b_i] = 0.0;
        icf[b_i] = true;
      }
      for (i = 0; i < 24; i++) {
        b_wtYerr[i] = (real_T)i + 1.0;
      }
      for (b_i = 0; b_i < 8; b_i++) {
        boolean_T bv1[24];
        for (i = 0; i < 24; i++) {
          d = d_obj_nonlcon_workspace_runtime[b_i + (i << 3)];
          b_x[i] = muDoubleScalarIsInf(d);
          bv1[i] = muDoubleScalarIsNaN(d);
        }
        for (i = 0; i < 24; i++) {
          d = b_wtYerr[i];
          if (((int32_T)d < 1) || ((int32_T)d > 384)) {
            emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, 384, &u_emlrtBCI,
                                          &e_st);
          }
          icf[(int32_T)d - 1] = ((!b_x[i]) && (!bv1[i]));
        }
        for (i = 0; i < 24; i++) {
          d = e_obj_nonlcon_workspace_runtime[b_i + (i << 3)];
          b_x[i] = muDoubleScalarIsInf(d);
          bv1[i] = muDoubleScalarIsNaN(d);
        }
        for (i = 0; i < 24; i++) {
          i1 = (int32_T)(b_wtYerr[i] + 24.0);
          if ((i1 < 1) || (i1 > 384)) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, 384, &v_emlrtBCI, &e_st);
          }
          icf[i1 - 1] = ((!b_x[i]) && (!bv1[i]));
        }
        for (i = 0; i < 24; i++) {
          i1 = (int32_T)b_wtYerr[i];
          if ((i1 < 1) || (i1 > 384)) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, 384, &w_emlrtBCI, &e_st);
          }
        }
        for (i = 0; i < 24; i++) {
          i1 = (int32_T)(b_wtYerr[i] + 24.0);
          if ((i1 < 1) || (i1 > 384)) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, 384, &w_emlrtBCI, &e_st);
          }
        }
        y = false;
        c_i = 0;
        exitg1 = false;
        while ((!exitg1) && (c_i <= 47)) {
          int32_T c_wtYerr[48];
          for (i = 0; i < 24; i++) {
            d = b_wtYerr[i];
            c_wtYerr[i] = (int32_T)d - 1;
            c_wtYerr[i + 24] = (int32_T)(d + 24.0) - 1;
          }
          if (icf[c_wtYerr[c_i]]) {
            y = true;
            exitg1 = true;
          } else {
            c_i++;
          }
        }
        if (y) {
          for (i = 0; i < 24; i++) {
            c[(int32_T)b_wtYerr[i] - 1] =
                (d_obj_nonlcon_workspace_runtime[b_i + (i << 3)] - e) -
                X[(b_i + 9 * i) + 1];
          }
          for (i = 0; i < 24; i++) {
            i1 = (int32_T)(b_wtYerr[i] + 24.0);
            if ((i1 < 1) || (i1 > 384)) {
              emlrtDynamicBoundsCheckR2012b(i1, 1, 384, &x_emlrtBCI, &e_st);
            }
            c[i1 - 1] = (X[(b_i + 9 * i) + 1] -
                         e_obj_nonlcon_workspace_runtime[b_i + (i << 3)]) -
                        e;
          }
        }
        for (i = 0; i < 24; i++) {
          b_wtYerr[i] += 48.0;
        }
      }
      X_tmp = 0;
      c_i = 0;
      for (b_i = 0; b_i < 384; b_i++) {
        if (icf[b_i]) {
          X_tmp++;
          tmp_data[c_i] = (int16_T)(b_i + 1);
          c_i++;
        }
      }
      for (i = 0; i < X_tmp; i++) {
        b_c_data[i] = c[tmp_data[i] - 1];
      }
      c_i = 1;
      if (X_tmp - 1 >= 0) {
        memcpy(&c_data[0], &b_c_data[0], X_tmp * sizeof(real_T));
      }
    }
    e_st.site = &qb_emlrtRSI;
    f_st.site = &eb_emlrtRSI;
    sizes_idx_1 = (int8_T)((X_tmp != 0) && (c_i != 0));
    g_st.site = &fb_emlrtRSI;
    if ((c_i != sizes_idx_1) && ((X_tmp != 0) && (c_i != 0))) {
      emlrtErrorWithMessageIdR2018a(
          &g_st, &d_emlrtRTEI, "MATLAB:catenate:matrixDimensionMismatch",
          "MATLAB:catenate:matrixDimensionMismatch", 0);
    }
    if ((sizes_idx_1 == 0) || ((X_tmp != 0) && (c_i != 0))) {
      input_sizes_idx_0 = (int16_T)X_tmp;
    } else {
      input_sizes_idx_0 = 0;
    }
    c_i = input_sizes_idx_0;
    X_tmp = sizes_idx_1;
    if ((X_tmp - 1 >= 0) && (c_i - 1 >= 0)) {
      memcpy(&c[0], &c_data[0], c_i * sizeof(real_T));
    }
    if (ineq0 > ineqEnd) {
      i = -1;
      i1 = -1;
    } else {
      i = *Cineq_workspace_size;
      if ((ineq0 < 1) || (ineq0 > i)) {
        emlrtDynamicBoundsCheckR2012b(ineq0, 1, i, &oe_emlrtBCI, &st);
      }
      i = ineq0 - 2;
      i1 = *Cineq_workspace_size;
      if ((ineqEnd < 1) || (ineqEnd > i1)) {
        emlrtDynamicBoundsCheckR2012b(ineqEnd, 1, i1, &oe_emlrtBCI, &st);
      }
      i1 = ineqEnd - 1;
    }
    X_tmp = i1 - i;
    i1 = input_sizes_idx_0 * sizes_idx_1;
    if (X_tmp != i1) {
      emlrtSubAssignSizeCheck1dR2017a(X_tmp, i1, &emlrtECI, &st);
    }
    for (i1 = 0; i1 < X_tmp; i1++) {
      Cineq_workspace_data[(i + i1) + 1] = c[i1];
    }
    memcpy(&Ceq_workspace[0], &reshapes_f1[0], 192U * sizeof(real_T));
    b_st.site = &ai_emlrtRSI;
    *status = checkVectorNonFinite(&b_st, obj_mCineq, Cineq_workspace_data,
                                   *Cineq_workspace_size, ineq0);
    if (*status == 1) {
      b_st.site = &ai_emlrtRSI;
      *status = b_checkVectorNonFinite(&b_st, Ceq_workspace);
    }
  }
}

/* End of code generation (evalObjAndConstr.c) */
