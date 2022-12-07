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

static emlrtRSInfo wh_emlrtRSI = {
    1,                  /* lineNo */
    "evalObjAndConstr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+utils\\+"
    "ObjNonlinEvaluator\\evalObjAndConstr.p" /* pathName */
};

static emlrtRSInfo xh_emlrtRSI = {
    1,                   /* lineNo */
    "computeObjective_", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+utils\\+"
    "ObjNonlinEvaluator\\computeObjective_.p" /* pathName */
};

static emlrtRSInfo yh_emlrtRSI = {
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

static emlrtBCInfo pe_emlrtBCI = {
    1,     /* iFirst */
    60,    /* iLast */
    28,    /* lineNo */
    14,    /* colNo */
    "",    /* aName */
    "cat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m", /* pName
                                                                           */
    3 /* checkKind */
};

/* Function Definitions */
void evalObjAndConstr(const emlrtStack *sp,
                      const real_T c_obj_objfun_workspace_runtimed[6],
                      const real_T d_obj_objfun_workspace_runtimed[6],
                      const real_T e_obj_objfun_workspace_runtimed[60],
                      const real_T f_obj_objfun_workspace_runtimed[60],
                      const real_T g_obj_objfun_workspace_runtimed[60],
                      const real_T h_obj_objfun_workspace_runtimed[60],
                      const real_T i_obj_objfun_workspace_runtimed[60],
                      const real_T c_obj_nonlcon_workspace_runtime[6],
                      const real_T d_obj_nonlcon_workspace_runtime[60],
                      const real_T e_obj_nonlcon_workspace_runtime[60],
                      int32_T obj_mCineq, const real_T x[79],
                      real_T Cineq_workspace_data[],
                      const int32_T *Cineq_workspace_size, int32_T ineq0,
                      real_T Ceq_workspace[60], real_T *fval, int32_T *status)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack st;
  real_T b_c_data[120];
  real_T c[120];
  real_T c_data[120];
  real_T U[66];
  real_T X[66];
  real_T b_U[66];
  real_T b_X[66];
  real_T reshapes_f1[60];
  real_T duk[6];
  real_T wtYerr[6];
  real_T b_duk;
  real_T d;
  real_T e;
  real_T t4;
  real_T t8;
  int32_T X_tmp;
  int32_T b_i;
  int32_T c_i;
  int32_T i;
  int32_T i1;
  int8_T tmp_data[120];
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &wh_emlrtRSI;
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
  b_st.site = &xh_emlrtRSI;
  c_st.site = &lb_emlrtRSI;
  d_st.site = &cc_emlrtRSI;
  e_st.site = &dc_emlrtRSI;
  znlmpc_getXUe(x, c_obj_objfun_workspace_runtimed, X, U, &e);
  t4 = 0.0;
  for (i = 0; i < 11; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      c_i = i + 11 * i1;
      X_tmp = i1 + 6 * i;
      b_X[X_tmp] = X[c_i];
      b_U[X_tmp] = U[c_i];
    }
  }
  for (b_i = 0; b_i < 10; b_i++) {
    t8 = 0.0;
    for (i = 0; i < 6; i++) {
      i1 = b_i + 10 * i;
      d = f_obj_objfun_workspace_runtimed[i1] *
          (b_X[i + 6 * (b_i + 1)] - e_obj_objfun_workspace_runtimed[i1]);
      t8 += d * d;
      wtYerr[i] = b_U[i + 6 * b_i];
    }
    t4 += t8;
    if (b_i + 1 == 1) {
      for (c_i = 0; c_i < 6; c_i++) {
        duk[c_i] = wtYerr[c_i] - d_obj_objfun_workspace_runtimed[c_i];
      }
    } else {
      for (i = 0; i < 6; i++) {
        duk[i] = wtYerr[i] - b_U[i + 6 * (b_i - 1)];
      }
    }
    t8 = 0.0;
    b_duk = 0.0;
    for (i = 0; i < 6; i++) {
      i1 = b_i + 10 * i;
      d = g_obj_objfun_workspace_runtimed[i1] *
          (wtYerr[i] - i_obj_objfun_workspace_runtimed[i1]);
      wtYerr[i] = d;
      t8 += d * d;
      d = h_obj_objfun_workspace_runtimed[i1] * duk[i];
      duk[i] = d;
      b_duk += d * d;
    }
    t4 += t8;
    t4 += b_duk;
  }
  *fval = t4 + 100000.0 * e * e;
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
    int8_T input_sizes_idx_0;
    int8_T sizes_idx_1;
    boolean_T bv[60];
    boolean_T b_x[6];
    boolean_T exitg1;
    boolean_T guard1 = false;
    boolean_T y;
    st.site = &wh_emlrtRSI;
    ineqEnd = (ineq0 + obj_mCineq) - 1;
    b_st.site = &yh_emlrtRSI;
    c_st.site = &lb_emlrtRSI;
    d_st.site = &mb_emlrtRSI;
    e_st.site = &nb_emlrtRSI;
    znlmpc_getXUe(x, c_obj_nonlcon_workspace_runtime, X, U, &e);
    e_st.site = &ob_emlrtRSI;
    memset(&reshapes_f1[0], 0, 60U * sizeof(real_T));
    for (i = 0; i < 6; i++) {
      wtYerr[i] = (real_T)i + 1.0;
    }
    for (i = 0; i < 11; i++) {
      for (i1 = 0; i1 < 6; i1++) {
        c_i = i + 11 * i1;
        X_tmp = i1 + 6 * i;
        b_U[X_tmp] = U[c_i];
        b_X[X_tmp] = X[c_i];
      }
    }
    for (b_i = 0; b_i < 10; b_i++) {
      real_T c_t4[6];
      real_T b_duk_tmp_tmp;
      real_T b_t4;
      real_T b_t8;
      real_T c_duk_tmp_tmp;
      real_T d1;
      real_T d2;
      real_T d3;
      real_T d4;
      real_T d_duk_tmp_tmp;
      real_T duk_tmp;
      real_T duk_tmp_tmp;
      int32_T i2;
      /* OneShipStateFcn */
      /*     OUT1 = OneShipStateFcn(IN1,IN2) */
      /*     This function was generated by the Symbolic Math Toolbox
       * version 9.1. */
      /*     28-May-2022 15:38:52 */
      i = 6 * b_i + 4;
      d = b_U[i];
      i1 = 6 * b_i + 5;
      d1 = b_U[i1];
      c_i = 6 * b_i + 2;
      d2 = b_X[c_i];
      t4 = muDoubleScalarCos(d2);
      X_tmp = 6 * b_i + 3;
      duk_tmp_tmp = b_U[X_tmp];
      t8 = muDoubleScalarSin(d2);
      /* OneShipStateFcn */
      /*     OUT1 = OneShipStateFcn(IN1,IN2) */
      /*     This function was generated by the Symbolic Math Toolbox
       * version 9.1. */
      /*     28-May-2022 15:38:52 */
      i2 = 6 * (b_i + 1);
      d2 = b_X[i2 + 2];
      b_t4 = muDoubleScalarCos(d2);
      b_t8 = muDoubleScalarSin(d2);
      d2 = b_X[X_tmp];
      d3 = b_X[i];
      duk[0] = t4 * d2 - t8 * d3;
      duk[1] = t8 * d2 + t4 * d3;
      d4 = b_X[i1];
      duk[2] = d4;
      t4 = b_U[6 * b_i + 1];
      t8 = b_U[c_i];
      b_duk = b_U[6 * b_i];
      b_duk_tmp_tmp = t4 * muDoubleScalarCos(d);
      c_duk_tmp_tmp = t8 * muDoubleScalarCos(d1);
      duk_tmp = b_duk * muDoubleScalarCos(duk_tmp_tmp) * 0.0014269406392694061;
      duk[3] = (((d2 * -0.01 + b_duk_tmp_tmp * 0.0014269406392694061) +
                 c_duk_tmp_tmp * 0.0014269406392694061) +
                duk_tmp) +
               d4 * (d4 * 140000.0 - d3 * 994000.0) * -1.426940639269406E-6;
      d_duk_tmp_tmp = b_duk * muDoubleScalarSin(duk_tmp_tmp);
      duk_tmp_tmp = t4 * muDoubleScalarSin(d);
      t8 *= muDoubleScalarSin(d1);
      t4 = d2 * d4;
      b_duk = d2 * d3;
      duk[4] = ((((d4 * -0.007045416178974943 - d3 * 0.025011227435361039) -
                  b_duk_tmp_tmp * 8.6611644213800846E-6) +
                 d_duk_tmp_tmp * 0.000990384675421431) +
                ((c_duk_tmp_tmp * 8.6611644213800846E-6 +
                  duk_tmp_tmp * 0.00104484094476119) +
                 t8 * 0.00104484094476119)) +
               (t4 * -0.70489771090350661 - b_duk * 0.00094053829938838549);
      duk[5] = ((((d4 * -0.0500224548707221 - d3 * 7.9714791063442632E-5) -
                  b_duk_tmp_tmp * 6.14942673917986E-5) -
                 d_duk_tmp_tmp * 0.0001111259473649811) +
                (((c_duk_tmp_tmp * 6.14942673917986E-5 +
                   duk_tmp_tmp * 0.00027551356494730532) +
                  t8 * 0.00027551356494730532) +
                 t4 * 0.00094053829938838581)) +
               b_duk * -0.0066778219256575372;
      d = b_X[i2 + 3];
      d1 = b_X[i2 + 4];
      c_t4[0] = b_t4 * d - b_t8 * d1;
      c_t4[1] = b_t8 * d + b_t4 * d1;
      d2 = b_X[i2 + 5];
      c_t4[2] = d2;
      c_t4[3] = (((d * -0.01 + b_duk_tmp_tmp * 0.0014269406392694061) +
                  c_duk_tmp_tmp * 0.0014269406392694061) +
                 duk_tmp) +
                d2 * (d2 * 140000.0 - d1 * 994000.0) * -1.426940639269406E-6;
      t4 = d * d2;
      b_duk = d * d1;
      c_t4[4] = ((((d2 * -0.007045416178974943 - d1 * 0.025011227435361039) -
                   b_duk_tmp_tmp * 8.6611644213800846E-6) +
                  d_duk_tmp_tmp * 0.000990384675421431) +
                 ((c_duk_tmp_tmp * 8.6611644213800846E-6 +
                   duk_tmp_tmp * 0.00104484094476119) +
                  t8 * 0.00104484094476119)) +
                (t4 * -0.70489771090350661 - b_duk * 0.00094053829938838549);
      c_t4[5] = ((((d2 * -0.0500224548707221 - d1 * 7.9714791063442632E-5) -
                   b_duk_tmp_tmp * 6.14942673917986E-5) -
                  d_duk_tmp_tmp * 0.0001111259473649811) +
                 (((c_duk_tmp_tmp * 6.14942673917986E-5 +
                    duk_tmp_tmp * 0.00027551356494730532) +
                   t8 * 0.00027551356494730532) +
                  t4 * 0.00094053829938838581)) +
                b_duk * -0.0066778219256575372;
      for (i = 0; i < 6; i++) {
        d = wtYerr[i];
        if (((int32_T)d < 1) || ((int32_T)d > 60)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, 60, &pe_emlrtBCI, &e_st);
        }
        reshapes_f1[(int32_T)d - 1] =
            (b_X[i + 6 * b_i] + 0.5 * (duk[i] + c_t4[i])) - b_X[i + i2];
      }
      for (i = 0; i < 6; i++) {
        wtYerr[i] += 6.0;
      }
    }
    e_st.site = &pb_emlrtRSI;
    for (i = 0; i < 60; i++) {
      bv[i] = muDoubleScalarIsInf(d_obj_nonlcon_workspace_runtime[i]);
    }
    f_st.site = &wb_emlrtRSI;
    b_all(&f_st, bv, b_x);
    y = true;
    c_i = 0;
    exitg1 = false;
    while ((!exitg1) && (c_i < 6)) {
      if (!b_x[c_i]) {
        y = false;
        exitg1 = true;
      } else {
        c_i++;
      }
    }
    guard1 = false;
    if (y) {
      for (i = 0; i < 60; i++) {
        bv[i] = muDoubleScalarIsInf(e_obj_nonlcon_workspace_runtime[i]);
      }
      f_st.site = &wb_emlrtRSI;
      b_all(&f_st, bv, b_x);
      y = true;
      c_i = 0;
      exitg1 = false;
      while ((!exitg1) && (c_i < 6)) {
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
      boolean_T icf[120];
      for (b_i = 0; b_i < 120; b_i++) {
        c[b_i] = 0.0;
        icf[b_i] = true;
      }
      for (i = 0; i < 6; i++) {
        wtYerr[i] = (real_T)i + 1.0;
      }
      for (b_i = 0; b_i < 10; b_i++) {
        boolean_T bv1[6];
        for (i = 0; i < 6; i++) {
          d = d_obj_nonlcon_workspace_runtime[b_i + 10 * i];
          b_x[i] = muDoubleScalarIsInf(d);
          bv1[i] = muDoubleScalarIsNaN(d);
        }
        for (i = 0; i < 6; i++) {
          d = wtYerr[i];
          if (((int32_T)d < 1) || ((int32_T)d > 120)) {
            emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, 120, &d_emlrtBCI,
                                          &e_st);
          }
          icf[(int32_T)d - 1] = ((!b_x[i]) && (!bv1[i]));
        }
        for (i = 0; i < 6; i++) {
          d = e_obj_nonlcon_workspace_runtime[b_i + 10 * i];
          b_x[i] = muDoubleScalarIsInf(d);
          bv1[i] = muDoubleScalarIsNaN(d);
        }
        for (i = 0; i < 6; i++) {
          i1 = (int32_T)(wtYerr[i] + 6.0);
          if ((i1 < 1) || (i1 > 120)) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, 120, &e_emlrtBCI, &e_st);
          }
          icf[i1 - 1] = ((!b_x[i]) && (!bv1[i]));
        }
        for (i = 0; i < 6; i++) {
          i1 = (int32_T)wtYerr[i];
          if ((i1 < 1) || (i1 > 120)) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, 120, &f_emlrtBCI, &e_st);
          }
        }
        for (i = 0; i < 6; i++) {
          i1 = (int32_T)(wtYerr[i] + 6.0);
          if ((i1 < 1) || (i1 > 120)) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, 120, &f_emlrtBCI, &e_st);
          }
        }
        y = false;
        c_i = 0;
        exitg1 = false;
        while ((!exitg1) && (c_i <= 11)) {
          int32_T b_wtYerr[12];
          for (i = 0; i < 6; i++) {
            d = wtYerr[i];
            b_wtYerr[i] = (int32_T)d - 1;
            b_wtYerr[i + 6] = (int32_T)(d + 6.0) - 1;
          }
          if (icf[b_wtYerr[c_i]]) {
            y = true;
            exitg1 = true;
          } else {
            c_i++;
          }
        }
        if (y) {
          for (i = 0; i < 6; i++) {
            c[(int32_T)wtYerr[i] - 1] =
                (d_obj_nonlcon_workspace_runtime[b_i + 10 * i] - e) -
                X[(b_i + 11 * i) + 1];
          }
          for (i = 0; i < 6; i++) {
            i1 = (int32_T)(wtYerr[i] + 6.0);
            if ((i1 < 1) || (i1 > 120)) {
              emlrtDynamicBoundsCheckR2012b(i1, 1, 120, &g_emlrtBCI, &e_st);
            }
            c[i1 - 1] = (X[(b_i + 11 * i) + 1] -
                         e_obj_nonlcon_workspace_runtime[b_i + 10 * i]) -
                        e;
          }
        }
        for (i = 0; i < 6; i++) {
          wtYerr[i] += 12.0;
        }
      }
      X_tmp = 0;
      c_i = 0;
      for (b_i = 0; b_i < 120; b_i++) {
        if (icf[b_i]) {
          X_tmp++;
          tmp_data[c_i] = (int8_T)(b_i + 1);
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
          &g_st, &k_emlrtRTEI, "MATLAB:catenate:matrixDimensionMismatch",
          "MATLAB:catenate:matrixDimensionMismatch", 0);
    }
    if ((sizes_idx_1 == 0) || ((X_tmp != 0) && (c_i != 0))) {
      input_sizes_idx_0 = (int8_T)X_tmp;
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
    memcpy(&Ceq_workspace[0], &reshapes_f1[0], 60U * sizeof(real_T));
    b_st.site = &yh_emlrtRSI;
    *status = checkVectorNonFinite(&b_st, obj_mCineq, Cineq_workspace_data,
                                   *Cineq_workspace_size, ineq0);
    if (*status == 1) {
      b_st.site = &yh_emlrtRSI;
      *status = b_checkVectorNonFinite(&b_st, Ceq_workspace);
    }
  }
}

/* End of code generation (evalObjAndConstr.c) */
