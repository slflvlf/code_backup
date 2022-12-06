/*
 * evalObjAndConstr.c
 *
 * Code generation for function 'evalObjAndConstr'
 *
 */

/* Include files */
#include "evalObjAndConstr.h"
#include "OneShipStateFcn.h"
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
                      const real_T d_obj_objfun_workspace_runtimed[8],
                      const real_T e_obj_objfun_workspace_runtimed[60],
                      const real_T f_obj_objfun_workspace_runtimed[60],
                      const real_T g_obj_objfun_workspace_runtimed[80],
                      const real_T h_obj_objfun_workspace_runtimed[80],
                      const real_T i_obj_objfun_workspace_runtimed[80],
                      const real_T c_obj_nonlcon_workspace_runtime[6],
                      const real_T d_obj_nonlcon_workspace_runtime[60],
                      const real_T e_obj_nonlcon_workspace_runtime[60],
                      int32_T obj_mCineq, const real_T x[85],
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
  real_T U[88];
  real_T b_U[88];
  real_T X[66];
  real_T b_X[66];
  real_T reshapes_f1[60];
  real_T umvk[8];
  real_T d;
  real_T e;
  real_T fs;
  int32_T b_i;
  int32_T c_i;
  int32_T i;
  int32_T trueCount;
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
  fs = 0.0;
  for (i = 0; i < 11; i++) {
    for (c_i = 0; c_i < 6; c_i++) {
      b_X[c_i + 6 * i] = X[i + 11 * c_i];
    }
    for (c_i = 0; c_i < 8; c_i++) {
      b_U[c_i + (i << 3)] = U[i + 11 * c_i];
    }
  }
  for (b_i = 0; b_i < 10; b_i++) {
    real_T duk[8];
    real_T b_duk;
    real_T wtYerr;
    wtYerr = 0.0;
    for (i = 0; i < 6; i++) {
      c_i = b_i + 10 * i;
      d = f_obj_objfun_workspace_runtimed[c_i] *
          (b_X[i + 6 * (b_i + 1)] - e_obj_objfun_workspace_runtimed[c_i]);
      wtYerr += d * d;
    }
    fs += wtYerr;
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
    wtYerr = 0.0;
    b_duk = 0.0;
    for (i = 0; i < 8; i++) {
      c_i = b_i + 10 * i;
      d = g_obj_objfun_workspace_runtimed[c_i] *
          (umvk[i] - i_obj_objfun_workspace_runtimed[c_i]);
      umvk[i] = d;
      wtYerr += d * d;
      d = h_obj_objfun_workspace_runtimed[c_i] * duk[i];
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
    real_T b_wtYerr[6];
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
      b_wtYerr[i] = (real_T)i + 1.0;
    }
    for (i = 0; i < 11; i++) {
      for (c_i = 0; c_i < 8; c_i++) {
        b_U[c_i + (i << 3)] = U[i + 11 * c_i];
      }
      for (c_i = 0; c_i < 6; c_i++) {
        b_X[c_i + 6 * i] = X[i + 11 * c_i];
      }
    }
    for (b_i = 0; b_i < 10; b_i++) {
      real_T b_dv[6];
      real_T dv1[6];
      i = b_i << 3;
      OneShipStateFcn(*(real_T(*)[6]) & b_X[6 * b_i], *(real_T(*)[8]) & b_U[i],
                      b_dv);
      c_i = 6 * (b_i + 1);
      OneShipStateFcn(*(real_T(*)[6]) & b_X[c_i], *(real_T(*)[8]) & b_U[i],
                      dv1);
      for (i = 0; i < 6; i++) {
        d = b_wtYerr[i];
        if (((int32_T)d < 1) || ((int32_T)d > 60)) {
          emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, 60, &pe_emlrtBCI, &e_st);
        }
        reshapes_f1[(int32_T)d - 1] =
            (b_X[i + 6 * b_i] + 0.5 * (b_dv[i] + dv1[i])) - b_X[i + c_i];
      }
      for (i = 0; i < 6; i++) {
        b_wtYerr[i] += 6.0;
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
        trueCount = 0;
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
        b_wtYerr[i] = (real_T)i + 1.0;
      }
      for (b_i = 0; b_i < 10; b_i++) {
        boolean_T bv1[6];
        for (i = 0; i < 6; i++) {
          d = d_obj_nonlcon_workspace_runtime[b_i + 10 * i];
          b_x[i] = muDoubleScalarIsInf(d);
          bv1[i] = muDoubleScalarIsNaN(d);
        }
        for (i = 0; i < 6; i++) {
          d = b_wtYerr[i];
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
          c_i = (int32_T)(b_wtYerr[i] + 6.0);
          if ((c_i < 1) || (c_i > 120)) {
            emlrtDynamicBoundsCheckR2012b(c_i, 1, 120, &e_emlrtBCI, &e_st);
          }
          icf[c_i - 1] = ((!b_x[i]) && (!bv1[i]));
        }
        for (i = 0; i < 6; i++) {
          c_i = (int32_T)b_wtYerr[i];
          if ((c_i < 1) || (c_i > 120)) {
            emlrtDynamicBoundsCheckR2012b(c_i, 1, 120, &f_emlrtBCI, &e_st);
          }
        }
        for (i = 0; i < 6; i++) {
          c_i = (int32_T)(b_wtYerr[i] + 6.0);
          if ((c_i < 1) || (c_i > 120)) {
            emlrtDynamicBoundsCheckR2012b(c_i, 1, 120, &f_emlrtBCI, &e_st);
          }
        }
        y = false;
        c_i = 0;
        exitg1 = false;
        while ((!exitg1) && (c_i <= 11)) {
          int32_T c_wtYerr[12];
          for (i = 0; i < 6; i++) {
            d = b_wtYerr[i];
            c_wtYerr[i] = (int32_T)d - 1;
            c_wtYerr[i + 6] = (int32_T)(d + 6.0) - 1;
          }
          if (icf[c_wtYerr[c_i]]) {
            y = true;
            exitg1 = true;
          } else {
            c_i++;
          }
        }
        if (y) {
          for (i = 0; i < 6; i++) {
            c[(int32_T)b_wtYerr[i] - 1] =
                (d_obj_nonlcon_workspace_runtime[b_i + 10 * i] - e) -
                X[(b_i + 11 * i) + 1];
          }
          for (i = 0; i < 6; i++) {
            c_i = (int32_T)(b_wtYerr[i] + 6.0);
            if ((c_i < 1) || (c_i > 120)) {
              emlrtDynamicBoundsCheckR2012b(c_i, 1, 120, &g_emlrtBCI, &e_st);
            }
            c[c_i - 1] = (X[(b_i + 11 * i) + 1] -
                          e_obj_nonlcon_workspace_runtime[b_i + 10 * i]) -
                         e;
          }
        }
        for (i = 0; i < 6; i++) {
          b_wtYerr[i] += 12.0;
        }
      }
      trueCount = 0;
      c_i = 0;
      for (b_i = 0; b_i < 120; b_i++) {
        if (icf[b_i]) {
          trueCount++;
          tmp_data[c_i] = (int8_T)(b_i + 1);
          c_i++;
        }
      }
      for (i = 0; i < trueCount; i++) {
        b_c_data[i] = c[tmp_data[i] - 1];
      }
      c_i = 1;
      if (trueCount - 1 >= 0) {
        memcpy(&c_data[0], &b_c_data[0], trueCount * sizeof(real_T));
      }
    }
    e_st.site = &qb_emlrtRSI;
    f_st.site = &eb_emlrtRSI;
    sizes_idx_1 = (int8_T)((trueCount != 0) && (c_i != 0));
    g_st.site = &fb_emlrtRSI;
    if ((c_i != sizes_idx_1) && ((trueCount != 0) && (c_i != 0))) {
      emlrtErrorWithMessageIdR2018a(
          &g_st, &k_emlrtRTEI, "MATLAB:catenate:matrixDimensionMismatch",
          "MATLAB:catenate:matrixDimensionMismatch", 0);
    }
    if ((sizes_idx_1 == 0) || ((trueCount != 0) && (c_i != 0))) {
      input_sizes_idx_0 = (int8_T)trueCount;
    } else {
      input_sizes_idx_0 = 0;
    }
    c_i = input_sizes_idx_0;
    trueCount = sizes_idx_1;
    if ((trueCount - 1 >= 0) && (c_i - 1 >= 0)) {
      memcpy(&c[0], &c_data[0], c_i * sizeof(real_T));
    }
    if (ineq0 > ineqEnd) {
      i = -1;
      c_i = -1;
    } else {
      i = *Cineq_workspace_size;
      if ((ineq0 < 1) || (ineq0 > i)) {
        emlrtDynamicBoundsCheckR2012b(ineq0, 1, i, &oe_emlrtBCI, &st);
      }
      i = ineq0 - 2;
      c_i = *Cineq_workspace_size;
      if ((ineqEnd < 1) || (ineqEnd > c_i)) {
        emlrtDynamicBoundsCheckR2012b(ineqEnd, 1, c_i, &oe_emlrtBCI, &st);
      }
      c_i = ineqEnd - 1;
    }
    trueCount = c_i - i;
    c_i = input_sizes_idx_0 * sizes_idx_1;
    if (trueCount != c_i) {
      emlrtSubAssignSizeCheck1dR2017a(trueCount, c_i, &emlrtECI, &st);
    }
    for (c_i = 0; c_i < trueCount; c_i++) {
      Cineq_workspace_data[(i + c_i) + 1] = c[c_i];
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
