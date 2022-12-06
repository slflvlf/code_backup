/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * maxConstraintViolation.c
 *
 * Code generation for function 'maxConstraintViolation'
 *
 */

/* Include files */
#include "maxConstraintViolation.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo lf_emlrtRSI = {
    1,                        /* lineNo */
    "maxConstraintViolation", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\maxConstraintViolation.p" /* pathName */
};

static emlrtRSInfo mf_emlrtRSI = {
    1,                                           /* lineNo */
    "maxConstraintViolation_AMats_regularized_", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\maxConstraintViolation_AMats_regularized_.p" /* pathName */
};

static emlrtRSInfo nf_emlrtRSI = {
    1,                                              /* lineNo */
    "maxConstraintViolation_AMats_nonregularized_", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\maxConstraintViolation_AMats_nonregularized_"
    ".p" /* pathName */
};

static emlrtBCInfo qc_emlrtBCI = {
    -1,                       /* iFirst */
    -1,                       /* iLast */
    1,                        /* lineNo */
    1,                        /* colNo */
    "",                       /* aName */
    "maxConstraintViolation", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\maxConstraintViolation.p", /* pName */
    0                                       /* checkKind */
};

static emlrtBCInfo rc_emlrtBCI = {
    -1,                                             /* iFirst */
    -1,                                             /* iLast */
    1,                                              /* lineNo */
    1,                                              /* colNo */
    "",                                             /* aName */
    "maxConstraintViolation_AMats_nonregularized_", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\maxConstraintViolation_AMats_nonregularized_"
    ".p", /* pName */
    0     /* checkKind */
};

static emlrtBCInfo sc_emlrtBCI = {
    -1,                                          /* iFirst */
    -1,                                          /* iLast */
    1,                                           /* lineNo */
    1,                                           /* colNo */
    "",                                          /* aName */
    "maxConstraintViolation_AMats_regularized_", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\maxConstraintViolation_AMats_regularized_.p", /* pName */
    0                                                          /* checkKind */
};

/* Function Definitions */
real_T b_maxConstraintViolation(const emlrtStack *sp, h_struct_T *obj,
                                const real_T x_data[], int32_T ix0)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T v;
  int32_T i;
  int32_T idx;
  int32_T mFixed;
  int32_T mIneq;
  int32_T mLB;
  int32_T mUB;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  mLB = obj->sizes[3];
  mUB = obj->sizes[4];
  mFixed = obj->sizes[0];
  if (obj->probType == 2) {
    st.site = &lf_emlrtRSI;
    v = 0.0;
    mIneq = obj->sizes[2];
    if (obj->Aineq.size[0] != 0) {
      b_st.site = &mf_emlrtRSI;
      if (obj->sizes[2] >= 1) {
        n_t = (ptrdiff_t)obj->sizes[2];
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        dcopy(&n_t, &obj->bineq.data[0], &incx_t,
              &obj->maxConstrWorkspace.data[0], &incy_t);
      }
      b_st.site = &mf_emlrtRSI;
      e_xgemv(79, obj->sizes[2], obj->Aineq.data, obj->ldA, x_data, ix0,
              obj->maxConstrWorkspace.data);
      b_st.site = &mf_emlrtRSI;
      if (obj->sizes[2] > 2147483646) {
        c_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (idx = 0; idx < mIneq; idx++) {
        i = obj->maxConstrWorkspace.size[0];
        if ((idx + 1 < 1) || (idx + 1 > i)) {
          emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &sc_emlrtBCI, &st);
        }
        i = obj->maxConstrWorkspace.size[0];
        if (idx + 1 > i) {
          emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &sc_emlrtBCI, &st);
        }
        obj->maxConstrWorkspace.data[idx] -= x_data[(ix0 + idx) + 78];
        i = obj->maxConstrWorkspace.size[0];
        if (idx + 1 > i) {
          emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &sc_emlrtBCI, &st);
        }
        v = muDoubleScalarMax(v, obj->maxConstrWorkspace.data[idx]);
      }
    }
    b_st.site = &mf_emlrtRSI;
    memcpy(&obj->maxConstrWorkspace.data[0], &obj->beq[0],
           60U * sizeof(real_T));
    b_st.site = &mf_emlrtRSI;
    e_xgemv(79, 60, obj->Aeq.data, obj->ldA, x_data, ix0,
            obj->maxConstrWorkspace.data);
    b_st.site = &mf_emlrtRSI;
    for (idx = 0; idx < 60; idx++) {
      obj->maxConstrWorkspace.data[idx] =
          (obj->maxConstrWorkspace.data[idx] -
           x_data[((ix0 + mIneq) + idx) + 78]) +
          x_data[((ix0 + obj->sizes[2]) + idx) + 138];
      v = muDoubleScalarMax(
          v, muDoubleScalarAbs(obj->maxConstrWorkspace.data[idx]));
    }
  } else {
    st.site = &lf_emlrtRSI;
    v = 0.0;
    mIneq = obj->sizes[2];
    if (obj->Aineq.size[0] != 0) {
      b_st.site = &nf_emlrtRSI;
      if (obj->sizes[2] >= 1) {
        n_t = (ptrdiff_t)obj->sizes[2];
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        dcopy(&n_t, &obj->bineq.data[0], &incx_t,
              &obj->maxConstrWorkspace.data[0], &incy_t);
      }
      b_st.site = &nf_emlrtRSI;
      e_xgemv(obj->nVar, obj->sizes[2], obj->Aineq.data, obj->ldA, x_data, ix0,
              obj->maxConstrWorkspace.data);
      b_st.site = &nf_emlrtRSI;
      if (obj->sizes[2] > 2147483646) {
        c_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (idx = 0; idx < mIneq; idx++) {
        i = obj->maxConstrWorkspace.size[0];
        if ((idx + 1 < 1) || (idx + 1 > i)) {
          emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &rc_emlrtBCI, &st);
        }
        v = muDoubleScalarMax(v, obj->maxConstrWorkspace.data[idx]);
      }
    }
    b_st.site = &nf_emlrtRSI;
    memcpy(&obj->maxConstrWorkspace.data[0], &obj->beq[0],
           60U * sizeof(real_T));
    b_st.site = &nf_emlrtRSI;
    e_xgemv(obj->nVar, 60, obj->Aeq.data, obj->ldA, x_data, ix0,
            obj->maxConstrWorkspace.data);
    b_st.site = &nf_emlrtRSI;
    for (idx = 0; idx < 60; idx++) {
      v = muDoubleScalarMax(
          v, muDoubleScalarAbs(obj->maxConstrWorkspace.data[idx]));
    }
  }
  if (obj->sizes[3] > 0) {
    st.site = &lf_emlrtRSI;
    if (obj->sizes[3] > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    i = obj->indexLB.size[0];
    mIneq = obj->lb.size[0];
    for (idx = 0; idx < mLB; idx++) {
      int32_T idxLB;
      if ((idx + 1 < 1) || (idx + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &qc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      idxLB = obj->indexLB.data[idx] - 1;
      if ((obj->indexLB.data[idx] < 1) || (obj->indexLB.data[idx] > mIneq)) {
        emlrtDynamicBoundsCheckR2012b(obj->indexLB.data[idx], 1, mIneq,
                                      &qc_emlrtBCI, (emlrtCTX)sp);
      }
      v = muDoubleScalarMax(v,
                            -x_data[(ix0 + idxLB) - 1] - obj->lb.data[idxLB]);
    }
  }
  if (obj->sizes[4] > 0) {
    st.site = &lf_emlrtRSI;
    if (obj->sizes[4] > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    i = obj->indexUB.size[0];
    for (idx = 0; idx < mUB; idx++) {
      if ((idx + 1 < 1) || (idx + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &qc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      mIneq = obj->indexUB.data[idx] - 1;
      v = muDoubleScalarMax(v, x_data[(ix0 + mIneq) - 1] - obj->ub.data[mIneq]);
    }
  }
  if (obj->sizes[0] > 0) {
    st.site = &lf_emlrtRSI;
    if (obj->sizes[0] > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    i = obj->indexFixed.size[0];
    for (idx = 0; idx < mFixed; idx++) {
      if ((idx + 1 < 1) || (idx + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &qc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      v = muDoubleScalarMax(
          v, muDoubleScalarAbs(x_data[(ix0 + obj->indexFixed.data[idx]) - 2] -
                               obj->ub.data[obj->indexFixed.data[idx] - 1]));
    }
  }
  return v;
}

real_T c_maxConstraintViolation(const emlrtStack *sp, h_struct_T *obj,
                                const real_T x_data[], int32_T x_size)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T v;
  int32_T i;
  int32_T i1;
  int32_T idx;
  int32_T mFixed;
  int32_T mIneq;
  int32_T mLB;
  int32_T mUB;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  mLB = obj->sizes[3];
  mUB = obj->sizes[4];
  mFixed = obj->sizes[0];
  if (obj->probType == 2) {
    st.site = &lf_emlrtRSI;
    v = 0.0;
    mIneq = obj->sizes[2];
    if (obj->Aineq.size[0] != 0) {
      b_st.site = &mf_emlrtRSI;
      if (obj->sizes[2] >= 1) {
        n_t = (ptrdiff_t)obj->sizes[2];
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        dcopy(&n_t, &obj->bineq.data[0], &incx_t,
              &obj->maxConstrWorkspace.data[0], &incy_t);
      }
      b_st.site = &mf_emlrtRSI;
      h_xgemv(obj->sizes[2], obj->Aineq.data, obj->ldA, x_data,
              obj->maxConstrWorkspace.data);
      b_st.site = &mf_emlrtRSI;
      if (obj->sizes[2] > 2147483646) {
        c_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (idx = 0; idx < mIneq; idx++) {
        i = obj->maxConstrWorkspace.size[0];
        if ((idx + 1 < 1) || (idx + 1 > i)) {
          emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &sc_emlrtBCI, &st);
        }
        if (idx + 80 > x_size) {
          emlrtDynamicBoundsCheckR2012b(idx + 80, 1, x_size, &sc_emlrtBCI, &st);
        }
        i = obj->maxConstrWorkspace.size[0];
        if (idx + 1 > i) {
          emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &sc_emlrtBCI, &st);
        }
        obj->maxConstrWorkspace.data[idx] -= x_data[idx + 79];
        i = obj->maxConstrWorkspace.size[0];
        if (idx + 1 > i) {
          emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &sc_emlrtBCI, &st);
        }
        v = muDoubleScalarMax(v, obj->maxConstrWorkspace.data[idx]);
      }
    }
    b_st.site = &mf_emlrtRSI;
    memcpy(&obj->maxConstrWorkspace.data[0], &obj->beq[0],
           60U * sizeof(real_T));
    b_st.site = &mf_emlrtRSI;
    h_xgemv(60, obj->Aeq.data, obj->ldA, x_data, obj->maxConstrWorkspace.data);
    b_st.site = &mf_emlrtRSI;
    for (idx = 0; idx < 60; idx++) {
      i = (mIneq + idx) + 80;
      if ((i < 1) || (i > x_size)) {
        emlrtDynamicBoundsCheckR2012b(i, 1, x_size, &sc_emlrtBCI, &st);
      }
      i1 = (obj->sizes[2] + idx) + 140;
      if ((i1 < 1) || (i1 > x_size)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, x_size, &sc_emlrtBCI, &st);
      }
      obj->maxConstrWorkspace.data[idx] =
          (obj->maxConstrWorkspace.data[idx] - x_data[i - 1]) + x_data[i1 - 1];
      v = muDoubleScalarMax(
          v, muDoubleScalarAbs(obj->maxConstrWorkspace.data[idx]));
    }
  } else {
    st.site = &lf_emlrtRSI;
    v = 0.0;
    mIneq = obj->sizes[2];
    if (obj->Aineq.size[0] != 0) {
      b_st.site = &nf_emlrtRSI;
      if (obj->sizes[2] >= 1) {
        n_t = (ptrdiff_t)obj->sizes[2];
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        dcopy(&n_t, &obj->bineq.data[0], &incx_t,
              &obj->maxConstrWorkspace.data[0], &incy_t);
      }
      b_st.site = &nf_emlrtRSI;
      d_xgemv(obj->nVar, obj->sizes[2], obj->Aineq.data, obj->ldA, x_data,
              obj->maxConstrWorkspace.data);
      b_st.site = &nf_emlrtRSI;
      if (obj->sizes[2] > 2147483646) {
        c_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (idx = 0; idx < mIneq; idx++) {
        i = obj->maxConstrWorkspace.size[0];
        if ((idx + 1 < 1) || (idx + 1 > i)) {
          emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &rc_emlrtBCI, &st);
        }
        v = muDoubleScalarMax(v, obj->maxConstrWorkspace.data[idx]);
      }
    }
    b_st.site = &nf_emlrtRSI;
    memcpy(&obj->maxConstrWorkspace.data[0], &obj->beq[0],
           60U * sizeof(real_T));
    b_st.site = &nf_emlrtRSI;
    d_xgemv(obj->nVar, 60, obj->Aeq.data, obj->ldA, x_data,
            obj->maxConstrWorkspace.data);
    b_st.site = &nf_emlrtRSI;
    for (idx = 0; idx < 60; idx++) {
      v = muDoubleScalarMax(
          v, muDoubleScalarAbs(obj->maxConstrWorkspace.data[idx]));
    }
  }
  if (obj->sizes[3] > 0) {
    st.site = &lf_emlrtRSI;
    if (obj->sizes[3] > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    i = obj->indexLB.size[0];
    i1 = obj->lb.size[0];
    for (idx = 0; idx < mLB; idx++) {
      if ((idx + 1 < 1) || (idx + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &qc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      mIneq = obj->indexLB.data[idx] - 1;
      if ((obj->indexLB.data[idx] < 1) || (obj->indexLB.data[idx] > x_size)) {
        emlrtDynamicBoundsCheckR2012b(obj->indexLB.data[idx], 1, x_size,
                                      &qc_emlrtBCI, (emlrtCTX)sp);
      }
      if ((obj->indexLB.data[idx] < 1) || (obj->indexLB.data[idx] > i1)) {
        emlrtDynamicBoundsCheckR2012b(obj->indexLB.data[idx], 1, i1,
                                      &qc_emlrtBCI, (emlrtCTX)sp);
      }
      v = muDoubleScalarMax(v, -x_data[mIneq] - obj->lb.data[mIneq]);
    }
  }
  if (obj->sizes[4] > 0) {
    st.site = &lf_emlrtRSI;
    if (obj->sizes[4] > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    i = obj->indexUB.size[0];
    for (idx = 0; idx < mUB; idx++) {
      if ((idx + 1 < 1) || (idx + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &qc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      mIneq = obj->indexUB.data[idx] - 1;
      v = muDoubleScalarMax(v, x_data[mIneq] - obj->ub.data[mIneq]);
    }
  }
  if (obj->sizes[0] > 0) {
    st.site = &lf_emlrtRSI;
    if (obj->sizes[0] > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    i = obj->indexFixed.size[0];
    for (idx = 0; idx < mFixed; idx++) {
      if ((idx + 1 < 1) || (idx + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &qc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      v = muDoubleScalarMax(
          v, muDoubleScalarAbs(x_data[obj->indexFixed.data[idx] - 1] -
                               obj->ub.data[obj->indexFixed.data[idx] - 1]));
    }
  }
  return v;
}

real_T maxConstraintViolation(const emlrtStack *sp, h_struct_T *obj,
                              const real_T x_data[])
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T v;
  int32_T i;
  int32_T idx;
  int32_T mFixed;
  int32_T mIneq;
  int32_T mLB;
  int32_T mUB;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  mLB = obj->sizes[3];
  mUB = obj->sizes[4];
  mFixed = obj->sizes[0];
  if (obj->probType == 2) {
    st.site = &lf_emlrtRSI;
    v = 0.0;
    mIneq = obj->sizes[2];
    if (obj->Aineq.size[0] != 0) {
      b_st.site = &mf_emlrtRSI;
      if (obj->sizes[2] >= 1) {
        n_t = (ptrdiff_t)obj->sizes[2];
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        dcopy(&n_t, &obj->bineq.data[0], &incx_t,
              &obj->maxConstrWorkspace.data[0], &incy_t);
      }
      b_st.site = &mf_emlrtRSI;
      h_xgemv(obj->sizes[2], obj->Aineq.data, obj->ldA, x_data,
              obj->maxConstrWorkspace.data);
      b_st.site = &mf_emlrtRSI;
      if (obj->sizes[2] > 2147483646) {
        c_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (idx = 0; idx < mIneq; idx++) {
        i = obj->maxConstrWorkspace.size[0];
        if ((idx + 1 < 1) || (idx + 1 > i)) {
          emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &sc_emlrtBCI, &st);
        }
        i = obj->maxConstrWorkspace.size[0];
        if (idx + 1 > i) {
          emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &sc_emlrtBCI, &st);
        }
        obj->maxConstrWorkspace.data[idx] -= x_data[idx + 79];
        i = obj->maxConstrWorkspace.size[0];
        if (idx + 1 > i) {
          emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &sc_emlrtBCI, &st);
        }
        v = muDoubleScalarMax(v, obj->maxConstrWorkspace.data[idx]);
      }
    }
    b_st.site = &mf_emlrtRSI;
    memcpy(&obj->maxConstrWorkspace.data[0], &obj->beq[0],
           60U * sizeof(real_T));
    b_st.site = &mf_emlrtRSI;
    h_xgemv(60, obj->Aeq.data, obj->ldA, x_data, obj->maxConstrWorkspace.data);
    b_st.site = &mf_emlrtRSI;
    for (idx = 0; idx < 60; idx++) {
      obj->maxConstrWorkspace.data[idx] =
          (obj->maxConstrWorkspace.data[idx] - x_data[(mIneq + idx) + 79]) +
          x_data[(obj->sizes[2] + idx) + 139];
      v = muDoubleScalarMax(
          v, muDoubleScalarAbs(obj->maxConstrWorkspace.data[idx]));
    }
  } else {
    st.site = &lf_emlrtRSI;
    v = 0.0;
    mIneq = obj->sizes[2];
    if (obj->Aineq.size[0] != 0) {
      b_st.site = &nf_emlrtRSI;
      if (obj->sizes[2] >= 1) {
        n_t = (ptrdiff_t)obj->sizes[2];
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        dcopy(&n_t, &obj->bineq.data[0], &incx_t,
              &obj->maxConstrWorkspace.data[0], &incy_t);
      }
      b_st.site = &nf_emlrtRSI;
      d_xgemv(obj->nVar, obj->sizes[2], obj->Aineq.data, obj->ldA, x_data,
              obj->maxConstrWorkspace.data);
      b_st.site = &nf_emlrtRSI;
      if (obj->sizes[2] > 2147483646) {
        c_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (idx = 0; idx < mIneq; idx++) {
        i = obj->maxConstrWorkspace.size[0];
        if ((idx + 1 < 1) || (idx + 1 > i)) {
          emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &rc_emlrtBCI, &st);
        }
        v = muDoubleScalarMax(v, obj->maxConstrWorkspace.data[idx]);
      }
    }
    b_st.site = &nf_emlrtRSI;
    memcpy(&obj->maxConstrWorkspace.data[0], &obj->beq[0],
           60U * sizeof(real_T));
    b_st.site = &nf_emlrtRSI;
    d_xgemv(obj->nVar, 60, obj->Aeq.data, obj->ldA, x_data,
            obj->maxConstrWorkspace.data);
    b_st.site = &nf_emlrtRSI;
    for (idx = 0; idx < 60; idx++) {
      v = muDoubleScalarMax(
          v, muDoubleScalarAbs(obj->maxConstrWorkspace.data[idx]));
    }
  }
  if (obj->sizes[3] > 0) {
    st.site = &lf_emlrtRSI;
    if (obj->sizes[3] > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    i = obj->indexLB.size[0];
    mIneq = obj->lb.size[0];
    for (idx = 0; idx < mLB; idx++) {
      int32_T idxLB;
      if ((idx + 1 < 1) || (idx + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &qc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      idxLB = obj->indexLB.data[idx] - 1;
      if ((obj->indexLB.data[idx] < 1) || (obj->indexLB.data[idx] > mIneq)) {
        emlrtDynamicBoundsCheckR2012b(obj->indexLB.data[idx], 1, mIneq,
                                      &qc_emlrtBCI, (emlrtCTX)sp);
      }
      v = muDoubleScalarMax(v, -x_data[idxLB] - obj->lb.data[idxLB]);
    }
  }
  if (obj->sizes[4] > 0) {
    st.site = &lf_emlrtRSI;
    if (obj->sizes[4] > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    i = obj->indexUB.size[0];
    for (idx = 0; idx < mUB; idx++) {
      if ((idx + 1 < 1) || (idx + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &qc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      mIneq = obj->indexUB.data[idx] - 1;
      v = muDoubleScalarMax(v, x_data[mIneq] - obj->ub.data[mIneq]);
    }
  }
  if (obj->sizes[0] > 0) {
    st.site = &lf_emlrtRSI;
    if (obj->sizes[0] > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    i = obj->indexFixed.size[0];
    for (idx = 0; idx < mFixed; idx++) {
      if ((idx + 1 < 1) || (idx + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &qc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      v = muDoubleScalarMax(
          v, muDoubleScalarAbs(x_data[obj->indexFixed.data[idx] - 1] -
                               obj->ub.data[obj->indexFixed.data[idx] - 1]));
    }
  }
  return v;
}

/* End of code generation (maxConstraintViolation.c) */
