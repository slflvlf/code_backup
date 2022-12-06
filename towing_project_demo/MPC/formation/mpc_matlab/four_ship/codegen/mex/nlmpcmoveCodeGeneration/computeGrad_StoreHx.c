/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeGrad_StoreHx.c
 *
 * Code generation for function 'computeGrad_StoreHx'
 *
 */

/* Include files */
#include "computeGrad_StoreHx.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo wf_emlrtRSI = {
    1,                     /* lineNo */
    "computeGrad_StoreHx", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "Objective\\computeGrad_StoreHx.p" /* pathName */
};

static emlrtBCInfo fd_emlrtBCI = {
    -1,                    /* iFirst */
    -1,                    /* iLast */
    1,                     /* lineNo */
    1,                     /* colNo */
    "",                    /* aName */
    "computeGrad_StoreHx", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "Objective\\computeGrad_StoreHx.p", /* pName */
    0                                   /* checkKind */
};

/* Function Definitions */
void computeGrad_StoreHx(const emlrtStack *sp, f_struct_T *obj,
                         const real_T H[70225], const real_T f_data[],
                         const real_T x_data[], int32_T x_size)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack st;
  real_T a;
  int32_T idx;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  switch (obj->objtype) {
  case 5: {
    int32_T a_tmp;
    int32_T i;
    a_tmp = obj->nvar;
    st.site = &wf_emlrtRSI;
    if (obj->nvar - 1 > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx = 0; idx <= a_tmp - 2; idx++) {
      i = obj->grad.size[0];
      if ((idx + 1 < 1) || (idx + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &fd_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      obj->grad.data[idx] = 0.0;
    }
    i = obj->grad.size[0];
    if ((obj->nvar < 1) || (obj->nvar > i)) {
      emlrtDynamicBoundsCheckR2012b(obj->nvar, 1, i, &fd_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    obj->grad.data[obj->nvar - 1] = obj->gammaScalar;
  } break;
  case 3: {
    int32_T a_tmp;
    st.site = &wf_emlrtRSI;
    f_xgemv(obj->nvar, obj->nvar, H, obj->nvar, x_data, obj->Hx.data);
    a_tmp = obj->nvar;
    st.site = &wf_emlrtRSI;
    if (obj->nvar > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx = 0; idx < a_tmp; idx++) {
      int32_T i;
      i = obj->Hx.size[0];
      if ((idx + 1 < 1) || (idx + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &fd_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      i = obj->grad.size[0];
      if (idx + 1 > i) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &fd_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      obj->grad.data[idx] = obj->Hx.data[idx];
    }
    if (obj->hasLinear) {
      st.site = &wf_emlrtRSI;
      if (obj->nvar >= 1) {
        a = 1.0;
        n_t = (ptrdiff_t)obj->nvar;
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        daxpy(&n_t, &a, &f_data[0], &incx_t, &obj->grad.data[0], &incy_t);
      }
    }
  } break;
  default: {
    int32_T a_tmp;
    int32_T i;
    int32_T iy;
    iy = obj->maxVar - 1;
    st.site = &wf_emlrtRSI;
    f_xgemv(obj->nvar, obj->nvar, H, obj->nvar, x_data, obj->Hx.data);
    a_tmp = obj->nvar + 1;
    st.site = &wf_emlrtRSI;
    if ((a_tmp <= iy) && (iy > 2147483646)) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx = a_tmp; idx <= iy; idx++) {
      if ((idx < 1) || (idx > x_size)) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, x_size, &fd_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      i = obj->Hx.size[0];
      if (idx > i) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &fd_emlrtBCI, (emlrtCTX)sp);
      }
      obj->Hx.data[idx - 1] = obj->beta * x_data[idx - 1];
    }
    st.site = &wf_emlrtRSI;
    if (obj->maxVar - 1 > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx = 0; idx < iy; idx++) {
      i = obj->Hx.size[0];
      if ((idx + 1 < 1) || (idx + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &fd_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      i = obj->grad.size[0];
      if (idx + 1 > i) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &fd_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      obj->grad.data[idx] = obj->Hx.data[idx];
    }
    if (obj->hasLinear) {
      st.site = &wf_emlrtRSI;
      if (obj->nvar >= 1) {
        a = 1.0;
        n_t = (ptrdiff_t)obj->nvar;
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        daxpy(&n_t, &a, &f_data[0], &incx_t, &obj->grad.data[0], &incy_t);
      }
    }
    a_tmp = (obj->maxVar - obj->nvar) - 1;
    if (a_tmp >= 1) {
      iy = obj->nvar;
      i = a_tmp - 1;
      for (idx = 0; idx <= i; idx++) {
        a_tmp = iy + idx;
        obj->grad.data[a_tmp] += obj->rho;
      }
    }
  } break;
  }
}

/* End of code generation (computeGrad_StoreHx.c) */
