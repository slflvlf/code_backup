/*
 * computeFval_ReuseHx.c
 *
 * Code generation for function 'computeFval_ReuseHx'
 *
 */

/* Include files */
#include "computeFval_ReuseHx.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_internal_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo wf_emlrtRSI = {
    1,                     /* lineNo */
    "computeFval_ReuseHx", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "Objective\\computeFval_ReuseHx.p" /* pathName */
};

static emlrtBCInfo gd_emlrtBCI = {
    -1,                    /* iFirst */
    -1,                    /* iLast */
    1,                     /* lineNo */
    1,                     /* colNo */
    "",                    /* aName */
    "computeFval_ReuseHx", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "Objective\\computeFval_ReuseHx.p", /* pName */
    0                                   /* checkKind */
};

/* Function Definitions */
real_T computeFval_ReuseHx(const emlrtStack *sp, const f_struct_T *obj,
                           real_T workspace_data[], const real_T f_data[],
                           int32_T f_size, const real_T x_data[],
                           int32_T x_size)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack st;
  real_T val;
  int32_T i;
  int32_T idx;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  switch (obj->objtype) {
  case 5:
    if ((obj->nvar < 1) || (obj->nvar > x_size)) {
      emlrtDynamicBoundsCheckR2012b(obj->nvar, 1, x_size, &gd_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    val = obj->gammaScalar * x_data[obj->nvar - 1];
    break;
  case 3: {
    if (obj->hasLinear) {
      int32_T a_tmp;
      a_tmp = obj->nvar;
      st.site = &wf_emlrtRSI;
      if (obj->nvar > 2147483646) {
        b_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }
      for (i = 0; i < a_tmp; i++) {
        idx = obj->Hx.size[0];
        if ((i + 1 < 1) || (i + 1 > idx)) {
          emlrtDynamicBoundsCheckR2012b(i + 1, 1, idx, &gd_emlrtBCI,
                                        (emlrtCTX)sp);
        }
        if ((i + 1 < 1) || (i + 1 > f_size)) {
          emlrtDynamicBoundsCheckR2012b(i + 1, 1, f_size, &gd_emlrtBCI,
                                        (emlrtCTX)sp);
        }
        workspace_data[i] = 0.5 * obj->Hx.data[i] + f_data[i];
      }
      if (obj->nvar < 1) {
        val = 0.0;
      } else {
        n_t = (ptrdiff_t)obj->nvar;
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        val = ddot(&n_t, &x_data[0], &incx_t, &workspace_data[0], &incy_t);
      }
    } else {
      if (obj->nvar < 1) {
        val = 0.0;
      } else {
        n_t = (ptrdiff_t)obj->nvar;
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        val = ddot(&n_t, &x_data[0], &incx_t, &obj->Hx.data[0], &incy_t);
      }
      val *= 0.5;
    }
  } break;
  default: {
    int32_T maxRegVar_tmp;
    maxRegVar_tmp = obj->maxVar - 1;
    if (obj->hasLinear) {
      int32_T a_tmp;
      a_tmp = obj->nvar;
      st.site = &wf_emlrtRSI;
      if (obj->nvar > 2147483646) {
        b_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }
      for (i = 0; i < a_tmp; i++) {
        if ((i + 1 < 1) || (i + 1 > f_size)) {
          emlrtDynamicBoundsCheckR2012b(i + 1, 1, f_size, &gd_emlrtBCI,
                                        (emlrtCTX)sp);
        }
        workspace_data[i] = f_data[i];
      }
      a_tmp = (obj->maxVar - obj->nvar) - 1;
      st.site = &wf_emlrtRSI;
      if (a_tmp > 2147483646) {
        b_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }
      for (i = 0; i < a_tmp; i++) {
        workspace_data[obj->nvar + i] = obj->rho;
      }
      st.site = &wf_emlrtRSI;
      if (maxRegVar_tmp > 2147483646) {
        b_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }
      idx = obj->Hx.size[0];
      for (i = 0; i < maxRegVar_tmp; i++) {
        if ((i + 1 < 1) || (i + 1 > idx)) {
          emlrtDynamicBoundsCheckR2012b(i + 1, 1, idx, &gd_emlrtBCI,
                                        (emlrtCTX)sp);
        }
        workspace_data[i] += 0.5 * obj->Hx.data[i];
      }
      if (maxRegVar_tmp < 1) {
        val = 0.0;
      } else {
        n_t = (ptrdiff_t)(obj->maxVar - 1);
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        val = ddot(&n_t, &x_data[0], &incx_t, &workspace_data[0], &incy_t);
      }
    } else {
      int32_T a_tmp;
      if (maxRegVar_tmp < 1) {
        val = 0.0;
      } else {
        n_t = (ptrdiff_t)(obj->maxVar - 1);
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        val = ddot(&n_t, &x_data[0], &incx_t, &obj->Hx.data[0], &incy_t);
      }
      val *= 0.5;
      a_tmp = obj->nvar + 1;
      st.site = &wf_emlrtRSI;
      if ((a_tmp <= maxRegVar_tmp) && (maxRegVar_tmp > 2147483646)) {
        b_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }
      for (idx = a_tmp; idx <= maxRegVar_tmp; idx++) {
        if ((idx < 1) || (idx > x_size)) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, x_size, &gd_emlrtBCI,
                                        (emlrtCTX)sp);
        }
        val += x_data[idx - 1] * obj->rho;
      }
    }
  } break;
  }
  return val;
}

/* End of code generation (computeFval_ReuseHx.c) */
