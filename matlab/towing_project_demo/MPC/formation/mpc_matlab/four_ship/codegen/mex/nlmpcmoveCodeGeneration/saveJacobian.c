/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * saveJacobian.c
 *
 * Code generation for function 'saveJacobian'
 *
 */

/* Include files */
#include "saveJacobian.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo he_emlrtRSI = {
    1,              /* lineNo */
    "saveJacobian", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "TrialState\\saveJacobian.p" /* pathName */
};

/* Function Definitions */
void saveJacobian(c_nlmpcmoveCodeGenerationStackD *SD, const emlrtStack *sp,
                  e_struct_T *obj, int32_T nVar, int32_T mIneq,
                  const real_T JacCineqTrans_data[], int32_T ineqCol0,
                  const real_T JacCeqTrans_data[], int32_T ldJ)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack st;
  int32_T b;
  int32_T iCol;
  int32_T idx_col;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  iCol = ldJ * (ineqCol0 - 1);
  b = (mIneq - ineqCol0) + 1;
  st.site = &he_emlrtRSI;
  if (b > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  if (b - 1 >= 0) {
    n_t = (ptrdiff_t)nVar;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
  }
  for (idx_col = 0; idx_col < b; idx_col++) {
    int32_T loop_ub_tmp;
    int32_T y_size_idx_0;
    int32_T y_size_idx_1;
    st.site = &he_emlrtRSI;
    y_size_idx_0 = obj->JacCineqTrans_old.size[0];
    y_size_idx_1 = obj->JacCineqTrans_old.size[1];
    loop_ub_tmp =
        obj->JacCineqTrans_old.size[0] * obj->JacCineqTrans_old.size[1];
    if (loop_ub_tmp - 1 >= 0) {
      memcpy(&SD->u1.f0.y_data[0], &obj->JacCineqTrans_old.data[0],
             loop_ub_tmp * sizeof(real_T));
    }
    dcopy(&n_t, &JacCineqTrans_data[iCol + idx_col * ldJ], &incx_t,
          &SD->u1.f0.y_data[idx_col * ldJ], &incy_t);
    obj->JacCineqTrans_old.size[0] = y_size_idx_0;
    obj->JacCineqTrans_old.size[1] = y_size_idx_1;
    y_size_idx_0 *= y_size_idx_1;
    if (y_size_idx_0 - 1 >= 0) {
      memcpy(&obj->JacCineqTrans_old.data[0], &SD->u1.f0.y_data[0],
             y_size_idx_0 * sizeof(real_T));
    }
  }
  st.site = &he_emlrtRSI;
  n_t = (ptrdiff_t)nVar;
  incx_t = (ptrdiff_t)1;
  incy_t = (ptrdiff_t)1;
  for (idx_col = 0; idx_col < 192; idx_col++) {
    st.site = &he_emlrtRSI;
    dcopy(&n_t, &JacCeqTrans_data[idx_col * ldJ], &incx_t,
          &obj->JacCeqTrans_old.data[idx_col * ldJ], &incy_t);
  }
}

/* End of code generation (saveJacobian.c) */
