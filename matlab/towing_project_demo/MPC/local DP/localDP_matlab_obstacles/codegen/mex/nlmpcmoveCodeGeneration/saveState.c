/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * saveState.c
 *
 * Code generation for function 'saveState'
 *
 */

/* Include files */
#include "saveState.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void saveState(e_struct_T *obj)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  real_T y_data[1059];
  int32_T k;
  int32_T loop_ub;
  obj->sqpFval_old = obj->sqpFval;
  for (k = 0; k < 79; k++) {
    obj->xstarsqp_old[k] = obj->xstarsqp[k];
    obj->grad_old.data[k] = obj->grad.data[k];
  }
  k = obj->cIneq_old.size[0];
  loop_ub = obj->cIneq_old.size[0];
  if (loop_ub - 1 >= 0) {
    memcpy(&y_data[0], &obj->cIneq_old.data[0], loop_ub * sizeof(real_T));
  }
  if (obj->mIneq >= 1) {
    n_t = (ptrdiff_t)obj->mIneq;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dcopy(&n_t, &obj->cIneq.data[0], &incx_t, &y_data[0], &incy_t);
  }
  obj->cIneq_old.size[0] = k;
  if (k - 1 >= 0) {
    memcpy(&obj->cIneq_old.data[0], &y_data[0], k * sizeof(real_T));
  }
  memcpy(&obj->cEq_old[0], &obj->cEq[0], 60U * sizeof(real_T));
}

/* End of code generation (saveState.c) */
