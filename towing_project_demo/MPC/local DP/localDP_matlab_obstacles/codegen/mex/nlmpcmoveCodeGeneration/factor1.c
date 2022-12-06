/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * factor1.c
 *
 * Code generation for function 'factor1'
 *
 */

/* Include files */
#include "factor1.h"
#include "eml_int_forloop_overflow_check.h"
#include "fullColLDL2_.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "partialColLDL3_.h"
#include "rt_nonfinite.h"
#include "xcopy.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void b_factor(const emlrtStack *sp, j_struct_T *obj, const real_T A[6241],
              int32_T ndims, int32_T ldA)
{
  ptrdiff_t incx_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack st;
  int32_T A_maxDiag_idx;
  int32_T LD_diagOffset;
  int32_T LDimSizeP1;
  int32_T order;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  LDimSizeP1 = obj->ldm + 1;
  obj->ndims = ndims;
  st.site = &hg_emlrtRSI;
  if (ndims > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (order = 0; order < ndims; order++) {
    st.site = &hg_emlrtRSI;
    e_xcopy(&st, ndims, A, ldA * order + 1, obj->FMat.data,
            obj->ldm * order + 1);
  }
  if (ndims < 1) {
    A_maxDiag_idx = -1;
  } else {
    n_t = (ptrdiff_t)ndims;
    incx_t = (ptrdiff_t)(obj->ldm + 1);
    n_t = idamax(&n_t, &obj->FMat.data[0], &incx_t);
    A_maxDiag_idx = (int32_T)n_t - 1;
  }
  LD_diagOffset = obj->FMat.size[0] * obj->FMat.size[1];
  A_maxDiag_idx += obj->ldm * A_maxDiag_idx;
  if ((A_maxDiag_idx + 1 < 1) || (A_maxDiag_idx + 1 > LD_diagOffset)) {
    emlrtDynamicBoundsCheckR2012b(A_maxDiag_idx + 1, 1, LD_diagOffset,
                                  &jd_emlrtBCI, (emlrtCTX)sp);
  }
  obj->regTol_ = muDoubleScalarMax(
      muDoubleScalarAbs(obj->FMat.data[A_maxDiag_idx]) * 2.2204460492503131E-16,
      0.0);
  if (ndims > 128) {
    boolean_T exitg1;
    A_maxDiag_idx = 0;
    exitg1 = false;
    while ((!exitg1) && (A_maxDiag_idx < ndims)) {
      LD_diagOffset = LDimSizeP1 * A_maxDiag_idx + 1;
      order = ndims - A_maxDiag_idx;
      if (A_maxDiag_idx + 48 <= ndims) {
        st.site = &hg_emlrtRSI;
        partialColLDL3_(&st, obj, LD_diagOffset, order);
        A_maxDiag_idx += 48;
      } else {
        st.site = &hg_emlrtRSI;
        fullColLDL2_(&st, obj, LD_diagOffset, order);
        exitg1 = true;
      }
    }
  } else {
    st.site = &hg_emlrtRSI;
    fullColLDL2_(&st, obj, 1, ndims);
  }
  if (obj->ConvexCheck) {
    st.site = &hg_emlrtRSI;
    order = 0;
    int32_T exitg2;
    do {
      exitg2 = 0;
      if (order <= ndims - 1) {
        LD_diagOffset = obj->FMat.size[0] * obj->FMat.size[1];
        A_maxDiag_idx = (order + obj->ldm * order) + 1;
        if ((A_maxDiag_idx < 1) || (A_maxDiag_idx > LD_diagOffset)) {
          emlrtDynamicBoundsCheckR2012b(A_maxDiag_idx, 1, LD_diagOffset,
                                        &jd_emlrtBCI, (emlrtCTX)sp);
        }
        if (obj->FMat.data[A_maxDiag_idx - 1] <= 0.0) {
          obj->info = -(order + 1);
          exitg2 = 1;
        } else {
          order++;
        }
      } else {
        obj->ConvexCheck = false;
        exitg2 = 1;
      }
    } while (exitg2 == 0);
  }
}

/* End of code generation (factor1.c) */
