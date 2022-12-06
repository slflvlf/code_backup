/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sortLambdaQP.c
 *
 * Code generation for function 'sortLambdaQP'
 *
 */

/* Include files */
#include "sortLambdaQP.h"
#include "rt_nonfinite.h"
#include "xcopy.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ge_emlrtRSI = {
    1,              /* lineNo */
    "sortLambdaQP", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "parseoutput\\sortLambdaQP.p" /* pathName */
};

static emlrtBCInfo rb_emlrtBCI = {
    -1,             /* iFirst */
    -1,             /* iLast */
    1,              /* lineNo */
    1,              /* colNo */
    "",             /* aName */
    "sortLambdaQP", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "parseoutput\\sortLambdaQP.p", /* pName */
    0                              /* checkKind */
};

/* Function Definitions */
void sortLambdaQP(const emlrtStack *sp, real_T lambda_data[],
                  const int32_T *lambda_size, int32_T WorkingSet_nActiveConstr,
                  const int32_T WorkingSet_sizes[5],
                  const int32_T WorkingSet_isActiveIdx[6],
                  const int32_T WorkingSet_Wid_data[],
                  int32_T WorkingSet_Wid_size,
                  const int32_T WorkingSet_Wlocalidx_data[],
                  int32_T WorkingSet_Wlocalidx_size, real_T workspace_data[],
                  const int32_T workspace_size[2])
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  if (WorkingSet_nActiveConstr != 0) {
    int32_T currentMplier;
    int32_T i;
    int32_T idx;
    int32_T mAll;
    boolean_T exitg1;
    mAll =
        (((WorkingSet_sizes[0] + WorkingSet_sizes[3]) + WorkingSet_sizes[4]) +
         WorkingSet_sizes[2]) +
        192;
    if (mAll >= 1) {
      n_t = (ptrdiff_t)mAll;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      dcopy(&n_t, &lambda_data[0], &incx_t, &workspace_data[0], &incy_t);
    }
    st.site = &ge_emlrtRSI;
    g_xcopy(&st, mAll, lambda_data);
    currentMplier = 1;
    idx = 1;
    exitg1 = false;
    while ((!exitg1) && (idx <= WorkingSet_nActiveConstr)) {
      if (idx > WorkingSet_Wid_size) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, WorkingSet_Wid_size, &rb_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      i = WorkingSet_Wid_data[idx - 1];
      if (i <= 2) {
        if (idx > WorkingSet_Wlocalidx_size) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, WorkingSet_Wlocalidx_size,
                                        &rb_emlrtBCI, (emlrtCTX)sp);
        }
        if (idx > WorkingSet_Wid_size) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, WorkingSet_Wid_size,
                                        &rb_emlrtBCI, (emlrtCTX)sp);
        }
        if (i == 1) {
          mAll = 0;
        } else {
          mAll = WorkingSet_isActiveIdx[1] - 1;
        }
        i = workspace_size[0] * workspace_size[1];
        if (currentMplier > i) {
          emlrtDynamicBoundsCheckR2012b(currentMplier, 1, i, &rb_emlrtBCI,
                                        (emlrtCTX)sp);
        }
        i = *lambda_size;
        mAll += WorkingSet_Wlocalidx_data[idx - 1];
        if ((mAll < 1) || (mAll > i)) {
          emlrtDynamicBoundsCheckR2012b(mAll, 1, i, &rb_emlrtBCI, (emlrtCTX)sp);
        }
        lambda_data[mAll - 1] = workspace_data[currentMplier - 1];
        currentMplier++;
        idx++;
      } else {
        exitg1 = true;
      }
    }
    while (idx <= WorkingSet_nActiveConstr) {
      if (idx > WorkingSet_Wlocalidx_size) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, WorkingSet_Wlocalidx_size,
                                      &rb_emlrtBCI, (emlrtCTX)sp);
      }
      if (idx > WorkingSet_Wid_size) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, WorkingSet_Wid_size, &rb_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      switch (WorkingSet_Wid_data[idx - 1]) {
      case 3:
        mAll = WorkingSet_isActiveIdx[2];
        break;
      case 4:
        mAll = WorkingSet_isActiveIdx[3];
        break;
      default:
        mAll = WorkingSet_isActiveIdx[4];
        break;
      }
      i = workspace_size[0] * workspace_size[1];
      if (currentMplier > i) {
        emlrtDynamicBoundsCheckR2012b(currentMplier, 1, i, &rb_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      i = *lambda_size;
      mAll = (mAll + WorkingSet_Wlocalidx_data[idx - 1]) - 1;
      if ((mAll < 1) || (mAll > i)) {
        emlrtDynamicBoundsCheckR2012b(mAll, 1, i, &rb_emlrtBCI, (emlrtCTX)sp);
      }
      lambda_data[mAll - 1] = workspace_data[currentMplier - 1];
      currentMplier++;
      idx++;
    }
  }
}

/* End of code generation (sortLambdaQP.c) */
