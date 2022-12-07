/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeConstraintsAndUserJacobian_.c
 *
 * Code generation for function 'computeConstraintsAndUserJacobian_'
 *
 */

/* Include files */
#include "computeConstraintsAndUserJacobian_.h"
#include "checkMatrixNonFinite.h"
#include "checkVectorNonFinite.h"
#include "nlmpcmoveCodeGeneration.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "xcopy.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo mc_emlrtRSI = {
    1,                                    /* lineNo */
    "computeConstraintsAndUserJacobian_", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+utils\\+"
    "ObjNonlinEvaluator\\computeConstraintsAndUserJacobian_.p" /* pathName */
};

static emlrtBCInfo me_emlrtBCI = {
    -1,                                   /* iFirst */
    -1,                                   /* iLast */
    1,                                    /* lineNo */
    1,                                    /* colNo */
    "",                                   /* aName */
    "computeConstraintsAndUserJacobian_", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+utils\\+"
    "ObjNonlinEvaluator\\computeConstraintsAndUserJacobian_.p", /* pName */
    0                                                           /* checkKind */
};

/* Function Definitions */
int32_T c_computeConstraintsAndUserJaco(
    c_nlmpcmoveCodeGenerationStackD *SD, const emlrtStack *sp,
    const real_T c_obj_nonlcon_workspace_runtime[24],
    const real_T d_obj_nonlcon_workspace_runtime[192],
    const real_T e_obj_nonlcon_workspace_runtime[192], int32_T obj_mCineq,
    const real_T x[265], real_T Cineq_workspace_data[],
    const int32_T *Cineq_workspace_size, int32_T ineq0,
    real_T Ceq_workspace[192], real_T JacIneqTrans_workspace_data[],
    const int32_T *JacIneqTrans_workspace_size, int32_T iJI_col, int32_T ldJI,
    real_T JacEqTrans_workspace_data[],
    const int32_T *JacEqTrans_workspace_size, int32_T ldJE)
{
  emlrtStack b_st;
  emlrtStack st;
  real_T varargout_1_data[384];
  real_T varargout_2[192];
  int32_T idx_col;
  int32_T idx_row;
  int32_T status;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  if (obj_mCineq > 0) {
    int32_T varargout_1_size[2];
    int32_T varargout_3_size[2];
    int32_T i;
    int32_T i1;
    st.site = &mc_emlrtRSI;
    b_st.site = &lb_emlrtRSI;
    c_nlmpcmoveCodeGeneration_anonF(
        SD, &b_st, c_obj_nonlcon_workspace_runtime,
        d_obj_nonlcon_workspace_runtime, e_obj_nonlcon_workspace_runtime, x,
        varargout_1_data, varargout_1_size, varargout_2,
        SD->u4.f9.varargout_3_data, varargout_3_size, SD->u4.f9.varargout_4);
    st.site = &mc_emlrtRSI;
    b_xcopy(obj_mCineq, varargout_1_data, Cineq_workspace_data, ineq0);
    st.site = &mc_emlrtRSI;
    memcpy(&Ceq_workspace[0], &varargout_2[0], 192U * sizeof(real_T));
    i = varargout_3_size[0];
    st.site = &mc_emlrtRSI;
    for (idx_row = 0; idx_row < i; idx_row++) {
      i1 = varargout_3_size[1];
      st.site = &mc_emlrtRSI;
      for (idx_col = 0; idx_col < i1; idx_col++) {
        int32_T i2;
        int32_T i3;
        if (idx_row + 1 > varargout_3_size[0]) {
          emlrtDynamicBoundsCheckR2012b(idx_row + 1, 1, varargout_3_size[0],
                                        &me_emlrtBCI, (emlrtCTX)sp);
        }
        if (idx_col + 1 > varargout_3_size[1]) {
          emlrtDynamicBoundsCheckR2012b(idx_col + 1, 1, varargout_3_size[1],
                                        &me_emlrtBCI, (emlrtCTX)sp);
        }
        i2 = *JacIneqTrans_workspace_size;
        i3 = (idx_row + ldJI * ((iJI_col + idx_col) - 1)) + 1;
        if ((i3 < 1) || (i3 > i2)) {
          emlrtDynamicBoundsCheckR2012b(i3, 1, i2, &me_emlrtBCI, (emlrtCTX)sp);
        }
        JacIneqTrans_workspace_data[i3 - 1] =
            SD->u4.f9.varargout_3_data[idx_row + varargout_3_size[0] * idx_col];
      }
    }
    i = *JacEqTrans_workspace_size;
    for (idx_row = 0; idx_row < 265; idx_row++) {
      for (idx_col = 0; idx_col < 192; idx_col++) {
        i1 = (idx_row + ldJE * idx_col) + 1;
        if ((i1 < 1) || (i1 > i)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, i, &me_emlrtBCI, (emlrtCTX)sp);
        }
        JacEqTrans_workspace_data[i1 - 1] =
            SD->u4.f9.varargout_4[idx_row + 265 * idx_col];
      }
    }
  } else {
    int32_T varargout_1_size[2];
    int32_T varargout_3_size[2];
    int32_T i;
    st.site = &mc_emlrtRSI;
    b_st.site = &lb_emlrtRSI;
    c_nlmpcmoveCodeGeneration_anonF(
        SD, &b_st, c_obj_nonlcon_workspace_runtime,
        d_obj_nonlcon_workspace_runtime, e_obj_nonlcon_workspace_runtime, x,
        varargout_1_data, varargout_1_size, varargout_2,
        SD->u4.f9.varargout_3_data, varargout_3_size, SD->u4.f9.varargout_4);
    st.site = &mc_emlrtRSI;
    memcpy(&Ceq_workspace[0], &varargout_2[0], 192U * sizeof(real_T));
    i = *JacEqTrans_workspace_size;
    for (idx_row = 0; idx_row < 265; idx_row++) {
      for (idx_col = 0; idx_col < 192; idx_col++) {
        int32_T i1;
        i1 = (idx_row + ldJE * idx_col) + 1;
        if ((i1 < 1) || (i1 > i)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, i, &me_emlrtBCI, (emlrtCTX)sp);
        }
        JacEqTrans_workspace_data[i1 - 1] =
            SD->u4.f9.varargout_4[idx_row + 265 * idx_col];
      }
    }
  }
  st.site = &mc_emlrtRSI;
  status = checkVectorNonFinite(&st, obj_mCineq, Cineq_workspace_data,
                                *Cineq_workspace_size, ineq0);
  if (status == 1) {
    st.site = &mc_emlrtRSI;
    status = b_checkVectorNonFinite(&st, Ceq_workspace);
    if (status == 1) {
      st.site = &mc_emlrtRSI;
      status =
          checkMatrixNonFinite(&st, obj_mCineq, JacIneqTrans_workspace_data,
                               *JacIneqTrans_workspace_size, iJI_col, ldJI);
      if (status == 1) {
        st.site = &mc_emlrtRSI;
        status = checkMatrixNonFinite(&st, 192, JacEqTrans_workspace_data,
                                      *JacEqTrans_workspace_size, 1, ldJE);
      }
    }
  }
  return status;
}

/* End of code generation (computeConstraintsAndUserJacobian_.c) */
