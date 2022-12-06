/*
 * fullColLDL2_.c
 *
 * Code generation for function 'fullColLDL2_'
 *
 */

/* Include files */
#include "fullColLDL2_.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo td_emlrtRSI =
    {
        45,     /* lineNo */
        "xger", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
        "blas\\xger.m" /* pathName */
};

static emlrtRSInfo vd_emlrtRSI = {
    15,     /* lineNo */
    "xger", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "refblas\\xger.m" /* pathName */
};

static emlrtRSInfo wd_emlrtRSI = {
    41,      /* lineNo */
    "xgerx", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "refblas\\xgerx.m" /* pathName */
};

static emlrtRSInfo pg_emlrtRSI = {
    1,              /* lineNo */
    "fullColLDL2_", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "DynamicRegCholManager\\fullColLDL2_.p" /* pathName */
};

static emlrtRSInfo qg_emlrtRSI = {
    54,      /* lineNo */
    "xgerx", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "refblas\\xgerx.m" /* pathName */
};

static emlrtBCInfo we_emlrtBCI = {
    -1,             /* iFirst */
    -1,             /* iLast */
    1,              /* lineNo */
    1,              /* colNo */
    "",             /* aName */
    "fullColLDL2_", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "DynamicRegCholManager\\fullColLDL2_.p", /* pName */
    0                                        /* checkKind */
};

static emlrtBCInfo xe_emlrtBCI = {
    1,              /* iFirst */
    1,              /* iLast */
    1,              /* lineNo */
    1,              /* colNo */
    "",             /* aName */
    "fullColLDL2_", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "DynamicRegCholManager\\fullColLDL2_.p", /* pName */
    3                                        /* checkKind */
};

/* Function Definitions */
void fullColLDL2_(const emlrtStack *sp, j_struct_T *obj, int32_T LD_offset,
                  int32_T NColsRemain)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  int32_T LDimSizeP1_tmp_tmp;
  int32_T idx;
  int32_T j;
  int32_T jA;
  int32_T k;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  LDimSizeP1_tmp_tmp = obj->ldm + 1;
  st.site = &pg_emlrtRSI;
  if (NColsRemain > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (k = 0; k < NColsRemain; k++) {
    real_T alpha1;
    real_T y;
    int32_T LD_diagOffset;
    int32_T b;
    int32_T subMatrixDim;
    LD_diagOffset = LD_offset + LDimSizeP1_tmp_tmp * k;
    j = obj->FMat.size[0] * obj->FMat.size[1];
    if ((LD_diagOffset < 1) || (LD_diagOffset > j)) {
      emlrtDynamicBoundsCheckR2012b(LD_diagOffset, 1, j, &we_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    alpha1 = obj->FMat.data[LD_diagOffset - 1];
    if ((muDoubleScalarAbs(alpha1) <= obj->regTol_) && (LD_diagOffset > j)) {
      emlrtDynamicBoundsCheckR2012b(LD_diagOffset, 1, j, &we_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    if (LD_diagOffset > j) {
      emlrtDynamicBoundsCheckR2012b(LD_diagOffset, 1, j, &we_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    alpha1 = -1.0 / alpha1;
    subMatrixDim = (NColsRemain - k) - 1;
    j = subMatrixDim - 1;
    for (idx = 0; idx <= j; idx++) {
      if ((idx + 1 < 1) || (idx + 1 > 1)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, 1, &xe_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      jA = obj->FMat.size[0] * obj->FMat.size[1];
      b = (LD_diagOffset + idx) + 1;
      if ((b < 1) || (b > jA)) {
        emlrtDynamicBoundsCheckR2012b(b, 1, jA, &we_emlrtBCI, (emlrtCTX)sp);
      }
      obj->workspace_ = obj->FMat.data[b - 1];
    }
    st.site = &pg_emlrtRSI;
    y = obj->workspace_;
    b_st.site = &td_emlrtRSI;
    c_st.site = &vd_emlrtRSI;
    if (!(alpha1 == 0.0)) {
      jA = LD_diagOffset + LDimSizeP1_tmp_tmp;
      d_st.site = &wd_emlrtRSI;
      if (subMatrixDim > 2147483646) {
        e_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&e_st);
      }
      for (j = 0; j < subMatrixDim; j++) {
        if (y != 0.0) {
          real_T temp;
          temp = y * alpha1;
          b = (subMatrixDim + jA) - 1;
          d_st.site = &qg_emlrtRSI;
          if ((jA <= b) && (b > 2147483646)) {
            e_st.site = &db_emlrtRSI;
            check_forloop_overflow_error(&e_st);
          }
          for (idx = jA; idx <= b; idx++) {
            obj->FMat.data[idx - 1] += obj->workspace_ * temp;
          }
        }
        jA += obj->ldm;
      }
    }
    st.site = &pg_emlrtRSI;
    if (subMatrixDim > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx = 0; idx < subMatrixDim; idx++) {
      j = obj->FMat.size[0] * obj->FMat.size[1];
      jA = (LD_diagOffset + idx) + 1;
      if ((jA < 1) || (jA > j)) {
        emlrtDynamicBoundsCheckR2012b(jA, 1, j, &we_emlrtBCI, (emlrtCTX)sp);
      }
      j = obj->FMat.size[0] * obj->FMat.size[1];
      if (LD_diagOffset > j) {
        emlrtDynamicBoundsCheckR2012b(LD_diagOffset, 1, j, &we_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      j = obj->FMat.size[0] * obj->FMat.size[1];
      if (jA > j) {
        emlrtDynamicBoundsCheckR2012b(jA, 1, j, &we_emlrtBCI, (emlrtCTX)sp);
      }
      obj->FMat.data[jA - 1] /= obj->FMat.data[LD_diagOffset - 1];
    }
  }
  jA = LD_offset + (obj->ldm + 1) * (NColsRemain - 1);
  j = obj->FMat.size[0] * obj->FMat.size[1];
  if ((jA < 1) || (jA > j)) {
    emlrtDynamicBoundsCheckR2012b(jA, 1, j, &we_emlrtBCI, (emlrtCTX)sp);
  }
  if ((muDoubleScalarAbs(obj->FMat.data[jA - 1]) <= obj->regTol_) && (jA > j)) {
    emlrtDynamicBoundsCheckR2012b(jA, 1, j, &we_emlrtBCI, (emlrtCTX)sp);
  }
}

/* End of code generation (fullColLDL2_.c) */
