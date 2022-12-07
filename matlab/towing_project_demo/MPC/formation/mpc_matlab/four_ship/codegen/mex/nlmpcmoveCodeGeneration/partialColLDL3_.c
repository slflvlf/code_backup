/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * partialColLDL3_.c
 *
 * Code generation for function 'partialColLDL3_'
 *
 */

/* Include files */
#include "partialColLDL3_.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo dd_emlrtRSI = {
    51,      /* lineNo */
    "xgemv", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "refblas\\xgemv.m" /* pathName */
};

static emlrtRSInfo td_emlrtRSI = {
    64,      /* lineNo */
    "xgemv", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+blas\\xgemv."
    "m" /* pathName */
};

static emlrtRSInfo tf_emlrtRSI = {
    58,      /* lineNo */
    "xgemv", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "refblas\\xgemv.m" /* pathName */
};

static emlrtRSInfo jg_emlrtRSI = {
    1,                 /* lineNo */
    "partialColLDL3_", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "DynamicRegCholManager\\partialColLDL3_.p" /* pathName */
};

static emlrtRSInfo kg_emlrtRSI = {
    63,      /* lineNo */
    "xgemm", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+blas\\xgemm."
    "m" /* pathName */
};

static emlrtRSInfo lg_emlrtRSI = {
    70,      /* lineNo */
    "xgemm", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "refblas\\xgemm.m" /* pathName */
};

static emlrtRSInfo mg_emlrtRSI = {
    71,      /* lineNo */
    "xgemm", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "refblas\\xgemm.m" /* pathName */
};

static emlrtRSInfo ng_emlrtRSI = {
    125,     /* lineNo */
    "xgemm", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "refblas\\xgemm.m" /* pathName */
};

static emlrtRSInfo og_emlrtRSI = {
    128,     /* lineNo */
    "xgemm", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "refblas\\xgemm.m" /* pathName */
};

static emlrtRSInfo pg_emlrtRSI = {
    135,     /* lineNo */
    "xgemm", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "refblas\\xgemm.m" /* pathName */
};

static emlrtBCInfo se_emlrtBCI = {
    1,                 /* iFirst */
    1,                 /* iLast */
    1,                 /* lineNo */
    1,                 /* colNo */
    "",                /* aName */
    "partialColLDL3_", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "DynamicRegCholManager\\partialColLDL3_.p", /* pName */
    3                                           /* checkKind */
};

static emlrtBCInfo te_emlrtBCI = {
    1,                 /* iFirst */
    1,                 /* iLast */
    1,                 /* lineNo */
    1,                 /* colNo */
    "",                /* aName */
    "partialColLDL3_", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "DynamicRegCholManager\\partialColLDL3_.p", /* pName */
    0                                           /* checkKind */
};

static emlrtBCInfo ue_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    1,                 /* lineNo */
    1,                 /* colNo */
    "",                /* aName */
    "partialColLDL3_", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "DynamicRegCholManager\\partialColLDL3_.p", /* pName */
    0                                           /* checkKind */
};

static emlrtRTEInfo p_emlrtRTEI = {
    18,                               /* lineNo */
    27,                               /* colNo */
    "eml_int_forloop_overflow_check", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\lib\\matlab\\eml\\eml_int_forloop_"
    "overflow_check.m" /* pName */
};

/* Function Definitions */
void partialColLDL3_(const emlrtStack *sp, j_struct_T *obj, int32_T LD_offset,
                     int32_T NColsRemain)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  int32_T LD_diagOffset;
  int32_T LD_diagOffset_tmp;
  int32_T LDimSizeP1;
  int32_T b;
  int32_T b_b;
  int32_T i;
  int32_T iac;
  int32_T idx;
  int32_T ix;
  int32_T j;
  int32_T k;
  int32_T lda;
  int32_T offsetColK;
  int32_T subBlockSize;
  int32_T subRows;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  LDimSizeP1 = obj->ldm + 1;
  b = NColsRemain - 1;
  for (k = 0; k < 48; k++) {
    real_T y;
    subRows = (NColsRemain - k) - 1;
    LD_diagOffset_tmp = LDimSizeP1 * k;
    LD_diagOffset = LD_offset + LD_diagOffset_tmp;
    for (idx = 0; idx <= subRows; idx++) {
      i = (LD_diagOffset_tmp + idx) + 1;
      if ((i < 1) || (i > 1)) {
        emlrtDynamicBoundsCheckR2012b(i, 1, 1, &se_emlrtBCI, (emlrtCTX)sp);
      }
      i = obj->FMat.size[0] * obj->FMat.size[1];
      lda = LD_diagOffset + idx;
      if ((lda < 1) || (lda > i)) {
        emlrtDynamicBoundsCheckR2012b(lda, 1, i, &ue_emlrtBCI, (emlrtCTX)sp);
      }
      obj->workspace_ = obj->FMat.data[lda - 1];
    }
    offsetColK = obj->ldm * k + 1;
    for (idx = 0; idx <= b; idx++) {
      i = offsetColK + idx;
      if ((i < 1) || (i > 1)) {
        emlrtDynamicBoundsCheckR2012b(i, 1, 1, &te_emlrtBCI, (emlrtCTX)sp);
      }
      if ((idx + 1 < 1) || (idx + 1 > 1)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, 1, &se_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      obj->workspace2_ = obj->workspace_;
    }
    st.site = &jg_emlrtRSI;
    lda = obj->ldm;
    y = obj->workspace2_;
    b_st.site = &td_emlrtRSI;
    if ((NColsRemain != 0) && (k != 0)) {
      ix = LD_offset + k;
      i = obj->ldm * (k - 1) + 1;
      c_st.site = &dd_emlrtRSI;
      if (obj->ldm == 0) {
        emlrtErrorWithMessageIdR2018a(&c_st, &p_emlrtRTEI,
                                      "Coder:builtins:VectorStride",
                                      "Coder:builtins:VectorStride", 0);
      }
      for (iac = 1; lda < 0 ? iac >= i : iac <= i; iac += lda) {
        b_b = (iac + NColsRemain) - 1;
        c_st.site = &tf_emlrtRSI;
        if ((iac <= b_b) && (b_b > 2147483646)) {
          d_st.site = &db_emlrtRSI;
          check_forloop_overflow_error(&d_st);
        }
        for (idx = iac; idx <= b_b; idx++) {
          y += obj->workspace_ * -obj->FMat.data[ix - 1];
        }
        ix += obj->ldm;
      }
    }
    obj->workspace2_ = y;
    for (idx = 0; idx <= b; idx++) {
      if ((idx + 1 < 1) || (idx + 1 > 1)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, 1, &te_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      i = offsetColK + idx;
      if ((i < 1) || (i > 1)) {
        emlrtDynamicBoundsCheckR2012b(i, 1, 1, &se_emlrtBCI, (emlrtCTX)sp);
      }
      obj->workspace_ = y;
    }
    for (idx = 0; idx <= subRows; idx++) {
      i = (LD_diagOffset_tmp + idx) + 1;
      if ((i < 1) || (i > 1)) {
        emlrtDynamicBoundsCheckR2012b(i, 1, 1, &te_emlrtBCI, (emlrtCTX)sp);
      }
      i = obj->FMat.size[0] * obj->FMat.size[1];
      lda = LD_diagOffset + idx;
      if ((lda < 1) || (lda > i)) {
        emlrtDynamicBoundsCheckR2012b(lda, 1, i, &ue_emlrtBCI, (emlrtCTX)sp);
      }
      obj->FMat.data[lda - 1] = obj->workspace_;
    }
    i = obj->FMat.size[0] * obj->FMat.size[1];
    if ((LD_diagOffset < 1) || (LD_diagOffset > i)) {
      emlrtDynamicBoundsCheckR2012b(LD_diagOffset, 1, i, &ue_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    if (muDoubleScalarAbs(obj->FMat.data[LD_diagOffset - 1]) <= obj->regTol_) {
      i = obj->FMat.size[0] * obj->FMat.size[1];
      if (LD_diagOffset > i) {
        emlrtDynamicBoundsCheckR2012b(LD_diagOffset, 1, i, &ue_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      i = obj->FMat.size[0] * obj->FMat.size[1];
      if (LD_diagOffset > i) {
        emlrtDynamicBoundsCheckR2012b(LD_diagOffset, 1, i, &ue_emlrtBCI,
                                      (emlrtCTX)sp);
      }
    }
    st.site = &jg_emlrtRSI;
    if (subRows > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx = 0; idx < subRows; idx++) {
      i = obj->FMat.size[0] * obj->FMat.size[1];
      lda = (LD_diagOffset + idx) + 1;
      if ((lda < 1) || (lda > i)) {
        emlrtDynamicBoundsCheckR2012b(lda, 1, i, &ue_emlrtBCI, (emlrtCTX)sp);
      }
      i = obj->FMat.size[0] * obj->FMat.size[1];
      if (LD_diagOffset > i) {
        emlrtDynamicBoundsCheckR2012b(LD_diagOffset, 1, i, &ue_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      i = obj->FMat.size[0] * obj->FMat.size[1];
      if (lda > i) {
        emlrtDynamicBoundsCheckR2012b(lda, 1, i, &ue_emlrtBCI, (emlrtCTX)sp);
      }
      obj->FMat.data[lda - 1] /= obj->FMat.data[LD_diagOffset - 1];
    }
  }
  st.site = &jg_emlrtRSI;
  if (NColsRemain - 1 > 2147483599) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (j = 48; j <= b; j += 48) {
    int32_T c_b;
    int32_T m;
    subRows = NColsRemain - j;
    subBlockSize = muIntScalarMin_sint32(48, subRows);
    LD_diagOffset_tmp = j + subBlockSize;
    b_b = LD_diagOffset_tmp - 1;
    st.site = &jg_emlrtRSI;
    if ((j <= LD_diagOffset_tmp - 1) && (LD_diagOffset_tmp - 1 > 2147483646)) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (k = j; k <= b_b; k++) {
      m = LD_diagOffset_tmp - k;
      LD_diagOffset = (LD_offset + LDimSizeP1 * k) - 1;
      for (idx = 0; idx < 48; idx++) {
        if (idx + 1 > 1) {
          emlrtDynamicBoundsCheckR2012b(idx + 1, 1, 1, &se_emlrtBCI,
                                        (emlrtCTX)sp);
        }
        i = obj->FMat.size[0] * obj->FMat.size[1];
        lda = (LD_offset + k) + idx * obj->ldm;
        if ((lda < 1) || (lda > i)) {
          emlrtDynamicBoundsCheckR2012b(lda, 1, i, &ue_emlrtBCI, (emlrtCTX)sp);
        }
        obj->workspace2_ = obj->FMat.data[lda - 1];
      }
      st.site = &jg_emlrtRSI;
      ix = k + 1;
      lda = obj->ldm;
      b_st.site = &td_emlrtRSI;
      if (m != 0) {
        c_b = (k + obj->ldm * 47) + 1;
        c_st.site = &dd_emlrtRSI;
        if ((k + 1 <= c_b) && (c_b > MAX_int32_T - obj->ldm)) {
          d_st.site = &db_emlrtRSI;
          check_forloop_overflow_error(&d_st);
        }
        for (iac = ix; lda < 0 ? iac >= c_b : iac <= c_b; iac += lda) {
          offsetColK = (iac + m) - 1;
          c_st.site = &tf_emlrtRSI;
          if ((iac <= offsetColK) && (offsetColK > 2147483646)) {
            d_st.site = &db_emlrtRSI;
            check_forloop_overflow_error(&d_st);
          }
          for (idx = iac; idx <= offsetColK; idx++) {
            i = (LD_diagOffset + idx) - iac;
            obj->FMat.data[i] += obj->workspace_ * -obj->workspace2_;
          }
        }
      }
    }
    if (LD_diagOffset_tmp < NColsRemain) {
      m = subRows - subBlockSize;
      iac = ((LD_offset + subBlockSize) + LDimSizeP1 * j) - 1;
      i = subBlockSize - 1;
      for (idx = 0; idx < 48; idx++) {
        ix = idx * obj->ldm;
        offsetColK = (LD_offset + j) + ix;
        for (LD_diagOffset_tmp = 0; LD_diagOffset_tmp <= i;
             LD_diagOffset_tmp++) {
          lda = (ix + LD_diagOffset_tmp) + 1;
          if ((lda < 1) || (lda > 1)) {
            emlrtDynamicBoundsCheckR2012b(lda, 1, 1, &se_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          lda = obj->FMat.size[0] * obj->FMat.size[1];
          subRows = offsetColK + LD_diagOffset_tmp;
          if ((subRows < 1) || (subRows > lda)) {
            emlrtDynamicBoundsCheckR2012b(subRows, 1, lda, &ue_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          obj->workspace2_ = obj->FMat.data[subRows - 1];
        }
      }
      st.site = &jg_emlrtRSI;
      LD_diagOffset_tmp = obj->ldm;
      LD_diagOffset = obj->ldm;
      b_st.site = &kg_emlrtRSI;
      if ((m != 0) && (subBlockSize != 0)) {
        ix = iac + obj->ldm * (subBlockSize - 1);
        c_st.site = &lg_emlrtRSI;
        if ((iac <= ix) && (ix > MAX_int32_T - obj->ldm)) {
          d_st.site = &db_emlrtRSI;
          check_forloop_overflow_error(&d_st);
        }
        for (lda = iac; LD_diagOffset < 0 ? lda >= ix : lda <= ix;
             lda += LD_diagOffset) {
          b_b = lda + m;
          c_st.site = &mg_emlrtRSI;
          if ((lda + 1 <= b_b) && (b_b > 2147483646)) {
            d_st.site = &db_emlrtRSI;
            check_forloop_overflow_error(&d_st);
          }
        }
        offsetColK = 0;
        c_st.site = &ng_emlrtRSI;
        if ((iac <= ix) && (ix > MAX_int32_T - obj->ldm)) {
          d_st.site = &db_emlrtRSI;
          check_forloop_overflow_error(&d_st);
        }
        for (lda = iac; LD_diagOffset < 0 ? lda >= ix : lda <= ix;
             lda += LD_diagOffset) {
          offsetColK++;
          b_b = offsetColK + LD_diagOffset_tmp * 47;
          c_st.site = &og_emlrtRSI;
          if ((offsetColK <= b_b) && (b_b > MAX_int32_T - LD_diagOffset_tmp)) {
            d_st.site = &db_emlrtRSI;
            check_forloop_overflow_error(&d_st);
          }
          for (subBlockSize = offsetColK;
               LD_diagOffset_tmp < 0 ? subBlockSize >= b_b
                                     : subBlockSize <= b_b;
               subBlockSize += LD_diagOffset_tmp) {
            subRows = lda + 1;
            c_b = lda + m;
            c_st.site = &pg_emlrtRSI;
            if ((lda + 1 <= c_b) && (c_b > 2147483646)) {
              d_st.site = &db_emlrtRSI;
              check_forloop_overflow_error(&d_st);
            }
            for (idx = subRows; idx <= c_b; idx++) {
              obj->FMat.data[idx - 1] += -obj->workspace2_ * obj->workspace_;
            }
          }
        }
      }
    }
  }
}

/* End of code generation (partialColLDL3_.c) */
