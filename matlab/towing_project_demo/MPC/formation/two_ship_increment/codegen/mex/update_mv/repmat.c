/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * repmat.c
 *
 * Code generation for function 'repmat'
 *
 */

/* Include files */
#include "repmat.h"
#include "eml_int_forloop_overflow_check.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "update_mv.h"
#include "update_mv_data.h"
#include "update_mv_emxutil.h"

/* Variable Definitions */
static emlrtRSInfo cc_emlrtRSI = { 28, /* lineNo */
  "repmat",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m"/* pathName */
};

static emlrtRSInfo dc_emlrtRSI = { 69, /* lineNo */
  "repmat",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m"/* pathName */
};

static emlrtMCInfo d_emlrtMCI = { 47,  /* lineNo */
  5,                                   /* colNo */
  "repmat",                            /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m"/* pName */
};

static emlrtDCInfo q_emlrtDCI = { 31,  /* lineNo */
  14,                                  /* colNo */
  "repmat",                            /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m",/* pName */
  4                                    /* checkKind */
};

static emlrtRTEInfo pb_emlrtRTEI = { 1,/* lineNo */
  14,                                  /* colNo */
  "repmat",                            /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m"/* pName */
};

static emlrtRSInfo ud_emlrtRSI = { 47, /* lineNo */
  "repmat",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m"/* pathName */
};

/* Function Declarations */
static void error(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location);

/* Function Definitions */
static void error(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = b;
  emlrtCallMATLABR2012b(sp, 0, NULL, 1, &pArray, "error", true, location);
}

void repmat(const emlrtStack *sp, const real_T a[6], real_T varargin_1,
            emxArray_real_T *b)
{
  real_T b_varargin_1;
  int32_T outsize_idx_0_tmp;
  int32_T outsize_idx_0;
  int32_T itilerow;
  const mxArray *y;
  const mxArray *m;
  static const int32_T iv[2] = { 1, 15 };

  static const char_T u[15] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'p', 'm', 'a',
    'x', 's', 'i', 'z', 'e' };

  int32_T k;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &cc_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  if ((varargin_1 != muDoubleScalarFloor(varargin_1)) || muDoubleScalarIsInf
      (varargin_1) || (varargin_1 < -2.147483648E+9) || (varargin_1 >
       2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(&st, &f_emlrtRTEI,
      "Coder:MATLAB:NonIntegerInput", "Coder:MATLAB:NonIntegerInput", 4, 12,
      MIN_int32_T, 12, MAX_int32_T);
  }

  if (varargin_1 <= 0.0) {
    b_varargin_1 = 0.0;
  } else {
    b_varargin_1 = varargin_1;
  }

  if (!(b_varargin_1 <= 2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(&st, &g_emlrtRTEI, "Coder:MATLAB:pmaxsize",
      "Coder:MATLAB:pmaxsize", 0);
  }

  if (!(varargin_1 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(varargin_1, &q_emlrtDCI, sp);
  }

  outsize_idx_0_tmp = (int32_T)varargin_1;
  outsize_idx_0 = 6 * outsize_idx_0_tmp;
  if (!(outsize_idx_0 == 6.0 * (real_T)outsize_idx_0_tmp)) {
    y = NULL;
    m = emlrtCreateCharArray(2, &iv[0]);
    emlrtInitCharArrayR2013a(sp, 15, m, &u[0]);
    emlrtAssign(&y, m);
    st.site = &ud_emlrtRSI;
    error(&st, y, &d_emlrtMCI);
  }

  itilerow = b->size[0];
  b->size[0] = outsize_idx_0;
  emxEnsureCapacity_real_T(sp, b, itilerow, &pb_emlrtRTEI);
  st.site = &dc_emlrtRSI;
  if ((1 <= outsize_idx_0_tmp) && (outsize_idx_0_tmp > 2147483646)) {
    b_st.site = &ec_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (itilerow = 0; itilerow < outsize_idx_0_tmp; itilerow++) {
    outsize_idx_0 = itilerow * 6;
    for (k = 0; k < 6; k++) {
      b->data[outsize_idx_0 + k] = a[k];
    }
  }
}

/* End of code generation (repmat.c) */
