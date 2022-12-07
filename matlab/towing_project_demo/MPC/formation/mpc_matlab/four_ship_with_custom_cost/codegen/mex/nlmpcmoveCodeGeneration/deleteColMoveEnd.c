/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * deleteColMoveEnd.c
 *
 * Code generation for function 'deleteColMoveEnd'
 *
 */

/* Include files */
#include "deleteColMoveEnd.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo dg_emlrtRSI = {
    1,                  /* lineNo */
    "deleteColMoveEnd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "QRManager\\deleteColMoveEnd.p" /* pathName */
};

static emlrtBCInfo id_emlrtBCI = {
    -1,                 /* iFirst */
    -1,                 /* iLast */
    1,                  /* lineNo */
    1,                  /* colNo */
    "",                 /* aName */
    "deleteColMoveEnd", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "QRManager\\deleteColMoveEnd.p", /* pName */
    0                                /* checkKind */
};

/* Function Definitions */
void deleteColMoveEnd(const emlrtStack *sp, i_struct_T *obj, int32_T idx)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T c;
  real_T d;
  real_T s;
  real_T temp;
  int32_T b_i;
  int32_T b_k;
  int32_T i;
  int32_T k;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  if (obj->usedPivoting) {
    boolean_T exitg1;
    i = 1;
    exitg1 = false;
    while ((!exitg1) && (i <= obj->ncols)) {
      b_i = obj->jpvt.size[0];
      if (i > b_i) {
        emlrtDynamicBoundsCheckR2012b(i, 1, b_i, &id_emlrtBCI, (emlrtCTX)sp);
      }
      if (obj->jpvt.data[i - 1] != idx) {
        i++;
      } else {
        exitg1 = true;
      }
    }
    idx = i;
  }
  if (idx >= obj->ncols) {
    obj->ncols--;
  } else {
    int32_T i1;
    int32_T ix;
    b_i = obj->jpvt.size[0];
    if ((obj->ncols < 1) || (obj->ncols > b_i)) {
      emlrtDynamicBoundsCheckR2012b(obj->ncols, 1, b_i, &id_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    b_i = obj->jpvt.size[0];
    if ((idx < 1) || (idx > b_i)) {
      emlrtDynamicBoundsCheckR2012b(idx, 1, b_i, &id_emlrtBCI, (emlrtCTX)sp);
    }
    obj->jpvt.data[idx - 1] = obj->jpvt.data[obj->ncols - 1];
    i = obj->minRowCol;
    st.site = &dg_emlrtRSI;
    if (obj->minRowCol > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (k = 0; k < i; k++) {
      b_i = obj->QR.size[0] * obj->QR.size[1];
      i1 = (k + obj->ldq * (obj->ncols - 1)) + 1;
      if ((i1 < 1) || (i1 > b_i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, b_i, &id_emlrtBCI, (emlrtCTX)sp);
      }
      ix = (k + obj->ldq * (idx - 1)) + 1;
      if ((ix < 1) || (ix > b_i)) {
        emlrtDynamicBoundsCheckR2012b(ix, 1, b_i, &id_emlrtBCI, (emlrtCTX)sp);
      }
      obj->QR.data[ix - 1] = obj->QR.data[i1 - 1];
    }
    obj->ncols--;
    obj->minRowCol = muIntScalarMin_sint32(obj->mrows, obj->ncols);
    if (idx < obj->mrows) {
      int32_T endIdx;
      int32_T idxRotGCol;
      int32_T n;
      int32_T temp_tmp;
      i = obj->mrows - 1;
      endIdx = muIntScalarMin_sint32(i, obj->ncols);
      k = endIdx;
      idxRotGCol = obj->ldq * (idx - 1);
      while (k >= idx) {
        st.site = &dg_emlrtRSI;
        b_i = obj->QR.size[0] * obj->QR.size[1];
        i1 = k + idxRotGCol;
        if ((i1 < 1) || (i1 > b_i)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, b_i, &id_emlrtBCI, &st);
        }
        temp = obj->QR.data[i1 - 1];
        b_i = obj->QR.size[0] * obj->QR.size[1];
        if ((i1 + 1 < 1) || (i1 + 1 > b_i)) {
          emlrtDynamicBoundsCheckR2012b(i1 + 1, 1, b_i, &id_emlrtBCI, &st);
        }
        d = obj->QR.data[i1];
        b_st.site = &ag_emlrtRSI;
        c = 0.0;
        s = 0.0;
        drotg(&temp, &d, &c, &s);
        b_i = obj->QR.size[0] * obj->QR.size[1];
        if ((i1 < 1) || (i1 > b_i)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, b_i, &id_emlrtBCI, (emlrtCTX)sp);
        }
        obj->QR.data[i1 - 1] = temp;
        b_i = obj->QR.size[0] * obj->QR.size[1];
        if ((i1 + 1 < 1) || (i1 + 1 > b_i)) {
          emlrtDynamicBoundsCheckR2012b(i1 + 1, 1, b_i, &id_emlrtBCI,
                                        (emlrtCTX)sp);
        }
        obj->QR.data[i1] = d;
        b_i = obj->QR.size[0] * obj->QR.size[1];
        i1 = (k + obj->ldq * (k - 1)) + 1;
        if ((i1 < 1) || (i1 > b_i)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, b_i, &id_emlrtBCI, (emlrtCTX)sp);
        }
        obj->QR.data[i1 - 1] = 0.0;
        i = k + obj->ldq * idx;
        st.site = &dg_emlrtRSI;
        n = obj->ncols - idx;
        if (n >= 1) {
          ix = i - 1;
          b_st.site = &cg_emlrtRSI;
          if (n > 2147483646) {
            c_st.site = &db_emlrtRSI;
            check_forloop_overflow_error(&c_st);
          }
          for (b_k = 0; b_k < n; b_k++) {
            temp = c * obj->QR.data[ix] + s * obj->QR.data[i];
            obj->QR.data[i] = c * obj->QR.data[i] - s * obj->QR.data[ix];
            obj->QR.data[ix] = temp;
            i += obj->ldq;
            ix += obj->ldq;
          }
        }
        b_i = obj->ldq * (k - 1);
        i = obj->ldq + b_i;
        st.site = &dg_emlrtRSI;
        n = obj->mrows;
        b_st.site = &cg_emlrtRSI;
        if (obj->mrows > 2147483646) {
          c_st.site = &db_emlrtRSI;
          check_forloop_overflow_error(&c_st);
        }
        for (b_k = 0; b_k < n; b_k++) {
          ix = i + b_k;
          temp_tmp = b_i + b_k;
          temp = c * obj->Q.data[temp_tmp] + s * obj->Q.data[ix];
          obj->Q.data[ix] = c * obj->Q.data[ix] - s * obj->Q.data[temp_tmp];
          obj->Q.data[temp_tmp] = temp;
        }
        k--;
      }
      b_i = idx + 1;
      st.site = &dg_emlrtRSI;
      for (k = b_i; k <= endIdx; k++) {
        st.site = &dg_emlrtRSI;
        i1 = obj->QR.size[0] * obj->QR.size[1];
        ix = k + obj->ldq * (k - 1);
        if ((ix < 1) || (ix > i1)) {
          emlrtDynamicBoundsCheckR2012b(ix, 1, i1, &id_emlrtBCI, &st);
        }
        temp = obj->QR.data[ix - 1];
        i1 = obj->QR.size[0] * obj->QR.size[1];
        if ((ix + 1 < 1) || (ix + 1 > i1)) {
          emlrtDynamicBoundsCheckR2012b(ix + 1, 1, i1, &id_emlrtBCI, &st);
        }
        d = obj->QR.data[ix];
        b_st.site = &ag_emlrtRSI;
        c = 0.0;
        s = 0.0;
        drotg(&temp, &d, &c, &s);
        i1 = obj->QR.size[0] * obj->QR.size[1];
        if (ix > i1) {
          emlrtDynamicBoundsCheckR2012b(ix, 1, i1, &id_emlrtBCI, (emlrtCTX)sp);
        }
        obj->QR.data[ix - 1] = temp;
        i1 = obj->QR.size[0] * obj->QR.size[1];
        if ((ix + 1 < 1) || (ix + 1 > i1)) {
          emlrtDynamicBoundsCheckR2012b(ix + 1, 1, i1, &id_emlrtBCI,
                                        (emlrtCTX)sp);
        }
        obj->QR.data[ix] = d;
        i = k * (obj->ldq + 1);
        st.site = &dg_emlrtRSI;
        n = obj->ncols - k;
        if (n >= 1) {
          ix = i - 1;
          b_st.site = &cg_emlrtRSI;
          if (n > 2147483646) {
            c_st.site = &db_emlrtRSI;
            check_forloop_overflow_error(&c_st);
          }
          for (b_k = 0; b_k < n; b_k++) {
            temp = c * obj->QR.data[ix] + s * obj->QR.data[i];
            obj->QR.data[i] = c * obj->QR.data[i] - s * obj->QR.data[ix];
            obj->QR.data[ix] = temp;
            i += obj->ldq;
            ix += obj->ldq;
          }
        }
        i1 = obj->ldq * (k - 1);
        i = obj->ldq + i1;
        st.site = &dg_emlrtRSI;
        n = obj->mrows;
        b_st.site = &cg_emlrtRSI;
        if (obj->mrows > 2147483646) {
          c_st.site = &db_emlrtRSI;
          check_forloop_overflow_error(&c_st);
        }
        for (b_k = 0; b_k < n; b_k++) {
          ix = i + b_k;
          temp_tmp = i1 + b_k;
          temp = c * obj->Q.data[temp_tmp] + s * obj->Q.data[ix];
          obj->Q.data[ix] = c * obj->Q.data[ix] - s * obj->Q.data[temp_tmp];
          obj->Q.data[temp_tmp] = temp;
        }
      }
    }
  }
}

/* End of code generation (deleteColMoveEnd.c) */
