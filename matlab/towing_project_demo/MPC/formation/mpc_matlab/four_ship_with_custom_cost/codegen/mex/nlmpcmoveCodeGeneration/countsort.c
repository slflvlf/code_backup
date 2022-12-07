/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * countsort.c
 *
 * Code generation for function 'countsort'
 *
 */

/* Include files */
#include "countsort.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo re_emlrtRSI =
    {
        1,           /* lineNo */
        "countsort", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
        "utils\\countsort.p" /* pathName */
};

static emlrtBCInfo lc_emlrtBCI = {
    -1,          /* iFirst */
    -1,          /* iLast */
    1,           /* lineNo */
    1,           /* colNo */
    "",          /* aName */
    "countsort", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+utils\\countsort."
    "p", /* pName */
    0    /* checkKind */
};

/* Function Definitions */
void countsort(const emlrtStack *sp, int32_T x_data[], const int32_T *x_size,
               int32_T xLen, int32_T workspace_data[],
               const int32_T *workspace_size, int32_T xMin, int32_T xMax)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T idx;
  int32_T idxFill;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  if ((xLen > 1) && (xMax > xMin)) {
    int32_T b_tmp;
    int32_T i;
    int32_T idxEnd;
    int32_T idxStart;
    b_tmp = (xMax - xMin) + 1;
    st.site = &re_emlrtRSI;
    if (b_tmp > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx = 0; idx < b_tmp; idx++) {
      i = *workspace_size;
      if ((idx + 1 < 1) || (idx + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &lc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      workspace_data[idx] = 0;
    }
    st.site = &re_emlrtRSI;
    if (xLen > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    i = *workspace_size;
    idxStart = *x_size;
    for (idx = 0; idx < xLen; idx++) {
      if (idx + 1 > idxStart) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, idxStart, &lc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      idxEnd = (x_data[idx] - xMin) + 1;
      if ((idxEnd < 1) || (idxEnd > i)) {
        emlrtDynamicBoundsCheckR2012b(idxEnd, 1, i, &lc_emlrtBCI, (emlrtCTX)sp);
      }
      if (idx + 1 > idxStart) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, idxStart, &lc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      if ((idxEnd < 1) || (idxEnd > i)) {
        emlrtDynamicBoundsCheckR2012b(idxEnd, 1, i, &lc_emlrtBCI, (emlrtCTX)sp);
      }
      workspace_data[idxEnd - 1]++;
    }
    st.site = &re_emlrtRSI;
    for (idx = 2; idx <= b_tmp; idx++) {
      if (idx > i) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &lc_emlrtBCI, (emlrtCTX)sp);
      }
      if (idx - 1 > i) {
        emlrtDynamicBoundsCheckR2012b(idx - 1, 1, i, &lc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      if (idx > i) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &lc_emlrtBCI, (emlrtCTX)sp);
      }
      workspace_data[idx - 1] += workspace_data[idx - 2];
    }
    idxStart = 1;
    idxEnd = workspace_data[0];
    st.site = &re_emlrtRSI;
    if (b_tmp - 1 > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx = 0; idx <= b_tmp - 2; idx++) {
      st.site = &re_emlrtRSI;
      if ((idxStart <= idxEnd) && (idxEnd > 2147483646)) {
        b_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }
      for (idxFill = idxStart; idxFill <= idxEnd; idxFill++) {
        i = *x_size;
        if ((idxFill < 1) || (idxFill > i)) {
          emlrtDynamicBoundsCheckR2012b(idxFill, 1, i, &lc_emlrtBCI,
                                        (emlrtCTX)sp);
        }
        x_data[idxFill - 1] = idx + xMin;
      }
      i = *workspace_size;
      if ((idx + 1 < 1) || (idx + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &lc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      idxStart = workspace_data[idx] + 1;
      if ((idx + 2 < 1) || (idx + 2 > i)) {
        emlrtDynamicBoundsCheckR2012b(idx + 2, 1, i, &lc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      idxEnd = workspace_data[idx + 1];
    }
    st.site = &re_emlrtRSI;
    if ((idxStart <= idxEnd) && (idxEnd > 2147483646)) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx = idxStart; idx <= idxEnd; idx++) {
      i = *x_size;
      if ((idx < 1) || (idx > i)) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &lc_emlrtBCI, (emlrtCTX)sp);
      }
      x_data[idx - 1] = xMax;
    }
  }
}

/* End of code generation (countsort.c) */
