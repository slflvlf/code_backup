/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * solve1.c
 *
 * Code generation for function 'solve1'
 *
 */

/* Include files */
#include "solve1.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo tg_emlrtRSI = {
    1,       /* lineNo */
    "solve", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "DynamicRegCholManager\\solve.p" /* pathName */
};

static emlrtBCInfo md_emlrtBCI = {
    -1,      /* iFirst */
    -1,      /* iLast */
    1,       /* lineNo */
    1,       /* colNo */
    "",      /* aName */
    "solve", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "DynamicRegCholManager\\solve.p", /* pName */
    0                                 /* checkKind */
};

/* Function Definitions */
void b_solve(const emlrtStack *sp, const j_struct_T *obj, real_T rhs_data[],
             const int32_T *rhs_size)
{
  ptrdiff_t incx_t;
  ptrdiff_t lda_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack st;
  int32_T b;
  int32_T idx;
  char_T DIAGA1;
  char_T TRANSA1;
  char_T UPLO1;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &tg_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  if (obj->ndims >= 1) {
    DIAGA1 = 'U';
    TRANSA1 = 'N';
    UPLO1 = 'L';
    n_t = (ptrdiff_t)obj->ndims;
    lda_t = (ptrdiff_t)obj->ldm;
    incx_t = (ptrdiff_t)1;
    dtrsv(&UPLO1, &TRANSA1, &DIAGA1, &n_t, &obj->FMat.data[0], &lda_t,
          &rhs_data[0], &incx_t);
  }
  b = obj->ndims;
  st.site = &tg_emlrtRSI;
  if (obj->ndims > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = 0; idx < b; idx++) {
    int32_T i;
    int32_T i1;
    i = *rhs_size;
    if ((idx + 1 < 1) || (idx + 1 > i)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &md_emlrtBCI, (emlrtCTX)sp);
    }
    i = obj->FMat.size[0] * obj->FMat.size[1];
    i1 = (idx + obj->ldm * idx) + 1;
    if ((i1 < 1) || (i1 > i)) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i, &md_emlrtBCI, (emlrtCTX)sp);
    }
    i = *rhs_size;
    if (idx + 1 > i) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &md_emlrtBCI, (emlrtCTX)sp);
    }
    rhs_data[idx] /= obj->FMat.data[i1 - 1];
  }
  st.site = &tg_emlrtRSI;
  if (obj->ndims >= 1) {
    DIAGA1 = 'U';
    TRANSA1 = 'T';
    UPLO1 = 'L';
    n_t = (ptrdiff_t)obj->ndims;
    lda_t = (ptrdiff_t)obj->ldm;
    incx_t = (ptrdiff_t)1;
    dtrsv(&UPLO1, &TRANSA1, &DIAGA1, &n_t, &obj->FMat.data[0], &lda_t,
          &rhs_data[0], &incx_t);
  }
}

void c_solve(const emlrtStack *sp, const j_struct_T *obj, real_T rhs_data[],
             const int32_T rhs_size[2])
{
  ptrdiff_t incx_t;
  ptrdiff_t lda_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack st;
  int32_T b;
  int32_T idx;
  char_T DIAGA1;
  char_T TRANSA1;
  char_T UPLO1;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &tg_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  if (obj->ndims >= 1) {
    DIAGA1 = 'U';
    TRANSA1 = 'N';
    UPLO1 = 'L';
    n_t = (ptrdiff_t)obj->ndims;
    lda_t = (ptrdiff_t)obj->ldm;
    incx_t = (ptrdiff_t)1;
    dtrsv(&UPLO1, &TRANSA1, &DIAGA1, &n_t, &obj->FMat.data[0], &lda_t,
          &rhs_data[0], &incx_t);
  }
  b = obj->ndims;
  st.site = &tg_emlrtRSI;
  if (obj->ndims > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = 0; idx < b; idx++) {
    int32_T i;
    int32_T i1;
    int32_T i2;
    i = rhs_size[0] * rhs_size[1];
    if ((idx + 1 < 1) || (idx + 1 > i)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &md_emlrtBCI, (emlrtCTX)sp);
    }
    i1 = obj->FMat.size[0] * obj->FMat.size[1];
    i2 = (idx + obj->ldm * idx) + 1;
    if ((i2 < 1) || (i2 > i1)) {
      emlrtDynamicBoundsCheckR2012b(i2, 1, i1, &md_emlrtBCI, (emlrtCTX)sp);
    }
    if (idx + 1 > i) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &md_emlrtBCI, (emlrtCTX)sp);
    }
    rhs_data[idx] /= obj->FMat.data[i2 - 1];
  }
  st.site = &tg_emlrtRSI;
  if (obj->ndims >= 1) {
    DIAGA1 = 'U';
    TRANSA1 = 'T';
    UPLO1 = 'L';
    n_t = (ptrdiff_t)obj->ndims;
    lda_t = (ptrdiff_t)obj->ldm;
    incx_t = (ptrdiff_t)1;
    dtrsv(&UPLO1, &TRANSA1, &DIAGA1, &n_t, &obj->FMat.data[0], &lda_t,
          &rhs_data[0], &incx_t);
  }
}

/* End of code generation (solve1.c) */
