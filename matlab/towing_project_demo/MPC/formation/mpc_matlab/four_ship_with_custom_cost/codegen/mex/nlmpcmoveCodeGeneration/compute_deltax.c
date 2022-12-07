/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compute_deltax.c
 *
 * Code generation for function 'compute_deltax'
 *
 */

/* Include files */
#include "compute_deltax.h"
#include "eml_int_forloop_overflow_check.h"
#include "factor.h"
#include "factor1.h"
#include "fullColLDL2_.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "partialColLDL3_.h"
#include "rt_nonfinite.h"
#include "solve.h"
#include "solve1.h"
#include "xgemm.h"
#include "xgemv.h"
#include "xpotrf.h"
#include "xtrsv.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo jc_emlrtRSI = {
    37,      /* lineNo */
    "xscal", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+blas\\xscal."
    "m" /* pathName */
};

static emlrtRSInfo kc_emlrtRSI = {
    49,           /* lineNo */
    "xscal_blas", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+blas\\xscal."
    "m" /* pathName */
};

static emlrtRSInfo rc_emlrtRSI = {
    91,           /* lineNo */
    "xgemv_blas", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\eml\\+coder\\+internal\\+blas\\xgemv."
    "m" /* pathName */
};

static emlrtRSInfo eg_emlrtRSI = {
    1,                /* lineNo */
    "compute_deltax", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "qpactiveset\\compute_deltax.p" /* pathName */
};

static emlrtRSInfo sg_emlrtRSI = {
    1,       /* lineNo */
    "solve", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "CholManager\\solve.p" /* pathName */
};

static emlrtRSInfo ug_emlrtRSI = {
    1,                         /* lineNo */
    "computeProjectedHessian", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "qpactiveset\\computeProjectedHessian.p" /* pathName */
};

static emlrtRSInfo vg_emlrtRSI = {
    1,                                     /* lineNo */
    "computeProjectedHessian_regularized", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\+relaxed\\computeProjectedHessian_regularized.p" /* pathName */
};

static emlrtRSInfo wg_emlrtRSI = {
    1,        /* lineNo */
    "factor", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "qpactiveset\\compute_deltax.p" /* pathName */
};

static emlrtRSInfo xg_emlrtRSI = {
    1,       /* lineNo */
    "solve", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "qpactiveset\\compute_deltax.p" /* pathName */
};

static emlrtBCInfo kd_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    1,                /* lineNo */
    1,                /* colNo */
    "",               /* aName */
    "compute_deltax", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "qpactiveset\\compute_deltax.p", /* pName */
    0                                /* checkKind */
};

static emlrtBCInfo ld_emlrtBCI = {
    -1,                                    /* iFirst */
    -1,                                    /* iLast */
    1,                                     /* lineNo */
    1,                                     /* colNo */
    "",                                    /* aName */
    "computeProjectedHessian_regularized", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\+relaxed\\computeProjectedHessian_regularized.p", /* pName */
    0                                                        /* checkKind */
};

/* Function Definitions */
void compute_deltax(const emlrtStack *sp, const real_T H[70225],
                    e_struct_T *solution, g_struct_T *memspace,
                    const i_struct_T *qrmanager, j_struct_T *cholmanager,
                    const f_struct_T *objective, boolean_T alwaysPositiveDef)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  real_T alpha1;
  real_T beta1;
  int32_T LDimSizeP1;
  int32_T idx_row;
  int32_T mNull_tmp;
  int32_T order;
  char_T TRANSA;
  char_T TRANSA1;
  char_T UPLO1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  order = qrmanager->mrows;
  mNull_tmp = qrmanager->mrows - qrmanager->ncols;
  if (mNull_tmp <= 0) {
    st.site = &eg_emlrtRSI;
    if (qrmanager->mrows > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (LDimSizeP1 = 0; LDimSizeP1 < order; LDimSizeP1++) {
      int32_T i;
      i = solution->searchDir.size[0];
      if ((LDimSizeP1 + 1 < 1) || (LDimSizeP1 + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(LDimSizeP1 + 1, 1, i, &kd_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      solution->searchDir.data[LDimSizeP1] = 0.0;
    }
  } else {
    int32_T i;
    st.site = &eg_emlrtRSI;
    if (qrmanager->mrows > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (LDimSizeP1 = 0; LDimSizeP1 < order; LDimSizeP1++) {
      i = objective->grad.size[0];
      if ((LDimSizeP1 + 1 < 1) || (LDimSizeP1 + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(LDimSizeP1 + 1, 1, i, &kd_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      i = solution->searchDir.size[0];
      if (LDimSizeP1 + 1 > i) {
        emlrtDynamicBoundsCheckR2012b(LDimSizeP1 + 1, 1, i, &kd_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      solution->searchDir.data[LDimSizeP1] = -objective->grad.data[LDimSizeP1];
    }
    if (qrmanager->ncols <= 0) {
      switch (objective->objtype) {
      case 5:
        break;
      case 3:
        st.site = &eg_emlrtRSI;
        if (alwaysPositiveDef) {
          b_st.site = &eg_emlrtRSI;
          factor(&b_st, cholmanager, H, qrmanager->mrows, qrmanager->mrows);
        } else {
          b_st.site = &eg_emlrtRSI;
          b_factor(&b_st, cholmanager, H, qrmanager->mrows, qrmanager->mrows);
        }
        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          st.site = &eg_emlrtRSI;
          if (alwaysPositiveDef) {
            b_st.site = &eg_emlrtRSI;
            solve(cholmanager, solution->searchDir.data);
          } else {
            b_st.site = &eg_emlrtRSI;
            b_solve(&b_st, cholmanager, solution->searchDir.data,
                    &solution->searchDir.size[0]);
          }
        }
        break;
      default: {
        if (alwaysPositiveDef) {
          st.site = &eg_emlrtRSI;
          b_st.site = &eg_emlrtRSI;
          factor(&b_st, cholmanager, H, objective->nvar, objective->nvar);
          if (cholmanager->info != 0) {
            solution->state = -6;
          } else {
            int32_T k;
            st.site = &eg_emlrtRSI;
            b_st.site = &eg_emlrtRSI;
            solve(cholmanager, solution->searchDir.data);
            st.site = &eg_emlrtRSI;
            k = qrmanager->mrows - objective->nvar;
            alpha1 = 1.0 / objective->beta;
            if (k >= 1) {
              b_st.site = &jc_emlrtRSI;
              c_st.site = &kc_emlrtRSI;
              n_t = (ptrdiff_t)k;
              incx_t = (ptrdiff_t)1;
              dscal(&n_t, &alpha1, &solution->searchDir.data[objective->nvar],
                    &incx_t);
            }
          }
        }
      } break;
      }
    } else {
      int32_T nullStartIdx_tmp;
      nullStartIdx_tmp = qrmanager->ldq * qrmanager->ncols + 1;
      if (objective->objtype == 5) {
        st.site = &eg_emlrtRSI;
        if (mNull_tmp > 2147483646) {
          b_st.site = &db_emlrtRSI;
          check_forloop_overflow_error(&b_st);
        }
        i = qrmanager->Q.size[0] * qrmanager->Q.size[1];
        for (LDimSizeP1 = 0; LDimSizeP1 < mNull_tmp; LDimSizeP1++) {
          int32_T i1;
          int32_T k;
          i1 = order + qrmanager->ldq * (qrmanager->ncols + LDimSizeP1);
          if ((i1 < 1) || (i1 > i)) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, i, &kd_emlrtBCI, (emlrtCTX)sp);
          }
          k = memspace->workspace_double.size[0] *
              memspace->workspace_double.size[1];
          if (LDimSizeP1 + 1 > k) {
            emlrtDynamicBoundsCheckR2012b(LDimSizeP1 + 1, 1, k, &kd_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          memspace->workspace_double.data[LDimSizeP1] =
              -qrmanager->Q.data[i1 - 1];
        }
        st.site = &eg_emlrtRSI;
        g_xgemv(qrmanager->mrows, mNull_tmp, qrmanager->Q.data,
                nullStartIdx_tmp, qrmanager->ldq,
                memspace->workspace_double.data, solution->searchDir.data);
      } else {
        int32_T i1;
        int32_T k;
        int32_T ldw;
        if (objective->objtype == 3) {
          st.site = &eg_emlrtRSI;
          b_st.site = &ug_emlrtRSI;
          c_xgemm(qrmanager->mrows, mNull_tmp, qrmanager->mrows, H,
                  qrmanager->mrows, qrmanager->Q.data, nullStartIdx_tmp,
                  qrmanager->ldq, memspace->workspace_double.data,
                  memspace->workspace_double.size[0]);
          b_st.site = &ug_emlrtRSI;
          d_xgemm(mNull_tmp, mNull_tmp, qrmanager->mrows, qrmanager->Q.data,
                  nullStartIdx_tmp, qrmanager->ldq,
                  memspace->workspace_double.data,
                  memspace->workspace_double.size[0], cholmanager->FMat.data,
                  cholmanager->ldm);
        } else if (alwaysPositiveDef) {
          k = objective->nvar + 1;
          st.site = &eg_emlrtRSI;
          ldw = memspace->workspace_double.size[0];
          b_st.site = &vg_emlrtRSI;
          c_xgemm(objective->nvar, mNull_tmp, objective->nvar, H,
                  objective->nvar, qrmanager->Q.data, nullStartIdx_tmp,
                  qrmanager->ldq, memspace->workspace_double.data,
                  memspace->workspace_double.size[0]);
          b_st.site = &vg_emlrtRSI;
          if (mNull_tmp > 2147483646) {
            c_st.site = &db_emlrtRSI;
            check_forloop_overflow_error(&c_st);
          }
          for (LDimSizeP1 = 0; LDimSizeP1 < mNull_tmp; LDimSizeP1++) {
            b_st.site = &vg_emlrtRSI;
            if ((k <= order) && (order > 2147483646)) {
              c_st.site = &db_emlrtRSI;
              check_forloop_overflow_error(&c_st);
            }
            for (idx_row = k; idx_row <= order; idx_row++) {
              i = qrmanager->Q.size[0];
              if ((idx_row < 1) || (idx_row > i)) {
                emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &ld_emlrtBCI, &st);
              }
              i = qrmanager->Q.size[1];
              i1 = (LDimSizeP1 + qrmanager->ncols) + 1;
              if ((i1 < 1) || (i1 > i)) {
                emlrtDynamicBoundsCheckR2012b(i1, 1, i, &ld_emlrtBCI, &st);
              }
              i = memspace->workspace_double.size[0];
              if (idx_row > i) {
                emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &ld_emlrtBCI, &st);
              }
              i = memspace->workspace_double.size[1];
              if (LDimSizeP1 + 1 > i) {
                emlrtDynamicBoundsCheckR2012b(LDimSizeP1 + 1, 1, i,
                                              &ld_emlrtBCI, &st);
              }
              memspace->workspace_double
                  .data[(idx_row +
                         memspace->workspace_double.size[0] * LDimSizeP1) -
                        1] =
                  objective->beta *
                  qrmanager->Q
                      .data[(idx_row + qrmanager->Q.size[0] * (i1 - 1)) - 1];
            }
          }
          b_st.site = &vg_emlrtRSI;
          d_xgemm(mNull_tmp, mNull_tmp, qrmanager->mrows, qrmanager->Q.data,
                  nullStartIdx_tmp, qrmanager->ldq,
                  memspace->workspace_double.data, ldw, cholmanager->FMat.data,
                  cholmanager->ldm);
        }
        st.site = &eg_emlrtRSI;
        if (alwaysPositiveDef) {
          b_st.site = &wg_emlrtRSI;
          cholmanager->ndims = mNull_tmp;
          c_st.site = &fg_emlrtRSI;
          cholmanager->info = xpotrf(&c_st, mNull_tmp, cholmanager->FMat.data,
                                     cholmanager->ldm);
        } else {
          b_st.site = &wg_emlrtRSI;
          LDimSizeP1 = cholmanager->ldm + 1;
          cholmanager->ndims = mNull_tmp;
          n_t = (ptrdiff_t)mNull_tmp;
          incx_t = (ptrdiff_t)(cholmanager->ldm + 1);
          n_t = idamax(&n_t, &cholmanager->FMat.data[0], &incx_t);
          i = cholmanager->FMat.size[0] * cholmanager->FMat.size[1];
          i1 = (int32_T)n_t + cholmanager->ldm * ((int32_T)n_t - 1);
          if ((i1 < 1) || (i1 > i)) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, i, &jd_emlrtBCI, &b_st);
          }
          cholmanager->regTol_ = muDoubleScalarMax(
              muDoubleScalarAbs(
                  cholmanager->FMat
                      .data[((int32_T)n_t +
                             cholmanager->ldm * ((int32_T)n_t - 1)) -
                            1]) *
                  2.2204460492503131E-16,
              0.0);
          if (mNull_tmp > 128) {
            boolean_T exitg1;
            k = 0;
            exitg1 = false;
            while ((!exitg1) && (k < mNull_tmp)) {
              ldw = LDimSizeP1 * k + 1;
              order = mNull_tmp - k;
              if (k + 48 <= mNull_tmp) {
                c_st.site = &ig_emlrtRSI;
                partialColLDL3_(&c_st, cholmanager, ldw, order);
                k += 48;
              } else {
                c_st.site = &ig_emlrtRSI;
                fullColLDL2_(&c_st, cholmanager, ldw, order);
                exitg1 = true;
              }
            }
          } else {
            c_st.site = &ig_emlrtRSI;
            fullColLDL2_(&c_st, cholmanager, 1, mNull_tmp);
          }
          if (cholmanager->ConvexCheck) {
            c_st.site = &ig_emlrtRSI;
            if (mNull_tmp > 2147483646) {
              d_st.site = &db_emlrtRSI;
              check_forloop_overflow_error(&d_st);
            }
            LDimSizeP1 = 0;
            int32_T exitg2;
            do {
              exitg2 = 0;
              if (LDimSizeP1 <= mNull_tmp - 1) {
                i = cholmanager->FMat.size[0] * cholmanager->FMat.size[1];
                i1 = (LDimSizeP1 + cholmanager->ldm * LDimSizeP1) + 1;
                if ((i1 < 1) || (i1 > i)) {
                  emlrtDynamicBoundsCheckR2012b(i1, 1, i, &jd_emlrtBCI, &b_st);
                }
                if (cholmanager->FMat.data[i1 - 1] <= 0.0) {
                  cholmanager->info = -(LDimSizeP1 + 1);
                  exitg2 = 1;
                } else {
                  LDimSizeP1++;
                }
              } else {
                cholmanager->ConvexCheck = false;
                exitg2 = 1;
              }
            } while (exitg2 == 0);
          }
        }
        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          st.site = &eg_emlrtRSI;
          if (qrmanager->mrows >= 1) {
            b_st.site = &qc_emlrtRSI;
            c_st.site = &rc_emlrtRSI;
            alpha1 = -1.0;
            beta1 = 0.0;
            TRANSA = 'T';
            m_t = (ptrdiff_t)qrmanager->mrows;
            n_t = (ptrdiff_t)mNull_tmp;
            lda_t = (ptrdiff_t)qrmanager->ldq;
            incx_t = (ptrdiff_t)1;
            incy_t = (ptrdiff_t)1;
            dgemv(&TRANSA, &m_t, &n_t, &alpha1,
                  &qrmanager->Q.data[nullStartIdx_tmp - 1], &lda_t,
                  &objective->grad.data[0], &incx_t, &beta1,
                  &memspace->workspace_double.data[0], &incy_t);
          }
          st.site = &eg_emlrtRSI;
          if (alwaysPositiveDef) {
            b_st.site = &xg_emlrtRSI;
            c_st.site = &sg_emlrtRSI;
            if (cholmanager->ndims >= 1) {
              TRANSA = 'N';
              TRANSA1 = 'T';
              UPLO1 = 'U';
              n_t = (ptrdiff_t)cholmanager->ndims;
              lda_t = (ptrdiff_t)cholmanager->ldm;
              incx_t = (ptrdiff_t)1;
              dtrsv(&UPLO1, &TRANSA1, &TRANSA, &n_t, &cholmanager->FMat.data[0],
                    &lda_t, &memspace->workspace_double.data[0], &incx_t);
            }
            c_st.site = &sg_emlrtRSI;
            xtrsv(cholmanager->ndims, cholmanager->FMat.data, cholmanager->ldm,
                  memspace->workspace_double.data);
          } else {
            b_st.site = &xg_emlrtRSI;
            c_solve(&b_st, cholmanager, memspace->workspace_double.data,
                    memspace->workspace_double.size);
          }
          st.site = &eg_emlrtRSI;
          g_xgemv(qrmanager->mrows, mNull_tmp, qrmanager->Q.data,
                  nullStartIdx_tmp, qrmanager->ldq,
                  memspace->workspace_double.data, solution->searchDir.data);
        }
      }
    }
  }
}

/* End of code generation (compute_deltax.c) */
