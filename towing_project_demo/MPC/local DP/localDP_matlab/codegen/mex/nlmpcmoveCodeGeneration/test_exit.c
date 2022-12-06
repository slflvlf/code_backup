/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * test_exit.c
 *
 * Code generation for function 'test_exit'
 *
 */

/* Include files */
#include "test_exit.h"
#include "computeComplError.h"
#include "computeDualFeasError.h"
#include "computeGradLag.h"
#include "computePrimalFeasError.h"
#include "computeQ_.h"
#include "eml_int_forloop_overflow_check.h"
#include "factorQRE.h"
#include "isDeltaXTooSmall.h"
#include "ixamax.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_internal_types.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
#include "updateWorkingSetForNewQP.h"
#include "xcopy.h"
#include "xgemv.h"
#include "xtrsv.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ad_emlrtRSI = {
    1,           /* lineNo */
    "test_exit", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\test_"
    "exit.p" /* pathName */
};

static emlrtRSInfo hd_emlrtRSI = {
    1,                  /* lineNo */
    "computeLambdaLSQ", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "stopping\\computeLambdaLSQ.p" /* pathName */
};

static emlrtBCInfo qb_emlrtBCI = {
    -1,          /* iFirst */
    -1,          /* iLast */
    1,           /* lineNo */
    1,           /* colNo */
    "",          /* aName */
    "test_exit", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\test_"
    "exit.p", /* pName */
    0         /* checkKind */
};

static emlrtBCInfo fe_emlrtBCI = {
    -1,                 /* iFirst */
    -1,                 /* iLast */
    1,                  /* lineNo */
    1,                  /* colNo */
    "",                 /* aName */
    "computeLambdaLSQ", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "stopping\\computeLambdaLSQ.p", /* pName */
    0                               /* checkKind */
};

/* Function Definitions */
void b_test_exit(const emlrtStack *sp, b_struct_T *Flags, g_struct_T *memspace,
                 struct_T *MeritFunction, int32_T fscales_lineq_constraint_size,
                 int32_T fscales_cineq_constraint_size, h_struct_T *WorkingSet,
                 e_struct_T *TrialState, i_struct_T *QRManager,
                 const real_T lb[79])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T optimRelativeFactor;
  real_T tol;
  int32_T i;
  int32_T mFixed;
  int32_T mIneq;
  int32_T mLB;
  int32_T mLambda;
  int32_T mUB;
  int32_T nActiveConstr;
  int32_T nVar;
  int32_T rankR;
  boolean_T dxTooSmall;
  boolean_T isFeasible;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  nVar = WorkingSet->nVar;
  mFixed = WorkingSet->sizes[0] + 1;
  mIneq = WorkingSet->sizes[2];
  mLB = WorkingSet->sizes[3];
  mUB = WorkingSet->sizes[4];
  mLambda =
      (((WorkingSet->sizes[0] + WorkingSet->sizes[2]) + WorkingSet->sizes[3]) +
       WorkingSet->sizes[4]) +
      60;
  st.site = &ad_emlrtRSI;
  b_xcopy(mLambda, TrialState->lambdasqp.data, TrialState->lambdaStopTest.data);
  st.site = &ad_emlrtRSI;
  computeGradLag(&st, TrialState->gradLag.data, &TrialState->gradLag.size[0],
                 WorkingSet->ldA, WorkingSet->nVar, TrialState->grad.data,
                 TrialState->grad.size[0], WorkingSet->sizes[2],
                 WorkingSet->Aineq.data, WorkingSet->Aeq.data,
                 WorkingSet->indexFixed.data, WorkingSet->indexFixed.size[0],
                 WorkingSet->sizes[0], WorkingSet->indexLB.data,
                 WorkingSet->indexLB.size[0], WorkingSet->sizes[3],
                 WorkingSet->indexUB.data, WorkingSet->indexUB.size[0],
                 WorkingSet->sizes[4], TrialState->lambdaStopTest.data,
                 TrialState->lambdaStopTest.size[0]);
  i = TrialState->grad.size[0];
  nActiveConstr = ixamax(WorkingSet->nVar, TrialState->grad.data);
  if ((nActiveConstr < 1) || (nActiveConstr > i)) {
    emlrtDynamicBoundsCheckR2012b(nActiveConstr, 1, i, &qb_emlrtBCI,
                                  (emlrtCTX)sp);
  }
  optimRelativeFactor = muDoubleScalarMax(
      1.0, muDoubleScalarAbs(TrialState->grad.data[nActiveConstr - 1]));
  if (muDoubleScalarIsInf(optimRelativeFactor)) {
    optimRelativeFactor = 1.0;
  }
  st.site = &ad_emlrtRSI;
  MeritFunction->nlpPrimalFeasError = computePrimalFeasError(
      &st, TrialState->xstarsqp, WorkingSet->sizes[2] - TrialState->mNonlinIneq,
      TrialState->mNonlinIneq, TrialState->cIneq.data,
      TrialState->cIneq.size[0], TrialState->cEq, WorkingSet->indexLB.data,
      WorkingSet->indexLB.size[0], WorkingSet->sizes[3], lb,
      WorkingSet->indexUB.data, WorkingSet->indexUB.size[0],
      WorkingSet->sizes[4]);
  if (TrialState->sqpIterations == 0) {
    MeritFunction->feasRelativeFactor =
        muDoubleScalarMax(1.0, MeritFunction->nlpPrimalFeasError);
  }
  isFeasible = (MeritFunction->nlpPrimalFeasError <=
                1.0E-6 * MeritFunction->feasRelativeFactor);
  st.site = &ad_emlrtRSI;
  computeDualFeasError(&st, WorkingSet->nVar, TrialState->gradLag.data,
                       TrialState->gradLag.size[0], &Flags->gradOK,
                       &MeritFunction->nlpDualFeasError);
  if (!Flags->gradOK) {
    Flags->done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    real_T nlpComplErrorTmp;
    st.site = &ad_emlrtRSI;
    MeritFunction->nlpComplError = computeComplError(
        &st, fscales_lineq_constraint_size, fscales_cineq_constraint_size,
        TrialState->xstarsqp, WorkingSet->sizes[2], TrialState->cIneq.data,
        TrialState->cIneq.size[0], WorkingSet->indexLB.data,
        WorkingSet->indexLB.size[0], WorkingSet->sizes[3], lb,
        WorkingSet->indexUB.data, WorkingSet->indexUB.size[0],
        WorkingSet->sizes[4], TrialState->lambdaStopTest.data,
        TrialState->lambdaStopTest.size[0], WorkingSet->sizes[0] + 61);
    MeritFunction->firstOrderOpt = muDoubleScalarMax(
        MeritFunction->nlpDualFeasError, MeritFunction->nlpComplError);
    if (TrialState->sqpIterations > 1) {
      real_T d;
      st.site = &ad_emlrtRSI;
      b_computeGradLag(
          &st, memspace->workspace_double.data, memspace->workspace_double.size,
          WorkingSet->ldA, WorkingSet->nVar, TrialState->grad.data,
          TrialState->grad.size[0], WorkingSet->sizes[2],
          WorkingSet->Aineq.data, WorkingSet->Aeq.data,
          WorkingSet->indexFixed.data, WorkingSet->indexFixed.size[0],
          WorkingSet->sizes[0], WorkingSet->indexLB.data,
          WorkingSet->indexLB.size[0], WorkingSet->sizes[3],
          WorkingSet->indexUB.data, WorkingSet->indexUB.size[0],
          WorkingSet->sizes[4], TrialState->lambdaStopTestPrev.data,
          TrialState->lambdaStopTestPrev.size[0]);
      st.site = &ad_emlrtRSI;
      b_computeDualFeasError(&st, WorkingSet->nVar,
                             memspace->workspace_double.data, &dxTooSmall,
                             &tol);
      st.site = &ad_emlrtRSI;
      nlpComplErrorTmp = computeComplError(
          &st, fscales_lineq_constraint_size, fscales_cineq_constraint_size,
          TrialState->xstarsqp, WorkingSet->sizes[2], TrialState->cIneq.data,
          TrialState->cIneq.size[0], WorkingSet->indexLB.data,
          WorkingSet->indexLB.size[0], WorkingSet->sizes[3], lb,
          WorkingSet->indexUB.data, WorkingSet->indexUB.size[0],
          WorkingSet->sizes[4], TrialState->lambdaStopTestPrev.data,
          TrialState->lambdaStopTestPrev.size[0], WorkingSet->sizes[0] + 61);
      d = muDoubleScalarMax(tol, nlpComplErrorTmp);
      if (d < muDoubleScalarMax(MeritFunction->nlpDualFeasError,
                                MeritFunction->nlpComplError)) {
        MeritFunction->nlpDualFeasError = tol;
        MeritFunction->nlpComplError = nlpComplErrorTmp;
        MeritFunction->firstOrderOpt = d;
        st.site = &ad_emlrtRSI;
        b_xcopy(mLambda, TrialState->lambdaStopTestPrev.data,
                TrialState->lambdaStopTest.data);
      } else {
        st.site = &ad_emlrtRSI;
        b_xcopy(mLambda, TrialState->lambdaStopTest.data,
                TrialState->lambdaStopTestPrev.data);
      }
    } else {
      st.site = &ad_emlrtRSI;
      b_xcopy(mLambda, TrialState->lambdaStopTest.data,
              TrialState->lambdaStopTestPrev.data);
    }
    if (isFeasible &&
        (MeritFunction->nlpDualFeasError <= 1.0E-6 * optimRelativeFactor) &&
        (MeritFunction->nlpComplError <= 1.0E-6 * optimRelativeFactor)) {
      Flags->done = true;
      TrialState->sqpExitFlag = 1;
    } else {
      Flags->done = false;
      if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
        Flags->done = true;
        TrialState->sqpExitFlag = -3;
      } else {
        boolean_T guard1 = false;
        guard1 = false;
        if (TrialState->sqpIterations > 0) {
          st.site = &ad_emlrtRSI;
          dxTooSmall =
              isDeltaXTooSmall(&st, TrialState->xstarsqp,
                               TrialState->delta_x.data, WorkingSet->nVar);
          if (dxTooSmall) {
            if (!isFeasible) {
              if (Flags->stepType != 2) {
                Flags->stepType = 2;
                Flags->failedLineSearch = false;
                Flags->stepAccepted = false;
                guard1 = true;
              } else {
                Flags->done = true;
                TrialState->sqpExitFlag = -2;
              }
            } else {
              nActiveConstr = WorkingSet->nActiveConstr;
              if (WorkingSet->nActiveConstr > 0) {
                int32_T fullRank_R;
                int32_T iQR_diag;
                boolean_T exitg1;
                st.site = &ad_emlrtRSI;
                b_updateWorkingSetForNewQP(
                    &st, TrialState->xstarsqp, WorkingSet, WorkingSet->sizes[2],
                    TrialState->mNonlinIneq, TrialState->cIneq.data,
                    TrialState->cIneq.size[0], TrialState->cEq,
                    WorkingSet->sizes[3], lb, WorkingSet->sizes[4],
                    WorkingSet->sizes[0]);
                st.site = &ad_emlrtRSI;
                b_st.site = &hd_emlrtRSI;
                f_xcopy(&b_st, nActiveConstr, TrialState->lambda.data);
                b_st.site = &hd_emlrtRSI;
                factorQRE(&b_st, QRManager, WorkingSet->ATwset.data, nVar,
                          nActiveConstr, WorkingSet->ldA);
                b_st.site = &hd_emlrtRSI;
                c_st.site = &yd_emlrtRSI;
                computeQ_(&c_st, QRManager, QRManager->mrows);
                b_st.site = &hd_emlrtRSI;
                b_xgemv(nVar, nVar, QRManager->Q.data, QRManager->ldq,
                        TrialState->grad.data, memspace->workspace_double.data);
                tol = muDoubleScalarAbs(QRManager->QR.data[0]) *
                      muDoubleScalarMin(
                          1.4901161193847656E-8,
                          (real_T)muIntScalarMax_sint32(nVar, nActiveConstr) *
                              2.2204460492503131E-16);
                fullRank_R = muIntScalarMin_sint32(nVar, nActiveConstr);
                rankR = 0;
                iQR_diag = 1;
                exitg1 = false;
                while ((!exitg1) && (rankR < fullRank_R)) {
                  i = QRManager->QR.size[0] * QRManager->QR.size[1];
                  if ((iQR_diag < 1) || (iQR_diag > i)) {
                    emlrtDynamicBoundsCheckR2012b(iQR_diag, 1, i, &fe_emlrtBCI,
                                                  &st);
                  }
                  if (muDoubleScalarAbs(QRManager->QR.data[iQR_diag - 1]) >
                      tol) {
                    rankR++;
                    iQR_diag = (iQR_diag + QRManager->ldq) + 1;
                  } else {
                    exitg1 = true;
                  }
                }
                b_st.site = &hd_emlrtRSI;
                xtrsv(rankR, QRManager->QR.data, QRManager->ldq,
                      memspace->workspace_double.data);
                fullRank_R = muIntScalarMin_sint32(nActiveConstr, fullRank_R);
                b_st.site = &hd_emlrtRSI;
                if (fullRank_R > 2147483646) {
                  c_st.site = &db_emlrtRSI;
                  check_forloop_overflow_error(&c_st);
                }
                i = QRManager->jpvt.size[0];
                for (rankR = 0; rankR < fullRank_R; rankR++) {
                  if ((rankR + 1 < 1) || (rankR + 1 > i)) {
                    emlrtDynamicBoundsCheckR2012b(rankR + 1, 1, i, &fe_emlrtBCI,
                                                  &st);
                  }
                  nActiveConstr = TrialState->lambda.size[0];
                  if ((QRManager->jpvt.data[rankR] < 1) ||
                      (QRManager->jpvt.data[rankR] > nActiveConstr)) {
                    emlrtDynamicBoundsCheckR2012b(QRManager->jpvt.data[rankR],
                                                  1, nActiveConstr,
                                                  &fe_emlrtBCI, &st);
                  }
                  TrialState->lambda.data[QRManager->jpvt.data[rankR] - 1] =
                      memspace->workspace_double.data[rankR];
                }
                nActiveConstr = mFixed + 59;
                st.site = &ad_emlrtRSI;
                if ((mFixed <= mFixed + 59) && (mFixed + 59 > 2147483646)) {
                  b_st.site = &db_emlrtRSI;
                  check_forloop_overflow_error(&b_st);
                }
                for (rankR = mFixed; rankR <= nActiveConstr; rankR++) {
                  i = TrialState->lambda.size[0];
                  if ((rankR < 1) || (rankR > i)) {
                    emlrtDynamicBoundsCheckR2012b(rankR, 1, i, &qb_emlrtBCI,
                                                  (emlrtCTX)sp);
                  }
                  i = TrialState->lambda.size[0];
                  if (rankR > i) {
                    emlrtDynamicBoundsCheckR2012b(rankR, 1, i, &qb_emlrtBCI,
                                                  (emlrtCTX)sp);
                  }
                  TrialState->lambda.data[rankR - 1] =
                      -TrialState->lambda.data[rankR - 1];
                }
                st.site = &ad_emlrtRSI;
                sortLambdaQP(
                    &st, TrialState->lambda.data, &TrialState->lambda.size[0],
                    WorkingSet->nActiveConstr, WorkingSet->sizes,
                    WorkingSet->isActiveIdx, WorkingSet->Wid.data,
                    WorkingSet->Wid.size[0], WorkingSet->Wlocalidx.data,
                    WorkingSet->Wlocalidx.size[0],
                    memspace->workspace_double.data,
                    memspace->workspace_double.size);
                st.site = &ad_emlrtRSI;
                b_computeGradLag(
                    &st, memspace->workspace_double.data,
                    memspace->workspace_double.size, WorkingSet->ldA, nVar,
                    TrialState->grad.data, TrialState->grad.size[0], mIneq,
                    WorkingSet->Aineq.data, WorkingSet->Aeq.data,
                    WorkingSet->indexFixed.data, WorkingSet->indexFixed.size[0],
                    mFixed - 1, WorkingSet->indexLB.data,
                    WorkingSet->indexLB.size[0], mLB, WorkingSet->indexUB.data,
                    WorkingSet->indexUB.size[0], mUB, TrialState->lambda.data,
                    TrialState->lambda.size[0]);
                st.site = &ad_emlrtRSI;
                b_computeDualFeasError(&st, nVar,
                                       memspace->workspace_double.data,
                                       &dxTooSmall, &tol);
                st.site = &ad_emlrtRSI;
                nlpComplErrorTmp = computeComplError(
                    &st, fscales_lineq_constraint_size,
                    fscales_cineq_constraint_size, TrialState->xstarsqp, mIneq,
                    TrialState->cIneq.data, TrialState->cIneq.size[0],
                    WorkingSet->indexLB.data, WorkingSet->indexLB.size[0], mLB,
                    lb, WorkingSet->indexUB.data, WorkingSet->indexUB.size[0],
                    mUB, TrialState->lambda.data, TrialState->lambda.size[0],
                    mFixed);
                if ((tol <= 1.0E-6 * optimRelativeFactor) &&
                    (nlpComplErrorTmp <= 1.0E-6 * optimRelativeFactor)) {
                  MeritFunction->nlpDualFeasError = tol;
                  MeritFunction->nlpComplError = nlpComplErrorTmp;
                  MeritFunction->firstOrderOpt =
                      muDoubleScalarMax(tol, nlpComplErrorTmp);
                  st.site = &ad_emlrtRSI;
                  b_xcopy(mLambda, TrialState->lambda.data,
                          TrialState->lambdaStopTest.data);
                  Flags->done = true;
                  TrialState->sqpExitFlag = 1;
                } else {
                  Flags->done = true;
                  TrialState->sqpExitFlag = 2;
                }
              } else {
                Flags->done = true;
                TrialState->sqpExitFlag = 2;
              }
            }
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }
        if (guard1) {
          if (TrialState->sqpIterations >= 400) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          } else if (TrialState->FunctionEvaluations >= 7900) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          }
        }
      }
    }
  }
}

void test_exit(const emlrtStack *sp, struct_T *MeritFunction,
               int32_T fscales_lineq_constraint_size,
               int32_T fscales_cineq_constraint_size,
               const h_struct_T *WorkingSet, e_struct_T *TrialState,
               const real_T lb[79], boolean_T *Flags_gradOK,
               boolean_T *Flags_fevalOK, boolean_T *Flags_done,
               boolean_T *Flags_stepAccepted, boolean_T *Flags_failedLineSearch,
               int32_T *Flags_stepType)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  emlrtStack st;
  real_T optimRelativeFactor;
  int32_T i;
  int32_T idx_max;
  int32_T mLambda;
  boolean_T isFeasible;
  st.prev = sp;
  st.tls = sp->tls;
  *Flags_fevalOK = true;
  *Flags_done = false;
  *Flags_stepAccepted = false;
  *Flags_failedLineSearch = false;
  *Flags_stepType = 1;
  mLambda =
      (((WorkingSet->sizes[0] + WorkingSet->sizes[2]) + WorkingSet->sizes[3]) +
       WorkingSet->sizes[4]) +
      60;
  if (mLambda >= 1) {
    n_t = (ptrdiff_t)mLambda;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dcopy(&n_t, &TrialState->lambdasqp.data[0], &incx_t,
          &TrialState->lambdaStopTest.data[0], &incy_t);
  }
  st.site = &ad_emlrtRSI;
  computeGradLag(&st, TrialState->gradLag.data, &TrialState->gradLag.size[0],
                 WorkingSet->ldA, WorkingSet->nVar, TrialState->grad.data,
                 TrialState->grad.size[0], WorkingSet->sizes[2],
                 WorkingSet->Aineq.data, WorkingSet->Aeq.data,
                 WorkingSet->indexFixed.data, WorkingSet->indexFixed.size[0],
                 WorkingSet->sizes[0], WorkingSet->indexLB.data,
                 WorkingSet->indexLB.size[0], WorkingSet->sizes[3],
                 WorkingSet->indexUB.data, WorkingSet->indexUB.size[0],
                 WorkingSet->sizes[4], TrialState->lambdaStopTest.data,
                 TrialState->lambdaStopTest.size[0]);
  if (WorkingSet->nVar < 1) {
    idx_max = 0;
  } else {
    n_t = (ptrdiff_t)WorkingSet->nVar;
    incx_t = (ptrdiff_t)1;
    n_t = idamax(&n_t, &TrialState->grad.data[0], &incx_t);
    idx_max = (int32_T)n_t;
  }
  i = TrialState->grad.size[0];
  if ((idx_max < 1) || (idx_max > i)) {
    emlrtDynamicBoundsCheckR2012b(idx_max, 1, i, &qb_emlrtBCI, (emlrtCTX)sp);
  }
  optimRelativeFactor = muDoubleScalarMax(
      1.0, muDoubleScalarAbs(TrialState->grad.data[idx_max - 1]));
  if (muDoubleScalarIsInf(optimRelativeFactor)) {
    optimRelativeFactor = 1.0;
  }
  st.site = &ad_emlrtRSI;
  MeritFunction->nlpPrimalFeasError = computePrimalFeasError(
      &st, TrialState->xstarsqp, WorkingSet->sizes[2] - TrialState->mNonlinIneq,
      TrialState->mNonlinIneq, TrialState->cIneq.data,
      TrialState->cIneq.size[0], TrialState->cEq, WorkingSet->indexLB.data,
      WorkingSet->indexLB.size[0], WorkingSet->sizes[3], lb,
      WorkingSet->indexUB.data, WorkingSet->indexUB.size[0],
      WorkingSet->sizes[4]);
  MeritFunction->feasRelativeFactor =
      muDoubleScalarMax(1.0, MeritFunction->nlpPrimalFeasError);
  isFeasible = (MeritFunction->nlpPrimalFeasError <=
                1.0E-6 * MeritFunction->feasRelativeFactor);
  st.site = &ad_emlrtRSI;
  computeDualFeasError(&st, WorkingSet->nVar, TrialState->gradLag.data,
                       TrialState->gradLag.size[0], Flags_gradOK,
                       &MeritFunction->nlpDualFeasError);
  if (!*Flags_gradOK) {
    *Flags_done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    st.site = &ad_emlrtRSI;
    MeritFunction->nlpComplError = computeComplError(
        &st, fscales_lineq_constraint_size, fscales_cineq_constraint_size,
        TrialState->xstarsqp, WorkingSet->sizes[2], TrialState->cIneq.data,
        TrialState->cIneq.size[0], WorkingSet->indexLB.data,
        WorkingSet->indexLB.size[0], WorkingSet->sizes[3], lb,
        WorkingSet->indexUB.data, WorkingSet->indexUB.size[0],
        WorkingSet->sizes[4], TrialState->lambdaStopTest.data,
        TrialState->lambdaStopTest.size[0], WorkingSet->sizes[0] + 61);
    MeritFunction->firstOrderOpt = muDoubleScalarMax(
        MeritFunction->nlpDualFeasError, MeritFunction->nlpComplError);
    if (mLambda >= 1) {
      n_t = (ptrdiff_t)mLambda;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      dcopy(&n_t, &TrialState->lambdaStopTest.data[0], &incx_t,
            &TrialState->lambdaStopTestPrev.data[0], &incy_t);
    }
    if (isFeasible &&
        (MeritFunction->nlpDualFeasError <= 1.0E-6 * optimRelativeFactor) &&
        (MeritFunction->nlpComplError <= 1.0E-6 * optimRelativeFactor)) {
      *Flags_done = true;
      TrialState->sqpExitFlag = 1;
    } else if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
      *Flags_done = true;
      TrialState->sqpExitFlag = -3;
    }
  }
}

/* End of code generation (test_exit.c) */
