/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * normal.c
 *
 * Code generation for function 'normal'
 *
 */

/* Include files */
#include "normal.h"
#include "addAeqConstr.h"
#include "computeConstrViolationIneq_.h"
#include "driver1.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_internal_types.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo le_emlrtRSI = {
    1,        /* lineNo */
    "normal", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\normal.p" /* pathName */
};

static emlrtBCInfo xb_emlrtBCI = {
    1,        /* iFirst */
    192,      /* iLast */
    1,        /* lineNo */
    1,        /* colNo */
    "",       /* aName */
    "normal", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\normal.p", /* pName */
    0                 /* checkKind */
};

static emlrtBCInfo yb_emlrtBCI = {
    -1,       /* iFirst */
    -1,       /* iLast */
    1,        /* lineNo */
    1,        /* colNo */
    "",       /* aName */
    "normal", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\normal.p", /* pName */
    0                 /* checkKind */
};

/* Function Definitions */
void normal(c_nlmpcmoveCodeGenerationStackD *SD, const emlrtStack *sp,
            const real_T Hessian[70225], const real_T grad_data[],
            int32_T grad_size, e_struct_T *TrialState, struct_T *MeritFunction,
            g_struct_T *memspace, h_struct_T *WorkingSet, i_struct_T *QRManager,
            j_struct_T *CholManager, f_struct_T *QPObjective,
            const d_struct_T *qpoptions)
{
  d_struct_T b_qpoptions;
  d_struct_T c_qpoptions;
  emlrtStack b_st;
  emlrtStack st;
  int32_T idx;
  int32_T k;
  boolean_T exitg1;
  boolean_T nonlinEqRemoved;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  b_qpoptions = *qpoptions;
  c_qpoptions = *qpoptions;
  st.site = &le_emlrtRSI;
  b_driver(SD, &st, Hessian, grad_data, grad_size, TrialState, memspace,
           WorkingSet, QRManager, CholManager, QPObjective, &b_qpoptions,
           &c_qpoptions);
  if (TrialState->state > 0) {
    real_T constrViolationEq;
    real_T constrViolationIneq;
    real_T penaltyParamTrial;
    st.site = &le_emlrtRSI;
    penaltyParamTrial = MeritFunction->penaltyParam;
    constrViolationEq = 0.0;
    for (k = 0; k < 192; k++) {
      constrViolationEq += muDoubleScalarAbs(TrialState->cEq[k]);
    }
    b_st.site = &ih_emlrtRSI;
    constrViolationIneq = computeConstrViolationIneq_(
        &b_st, WorkingSet->sizes[2], TrialState->cIneq.data,
        TrialState->cIneq.size[0]);
    constrViolationIneq += constrViolationEq;
    constrViolationEq = MeritFunction->linearizedConstrViol;
    MeritFunction->linearizedConstrViol = 0.0;
    constrViolationEq += constrViolationIneq;
    if ((constrViolationEq > 2.2204460492503131E-16) &&
        (TrialState->fstar > 0.0)) {
      if (TrialState->sqpFval == 0.0) {
        penaltyParamTrial = 1.0;
      } else {
        penaltyParamTrial = 1.5;
      }
      penaltyParamTrial =
          penaltyParamTrial * TrialState->fstar / constrViolationEq;
    }
    if (penaltyParamTrial < MeritFunction->penaltyParam) {
      MeritFunction->phi =
          TrialState->sqpFval + penaltyParamTrial * constrViolationIneq;
      if ((MeritFunction->initFval +
           penaltyParamTrial * (MeritFunction->initConstrViolationEq +
                                MeritFunction->initConstrViolationIneq)) -
              MeritFunction->phi >
          (real_T)MeritFunction->nPenaltyDecreases * MeritFunction->threshold) {
        MeritFunction->nPenaltyDecreases++;
        if ((MeritFunction->nPenaltyDecreases << 1) >
            TrialState->sqpIterations) {
          MeritFunction->threshold *= 10.0;
        }
        MeritFunction->penaltyParam =
            muDoubleScalarMax(penaltyParamTrial, 1.0E-10);
      } else {
        MeritFunction->phi = TrialState->sqpFval +
                             MeritFunction->penaltyParam * constrViolationIneq;
      }
    } else {
      MeritFunction->penaltyParam =
          muDoubleScalarMax(penaltyParamTrial, 1.0E-10);
      MeritFunction->phi = TrialState->sqpFval +
                           MeritFunction->penaltyParam * constrViolationIneq;
    }
    MeritFunction->phiPrimePlus = muDoubleScalarMin(
        TrialState->fstar - MeritFunction->penaltyParam * constrViolationIneq,
        0.0);
  }
  st.site = &le_emlrtRSI;
  sortLambdaQP(&st, TrialState->lambda.data, &TrialState->lambda.size[0],
               WorkingSet->nActiveConstr, WorkingSet->sizes,
               WorkingSet->isActiveIdx, WorkingSet->Wid.data,
               WorkingSet->Wid.size[0], WorkingSet->Wlocalidx.data,
               WorkingSet->Wlocalidx.size[0], memspace->workspace_double.data,
               memspace->workspace_double.size);
  nonlinEqRemoved = (WorkingSet->mEqRemoved > 0);
  exitg1 = false;
  while ((!exitg1) && (WorkingSet->mEqRemoved > 0)) {
    if ((WorkingSet->mEqRemoved < 1) || (WorkingSet->mEqRemoved > 192)) {
      emlrtDynamicBoundsCheckR2012b(WorkingSet->mEqRemoved, 1, 192,
                                    &xb_emlrtBCI, (emlrtCTX)sp);
    }
    k = WorkingSet->indexEqRemoved[WorkingSet->mEqRemoved - 1];
    if (k >= 1) {
      if ((WorkingSet->mEqRemoved < 1) || (WorkingSet->mEqRemoved > 192)) {
        emlrtDynamicBoundsCheckR2012b(WorkingSet->mEqRemoved, 1, 192,
                                      &xb_emlrtBCI, (emlrtCTX)sp);
      }
      st.site = &le_emlrtRSI;
      addAeqConstr(&st, WorkingSet, k);
      WorkingSet->mEqRemoved--;
    } else {
      exitg1 = true;
    }
  }
  if (nonlinEqRemoved) {
    st.site = &le_emlrtRSI;
    for (idx = 0; idx < 192; idx++) {
      int32_T i;
      k = WorkingSet->Wlocalidx.size[0];
      i = (WorkingSet->sizes[0] + idx) + 1;
      if ((i < 1) || (i > k)) {
        emlrtDynamicBoundsCheckR2012b(i, 1, k, &yb_emlrtBCI, (emlrtCTX)sp);
      }
      WorkingSet->Wlocalidx.data[i - 1] = idx + 1;
    }
  }
}

/* End of code generation (normal.c) */
