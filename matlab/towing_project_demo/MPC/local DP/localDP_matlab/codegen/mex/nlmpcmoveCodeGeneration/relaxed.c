/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * relaxed.c
 *
 * Code generation for function 'relaxed'
 *
 */

/* Include files */
#include "relaxed.h"
#include "assignResidualsToXSlack.h"
#include "driver1.h"
#include "eml_int_forloop_overflow_check.h"
#include "ixamax.h"
#include "moveConstraint_.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_internal_types.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "sortLambdaQP.h"
#include "updatePenaltyParam.h"
#include "xasum.h"
#include "xdot.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo oh_emlrtRSI = {
    1,         /* lineNo */
    "relaxed", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\relaxed.p" /* pathName */
};

static emlrtRSInfo ph_emlrtRSI = {
    1,                            /* lineNo */
    "findActiveSlackLowerBounds", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\+relaxed\\findActiveSlackLowerBounds.p" /* pathName */
};

static emlrtRSInfo qh_emlrtRSI = {
    1,                              /* lineNo */
    "removeActiveSlackLowerBounds", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\+relaxed\\removeActiveSlackLowerBounds.p" /* pathName */
};

static emlrtBCInfo ud_emlrtBCI = {
    -1,        /* iFirst */
    -1,        /* iLast */
    1,         /* lineNo */
    1,         /* colNo */
    "",        /* aName */
    "relaxed", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\relaxed.p", /* pName */
    0                  /* checkKind */
};

static emlrtBCInfo vd_emlrtBCI = {
    1,         /* iFirst */
    79,        /* iLast */
    1,         /* lineNo */
    1,         /* colNo */
    "",        /* aName */
    "relaxed", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\relaxed.p", /* pName */
    0                  /* checkKind */
};

static emlrtBCInfo wd_emlrtBCI = {
    -1,                           /* iFirst */
    -1,                           /* iLast */
    1,                            /* lineNo */
    1,                            /* colNo */
    "",                           /* aName */
    "findActiveSlackLowerBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\+relaxed\\findActiveSlackLowerBounds.p", /* pName */
    0                                               /* checkKind */
};

static emlrtBCInfo xd_emlrtBCI = {
    -1,                             /* iFirst */
    -1,                             /* iLast */
    1,                              /* lineNo */
    1,                              /* colNo */
    "",                             /* aName */
    "removeActiveSlackLowerBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\+relaxed\\removeActiveSlackLowerBounds.p", /* pName */
    0                                                 /* checkKind */
};

/* Function Definitions */
void relaxed(c_nlmpcmoveCodeGenerationStackD *SD, const emlrtStack *sp,
             const real_T Hessian[6241], const real_T grad_data[],
             int32_T grad_size, e_struct_T *TrialState, struct_T *MeritFunction,
             g_struct_T *memspace, h_struct_T *WorkingSet,
             i_struct_T *QRManager, j_struct_T *CholManager,
             f_struct_T *QPObjective, d_struct_T *qpoptions)
{
  d_struct_T b_qpoptions;
  d_struct_T c_qpoptions;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T beta;
  real_T rho;
  int32_T i;
  int32_T i1;
  int32_T idx;
  int32_T mFiniteLB;
  int32_T mIneq;
  int32_T nActiveLBArtificial;
  int32_T nVarOrig;
  int32_T temp;
  int32_T tmp_tmp_tmp;
  boolean_T tf;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  nVarOrig = WorkingSet->nVar + 1;
  mIneq = WorkingSet->sizes[2];
  beta = 0.0;
  st.site = &oh_emlrtRSI;
  if (WorkingSet->nVar > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = 0; idx <= nVarOrig - 2; idx++) {
    if ((idx + 1 < 1) || (idx + 1 > 79)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, 79, &vd_emlrtBCI, (emlrtCTX)sp);
    }
    beta += Hessian[idx + 79 * idx];
  }
  beta /= (real_T)WorkingSet->nVar;
  if (TrialState->sqpIterations <= 1) {
    i = ixamax(QPObjective->nvar, grad_data);
    if ((i < 1) || (i > grad_size)) {
      emlrtDynamicBoundsCheckR2012b(i, 1, grad_size, &ud_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    rho = 100.0 * muDoubleScalarMax(1.0, muDoubleScalarAbs(grad_data[i - 1]));
  } else {
    i = TrialState->lambdasqp.size[0];
    i1 = ixamax(WorkingSet->mConstr, TrialState->lambdasqp.data);
    if ((i1 < 1) || (i1 > i)) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i, &ud_emlrtBCI, (emlrtCTX)sp);
    }
    rho = muDoubleScalarAbs(TrialState->lambdasqp.data[i1 - 1]);
  }
  QPObjective->nvar = WorkingSet->nVar;
  QPObjective->beta = beta;
  QPObjective->rho = rho;
  QPObjective->hasLinear = true;
  QPObjective->objtype = 4;
  st.site = &oh_emlrtRSI;
  setProblemType(&st, WorkingSet, 2);
  st.site = &oh_emlrtRSI;
  assignResidualsToXSlack(&st, nVarOrig - 1, WorkingSet, TrialState, memspace);
  temp = qpoptions->MaxIterations;
  qpoptions->MaxIterations =
      ((qpoptions->MaxIterations + WorkingSet->nVar) - nVarOrig) + 1;
  b_qpoptions = *qpoptions;
  c_qpoptions = *qpoptions;
  st.site = &oh_emlrtRSI;
  b_driver(SD, &st, Hessian, grad_data, grad_size, TrialState, memspace,
           WorkingSet, QRManager, CholManager, QPObjective, &b_qpoptions,
           &c_qpoptions);
  qpoptions->MaxIterations = temp;
  st.site = &oh_emlrtRSI;
  temp = WorkingSet->sizes[2] - 2;
  mFiniteLB = WorkingSet->sizes[3] - 120;
  nActiveLBArtificial = 0;
  b_st.site = &ph_emlrtRSI;
  i = WorkingSet->isActiveConstr.size[0];
  i1 = WorkingSet->isActiveConstr.size[0];
  for (idx = 0; idx < 60; idx++) {
    boolean_T b_tf;
    b_st.site = &ph_emlrtRSI;
    tmp_tmp_tmp = (WorkingSet->isActiveIdx[3] + mFiniteLB) + idx;
    if ((tmp_tmp_tmp < 1) || (tmp_tmp_tmp > i)) {
      emlrtDynamicBoundsCheckR2012b(tmp_tmp_tmp, 1, i, &vc_emlrtBCI, &b_st);
    }
    tf = WorkingSet->isActiveConstr.data[tmp_tmp_tmp - 1];
    b_st.site = &ph_emlrtRSI;
    if ((tmp_tmp_tmp + 60 < 1) || (tmp_tmp_tmp + 60 > i1)) {
      emlrtDynamicBoundsCheckR2012b(tmp_tmp_tmp + 60, 1, i1, &vc_emlrtBCI,
                                    &b_st);
    }
    b_tf = WorkingSet->isActiveConstr.data[tmp_tmp_tmp + 59];
    memspace->workspace_int.data[idx] = tf;
    memspace->workspace_int.data[idx + 60] = b_tf;
    nActiveLBArtificial = (nActiveLBArtificial + tf) + b_tf;
  }
  b_st.site = &ph_emlrtRSI;
  if (WorkingSet->sizes[2] > 2147483646) {
    c_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&c_st);
  }
  for (idx = 0; idx <= temp + 1; idx++) {
    b_st.site = &ph_emlrtRSI;
    i = WorkingSet->isActiveConstr.size[0];
    i1 = (((WorkingSet->isActiveIdx[3] + mFiniteLB) - temp) + idx) - 2;
    if ((i1 < 1) || (i1 > i)) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i, &vc_emlrtBCI, &b_st);
    }
    tf = WorkingSet->isActiveConstr.data[i1 - 1];
    i = memspace->workspace_int.size[0];
    if ((idx + 121 < 1) || (idx + 121 > i)) {
      emlrtDynamicBoundsCheckR2012b(idx + 121, 1, i, &wd_emlrtBCI, &st);
    }
    memspace->workspace_int.data[idx + 120] = tf;
    nActiveLBArtificial += tf;
  }
  if (TrialState->state != -6) {
    temp = WorkingSet->nVarMax - nVarOrig;
    st.site = &oh_emlrtRSI;
    updatePenaltyParam(&st, MeritFunction, TrialState->sqpFval,
                       TrialState->cIneq.data, TrialState->cIneq.size[0], mIneq,
                       TrialState->cEq, TrialState->sqpIterations,
                       (TrialState->fstar -
                        rho * xasum(temp, TrialState->xstar.data, nVarOrig)) -
                           beta / 2.0 *
                               xdot(temp, TrialState->xstar.data, nVarOrig,
                                    TrialState->xstar.data, nVarOrig),
                       TrialState->xstar.data, nVarOrig,
                       WorkingSet->nVarMax - nVarOrig);
    temp = WorkingSet->isActiveIdx[1];
    st.site = &oh_emlrtRSI;
    for (idx = 0; idx < 60; idx++) {
      if ((memspace->workspace_int.data[idx] != 0) &&
          (memspace->workspace_int.data[idx + 60] != 0)) {
        tf = true;
      } else {
        tf = false;
      }
      i = TrialState->lambda.size[0];
      i1 = temp + idx;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &ud_emlrtBCI, (emlrtCTX)sp);
      }
      i = TrialState->lambda.size[0];
      if (i1 > i) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &ud_emlrtBCI, (emlrtCTX)sp);
      }
      TrialState->lambda.data[i1 - 1] *= (real_T)tf;
    }
    temp = WorkingSet->isActiveIdx[2];
    mFiniteLB = WorkingSet->nActiveConstr;
    st.site = &oh_emlrtRSI;
    if ((WorkingSet->isActiveIdx[2] <= WorkingSet->nActiveConstr) &&
        (WorkingSet->nActiveConstr > 2147483646)) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (idx = temp; idx <= mFiniteLB; idx++) {
      i = WorkingSet->Wlocalidx.size[0];
      if ((idx < 1) || (idx > i)) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ud_emlrtBCI, (emlrtCTX)sp);
      }
      i = WorkingSet->Wid.size[0];
      if (idx > i) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ud_emlrtBCI, (emlrtCTX)sp);
      }
      if (WorkingSet->Wid.data[idx - 1] == 3) {
        i = memspace->workspace_int.size[0];
        i1 = WorkingSet->Wlocalidx.data[idx - 1];
        if ((i1 + 120 < 1) || (i1 + 120 > i)) {
          emlrtDynamicBoundsCheckR2012b(i1 + 120, 1, i, &ud_emlrtBCI,
                                        (emlrtCTX)sp);
        }
        i = TrialState->lambda.size[0];
        if (idx > i) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ud_emlrtBCI, (emlrtCTX)sp);
        }
        i = TrialState->lambda.size[0];
        if (idx > i) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ud_emlrtBCI, (emlrtCTX)sp);
        }
        TrialState->lambda.data[idx - 1] *=
            (real_T)memspace->workspace_int.data[i1 + 119];
      }
    }
  }
  st.site = &oh_emlrtRSI;
  temp = WorkingSet->sizes[0];
  mFiniteLB = (WorkingSet->sizes[3] - WorkingSet->sizes[2]) - 120;
  idx = WorkingSet->nActiveConstr;
  while ((idx > temp + 60) && (nActiveLBArtificial > 0)) {
    i = WorkingSet->Wid.size[0];
    if ((idx < 1) || (idx > i)) {
      emlrtDynamicBoundsCheckR2012b(idx, 1, i, &xd_emlrtBCI, &st);
    }
    if (WorkingSet->Wid.data[idx - 1] == 4) {
      i = WorkingSet->Wlocalidx.size[0];
      if (idx > i) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &xd_emlrtBCI, &st);
      }
      i = WorkingSet->Wlocalidx.data[idx - 1];
      if (i > mFiniteLB) {
        i1 = TrialState->lambda.size[0];
        if ((WorkingSet->nActiveConstr < 1) ||
            (WorkingSet->nActiveConstr > i1)) {
          emlrtDynamicBoundsCheckR2012b(WorkingSet->nActiveConstr, 1, i1,
                                        &xd_emlrtBCI, &st);
        }
        tmp_tmp_tmp = WorkingSet->nActiveConstr - 1;
        beta = TrialState->lambda.data[tmp_tmp_tmp];
        i1 = TrialState->lambda.size[0];
        if ((WorkingSet->nActiveConstr < 1) ||
            (WorkingSet->nActiveConstr > i1)) {
          emlrtDynamicBoundsCheckR2012b(WorkingSet->nActiveConstr, 1, i1,
                                        &xd_emlrtBCI, &st);
        }
        TrialState->lambda.data[tmp_tmp_tmp] = 0.0;
        i1 = TrialState->lambda.size[0];
        if (idx > i1) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, i1, &xd_emlrtBCI, &st);
        }
        TrialState->lambda.data[idx - 1] = beta;
        b_st.site = &qh_emlrtRSI;
        i1 = WorkingSet->Wid.size[0];
        if (idx > i1) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, i1, &ec_emlrtBCI, &b_st);
        }
        i1 = WorkingSet->Wlocalidx.size[0];
        if (idx > i1) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, i1, &ec_emlrtBCI, &b_st);
        }
        i1 = WorkingSet->isActiveConstr.size[0];
        i = (WorkingSet->isActiveIdx[3] + i) - 1;
        if ((i < 1) || (i > i1)) {
          emlrtDynamicBoundsCheckR2012b(i, 1, i1, &ec_emlrtBCI, &b_st);
        }
        WorkingSet->isActiveConstr.data[i - 1] = false;
        c_st.site = &se_emlrtRSI;
        moveConstraint_(&c_st, WorkingSet, WorkingSet->nActiveConstr, idx);
        WorkingSet->nActiveConstr--;
        WorkingSet->nWConstr[3]--;
        nActiveLBArtificial--;
      }
    }
    idx--;
  }
  QPObjective->nvar = nVarOrig - 1;
  QPObjective->hasLinear = true;
  QPObjective->objtype = 3;
  st.site = &oh_emlrtRSI;
  setProblemType(&st, WorkingSet, 3);
  st.site = &oh_emlrtRSI;
  sortLambdaQP(&st, TrialState->lambda.data, &TrialState->lambda.size[0],
               WorkingSet->nActiveConstr, WorkingSet->sizes,
               WorkingSet->isActiveIdx, WorkingSet->Wid.data,
               WorkingSet->Wid.size[0], WorkingSet->Wlocalidx.data,
               WorkingSet->Wlocalidx.size[0], memspace->workspace_double.data,
               memspace->workspace_double.size);
}

/* End of code generation (relaxed.c) */
