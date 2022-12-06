/*
 * soc.c
 *
 * Code generation for function 'soc'
 *
 */

/* Include files */
#include "soc.h"
#include "addAeqConstr.h"
#include "addAineqConstr.h"
#include "addBoundToActiveSetMatrix_.h"
#include "driver1.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_internal_types.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "removeAllIneqConstr.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
#include "updateWorkingSet.h"
#include "xcopy.h"
#include "xnrm2.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo rh_emlrtRSI = {
    1,     /* lineNo */
    "soc", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\soc.p" /* pathName */
};

static emlrtRSInfo th_emlrtRSI = {
    1,                   /* lineNo */
    "restoreWorkingSet", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\+soc\\restoreWorkingSet.p" /* pathName */
};

static emlrtBCInfo yd_emlrtBCI = {
    1,     /* iFirst */
    60,    /* iLast */
    1,     /* lineNo */
    1,     /* colNo */
    "",    /* aName */
    "soc", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\soc.p", /* pName */
    0              /* checkKind */
};

static emlrtBCInfo ae_emlrtBCI = {
    -1,    /* iFirst */
    -1,    /* iLast */
    1,     /* lineNo */
    1,     /* colNo */
    "",    /* aName */
    "soc", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\soc.p", /* pName */
    0              /* checkKind */
};

static emlrtBCInfo be_emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    1,                   /* lineNo */
    1,                   /* colNo */
    "",                  /* aName */
    "restoreWorkingSet", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\+soc\\restoreWorkingSet.p", /* pName */
    0                                  /* checkKind */
};

/* Function Definitions */
boolean_T soc(c_nlmpcmoveCodeGenerationStackD *SD, const emlrtStack *sp,
              const real_T Hessian[7225], const real_T grad_data[],
              int32_T grad_size, e_struct_T *TrialState, g_struct_T *memspace,
              h_struct_T *WorkingSet, i_struct_T *QRManager,
              j_struct_T *CholManager, f_struct_T *QPObjective,
              const d_struct_T *qpoptions)
{
  d_struct_T b_qpoptions;
  d_struct_T c_qpoptions;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  int32_T i;
  int32_T idx;
  int32_T mConstrMax;
  int32_T mIneq;
  int32_T mLB;
  int32_T nVar;
  int32_T nWIneq_old;
  int32_T nWLower_old;
  int32_T nWUpper_old;
  boolean_T exitg1;
  boolean_T success;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  nWIneq_old = WorkingSet->nWConstr[2];
  nWLower_old = WorkingSet->nWConstr[3];
  nWUpper_old = WorkingSet->nWConstr[4];
  nVar = WorkingSet->nVar;
  mConstrMax = WorkingSet->mConstrMax;
  st.site = &rh_emlrtRSI;
  b_st.site = &gc_emlrtRSI;
  c_st.site = &hc_emlrtRSI;
  if (WorkingSet->nVar > 2147483646) {
    d_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&d_st);
  }
  memcpy(&TrialState->xstarsqp[0], &TrialState->xstarsqp_old[0],
         nVar * sizeof(real_T));
  st.site = &rh_emlrtRSI;
  if (WorkingSet->nVar > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (mIneq = 0; mIneq < nVar; mIneq++) {
    i = TrialState->xstar.size[0];
    if ((mIneq + 1 < 1) || (mIneq + 1 > i)) {
      emlrtDynamicBoundsCheckR2012b(mIneq + 1, 1, i, &ae_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    i = TrialState->socDirection.size[0];
    if (mIneq + 1 > i) {
      emlrtDynamicBoundsCheckR2012b(mIneq + 1, 1, i, &ae_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    TrialState->socDirection.data[mIneq] = TrialState->xstar.data[mIneq];
  }
  st.site = &rh_emlrtRSI;
  b_xcopy(WorkingSet->mConstrMax, TrialState->lambda.data,
          TrialState->lambdaStopTest.data);
  SD->u2.f4.WorkingSet = *WorkingSet;
  st.site = &rh_emlrtRSI;
  updateWorkingSet(&st, &SD->u2.f4.WorkingSet, TrialState->cIneq.data,
                   TrialState->cIneq.size[0], TrialState->cEq,
                   TrialState->searchDir.data, TrialState->workingset_old.data,
                   &TrialState->workingset_old.size[0]);
  st.site = &rh_emlrtRSI;
  xcopy(&st, WorkingSet->nVar, TrialState->xstarsqp, TrialState->xstar.data);
  *WorkingSet = SD->u2.f4.WorkingSet;
  b_qpoptions = *qpoptions;
  c_qpoptions = *qpoptions;
  st.site = &rh_emlrtRSI;
  b_driver(SD, &st, Hessian, grad_data, grad_size, TrialState, memspace,
           WorkingSet, QRManager, CholManager, QPObjective, &b_qpoptions,
           &c_qpoptions);
  exitg1 = false;
  while ((!exitg1) && (WorkingSet->mEqRemoved > 0)) {
    if ((WorkingSet->mEqRemoved < 1) || (WorkingSet->mEqRemoved > 60)) {
      emlrtDynamicBoundsCheckR2012b(WorkingSet->mEqRemoved, 1, 60, &yd_emlrtBCI,
                                    (emlrtCTX)sp);
    }
    if (WorkingSet->indexEqRemoved[WorkingSet->mEqRemoved - 1] >= 1) {
      if ((WorkingSet->mEqRemoved < 1) || (WorkingSet->mEqRemoved > 60)) {
        emlrtDynamicBoundsCheckR2012b(WorkingSet->mEqRemoved, 1, 60,
                                      &yd_emlrtBCI, (emlrtCTX)sp);
      }
      st.site = &rh_emlrtRSI;
      addAeqConstr(&st, WorkingSet,
                   WorkingSet->indexEqRemoved[WorkingSet->mEqRemoved - 1]);
      WorkingSet->mEqRemoved--;
    } else {
      exitg1 = true;
    }
  }
  st.site = &rh_emlrtRSI;
  if (nVar > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = 0; idx < nVar; idx++) {
    real_T oldDirIdx_tmp;
    i = TrialState->socDirection.size[0];
    if ((idx + 1 < 1) || (idx + 1 > i)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &ae_emlrtBCI, (emlrtCTX)sp);
    }
    oldDirIdx_tmp = TrialState->socDirection.data[idx];
    i = TrialState->xstar.size[0];
    if (idx + 1 > i) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &ae_emlrtBCI, (emlrtCTX)sp);
    }
    i = TrialState->socDirection.size[0];
    if (idx + 1 > i) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &ae_emlrtBCI, (emlrtCTX)sp);
    }
    TrialState->socDirection.data[idx] =
        TrialState->xstar.data[idx] - oldDirIdx_tmp;
    i = TrialState->xstar.size[0];
    if (idx + 1 > i) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &ae_emlrtBCI, (emlrtCTX)sp);
    }
    TrialState->xstar.data[idx] = oldDirIdx_tmp;
  }
  success = (xnrm2(nVar, TrialState->socDirection.data) <=
             2.0 * xnrm2(nVar, TrialState->xstar.data));
  st.site = &rh_emlrtRSI;
  mIneq = WorkingSet->sizes[2] + 1;
  mLB = WorkingSet->sizes[3];
  b_st.site = &th_emlrtRSI;
  for (idx = 0; idx < 60; idx++) {
    WorkingSet->beq[idx] = -TrialState->cEq[idx];
  }
  b_st.site = &th_emlrtRSI;
  c_st.site = &gc_emlrtRSI;
  for (nVar = 0; nVar < 60; nVar++) {
    WorkingSet->bwset.data[WorkingSet->sizes[0] + nVar] = WorkingSet->beq[nVar];
  }
  if (WorkingSet->sizes[2] > 0) {
    b_st.site = &th_emlrtRSI;
    if (WorkingSet->sizes[2] > 2147483646) {
      c_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&c_st);
    }
    i = TrialState->cIneq.size[0];
    for (idx = 0; idx <= mIneq - 2; idx++) {
      if ((idx + 1 < 1) || (idx + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &be_emlrtBCI, &st);
      }
      nVar = WorkingSet->bineq.size[0];
      if (idx + 1 > nVar) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, nVar, &be_emlrtBCI, &st);
      }
      WorkingSet->bineq.data[idx] = -TrialState->cIneq.data[idx];
    }
    if (!success) {
      b_st.site = &th_emlrtRSI;
      removeAllIneqConstr(&b_st, WorkingSet);
      b_st.site = &th_emlrtRSI;
      if (nWIneq_old > 2147483646) {
        c_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (idx = 0; idx < nWIneq_old; idx++) {
        i = TrialState->workingset_old.size[0];
        if ((idx + 1 < 1) || (idx + 1 > i)) {
          emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &be_emlrtBCI, &st);
        }
        b_st.site = &th_emlrtRSI;
        addAineqConstr(&b_st, WorkingSet, TrialState->workingset_old.data[idx]);
      }
      b_st.site = &th_emlrtRSI;
      if (nWLower_old > 2147483646) {
        c_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (idx = 0; idx < nWLower_old; idx++) {
        b_st.site = &th_emlrtRSI;
        i = TrialState->workingset_old.size[0];
        nVar = idx + mIneq;
        if ((nVar < 1) || (nVar > i)) {
          emlrtDynamicBoundsCheckR2012b(nVar, 1, i, &be_emlrtBCI, &b_st);
        }
        c_st.site = &ch_emlrtRSI;
        addBoundToActiveSetMatrix_(&c_st, WorkingSet, 4,
                                   TrialState->workingset_old.data[nVar - 1]);
      }
      b_st.site = &th_emlrtRSI;
      if (nWUpper_old > 2147483646) {
        c_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (idx = 0; idx < nWUpper_old; idx++) {
        b_st.site = &th_emlrtRSI;
        i = TrialState->workingset_old.size[0];
        nVar = (idx + mIneq) + mLB;
        if ((nVar < 1) || (nVar > i)) {
          emlrtDynamicBoundsCheckR2012b(nVar, 1, i, &be_emlrtBCI, &b_st);
        }
        c_st.site = &eh_emlrtRSI;
        addBoundToActiveSetMatrix_(&c_st, WorkingSet, 5,
                                   TrialState->workingset_old.data[nVar - 1]);
      }
    }
  }
  if (!success) {
    st.site = &rh_emlrtRSI;
    b_xcopy(mConstrMax, TrialState->lambdaStopTest.data,
            TrialState->lambda.data);
  } else {
    st.site = &rh_emlrtRSI;
    sortLambdaQP(&st, TrialState->lambda.data, &TrialState->lambda.size[0],
                 WorkingSet->nActiveConstr, WorkingSet->sizes,
                 WorkingSet->isActiveIdx, WorkingSet->Wid.data,
                 WorkingSet->Wid.size[0], WorkingSet->Wlocalidx.data,
                 WorkingSet->Wlocalidx.size[0], memspace->workspace_double.data,
                 memspace->workspace_double.size);
  }
  return success;
}

/* End of code generation (soc.c) */
