/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * step.c
 *
 * Code generation for function 'step'
 *
 */

/* Include files */
#include "step.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_internal_types.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "normal.h"
#include "relaxed.h"
#include "removeAllIneqConstr.h"
#include "rt_nonfinite.h"
#include "soc.h"
#include "xcopy.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ke_emlrtRSI =
    {
        1,      /* lineNo */
        "step", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
        "fminconsqp\\step.p" /* pathName */
};

static emlrtRSInfo nh_emlrtRSI = {
    1,                   /* lineNo */
    "makeBoundFeasible", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\makeBoundFeasible.p" /* pathName */
};

static emlrtRSInfo vh_emlrtRSI = {
    1,                   /* lineNo */
    "saturateDirection", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\saturateDirection.p" /* pathName */
};

static emlrtRSInfo wh_emlrtRSI = {
    1,           /* lineNo */
    "BFGSReset", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "fminconsqp\\BFGSReset.p" /* pathName */
};

static emlrtBCInfo sb_emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    1,                   /* lineNo */
    1,                   /* colNo */
    "",                  /* aName */
    "saturateDirection", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\saturateDirection.p", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo tb_emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    1,                   /* lineNo */
    1,                   /* colNo */
    "",                  /* aName */
    "makeBoundFeasible", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\makeBoundFeasible.p", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo ub_emlrtBCI = {
    1,                   /* iFirst */
    265,                 /* iLast */
    1,                   /* lineNo */
    1,                   /* colNo */
    "",                  /* aName */
    "makeBoundFeasible", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\makeBoundFeasible.p", /* pName */
    0                            /* checkKind */
};

static emlrtBCInfo vb_emlrtBCI = {
    -1,     /* iFirst */
    -1,     /* iLast */
    1,      /* lineNo */
    1,      /* colNo */
    "",     /* aName */
    "step", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\step."
    "p", /* pName */
    0    /* checkKind */
};

static emlrtBCInfo wb_emlrtBCI = {
    1,                   /* iFirst */
    265,                 /* iLast */
    1,                   /* lineNo */
    1,                   /* colNo */
    "",                  /* aName */
    "saturateDirection", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "step\\saturateDirection.p", /* pName */
    0                            /* checkKind */
};

/* Function Definitions */
boolean_T step(c_nlmpcmoveCodeGenerationStackD *SD, const emlrtStack *sp,
               int32_T *STEP_TYPE, real_T Hessian[70225], const real_T lb[265],
               e_struct_T *TrialState, struct_T *MeritFunction,
               g_struct_T *memspace, h_struct_T *WorkingSet,
               i_struct_T *QRManager, j_struct_T *CholManager,
               f_struct_T *QPObjective, d_struct_T *qpoptions)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T tmp_data[3411];
  real_T b_tmp_data[1802];
  real_T nrmDirInf;
  real_T nrmGradInf;
  int32_T iH0;
  int32_T idx;
  int32_T mLB;
  int32_T mUB;
  int32_T nVar;
  int32_T tmp_size;
  boolean_T checkBoundViolation;
  boolean_T stepSuccess;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  stepSuccess = true;
  checkBoundViolation = true;
  nVar = WorkingSet->nVar;
  if (*STEP_TYPE != 3) {
    st.site = &ke_emlrtRSI;
    xcopy(&st, WorkingSet->nVar, TrialState->xstarsqp, TrialState->xstar.data);
  } else {
    tmp_size = TrialState->searchDir.size[0];
    iH0 = TrialState->searchDir.size[0];
    if (iH0 - 1 >= 0) {
      memcpy(&tmp_data[0], &TrialState->searchDir.data[0],
             iH0 * sizeof(real_T));
    }
    st.site = &ke_emlrtRSI;
    c_xcopy(WorkingSet->nVar, TrialState->xstar.data, tmp_data);
    TrialState->searchDir.size[0] = tmp_size;
    if (tmp_size - 1 >= 0) {
      memcpy(&TrialState->searchDir.data[0], &tmp_data[0],
             tmp_size * sizeof(real_T));
    }
  }
  int32_T exitg1;
  boolean_T guard1 = false;
  do {
    exitg1 = 0;
    guard1 = false;
    switch (*STEP_TYPE) {
    case 1:
      st.site = &ke_emlrtRSI;
      normal(SD, &st, Hessian, TrialState->grad.data, TrialState->grad.size[0],
             TrialState, MeritFunction, memspace, WorkingSet, QRManager,
             CholManager, QPObjective, qpoptions);
      if ((TrialState->state <= 0) && (TrialState->state != -6)) {
        *STEP_TYPE = 2;
      } else {
        tmp_size = TrialState->delta_x.size[0];
        iH0 = TrialState->delta_x.size[0];
        if (iH0 - 1 >= 0) {
          memcpy(&tmp_data[0], &TrialState->delta_x.data[0],
                 iH0 * sizeof(real_T));
        }
        st.site = &ke_emlrtRSI;
        c_xcopy(nVar, TrialState->xstar.data, tmp_data);
        TrialState->delta_x.size[0] = tmp_size;
        if (tmp_size - 1 >= 0) {
          memcpy(&TrialState->delta_x.data[0], &tmp_data[0],
                 tmp_size * sizeof(real_T));
        }
        guard1 = true;
      }
      break;
    case 2:
      st.site = &ke_emlrtRSI;
      removeAllIneqConstr(&st, WorkingSet);
      st.site = &ke_emlrtRSI;
      tmp_size = TrialState->xstar.size[0];
      iH0 = TrialState->xstar.size[0];
      if (iH0 - 1 >= 0) {
        memcpy(&b_tmp_data[0], &TrialState->xstar.data[0],
               iH0 * sizeof(real_T));
      }
      mLB = WorkingSet->sizes[3];
      mUB = WorkingSet->sizes[4];
      b_st.site = &nh_emlrtRSI;
      if (WorkingSet->sizes[3] > 2147483646) {
        c_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (idx = 0; idx < mLB; idx++) {
        iH0 = WorkingSet->indexLB.size[0];
        if ((idx + 1 < 1) || (idx + 1 > iH0)) {
          emlrtDynamicBoundsCheckR2012b(idx + 1, 1, iH0, &tb_emlrtBCI, &st);
        }
        if ((WorkingSet->indexLB.data[idx] < 1) ||
            (WorkingSet->indexLB.data[idx] > tmp_size)) {
          emlrtDynamicBoundsCheckR2012b(WorkingSet->indexLB.data[idx], 1,
                                        tmp_size, &tb_emlrtBCI, &st);
        }
        iH0 = WorkingSet->lb.size[0];
        if ((WorkingSet->indexLB.data[idx] < 1) ||
            (WorkingSet->indexLB.data[idx] > iH0)) {
          emlrtDynamicBoundsCheckR2012b(WorkingSet->indexLB.data[idx], 1, iH0,
                                        &tb_emlrtBCI, &st);
        }
        nrmGradInf = WorkingSet->lb.data[WorkingSet->indexLB.data[idx] - 1];
        if (-b_tmp_data[WorkingSet->indexLB.data[idx] - 1] > nrmGradInf) {
          if ((WorkingSet->indexLB.data[idx] < 1) ||
              (WorkingSet->indexLB.data[idx] > 265)) {
            emlrtDynamicBoundsCheckR2012b(WorkingSet->indexLB.data[idx], 1, 265,
                                          &ub_emlrtBCI, &st);
          }
          b_tmp_data[WorkingSet->indexLB.data[idx] - 1] =
              -nrmGradInf + muDoubleScalarAbs(nrmGradInf);
        }
      }
      b_st.site = &nh_emlrtRSI;
      if (WorkingSet->sizes[4] > 2147483646) {
        c_st.site = &db_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (idx = 0; idx < mUB; idx++) {
        iH0 = WorkingSet->indexUB.size[0];
        if ((idx + 1 < 1) || (idx + 1 > iH0)) {
          emlrtDynamicBoundsCheckR2012b(idx + 1, 1, iH0, &tb_emlrtBCI, &st);
        }
      }
      TrialState->xstar.size[0] = tmp_size;
      if (tmp_size - 1 >= 0) {
        memcpy(&TrialState->xstar.data[0], &b_tmp_data[0],
               tmp_size * sizeof(real_T));
      }
      st.site = &ke_emlrtRSI;
      relaxed(SD, &st, Hessian, TrialState->grad.data, TrialState->grad.size[0],
              TrialState, MeritFunction, memspace, WorkingSet, QRManager,
              CholManager, QPObjective, qpoptions);
      tmp_size = TrialState->delta_x.size[0];
      iH0 = TrialState->delta_x.size[0];
      if (iH0 - 1 >= 0) {
        memcpy(&tmp_data[0], &TrialState->delta_x.data[0],
               iH0 * sizeof(real_T));
      }
      st.site = &ke_emlrtRSI;
      c_xcopy(nVar, TrialState->xstar.data, tmp_data);
      TrialState->delta_x.size[0] = tmp_size;
      if (tmp_size - 1 >= 0) {
        memcpy(&TrialState->delta_x.data[0], &tmp_data[0],
               tmp_size * sizeof(real_T));
      }
      guard1 = true;
      break;
    default:
      iH0 = TrialState->grad.size[0];
      if (iH0 - 1 >= 0) {
        memcpy(&b_tmp_data[0], &TrialState->grad.data[0], iH0 * sizeof(real_T));
      }
      st.site = &ke_emlrtRSI;
      stepSuccess = soc(SD, &st, Hessian, b_tmp_data, TrialState->grad.size[0],
                        TrialState, memspace, WorkingSet, QRManager,
                        CholManager, QPObjective, qpoptions);
      checkBoundViolation = stepSuccess;
      if (stepSuccess && (TrialState->state != -6)) {
        st.site = &ke_emlrtRSI;
        if (nVar > 2147483646) {
          b_st.site = &db_emlrtRSI;
          check_forloop_overflow_error(&b_st);
        }
        for (idx = 0; idx < nVar; idx++) {
          iH0 = TrialState->xstar.size[0];
          if ((idx + 1 < 1) || (idx + 1 > iH0)) {
            emlrtDynamicBoundsCheckR2012b(idx + 1, 1, iH0, &vb_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          iH0 = TrialState->socDirection.size[0];
          if (idx + 1 > iH0) {
            emlrtDynamicBoundsCheckR2012b(idx + 1, 1, iH0, &vb_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          iH0 = TrialState->delta_x.size[0];
          if (idx + 1 > iH0) {
            emlrtDynamicBoundsCheckR2012b(idx + 1, 1, iH0, &vb_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          TrialState->delta_x.data[idx] =
              TrialState->xstar.data[idx] + TrialState->socDirection.data[idx];
        }
      }
      guard1 = true;
      break;
    }
    if (guard1) {
      if (TrialState->state != -6) {
        exitg1 = 1;
      } else {
        st.site = &ke_emlrtRSI;
        nrmGradInf = 0.0;
        nrmDirInf = 1.0;
        for (idx = 0; idx < 265; idx++) {
          nrmGradInf = muDoubleScalarMax(
              nrmGradInf, muDoubleScalarAbs(TrialState->grad.data[idx]));
          nrmDirInf = muDoubleScalarMax(
              nrmDirInf, muDoubleScalarAbs(TrialState->xstar.data[idx]));
        }
        nrmGradInf =
            muDoubleScalarMax(2.2204460492503131E-16, nrmGradInf / nrmDirInf);
        for (tmp_size = 0; tmp_size < 265; tmp_size++) {
          iH0 = 265 * tmp_size;
          b_st.site = &wh_emlrtRSI;
          h_xcopy(&b_st, tmp_size, Hessian, iH0 + 1);
          Hessian[tmp_size + 265 * tmp_size] = nrmGradInf;
          iH0 += tmp_size;
          b_st.site = &wh_emlrtRSI;
          h_xcopy(&b_st, 264 - tmp_size, Hessian, iH0 + 2);
        }
      }
    }
  } while (exitg1 == 0);
  if (checkBoundViolation) {
    mLB = WorkingSet->sizes[3];
    mUB = WorkingSet->sizes[4];
    st.site = &ke_emlrtRSI;
    tmp_size = TrialState->delta_x.size[0];
    iH0 = TrialState->delta_x.size[0];
    if (iH0 - 1 >= 0) {
      memcpy(&b_tmp_data[0], &TrialState->delta_x.data[0],
             iH0 * sizeof(real_T));
    }
    b_st.site = &vh_emlrtRSI;
    if (WorkingSet->sizes[3] > 2147483646) {
      c_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&c_st);
    }
    for (idx = 0; idx < mLB; idx++) {
      iH0 = WorkingSet->indexLB.size[0];
      if ((idx + 1 < 1) || (idx + 1 > iH0)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, iH0, &sb_emlrtBCI, &st);
      }
      if ((WorkingSet->indexLB.data[idx] < 1) ||
          (WorkingSet->indexLB.data[idx] > 265)) {
        emlrtDynamicBoundsCheckR2012b(WorkingSet->indexLB.data[idx], 1, 265,
                                      &wb_emlrtBCI, &st);
      }
      nrmGradInf = b_tmp_data[WorkingSet->indexLB.data[idx] - 1];
      nrmDirInf = (TrialState->xstarsqp[WorkingSet->indexLB.data[idx] - 1] +
                   nrmGradInf) -
                  lb[WorkingSet->indexLB.data[idx] - 1];
      if (nrmDirInf < 0.0) {
        b_tmp_data[WorkingSet->indexLB.data[idx] - 1] = nrmGradInf - nrmDirInf;
        TrialState->xstar.data[WorkingSet->indexLB.data[idx] - 1] -= nrmDirInf;
      }
    }
    b_st.site = &vh_emlrtRSI;
    if (WorkingSet->sizes[4] > 2147483646) {
      c_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&c_st);
    }
    for (idx = 0; idx < mUB; idx++) {
      iH0 = WorkingSet->indexUB.size[0];
      if ((idx + 1 < 1) || (idx + 1 > iH0)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, iH0, &sb_emlrtBCI, &st);
      }
    }
    TrialState->delta_x.size[0] = tmp_size;
    if (tmp_size - 1 >= 0) {
      memcpy(&TrialState->delta_x.data[0], &b_tmp_data[0],
             tmp_size * sizeof(real_T));
    }
  }
  return stepSuccess;
}

/* End of code generation (step.c) */
