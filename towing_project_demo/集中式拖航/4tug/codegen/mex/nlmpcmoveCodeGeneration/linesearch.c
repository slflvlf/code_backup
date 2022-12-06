/*
 * linesearch.c
 *
 * Code generation for function 'linesearch'
 *
 */

/* Include files */
#include "linesearch.h"
#include "computeConstrViolationIneq_.h"
#include "computeLinearResiduals.h"
#include "eml_int_forloop_overflow_check.h"
#include "evalObjAndConstr.h"
#include "isDeltaXTooSmall.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_internal_types.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo bi_emlrtRSI = {
    1,            /* lineNo */
    "linesearch", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "fminconsqp\\linesearch.p" /* pathName */
};

static emlrtBCInfo de_emlrtBCI = {
    -1,           /* iFirst */
    -1,           /* iLast */
    1,            /* lineNo */
    1,            /* colNo */
    "",           /* aName */
    "linesearch", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "fminconsqp\\linesearch.p", /* pName */
    0                           /* checkKind */
};

static emlrtBCInfo ee_emlrtBCI = {
    1,            /* iFirst */
    85,           /* iLast */
    1,            /* lineNo */
    1,            /* colNo */
    "",           /* aName */
    "linesearch", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "fminconsqp\\linesearch.p", /* pName */
    3                           /* checkKind */
};

/* Function Definitions */
void linesearch(const emlrtStack *sp, boolean_T *evalWellDefined,
                const real_T bineq_data[], int32_T WorkingSet_nVar,
                int32_T WorkingSet_ldA, const real_T WorkingSet_Aineq_data[],
                e_struct_T *TrialState, real_T MeritFunction_penaltyParam,
                real_T MeritFunction_phi, real_T MeritFunction_phiPrimePlus,
                real_T MeritFunction_phiFullStep,
                const l_struct_T *c_FcnEvaluator_objfun_workspace,
                const l_struct_T *c_FcnEvaluator_nonlcon_workspac,
                int32_T FcnEvaluator_mCineq, boolean_T socTaken, real_T *alpha,
                int32_T *exitflag)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack st;
  real_T y_data[1231];
  real_T phi_alpha;
  int32_T loop_ub;
  int32_T mLinIneq;
  int32_T y_size;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  mLinIneq = TrialState->mIneq - TrialState->mNonlinIneq;
  *alpha = 1.0;
  *exitflag = 1;
  phi_alpha = MeritFunction_phiFullStep;
  st.site = &bi_emlrtRSI;
  y_size = TrialState->searchDir.size[0];
  loop_ub = TrialState->searchDir.size[0];
  if (loop_ub - 1 >= 0) {
    memcpy(&y_data[0], &TrialState->searchDir.data[0],
           loop_ub * sizeof(real_T));
  }
  if (WorkingSet_nVar >= 1) {
    n_t = (ptrdiff_t)WorkingSet_nVar;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dcopy(&n_t, &TrialState->delta_x.data[0], &incx_t, &y_data[0], &incy_t);
  }
  TrialState->searchDir.size[0] = y_size;
  if (y_size - 1 >= 0) {
    memcpy(&TrialState->searchDir.data[0], &y_data[0], y_size * sizeof(real_T));
  }
  int32_T exitg1;
  do {
    exitg1 = 0;
    if (TrialState->FunctionEvaluations < 8500) {
      if ((*evalWellDefined) &&
          (phi_alpha <=
           MeritFunction_phi + *alpha * 0.0001 * MeritFunction_phiPrimePlus)) {
        exitg1 = 1;
      } else {
        boolean_T tooSmallX;
        *alpha *= 0.7;
        st.site = &bi_emlrtRSI;
        if (WorkingSet_nVar > 2147483646) {
          b_st.site = &db_emlrtRSI;
          check_forloop_overflow_error(&b_st);
        }
        for (loop_ub = 0; loop_ub < WorkingSet_nVar; loop_ub++) {
          y_size = TrialState->xstar.size[0];
          if ((loop_ub + 1 < 1) || (loop_ub + 1 > y_size)) {
            emlrtDynamicBoundsCheckR2012b(loop_ub + 1, 1, y_size, &de_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          y_size = TrialState->delta_x.size[0];
          if (loop_ub + 1 > y_size) {
            emlrtDynamicBoundsCheckR2012b(loop_ub + 1, 1, y_size, &de_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          TrialState->delta_x.data[loop_ub] =
              *alpha * TrialState->xstar.data[loop_ub];
        }
        if (socTaken) {
          phi_alpha = *alpha * *alpha;
          st.site = &bi_emlrtRSI;
          if (WorkingSet_nVar >= 1) {
            n_t = (ptrdiff_t)WorkingSet_nVar;
            incx_t = (ptrdiff_t)1;
            incy_t = (ptrdiff_t)1;
            daxpy(&n_t, &phi_alpha, &TrialState->socDirection.data[0], &incx_t,
                  &TrialState->delta_x.data[0], &incy_t);
          }
        }
        st.site = &bi_emlrtRSI;
        tooSmallX = isDeltaXTooSmall(&st, TrialState->xstarsqp,
                                     TrialState->delta_x.data, WorkingSet_nVar);
        if (tooSmallX) {
          *exitflag = -2;
          exitg1 = 1;
        } else {
          st.site = &bi_emlrtRSI;
          for (loop_ub = 0; loop_ub < WorkingSet_nVar; loop_ub++) {
            if ((loop_ub + 1 < 1) || (loop_ub + 1 > 85)) {
              emlrtDynamicBoundsCheckR2012b(loop_ub + 1, 1, 85, &ee_emlrtBCI,
                                            (emlrtCTX)sp);
            }
            TrialState->xstarsqp[loop_ub] = TrialState->xstarsqp_old[loop_ub] +
                                            TrialState->delta_x.data[loop_ub];
          }
          st.site = &bi_emlrtRSI;
          evalObjAndConstr(&st, c_FcnEvaluator_objfun_workspace->x,
                           c_FcnEvaluator_objfun_workspace->lastMV,
                           c_FcnEvaluator_objfun_workspace->ref,
                           c_FcnEvaluator_objfun_workspace->OutputWeights,
                           c_FcnEvaluator_objfun_workspace->MVWeights,
                           c_FcnEvaluator_objfun_workspace->MVRateWeights,
                           c_FcnEvaluator_objfun_workspace->MVScaledTarget,
                           c_FcnEvaluator_nonlcon_workspac->x,
                           c_FcnEvaluator_nonlcon_workspac->OutputMin,
                           c_FcnEvaluator_nonlcon_workspac->OutputMax,
                           FcnEvaluator_mCineq, TrialState->xstarsqp,
                           TrialState->cIneq.data, &TrialState->cIneq.size[0],
                           TrialState->iNonIneq0, TrialState->cEq,
                           &TrialState->sqpFval, &y_size);
          st.site = &bi_emlrtRSI;
          b_computeLinearResiduals(
              TrialState->xstarsqp, WorkingSet_nVar, TrialState->cIneq.data,
              &TrialState->cIneq.size[0], mLinIneq, WorkingSet_Aineq_data,
              bineq_data, WorkingSet_ldA);
          TrialState->FunctionEvaluations++;
          *evalWellDefined = (y_size == 1);
          st.site = &bi_emlrtRSI;
          if (*evalWellDefined) {
            phi_alpha = 0.0;
            for (y_size = 0; y_size < 60; y_size++) {
              phi_alpha += muDoubleScalarAbs(TrialState->cEq[y_size]);
            }
            real_T constrViolationIneq;
            b_st.site = &ai_emlrtRSI;
            constrViolationIneq = computeConstrViolationIneq_(
                &b_st, TrialState->mIneq, TrialState->cIneq.data,
                TrialState->cIneq.size[0]);
            phi_alpha =
                TrialState->sqpFval +
                MeritFunction_penaltyParam * (phi_alpha + constrViolationIneq);
          } else {
            phi_alpha = rtInf;
          }
        }
      }
    } else {
      *exitflag = 0;
      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

/* End of code generation (linesearch.c) */
