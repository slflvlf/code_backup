/*
 * fmincon.c
 *
 * Code generation for function 'fmincon'
 *
 */

/* Include files */
#include "fmincon.h"
#include "checkLinearInputs.h"
#include "checkNonlinearInputs.h"
#include "checkX0.h"
#include "computeConstrViolationIneq_.h"
#include "computeLinearResiduals.h"
#include "driver.h"
#include "evalObjAndConstrAndDerivatives.h"
#include "initActiveSet.h"
#include "loadProblem.h"
#include "nlmpcmoveCodeGeneration_internal_types.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "updateWorkingSetForNewQP.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo gb_emlrtRSI = {
    1,         /* lineNo */
    "fmincon", /* fcnName */
    "C:\\Program Files\\MATLAB\\R2022a\\toolbox\\optim\\eml\\fmincon.p" /* pathName
                                                                         */
};

static emlrtRSInfo wc_emlrtRSI = {
    1,                  /* lineNo */
    "factoryConstruct", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "MeritFunction\\factoryConstruct.p" /* pathName */
};

static emlrtRTEInfo e_emlrtRTEI = {
    1,         /* lineNo */
    1,         /* colNo */
    "fmincon", /* fName */
    "C:\\Program Files\\MATLAB\\R2022a\\toolbox\\optim\\eml\\fmincon.p" /* pName
                                                                         */
};

static emlrtBCInfo emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    1,                /* lineNo */
    1,                /* colNo */
    "",               /* aName */
    "compressBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "initialize\\compressBounds.p", /* pName */
    0                               /* checkKind */
};

static emlrtBCInfo b_emlrtBCI = {
    -1,        /* iFirst */
    -1,        /* iLast */
    1,         /* lineNo */
    1,         /* colNo */
    "",        /* aName */
    "fmincon", /* fName */
    "C:\\Program Files\\MATLAB\\R2022a\\toolbox\\optim\\eml\\fmincon.p", /* pName
                                                                          */
    0 /* checkKind */
};

/* Function Definitions */
void fmincon(c_nlmpcmoveCodeGenerationStackD *SD, const emlrtStack *sp,
             const l_struct_T *fun_workspace_runtimedata,
             const k_struct_T *fun_workspace_userdata, const real_T x0[85],
             const real_T Aineq_data[], const int32_T Aineq_size[2],
             const real_T bineq_data[], int32_T bineq_size, const real_T lb[85],
             const real_T ub[85],
             const l_struct_T *nonlcon_workspace_runtimedata,
             const k_struct_T *nonlcon_workspace_userdata, real_T x[85],
             real_T *fval, real_T *exitflag, c_struct_T *output)
{
  emlrtStack b_st;
  emlrtStack st;
  f_struct_T QPObjective;
  n_struct_T FcnEvaluator;
  struct_T MeritFunction;
  real_T normResid;
  real_T scale;
  int32_T i;
  int32_T idx;
  int32_T mConstrMax;
  int32_T mIneq;
  int32_T mNonlinIneq;
  int32_T maxDims;
  int8_T tmp_data[646];
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &gb_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  checkX0(&st, x0);
  st.site = &gb_emlrtRSI;
  checkLinearInputs(&st, Aineq_data, Aineq_size, bineq_data, bineq_size, lb,
                    ub);
  st.site = &gb_emlrtRSI;
  mNonlinIneq =
      checkNonlinearInputs(SD, &st, x0, nonlcon_workspace_runtimedata->x,
                           nonlcon_workspace_runtimedata->OutputMin,
                           nonlcon_workspace_runtimedata->OutputMax);
  mIneq = (bineq_size + mNonlinIneq) + 206;
  mConstrMax = (mIneq + mIneq) - 61;
  maxDims = muIntScalarMax_sint32(mIneq, mConstrMax);
  SD->f7.TrialState.nVarMax = mIneq;
  SD->f7.TrialState.mNonlinIneq = mNonlinIneq;
  SD->f7.TrialState.mNonlinEq = 60;
  SD->f7.TrialState.mIneq = mIneq - 206;
  SD->f7.TrialState.mEq = 60;
  SD->f7.TrialState.iNonIneq0 = (mIneq - mNonlinIneq) - 205;
  SD->f7.TrialState.iNonEq0 = 1;
  SD->f7.TrialState.sqpFval_old = 0.0;
  SD->f7.TrialState.cIneq.size[0] = mIneq - 206;
  SD->f7.TrialState.cIneq_old.size[0] = mIneq - 206;
  SD->f7.TrialState.grad.size[0] = mIneq;
  SD->f7.TrialState.grad_old.size[0] = mIneq;
  SD->f7.TrialState.sqpIterations = 0;
  SD->f7.TrialState.sqpExitFlag = 0;
  SD->f7.TrialState.lambdasqp.size[0] = mConstrMax;
  if (mConstrMax - 1 >= 0) {
    memset(&SD->f7.TrialState.lambdasqp.data[0], 0,
           mConstrMax * sizeof(real_T));
  }
  SD->f7.TrialState.lambdaStopTest.size[0] = mConstrMax;
  SD->f7.TrialState.lambdaStopTestPrev.size[0] = mConstrMax;
  SD->f7.TrialState.steplength = 1.0;
  SD->f7.TrialState.delta_x.size[0] = mIneq;
  if (mIneq - 1 >= 0) {
    memset(&SD->f7.TrialState.delta_x.data[0], 0, mIneq * sizeof(real_T));
  }
  SD->f7.TrialState.socDirection.size[0] = mIneq;
  SD->f7.TrialState.workingset_old.size[0] = mConstrMax;
  if (mNonlinIneq > 0) {
    SD->f7.TrialState.JacCineqTrans_old.size[0] = mIneq;
    SD->f7.TrialState.JacCineqTrans_old.size[1] = mNonlinIneq;
  } else {
    SD->f7.TrialState.JacCineqTrans_old.size[0] = 0;
    SD->f7.TrialState.JacCineqTrans_old.size[1] = 0;
  }
  SD->f7.TrialState.JacCeqTrans_old.size[0] = mIneq;
  SD->f7.TrialState.JacCeqTrans_old.size[1] = 60;
  SD->f7.TrialState.gradLag.size[0] = mIneq;
  SD->f7.TrialState.delta_gradLag.size[0] = mIneq;
  SD->f7.TrialState.xstar.size[0] = mIneq;
  SD->f7.TrialState.fstar = 0.0;
  SD->f7.TrialState.firstorderopt = 0.0;
  SD->f7.TrialState.lambda.size[0] = mConstrMax;
  if (mConstrMax - 1 >= 0) {
    memset(&SD->f7.TrialState.lambda.data[0], 0, mConstrMax * sizeof(real_T));
  }
  SD->f7.TrialState.state = 0;
  SD->f7.TrialState.maxConstr = 0.0;
  SD->f7.TrialState.iterations = 0;
  SD->f7.TrialState.searchDir.size[0] = mIneq;
  memcpy(&SD->f7.TrialState.xstarsqp[0], &x0[0], 85U * sizeof(real_T));
  FcnEvaluator.mCineq = mNonlinIneq;
  FcnEvaluator.objfun.workspace.runtimedata = *fun_workspace_runtimedata;
  FcnEvaluator.objfun.workspace.userdata = *fun_workspace_userdata;
  FcnEvaluator.nonlcon.workspace.runtimedata = *nonlcon_workspace_runtimedata;
  FcnEvaluator.nonlcon.workspace.userdata = *nonlcon_workspace_userdata;
  FcnEvaluator.nVar = 85;
  FcnEvaluator.mCeq = 60;
  FcnEvaluator.NonFiniteSupport = true;
  FcnEvaluator.SpecifyObjectiveGradient = true;
  FcnEvaluator.SpecifyConstraintGradient = true;
  FcnEvaluator.ScaleProblem = false;
  QPObjective.grad.size[0] = mIneq;
  QPObjective.Hx.size[0] = mIneq - 1;
  QPObjective.maxVar = mIneq;
  QPObjective.beta = 0.0;
  QPObjective.rho = 0.0;
  QPObjective.prev_objtype = 3;
  QPObjective.prev_nvar = 0;
  QPObjective.prev_hasLinear = false;
  QPObjective.gammaScalar = 0.0;
  QPObjective.hasLinear = true;
  QPObjective.nvar = 85;
  QPObjective.objtype = 3;
  SD->f7.memspace.workspace_double.size[0] = maxDims;
  SD->f7.memspace.workspace_double.size[1] = mIneq;
  SD->f7.memspace.workspace_int.size[0] = maxDims;
  SD->f7.memspace.workspace_sort.size[0] = maxDims;
  SD->f7.WorkingSet.mConstr = 0;
  SD->f7.WorkingSet.mConstrOrig = 0;
  SD->f7.WorkingSet.mConstrMax = mConstrMax;
  SD->f7.WorkingSet.nVar = 85;
  SD->f7.WorkingSet.nVarOrig = 85;
  SD->f7.WorkingSet.nVarMax = mIneq;
  SD->f7.WorkingSet.ldA = mIneq;
  SD->f7.WorkingSet.Aineq.size[0] = (mIneq - 206) * mIneq;
  SD->f7.WorkingSet.bineq.size[0] = mIneq - 206;
  SD->f7.WorkingSet.Aeq.size[0] = 60 * mIneq;
  SD->f7.WorkingSet.lb.size[0] = mIneq;
  SD->f7.WorkingSet.ub.size[0] = mIneq;
  SD->f7.WorkingSet.indexUB.size[0] = mIneq;
  SD->f7.WorkingSet.indexFixed.size[0] = mIneq;
  SD->f7.WorkingSet.mEqRemoved = 0;
  SD->f7.WorkingSet.ATwset.size[0] = mIneq * mConstrMax;
  SD->f7.WorkingSet.bwset.size[0] = mConstrMax;
  SD->f7.WorkingSet.nActiveConstr = 0;
  SD->f7.WorkingSet.maxConstrWorkspace.size[0] = mConstrMax;
  for (i = 0; i < 5; i++) {
    SD->f7.WorkingSet.sizes[i] = 0;
    SD->f7.WorkingSet.sizesNormal[i] = 0;
    SD->f7.WorkingSet.sizesPhaseOne[i] = 0;
    SD->f7.WorkingSet.sizesRegularized[i] = 0;
    SD->f7.WorkingSet.sizesRegPhaseOne[i] = 0;
  }
  for (i = 0; i < 6; i++) {
    SD->f7.WorkingSet.isActiveIdx[i] = 0;
    SD->f7.WorkingSet.isActiveIdxNormal[i] = 0;
    SD->f7.WorkingSet.isActiveIdxPhaseOne[i] = 0;
    SD->f7.WorkingSet.isActiveIdxRegularized[i] = 0;
    SD->f7.WorkingSet.isActiveIdxRegPhaseOne[i] = 0;
  }
  SD->f7.WorkingSet.isActiveConstr.size[0] = mConstrMax;
  SD->f7.WorkingSet.Wid.size[0] = mConstrMax;
  SD->f7.WorkingSet.Wlocalidx.size[0] = mConstrMax;
  for (i = 0; i < 5; i++) {
    SD->f7.WorkingSet.nWConstr[i] = 0;
  }
  SD->f7.WorkingSet.probType = 3;
  SD->f7.WorkingSet.SLACK0 = 1.0E-5;
  st.site = &gb_emlrtRSI;
  for (idx = 0; idx < mIneq; idx++) {
    tmp_data[idx] = (int8_T)SD->f7.WorkingSet.indexLB.data[idx];
  }
  i = 0;
  for (idx = 0; idx < 85; idx++) {
    normResid = lb[idx];
    if ((!muDoubleScalarIsInf(normResid)) &&
        (!muDoubleScalarIsNaN(normResid))) {
      i++;
      if (i > mIneq) {
        emlrtDynamicBoundsCheckR2012b(i, 1, mIneq, &emlrtBCI, &st);
      }
      tmp_data[i - 1] = (int8_T)(idx + 1);
    }
  }
  SD->f7.WorkingSet.indexLB.size[0] = mIneq;
  for (idx = 0; idx < mIneq; idx++) {
    SD->f7.WorkingSet.indexLB.data[idx] = tmp_data[idx];
  }
  st.site = &gb_emlrtRSI;
  loadProblem(&st, &SD->f7.WorkingSet, mIneq - 206, bineq_size, Aineq_data,
              Aineq_size, i, mConstrMax);
  st.site = &gb_emlrtRSI;
  for (idx = 0; idx < i; idx++) {
    if (idx + 1 > SD->f7.WorkingSet.indexLB.size[0]) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1,
                                    SD->f7.WorkingSet.indexLB.size[0],
                                    &b_emlrtBCI, (emlrtCTX)sp);
    }
    SD->f7.TrialState.xstarsqp[SD->f7.WorkingSet.indexLB.data[idx] - 1] =
        muDoubleScalarMax(
            SD->f7.TrialState.xstarsqp[SD->f7.WorkingSet.indexLB.data[idx] - 1],
            lb[SD->f7.WorkingSet.indexLB.data[idx] - 1]);
  }
  st.site = &gb_emlrtRSI;
  st.site = &gb_emlrtRSI;
  st.site = &gb_emlrtRSI;
  evalObjAndConstrAndDerivatives(
      SD, &st, fun_workspace_runtimedata->x, fun_workspace_runtimedata->lastMV,
      fun_workspace_runtimedata->ref, fun_workspace_runtimedata->OutputWeights,
      fun_workspace_runtimedata->MVWeights,
      fun_workspace_runtimedata->MVRateWeights,
      fun_workspace_runtimedata->MVScaledTarget,
      nonlcon_workspace_runtimedata->x,
      nonlcon_workspace_runtimedata->OutputMin,
      nonlcon_workspace_runtimedata->OutputMax, mNonlinIneq,
      SD->f7.TrialState.xstarsqp, SD->f7.TrialState.grad.data,
      &SD->f7.TrialState.grad.size[0], SD->f7.TrialState.cIneq.data,
      &SD->f7.TrialState.cIneq.size[0], SD->f7.TrialState.iNonIneq0,
      SD->f7.TrialState.cEq, SD->f7.WorkingSet.Aineq.data,
      &SD->f7.WorkingSet.Aineq.size[0], SD->f7.TrialState.iNonIneq0,
      SD->f7.WorkingSet.ldA, SD->f7.WorkingSet.Aeq.data,
      &SD->f7.WorkingSet.Aeq.size[0], SD->f7.WorkingSet.ldA,
      &SD->f7.TrialState.sqpFval, &mConstrMax);
  if (mConstrMax != 1) {
    emlrtErrorWithMessageIdR2018a(sp, &e_emlrtRTEI,
                                  "optim_codegen:fmincon:UndefAtX0",
                                  "optim_codegen:fmincon:UndefAtX0", 0);
  }
  SD->f7.TrialState.FunctionEvaluations = 1;
  st.site = &gb_emlrtRSI;
  computeLinearResiduals(
      SD->f7.TrialState.xstarsqp, SD->f7.TrialState.cIneq.data,
      &SD->f7.TrialState.cIneq.size[0], bineq_size,
      SD->f7.WorkingSet.Aineq.data, bineq_data, SD->f7.WorkingSet.ldA);
  st.site = &gb_emlrtRSI;
  updateWorkingSetForNewQP(
      &st, x0, &SD->f7.WorkingSet, mIneq - 206, SD->f7.TrialState.cIneq.data,
      SD->f7.TrialState.cIneq.size[0], SD->f7.TrialState.cEq, i, lb);
  st.site = &gb_emlrtRSI;
  initActiveSet(&st, &SD->f7.WorkingSet);
  st.site = &gb_emlrtRSI;
  MeritFunction.initFval = SD->f7.TrialState.sqpFval;
  MeritFunction.penaltyParam = 1.0;
  MeritFunction.threshold = 0.0001;
  MeritFunction.nPenaltyDecreases = 0;
  MeritFunction.linearizedConstrViol = 0.0;
  normResid = 0.0;
  for (mConstrMax = 0; mConstrMax < 60; mConstrMax++) {
    normResid += muDoubleScalarAbs(SD->f7.TrialState.cEq[mConstrMax]);
  }
  MeritFunction.initConstrViolationEq = normResid;
  b_st.site = &wc_emlrtRSI;
  MeritFunction.initConstrViolationIneq = computeConstrViolationIneq_(
      &b_st, mIneq - 206, SD->f7.TrialState.cIneq.data,
      SD->f7.TrialState.cIneq.size[0]);
  MeritFunction.phi = 0.0;
  MeritFunction.phiPrimePlus = 0.0;
  MeritFunction.phiFullStep = 0.0;
  MeritFunction.feasRelativeFactor = 0.0;
  MeritFunction.nlpPrimalFeasError = 0.0;
  MeritFunction.nlpDualFeasError = 0.0;
  MeritFunction.nlpComplError = 0.0;
  MeritFunction.firstOrderOpt = 0.0;
  MeritFunction.hasObjective = true;
  SD->f7.obj.ldq = maxDims;
  SD->f7.obj.QR.size[0] = maxDims;
  SD->f7.obj.QR.size[1] = maxDims;
  SD->f7.obj.Q.size[0] = maxDims;
  SD->f7.obj.Q.size[1] = maxDims;
  mConstrMax = maxDims * maxDims;
  if (mConstrMax - 1 >= 0) {
    memset(&SD->f7.obj.Q.data[0], 0, mConstrMax * sizeof(real_T));
  }
  SD->f7.obj.jpvt.size[0] = maxDims;
  if (maxDims - 1 >= 0) {
    memset(&SD->f7.obj.jpvt.data[0], 0, maxDims * sizeof(int32_T));
  }
  SD->f7.obj.mrows = 0;
  SD->f7.obj.ncols = 0;
  SD->f7.obj.tau.size[0] = muIntScalarMin_sint32(maxDims, maxDims);
  SD->f7.obj.minRowCol = 0;
  SD->f7.obj.usedPivoting = false;
  SD->f7.b_obj.FMat.size[0] = maxDims;
  SD->f7.b_obj.FMat.size[1] = maxDims;
  SD->f7.b_obj.ldm = maxDims;
  SD->f7.b_obj.ndims = 0;
  SD->f7.b_obj.info = 0;
  SD->f7.b_obj.scaleFactor = 0.0;
  SD->f7.b_obj.ConvexCheck = true;
  SD->f7.b_obj.regTol_ = rtInf;
  SD->f7.b_obj.workspace_ = rtInf;
  SD->f7.b_obj.workspace2_ = rtInf;
  st.site = &gb_emlrtRSI;
  driver(SD, &st, bineq_data, lb, &SD->f7.TrialState, &MeritFunction,
         &FcnEvaluator, &SD->f7.memspace, &SD->f7.WorkingSet, &SD->f7.obj,
         &SD->f7.b_obj, &QPObjective, bineq_size, mNonlinIneq,
         SD->f7.unusedExpr);
  *fval = SD->f7.TrialState.sqpFval;
  *exitflag = SD->f7.TrialState.sqpExitFlag;
  output->iterations = SD->f7.TrialState.sqpIterations;
  output->funcCount = SD->f7.TrialState.FunctionEvaluations;
  output->algorithm[0] = 's';
  output->algorithm[1] = 'q';
  output->algorithm[2] = 'p';
  output->constrviolation = MeritFunction.nlpPrimalFeasError;
  normResid = 0.0;
  scale = 3.3121686421112381E-170;
  for (mConstrMax = 0; mConstrMax < 85; mConstrMax++) {
    real_T absxk;
    x[mConstrMax] = SD->f7.TrialState.xstarsqp[mConstrMax];
    absxk = muDoubleScalarAbs(SD->f7.TrialState.delta_x.data[mConstrMax]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      normResid = normResid * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      normResid += t * t;
    }
  }
  output->stepsize = scale * muDoubleScalarSqrt(normResid);
  output->lssteplength = SD->f7.TrialState.steplength;
  output->firstorderopt = MeritFunction.firstOrderOpt;
}

/* End of code generation (fmincon.c) */
