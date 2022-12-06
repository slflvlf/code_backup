/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * iterate.c
 *
 * Code generation for function 'iterate'
 *
 */

/* Include files */
#include "iterate.h"
#include "addAineqConstr.h"
#include "addBoundToActiveSetMatrix_.h"
#include "checkStoppingAndUpdateFval.h"
#include "checkUnboundedOrIllPosed.h"
#include "computeFval_ReuseHx.h"
#include "computeGrad_StoreHx.h"
#include "computeQ_.h"
#include "compute_deltax.h"
#include "deleteColMoveEnd.h"
#include "eml_int_forloop_overflow_check.h"
#include "factorQR.h"
#include "feasibleratiotest.h"
#include "moveConstraint_.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "squareQ_appendCol.h"
#include "strcmp.h"
#include "xaxpy.h"
#include "xcopy.h"
#include "xgemv.h"
#include "xnrm2.h"
#include "xtrsv.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo vf_emlrtRSI = {
    1,         /* lineNo */
    "iterate", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "qpactiveset\\iterate.p" /* pathName */
};

static emlrtRSInfo yg_emlrtRSI = {
    1,                /* lineNo */
    "compute_lambda", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "qpactiveset\\compute_lambda.p" /* pathName */
};

static emlrtRSInfo ah_emlrtRSI = {
    1,                 /* lineNo */
    "find_neg_lambda", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\find_"
    "neg_lambda.p" /* pathName */
};

static emlrtBCInfo bd_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    1,                 /* lineNo */
    1,                 /* colNo */
    "",                /* aName */
    "isNonDegenerate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "QRManager\\isNonDegenerate.p", /* pName */
    0                               /* checkKind */
};

static emlrtBCInfo cd_emlrtBCI = {
    -1,        /* iFirst */
    -1,        /* iLast */
    1,         /* lineNo */
    1,         /* colNo */
    "",        /* aName */
    "iterate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "qpactiveset\\iterate.p", /* pName */
    0                         /* checkKind */
};

static emlrtBCInfo dd_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    1,                /* lineNo */
    1,                /* colNo */
    "",               /* aName */
    "compute_lambda", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "qpactiveset\\compute_lambda.p", /* pName */
    0                                /* checkKind */
};

static emlrtBCInfo ed_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    1,                 /* lineNo */
    1,                 /* colNo */
    "",                /* aName */
    "find_neg_lambda", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\find_"
    "neg_lambda.p", /* pName */
    0               /* checkKind */
};

/* Function Definitions */
void iterate(c_nlmpcmoveCodeGenerationStackD *SD, const emlrtStack *sp,
             const real_T H[70225], const real_T f_data[], int32_T f_size,
             e_struct_T *solution, g_struct_T *memspace, h_struct_T *workingset,
             i_struct_T *qrmanager, j_struct_T *cholmanager,
             f_struct_T *objective, const char_T options_SolverName[7],
             real_T options_StepTolerance, real_T options_ObjectiveLimit,
             int32_T runTimeOptions_MaxIterations)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T tmp_data[3411];
  real_T minLambda;
  int32_T TYPE;
  int32_T activeSetChangeID;
  int32_T globalActiveConstrIdx;
  int32_T i;
  int32_T idx;
  int32_T nVar;
  int32_T tmp_size;
  boolean_T subProblemChanged;
  boolean_T updateFval;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  subProblemChanged = true;
  updateFval = true;
  activeSetChangeID = 0;
  TYPE = objective->objtype;
  nVar = workingset->nVar;
  globalActiveConstrIdx = 0;
  st.site = &vf_emlrtRSI;
  computeGrad_StoreHx(&st, objective, H, f_data, solution->xstar.data,
                      solution->xstar.size[0]);
  st.site = &vf_emlrtRSI;
  solution->fstar = computeFval_ReuseHx(
      &st, objective, memspace->workspace_double.data, f_data, f_size,
      solution->xstar.data, solution->xstar.size[0]);
  if (solution->iterations < runTimeOptions_MaxIterations) {
    solution->state = -5;
  } else {
    solution->state = 0;
  }
  st.site = &vf_emlrtRSI;
  g_xcopy(&st, workingset->mConstrMax, solution->lambda.data);
  int32_T exitg1;
  do {
    exitg1 = 0;
    if (solution->state == -5) {
      int32_T idxQR;
      boolean_T guard1 = false;
      guard1 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
        case 1:
          st.site = &vf_emlrtRSI;
          squareQ_appendCol(&st, qrmanager, workingset->ATwset.data,
                            workingset->ldA * (workingset->nActiveConstr - 1) +
                                1);
          break;
        case -1:
          st.site = &vf_emlrtRSI;
          deleteColMoveEnd(&st, qrmanager, globalActiveConstrIdx);
          break;
        default:
          st.site = &vf_emlrtRSI;
          factorQR(&st, qrmanager, workingset->ATwset.data, nVar,
                   workingset->nActiveConstr, workingset->ldA);
          st.site = &vf_emlrtRSI;
          b_st.site = &ae_emlrtRSI;
          computeQ_(&b_st, qrmanager, qrmanager->mrows);
          break;
        }
        st.site = &vf_emlrtRSI;
        compute_deltax(&st, H, solution, memspace, qrmanager, cholmanager,
                       objective, b_strcmp(options_SolverName));
        if (solution->state != -5) {
          exitg1 = 1;
        } else if ((xnrm2(nVar, solution->searchDir.data) <
                    options_StepTolerance) ||
                   (workingset->nActiveConstr >= nVar)) {
          guard1 = true;
        } else {
          st.site = &vf_emlrtRSI;
          feasibleratiotest(
              &st, solution->xstar.data, solution->xstar.size[0],
              solution->searchDir.data, solution->searchDir.size[0],
              memspace->workspace_double.data, memspace->workspace_double.size,
              workingset->nVar, workingset->ldA, workingset->Aineq.data,
              workingset->bineq.data, workingset->lb.data,
              workingset->lb.size[0], workingset->ub.data,
              workingset->indexLB.data, workingset->indexLB.size[0],
              workingset->indexUB.data, workingset->indexUB.size[0],
              workingset->sizes, workingset->isActiveIdx,
              workingset->isActiveConstr.data,
              workingset->isActiveConstr.size[0], workingset->nWConstr,
              TYPE == 5, &minLambda, &updateFval, &i, &tmp_size);
          if (updateFval) {
            switch (i) {
            case 3:
              st.site = &vf_emlrtRSI;
              addAineqConstr(&st, workingset, tmp_size);
              break;
            case 4:
              st.site = &vf_emlrtRSI;
              b_st.site = &dh_emlrtRSI;
              addBoundToActiveSetMatrix_(&b_st, workingset, 4, tmp_size);
              break;
            default:
              st.site = &vf_emlrtRSI;
              b_st.site = &fh_emlrtRSI;
              addBoundToActiveSetMatrix_(&b_st, workingset, 5, tmp_size);
              break;
            }
            activeSetChangeID = 1;
          } else {
            st.site = &vf_emlrtRSI;
            checkUnboundedOrIllPosed(solution, objective);
            subProblemChanged = false;
            if (workingset->nActiveConstr == 0) {
              solution->state = 1;
            }
          }
          st.site = &vf_emlrtRSI;
          xaxpy(nVar, minLambda, solution->searchDir.data,
                solution->xstar.data);
          st.site = &vf_emlrtRSI;
          computeGrad_StoreHx(&st, objective, H, f_data, solution->xstar.data,
                              solution->xstar.size[0]);
          updateFval = true;
          st.site = &vf_emlrtRSI;
          checkStoppingAndUpdateFval(
              SD, &st, &activeSetChangeID, f_data, f_size, solution, memspace,
              objective, workingset, qrmanager, options_ObjectiveLimit,
              runTimeOptions_MaxIterations, updateFval);
        }
      } else {
        tmp_size = solution->searchDir.size[0];
        idxQR = solution->searchDir.size[0];
        if (idxQR - 1 >= 0) {
          memcpy(&tmp_data[0], &solution->searchDir.data[0],
                 idxQR * sizeof(real_T));
        }
        st.site = &vf_emlrtRSI;
        g_xcopy(&st, nVar, tmp_data);
        solution->searchDir.size[0] = tmp_size;
        if (tmp_size - 1 >= 0) {
          memcpy(&solution->searchDir.data[0], &tmp_data[0],
                 tmp_size * sizeof(real_T));
        }
        guard1 = true;
      }
      if (guard1) {
        st.site = &vf_emlrtRSI;
        tmp_size = qrmanager->ncols;
        if (qrmanager->ncols > 0) {
          boolean_T b_guard1 = false;
          b_guard1 = false;
          if (objective->objtype != 4) {
            minLambda =
                100.0 * (real_T)qrmanager->mrows * 2.2204460492503131E-16;
            b_st.site = &yg_emlrtRSI;
            if ((qrmanager->mrows > 0) && (qrmanager->ncols > 0)) {
              updateFval = true;
            } else {
              updateFval = false;
            }
            if (updateFval) {
              boolean_T exitg2;
              boolean_T guard2 = false;
              idx = tmp_size;
              guard2 = false;
              if (qrmanager->mrows < qrmanager->ncols) {
                idxQR =
                    qrmanager->mrows + qrmanager->ldq * (qrmanager->ncols - 1);
                exitg2 = false;
                while ((!exitg2) && (idx > qrmanager->mrows)) {
                  i = qrmanager->QR.size[0] * qrmanager->QR.size[1];
                  if ((idxQR < 1) || (idxQR > i)) {
                    emlrtDynamicBoundsCheckR2012b(idxQR, 1, i, &bd_emlrtBCI,
                                                  &b_st);
                  }
                  if (muDoubleScalarAbs(qrmanager->QR.data[idxQR - 1]) >=
                      minLambda) {
                    idx--;
                    idxQR -= qrmanager->ldq;
                  } else {
                    exitg2 = true;
                  }
                }
                updateFval = (idx == qrmanager->mrows);
                if (updateFval) {
                  guard2 = true;
                }
              } else {
                guard2 = true;
              }
              if (guard2) {
                idxQR = idx + qrmanager->ldq * (idx - 1);
                exitg2 = false;
                while ((!exitg2) && (idx >= 1)) {
                  i = qrmanager->QR.size[0] * qrmanager->QR.size[1];
                  if ((idxQR < 1) || (idxQR > i)) {
                    emlrtDynamicBoundsCheckR2012b(idxQR, 1, i, &bd_emlrtBCI,
                                                  &b_st);
                  }
                  if (muDoubleScalarAbs(qrmanager->QR.data[idxQR - 1]) >=
                      minLambda) {
                    idx--;
                    idxQR = (idxQR - qrmanager->ldq) - 1;
                  } else {
                    exitg2 = true;
                  }
                }
                updateFval = (idx == 0);
              }
            }
            if (!updateFval) {
              solution->state = -7;
            } else {
              b_guard1 = true;
            }
          } else {
            b_guard1 = true;
          }
          if (b_guard1) {
            b_st.site = &yg_emlrtRSI;
            b_xgemv(qrmanager->mrows, qrmanager->ncols, qrmanager->Q.data,
                    qrmanager->ldq, objective->grad.data,
                    memspace->workspace_double.data);
            b_st.site = &yg_emlrtRSI;
            xtrsv(qrmanager->ncols, qrmanager->QR.data, qrmanager->ldq,
                  memspace->workspace_double.data);
            b_st.site = &yg_emlrtRSI;
            if (qrmanager->ncols > 2147483646) {
              c_st.site = &db_emlrtRSI;
              check_forloop_overflow_error(&c_st);
            }
            for (idx = 0; idx < tmp_size; idx++) {
              i = solution->lambda.size[0];
              if ((idx + 1 < 1) || (idx + 1 > i)) {
                emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &dd_emlrtBCI, &st);
              }
              solution->lambda.data[idx] =
                  -memspace->workspace_double.data[idx];
            }
          }
        }
        if ((solution->state != -7) || (workingset->nActiveConstr > nVar)) {
          int32_T idxMinLambda;
          st.site = &vf_emlrtRSI;
          idxMinLambda = 0;
          minLambda = 0.0;
          tmp_size = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
          idxQR = workingset->nActiveConstr;
          b_st.site = &ah_emlrtRSI;
          if ((tmp_size <= workingset->nActiveConstr) &&
              (workingset->nActiveConstr > 2147483646)) {
            c_st.site = &db_emlrtRSI;
            check_forloop_overflow_error(&c_st);
          }
          for (idx = tmp_size; idx <= idxQR; idx++) {
            real_T d;
            i = solution->lambda.size[0];
            if ((idx < 1) || (idx > i)) {
              emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ed_emlrtBCI, &st);
            }
            d = solution->lambda.data[idx - 1];
            if (d < minLambda) {
              i = solution->lambda.size[0];
              if (idx > i) {
                emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ed_emlrtBCI, &st);
              }
              minLambda = d;
              idxMinLambda = idx;
            }
          }
          if (idxMinLambda == 0) {
            solution->state = 1;
          } else {
            activeSetChangeID = -1;
            globalActiveConstrIdx = idxMinLambda;
            subProblemChanged = true;
            i = workingset->Wid.size[0];
            if (idxMinLambda > i) {
              emlrtDynamicBoundsCheckR2012b(idxMinLambda, 1, i, &cd_emlrtBCI,
                                            (emlrtCTX)sp);
            }
            i = workingset->Wlocalidx.size[0];
            if (idxMinLambda > i) {
              emlrtDynamicBoundsCheckR2012b(idxMinLambda, 1, i, &cd_emlrtBCI,
                                            (emlrtCTX)sp);
            }
            st.site = &vf_emlrtRSI;
            i = workingset->Wid.size[0];
            if (idxMinLambda > i) {
              emlrtDynamicBoundsCheckR2012b(idxMinLambda, 1, i, &ec_emlrtBCI,
                                            &st);
            }
            idxQR = workingset->Wid.data[idxMinLambda - 1];
            i = workingset->Wlocalidx.size[0];
            if (idxMinLambda > i) {
              emlrtDynamicBoundsCheckR2012b(idxMinLambda, 1, i, &ec_emlrtBCI,
                                            &st);
            }
            if ((idxQR < 1) || (idxQR > 6)) {
              emlrtDynamicBoundsCheckR2012b(
                  workingset->Wid.data[idxMinLambda - 1], 1, 6, &fc_emlrtBCI,
                  &st);
            }
            i = workingset->isActiveConstr.size[0];
            tmp_size = (workingset->isActiveIdx[idxQR - 1] +
                        workingset->Wlocalidx.data[idxMinLambda - 1]) -
                       1;
            if ((tmp_size < 1) || (tmp_size > i)) {
              emlrtDynamicBoundsCheckR2012b(tmp_size, 1, i, &ec_emlrtBCI, &st);
            }
            workingset->isActiveConstr.data[tmp_size - 1] = false;
            b_st.site = &te_emlrtRSI;
            moveConstraint_(&b_st, workingset, workingset->nActiveConstr,
                            idxMinLambda);
            workingset->nActiveConstr--;
            if (idxQR > 5) {
              emlrtDynamicBoundsCheckR2012b(6, 1, 5, &kc_emlrtBCI, &st);
            }
            workingset->nWConstr[idxQR - 1]--;
            i = solution->lambda.size[0];
            if (idxMinLambda > i) {
              emlrtDynamicBoundsCheckR2012b(idxMinLambda, 1, i, &cd_emlrtBCI,
                                            (emlrtCTX)sp);
            }
            solution->lambda.data[idxMinLambda - 1] = 0.0;
          }
        } else {
          int32_T idxMinLambda;
          idxMinLambda = workingset->nActiveConstr;
          activeSetChangeID = 0;
          globalActiveConstrIdx = workingset->nActiveConstr;
          subProblemChanged = true;
          i = workingset->Wid.size[0];
          if ((workingset->nActiveConstr < 1) ||
              (workingset->nActiveConstr > i)) {
            emlrtDynamicBoundsCheckR2012b(workingset->nActiveConstr, 1, i,
                                          &cd_emlrtBCI, (emlrtCTX)sp);
          }
          i = workingset->Wlocalidx.size[0];
          if ((workingset->nActiveConstr < 1) ||
              (workingset->nActiveConstr > i)) {
            emlrtDynamicBoundsCheckR2012b(workingset->nActiveConstr, 1, i,
                                          &cd_emlrtBCI, (emlrtCTX)sp);
          }
          st.site = &vf_emlrtRSI;
          i = workingset->Wid.size[0];
          if ((workingset->nActiveConstr < 1) ||
              (workingset->nActiveConstr > i)) {
            emlrtDynamicBoundsCheckR2012b(workingset->nActiveConstr, 1, i,
                                          &ec_emlrtBCI, &st);
          }
          tmp_size = workingset->nActiveConstr - 1;
          idxQR = workingset->Wid.data[tmp_size];
          i = workingset->Wlocalidx.size[0];
          if ((workingset->nActiveConstr < 1) ||
              (workingset->nActiveConstr > i)) {
            emlrtDynamicBoundsCheckR2012b(workingset->nActiveConstr, 1, i,
                                          &ec_emlrtBCI, &st);
          }
          if ((idxQR < 1) || (idxQR > 6)) {
            emlrtDynamicBoundsCheckR2012b(
                workingset->Wid.data[workingset->nActiveConstr - 1], 1, 6,
                &fc_emlrtBCI, &st);
          }
          i = workingset->isActiveConstr.size[0];
          tmp_size = (workingset->isActiveIdx[idxQR - 1] +
                      workingset->Wlocalidx.data[tmp_size]) -
                     1;
          if ((tmp_size < 1) || (tmp_size > i)) {
            emlrtDynamicBoundsCheckR2012b(tmp_size, 1, i, &ec_emlrtBCI, &st);
          }
          workingset->isActiveConstr.data[tmp_size - 1] = false;
          b_st.site = &te_emlrtRSI;
          moveConstraint_(&b_st, workingset, workingset->nActiveConstr,
                          workingset->nActiveConstr);
          workingset->nActiveConstr--;
          if (idxQR > 5) {
            emlrtDynamicBoundsCheckR2012b(6, 1, 5, &kc_emlrtBCI, &st);
          }
          workingset->nWConstr[idxQR - 1]--;
          i = solution->lambda.size[0];
          if ((idxMinLambda < 1) || (idxMinLambda > i)) {
            emlrtDynamicBoundsCheckR2012b(idxMinLambda, 1, i, &cd_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          solution->lambda.data[idxMinLambda - 1] = 0.0;
        }
        updateFval = false;
        st.site = &vf_emlrtRSI;
        checkStoppingAndUpdateFval(SD, &st, &activeSetChangeID, f_data, f_size,
                                   solution, memspace, objective, workingset,
                                   qrmanager, options_ObjectiveLimit,
                                   runTimeOptions_MaxIterations, updateFval);
      }
    } else {
      if (!updateFval) {
        st.site = &vf_emlrtRSI;
        solution->fstar = computeFval_ReuseHx(
            &st, objective, memspace->workspace_double.data, f_data, f_size,
            solution->xstar.data, solution->xstar.size[0]);
      }
      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

/* End of code generation (iterate.c) */
