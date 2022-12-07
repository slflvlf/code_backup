/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * phaseone.c
 *
 * Code generation for function 'phaseone'
 *
 */

/* Include files */
#include "phaseone.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "iterate.h"
#include "moveConstraint_.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_internal_types.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "removeAllIneqConstr.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo qf_emlrtRSI = {
    1,          /* lineNo */
    "phaseone", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "qpactiveset\\phaseone.p" /* pathName */
};

static emlrtBCInfo wc_emlrtBCI = {
    -1,         /* iFirst */
    -1,         /* iLast */
    1,          /* lineNo */
    1,          /* colNo */
    "",         /* aName */
    "phaseone", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "qpactiveset\\phaseone.p", /* pName */
    0                          /* checkKind */
};

/* Function Definitions */
void phaseone(c_nlmpcmoveCodeGenerationStackD *SD, const emlrtStack *sp,
              const real_T H[70225], const real_T f_data[], int32_T f_size,
              e_struct_T *solution, g_struct_T *memspace,
              h_struct_T *workingset, i_struct_T *qrmanager,
              j_struct_T *cholmanager, f_struct_T *objective,
              d_struct_T *options, const d_struct_T *runTimeOptions)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T PROBTYPE_ORIG;
  int32_T b_workingset;
  int32_T i;
  int32_T mConstr;
  int32_T mEqFixed;
  int32_T nVar;
  int32_T nVarP1_tmp;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  PROBTYPE_ORIG = workingset->probType;
  nVar = workingset->nVar;
  nVarP1_tmp = workingset->nVar + 1;
  b_workingset = solution->xstar.size[0];
  if ((nVarP1_tmp < 1) || (nVarP1_tmp > b_workingset)) {
    emlrtDynamicBoundsCheckR2012b(nVarP1_tmp, 1, b_workingset, &wc_emlrtBCI,
                                  (emlrtCTX)sp);
  }
  solution->xstar.data[nVarP1_tmp - 1] = solution->maxConstr + 1.0;
  if (workingset->probType == 3) {
    b_workingset = 1;
  } else {
    b_workingset = 4;
  }
  st.site = &qf_emlrtRSI;
  setProblemType(&st, workingset, b_workingset);
  st.site = &qf_emlrtRSI;
  removeAllIneqConstr(&st, workingset);
  objective->prev_objtype = objective->objtype;
  objective->prev_nvar = objective->nvar;
  objective->prev_hasLinear = objective->hasLinear;
  objective->objtype = 5;
  objective->nvar = nVarP1_tmp;
  objective->gammaScalar = 1.0;
  objective->hasLinear = true;
  options->ObjectiveLimit = 1.0E-6;
  options->StepTolerance = 1.4901161193847657E-10;
  st.site = &qf_emlrtRSI;
  solution->fstar =
      computeFval(&st, objective, memspace->workspace_double.data, H, f_data,
                  f_size, solution->xstar.data, solution->xstar.size[0]);
  solution->state = 5;
  st.site = &qf_emlrtRSI;
  iterate(SD, &st, H, f_data, f_size, solution, memspace, workingset, qrmanager,
          cholmanager, objective, options->SolverName, options->StepTolerance,
          options->ObjectiveLimit, runTimeOptions->MaxIterations);
  st.site = &qf_emlrtRSI;
  b_workingset = workingset->isActiveConstr.size[0];
  i = workingset->isActiveIdx[3] + workingset->sizes[3];
  if ((i - 1 < 1) || (i - 1 > b_workingset)) {
    emlrtDynamicBoundsCheckR2012b(i - 1, 1, b_workingset, &vc_emlrtBCI, &st);
  }
  if (workingset->isActiveConstr.data[i - 2]) {
    boolean_T exitg1;
    st.site = &qf_emlrtRSI;
    if ((workingset->sizes[0] + 193 <= workingset->nActiveConstr) &&
        (workingset->nActiveConstr > 2147483646)) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    mConstr = workingset->sizes[0] + 193;
    exitg1 = false;
    while ((!exitg1) && (mConstr <= workingset->nActiveConstr)) {
      b_workingset = workingset->Wid.size[0];
      if ((mConstr < 1) || (mConstr > b_workingset)) {
        emlrtDynamicBoundsCheckR2012b(mConstr, 1, b_workingset, &wc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      if (workingset->Wid.data[mConstr - 1] == 4) {
        b_workingset = workingset->Wlocalidx.size[0];
        if (mConstr > b_workingset) {
          emlrtDynamicBoundsCheckR2012b(mConstr, 1, b_workingset, &wc_emlrtBCI,
                                        (emlrtCTX)sp);
        }
        b_workingset = workingset->Wlocalidx.data[mConstr - 1];
        if (b_workingset == workingset->sizes[3]) {
          st.site = &qf_emlrtRSI;
          i = workingset->Wid.size[0];
          if (mConstr > i) {
            emlrtDynamicBoundsCheckR2012b(mConstr, 1, i, &ec_emlrtBCI, &st);
          }
          i = workingset->Wlocalidx.size[0];
          if (mConstr > i) {
            emlrtDynamicBoundsCheckR2012b(mConstr, 1, i, &ec_emlrtBCI, &st);
          }
          i = workingset->isActiveConstr.size[0];
          b_workingset = (workingset->isActiveIdx[3] + b_workingset) - 1;
          if ((b_workingset < 1) || (b_workingset > i)) {
            emlrtDynamicBoundsCheckR2012b(b_workingset, 1, i, &ec_emlrtBCI,
                                          &st);
          }
          workingset->isActiveConstr.data[b_workingset - 1] = false;
          b_st.site = &te_emlrtRSI;
          moveConstraint_(&b_st, workingset, workingset->nActiveConstr,
                          mConstr);
          workingset->nActiveConstr--;
          workingset->nWConstr[3]--;
          exitg1 = true;
        } else {
          mConstr++;
        }
      } else {
        mConstr++;
      }
    }
  }
  mConstr = workingset->nActiveConstr;
  mEqFixed = workingset->sizes[0] + 192;
  while ((mConstr > mEqFixed) && (mConstr > nVar)) {
    int32_T TYPE_tmp;
    st.site = &qf_emlrtRSI;
    b_workingset = workingset->Wid.size[0];
    if ((mConstr < 1) || (mConstr > b_workingset)) {
      emlrtDynamicBoundsCheckR2012b(mConstr, 1, b_workingset, &ec_emlrtBCI,
                                    &st);
    }
    TYPE_tmp = workingset->Wid.data[mConstr - 1];
    b_workingset = workingset->Wlocalidx.size[0];
    if (mConstr > b_workingset) {
      emlrtDynamicBoundsCheckR2012b(mConstr, 1, b_workingset, &ec_emlrtBCI,
                                    &st);
    }
    if ((TYPE_tmp < 1) || (TYPE_tmp > 6)) {
      emlrtDynamicBoundsCheckR2012b(workingset->Wid.data[mConstr - 1], 1, 6,
                                    &fc_emlrtBCI, &st);
    }
    b_workingset = workingset->isActiveConstr.size[0];
    i = (workingset->isActiveIdx[TYPE_tmp - 1] +
         workingset->Wlocalidx.data[mConstr - 1]) -
        1;
    if ((i < 1) || (i > b_workingset)) {
      emlrtDynamicBoundsCheckR2012b(i, 1, b_workingset, &ec_emlrtBCI, &st);
    }
    workingset->isActiveConstr.data[i - 1] = false;
    b_st.site = &te_emlrtRSI;
    moveConstraint_(&b_st, workingset, workingset->nActiveConstr, mConstr);
    workingset->nActiveConstr--;
    if (TYPE_tmp > 5) {
      emlrtDynamicBoundsCheckR2012b(6, 1, 5, &kc_emlrtBCI, &st);
    }
    workingset->nWConstr[TYPE_tmp - 1]--;
    mConstr--;
  }
  b_workingset = solution->xstar.size[0];
  if (nVarP1_tmp > b_workingset) {
    emlrtDynamicBoundsCheckR2012b(nVarP1_tmp, 1, b_workingset, &wc_emlrtBCI,
                                  (emlrtCTX)sp);
  }
  solution->maxConstr = solution->xstar.data[nVarP1_tmp - 1];
  st.site = &qf_emlrtRSI;
  setProblemType(&st, workingset, PROBTYPE_ORIG);
  objective->objtype = objective->prev_objtype;
  objective->nvar = objective->prev_nvar;
  objective->hasLinear = objective->prev_hasLinear;
  options->ObjectiveLimit = rtMinusInf;
  options->StepTolerance = 1.0E-6;
}

/* End of code generation (phaseone.c) */
