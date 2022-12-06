/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * checkStoppingAndUpdateFval.c
 *
 * Code generation for function 'checkStoppingAndUpdateFval'
 *
 */

/* Include files */
#include "checkStoppingAndUpdateFval.h"
#include "computeFval_ReuseHx.h"
#include "eml_int_forloop_overflow_check.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_internal_types.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "xcopy.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo gh_emlrtRSI = {
    1,                            /* lineNo */
    "checkStoppingAndUpdateFval", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "stopping\\checkStoppingAndUpdateFval.p" /* pathName */
};

static emlrtBCInfo qd_emlrtBCI = {
    -1,                           /* iFirst */
    -1,                           /* iLast */
    1,                            /* lineNo */
    1,                            /* colNo */
    "",                           /* aName */
    "checkStoppingAndUpdateFval", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "stopping\\checkStoppingAndUpdateFval.p", /* pName */
    0                                         /* checkKind */
};

/* Function Definitions */
void checkStoppingAndUpdateFval(
    c_nlmpcmoveCodeGenerationStackD *SD, const emlrtStack *sp,
    int32_T *activeSetChangeID, const real_T f_data[], int32_T f_size,
    e_struct_T *solution, g_struct_T *memspace, const f_struct_T *objective,
    h_struct_T *workingset, i_struct_T *qrmanager,
    real_T options_ObjectiveLimit, int32_T runTimeOptions_MaxIterations,
    boolean_T updateFval)
{
  emlrtStack b_st;
  emlrtStack st;
  real_T tmp_data[1059];
  int32_T loop_ub;
  int32_T nVar;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  solution->iterations++;
  nVar = objective->nvar;
  if ((solution->iterations >= runTimeOptions_MaxIterations) &&
      ((solution->state != 1) || (objective->objtype == 5))) {
    solution->state = 0;
  }
  if (solution->iterations - solution->iterations / 50 * 50 == 0) {
    real_T tempMaxConstr;
    int32_T tmp_size;
    st.site = &gh_emlrtRSI;
    solution->maxConstr = c_maxConstraintViolation(
        &st, workingset, solution->xstar.data, solution->xstar.size[0]);
    tempMaxConstr = solution->maxConstr;
    if (objective->objtype == 5) {
      tmp_size = solution->xstar.size[0];
      if ((objective->nvar < 1) || (objective->nvar > tmp_size)) {
        emlrtDynamicBoundsCheckR2012b(objective->nvar, 1, tmp_size,
                                      &qd_emlrtBCI, (emlrtCTX)sp);
      }
      tempMaxConstr =
          solution->maxConstr - solution->xstar.data[objective->nvar - 1];
    }
    if (tempMaxConstr > 1.0E-6) {
      boolean_T nonDegenerateWset;
      tmp_size = solution->searchDir.size[0];
      loop_ub = solution->searchDir.size[0];
      if (loop_ub - 1 >= 0) {
        memcpy(&tmp_data[0], &solution->searchDir.data[0],
               loop_ub * sizeof(real_T));
      }
      st.site = &gh_emlrtRSI;
      b_xcopy(objective->nvar, solution->xstar.data, tmp_data);
      solution->searchDir.size[0] = tmp_size;
      if (tmp_size - 1 >= 0) {
        memcpy(&solution->searchDir.data[0], &tmp_data[0],
               tmp_size * sizeof(real_T));
      }
      st.site = &gh_emlrtRSI;
      nonDegenerateWset = feasibleX0ForWorkingSet(
          SD, &st, memspace->workspace_double.data,
          memspace->workspace_double.size, solution->searchDir.data, workingset,
          qrmanager);
      if ((!nonDegenerateWset) && (solution->state != 0)) {
        solution->state = -2;
      }
      *activeSetChangeID = 0;
      st.site = &gh_emlrtRSI;
      tempMaxConstr =
          c_maxConstraintViolation(&st, workingset, solution->searchDir.data,
                                   solution->searchDir.size[0]);
      if (tempMaxConstr < solution->maxConstr) {
        st.site = &gh_emlrtRSI;
        if (objective->nvar > 2147483646) {
          b_st.site = &db_emlrtRSI;
          check_forloop_overflow_error(&b_st);
        }
        for (loop_ub = 0; loop_ub < nVar; loop_ub++) {
          tmp_size = solution->searchDir.size[0];
          if ((loop_ub + 1 < 1) || (loop_ub + 1 > tmp_size)) {
            emlrtDynamicBoundsCheckR2012b(loop_ub + 1, 1, tmp_size,
                                          &qd_emlrtBCI, (emlrtCTX)sp);
          }
          tmp_size = solution->xstar.size[0];
          if (loop_ub + 1 > tmp_size) {
            emlrtDynamicBoundsCheckR2012b(loop_ub + 1, 1, tmp_size,
                                          &qd_emlrtBCI, (emlrtCTX)sp);
          }
          solution->xstar.data[loop_ub] = solution->searchDir.data[loop_ub];
        }
        solution->maxConstr = tempMaxConstr;
      }
    }
  }
  if (updateFval && (options_ObjectiveLimit > rtMinusInf)) {
    st.site = &gh_emlrtRSI;
    solution->fstar = computeFval_ReuseHx(
        &st, objective, memspace->workspace_double.data, f_data, f_size,
        solution->xstar.data, solution->xstar.size[0]);
    if ((solution->fstar < options_ObjectiveLimit) &&
        ((solution->state != 0) || (objective->objtype != 5))) {
      solution->state = 2;
    }
  }
}

/* End of code generation (checkStoppingAndUpdateFval.c) */
