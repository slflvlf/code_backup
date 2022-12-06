/*
 * PresolveWorkingSet.c
 *
 * Code generation for function 'PresolveWorkingSet'
 *
 */

/* Include files */
#include "PresolveWorkingSet.h"
#include "RemoveDependentEq_.h"
#include "RemoveDependentIneq_.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "removeAllIneqConstr.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo me_emlrtRSI = {
    1,                    /* lineNo */
    "PresolveWorkingSet", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "initialize\\PresolveWorkingSet.p" /* pathName */
};

/* Function Definitions */
void PresolveWorkingSet(c_nlmpcmoveCodeGenerationStackD *SD,
                        const emlrtStack *sp, e_struct_T *solution,
                        g_struct_T *memspace, h_struct_T *workingset,
                        i_struct_T *qrmanager)
{
  emlrtStack st;
  int32_T i;
  st.prev = sp;
  st.tls = sp->tls;
  solution->state = 82;
  st.site = &me_emlrtRSI;
  i = RemoveDependentEq_(&st, memspace, workingset, qrmanager);
  if ((i != -1) && (workingset->nActiveConstr <= qrmanager->ldq)) {
    boolean_T guard1 = false;
    boolean_T okWorkingSet;
    st.site = &me_emlrtRSI;
    RemoveDependentIneq_(&st, workingset, qrmanager, memspace);
    st.site = &me_emlrtRSI;
    okWorkingSet =
        feasibleX0ForWorkingSet(SD, &st, memspace->workspace_double.data,
                                memspace->workspace_double.size,
                                solution->xstar.data, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      st.site = &me_emlrtRSI;
      b_RemoveDependentIneq_(&st, workingset, qrmanager, memspace);
      st.site = &me_emlrtRSI;
      okWorkingSet =
          feasibleX0ForWorkingSet(SD, &st, memspace->workspace_double.data,
                                  memspace->workspace_double.size,
                                  solution->xstar.data, workingset, qrmanager);
      if (!okWorkingSet) {
        solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1 && (workingset->nWConstr[0] + workingset->nWConstr[1] ==
                   workingset->nVar)) {
      real_T constrViolation;
      st.site = &me_emlrtRSI;
      constrViolation = c_maxConstraintViolation(
          &st, workingset, solution->xstar.data, solution->xstar.size[0]);
      if (constrViolation > 1.0E-6) {
        solution->state = -2;
      }
    }
  } else {
    solution->state = -3;
    st.site = &me_emlrtRSI;
    removeAllIneqConstr(&st, workingset);
  }
}

/* End of code generation (PresolveWorkingSet.c) */
