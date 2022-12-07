/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * addAineqConstr.c
 *
 * Code generation for function 'addAineqConstr'
 *
 */

/* Include files */
#include "addAineqConstr.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ch_emlrtRSI = {
    1,                /* lineNo */
    "addAineqConstr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\addAineqConstr.p" /* pathName */
};

static emlrtBCInfo od_emlrtBCI = {
    -1,               /* iFirst */
    -1,               /* iLast */
    1,                /* lineNo */
    1,                /* colNo */
    "",               /* aName */
    "addAineqConstr", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\addAineqConstr.p", /* pName */
    0                               /* checkKind */
};

/* Function Definitions */
void addAineqConstr(const emlrtStack *sp, h_struct_T *obj, int32_T idx_local)
{
  emlrtStack st;
  int32_T i;
  int32_T i1;
  int32_T iAineq0;
  int32_T iAw0;
  int32_T idx;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &ch_emlrtRSI;
  obj->nWConstr[2]++;
  i = obj->isActiveConstr.size[0];
  i1 = (obj->isActiveIdx[2] + idx_local) - 1;
  if ((i1 < 1) || (i1 > i)) {
    emlrtDynamicBoundsCheckR2012b(i1, 1, i, &nd_emlrtBCI, &st);
  }
  obj->isActiveConstr.data[i1 - 1] = true;
  obj->nActiveConstr++;
  i = obj->Wid.size[0];
  if ((obj->nActiveConstr < 1) || (obj->nActiveConstr > i)) {
    emlrtDynamicBoundsCheckR2012b(obj->nActiveConstr, 1, i, &nd_emlrtBCI, &st);
  }
  obj->Wid.data[obj->nActiveConstr - 1] = 3;
  i = obj->Wlocalidx.size[0];
  if ((obj->nActiveConstr < 1) || (obj->nActiveConstr > i)) {
    emlrtDynamicBoundsCheckR2012b(obj->nActiveConstr, 1, i, &nd_emlrtBCI, &st);
  }
  obj->Wlocalidx.data[obj->nActiveConstr - 1] = idx_local;
  iAineq0 = obj->ldA * (idx_local - 1) + 1;
  iAw0 = obj->ldA * (obj->nActiveConstr - 1) + 1;
  i = obj->nVar - 1;
  for (idx = 0; idx <= i; idx++) {
    int32_T i2;
    int32_T i3;
    i1 = obj->Aineq.size[0];
    i2 = iAineq0 + idx;
    if ((i2 < 1) || (i2 > i1)) {
      emlrtDynamicBoundsCheckR2012b(i2, 1, i1, &od_emlrtBCI, (emlrtCTX)sp);
    }
    i1 = obj->ATwset.size[0];
    i3 = iAw0 + idx;
    if ((i3 < 1) || (i3 > i1)) {
      emlrtDynamicBoundsCheckR2012b(i3, 1, i1, &od_emlrtBCI, (emlrtCTX)sp);
    }
    obj->ATwset.data[i3 - 1] = obj->Aineq.data[i2 - 1];
  }
  i = obj->bineq.size[0];
  if ((idx_local < 1) || (idx_local > i)) {
    emlrtDynamicBoundsCheckR2012b(idx_local, 1, i, &od_emlrtBCI, (emlrtCTX)sp);
  }
  i = obj->bwset.size[0];
  if ((obj->nActiveConstr < 1) || (obj->nActiveConstr > i)) {
    emlrtDynamicBoundsCheckR2012b(obj->nActiveConstr, 1, i, &od_emlrtBCI,
                                  (emlrtCTX)sp);
  }
  obj->bwset.data[obj->nActiveConstr - 1] = obj->bineq.data[idx_local - 1];
}

/* End of code generation (addAineqConstr.c) */
