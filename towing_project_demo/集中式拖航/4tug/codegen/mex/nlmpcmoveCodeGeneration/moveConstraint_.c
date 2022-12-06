/*
 * moveConstraint_.c
 *
 * Code generation for function 'moveConstraint_'
 *
 */

/* Include files */
#include "moveConstraint_.h"
#include "eml_int_forloop_overflow_check.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo te_emlrtRSI = {
    1,                 /* lineNo */
    "moveConstraint_", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\moveConstraint_.p" /* pathName */
};

static emlrtBCInfo mc_emlrtBCI = {
    -1,                /* iFirst */
    -1,                /* iLast */
    1,                 /* lineNo */
    1,                 /* colNo */
    "",                /* aName */
    "moveConstraint_", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "WorkingSet\\moveConstraint_.p", /* pName */
    0                                /* checkKind */
};

/* Function Definitions */
void moveConstraint_(const emlrtStack *sp, h_struct_T *obj,
                     int32_T idx_global_start, int32_T idx_global_dest)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T b;
  int32_T i;
  int32_T idx;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  i = obj->Wid.size[0];
  if ((idx_global_start < 1) || (idx_global_start > i)) {
    emlrtDynamicBoundsCheckR2012b(idx_global_start, 1, i, &mc_emlrtBCI,
                                  (emlrtCTX)sp);
  }
  i = obj->Wid.size[0];
  if ((idx_global_dest < 1) || (idx_global_dest > i)) {
    emlrtDynamicBoundsCheckR2012b(idx_global_dest, 1, i, &mc_emlrtBCI,
                                  (emlrtCTX)sp);
  }
  obj->Wid.data[idx_global_dest - 1] = obj->Wid.data[idx_global_start - 1];
  i = obj->Wlocalidx.size[0];
  if (idx_global_start > i) {
    emlrtDynamicBoundsCheckR2012b(idx_global_start, 1, i, &mc_emlrtBCI,
                                  (emlrtCTX)sp);
  }
  i = obj->Wlocalidx.size[0];
  if (idx_global_dest > i) {
    emlrtDynamicBoundsCheckR2012b(idx_global_dest, 1, i, &mc_emlrtBCI,
                                  (emlrtCTX)sp);
  }
  obj->Wlocalidx.data[idx_global_dest - 1] =
      obj->Wlocalidx.data[idx_global_start - 1];
  b = obj->nVar;
  st.site = &te_emlrtRSI;
  if (obj->nVar > 2147483646) {
    b_st.site = &db_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  for (idx = 0; idx < b; idx++) {
    int32_T i1;
    int32_T i2;
    i = obj->ATwset.size[0];
    i1 = (idx + obj->ldA * (idx_global_start - 1)) + 1;
    if ((i1 < 1) || (i1 > i)) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i, &mc_emlrtBCI, (emlrtCTX)sp);
    }
    i = obj->ATwset.size[0];
    i2 = (idx + obj->ldA * (idx_global_dest - 1)) + 1;
    if ((i2 < 1) || (i2 > i)) {
      emlrtDynamicBoundsCheckR2012b(i2, 1, i, &mc_emlrtBCI, (emlrtCTX)sp);
    }
    obj->ATwset.data[i2 - 1] = obj->ATwset.data[i1 - 1];
  }
  i = obj->bwset.size[0];
  if (idx_global_start > i) {
    emlrtDynamicBoundsCheckR2012b(idx_global_start, 1, i, &mc_emlrtBCI,
                                  (emlrtCTX)sp);
  }
  i = obj->bwset.size[0];
  if (idx_global_dest > i) {
    emlrtDynamicBoundsCheckR2012b(idx_global_dest, 1, i, &mc_emlrtBCI,
                                  (emlrtCTX)sp);
  }
  obj->bwset.data[idx_global_dest - 1] = obj->bwset.data[idx_global_start - 1];
}

/* End of code generation (moveConstraint_.c) */
