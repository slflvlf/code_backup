/*
 * update_mv_initialize.c
 *
 * Code generation for function 'update_mv_initialize'
 *
 */

/* Include files */
#include "update_mv_initialize.h"
#include "_coder_update_mv_mex.h"
#include "rt_nonfinite.h"
#include "update_mv_data.h"

/* Function Definitions */
void update_mv_initialize(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mex_InitInfAndNan();
  mexFunctionCreateRootTLS();
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2012b();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (update_mv_initialize.c) */
