/*
 * BBS_initialize.c
 *
 * Code generation for function 'BBS_initialize'
 *
 */

/* Include files */
#include "BBS_initialize.h"
#include "BBS_data.h"
#include "_coder_BBS_mex.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void BBS_initialize(void)
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

/* End of code generation (BBS_initialize.c) */
