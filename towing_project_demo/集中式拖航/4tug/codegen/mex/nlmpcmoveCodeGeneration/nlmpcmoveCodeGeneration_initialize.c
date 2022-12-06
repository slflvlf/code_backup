/*
 * nlmpcmoveCodeGeneration_initialize.c
 *
 * Code generation for function 'nlmpcmoveCodeGeneration_initialize'
 *
 */

/* Include files */
#include "nlmpcmoveCodeGeneration_initialize.h"
#include "_coder_nlmpcmoveCodeGeneration_mex.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void nlmpcmoveCodeGeneration_initialize(void)
{
  static const volatile char_T *emlrtBreakCheckR2012bFlagVar = NULL;
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
  emlrtLicenseCheckR2022a(
      &st, (const char_T *)"EMLRT:runTime:MexFunctionNeedsLicense",
      (const char_T *)"optimization_toolbox", 2);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (nlmpcmoveCodeGeneration_initialize.c) */
