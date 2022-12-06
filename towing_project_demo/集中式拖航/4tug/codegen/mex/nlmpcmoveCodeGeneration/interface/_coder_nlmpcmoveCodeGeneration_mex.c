/*
 * _coder_nlmpcmoveCodeGeneration_mex.c
 *
 * Code generation for function '_coder_nlmpcmoveCodeGeneration_mex'
 *
 */

/* Include files */
#include "_coder_nlmpcmoveCodeGeneration_mex.h"
#include "_coder_nlmpcmoveCodeGeneration_api.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_initialize.h"
#include "nlmpcmoveCodeGeneration_terminate.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  c_nlmpcmoveCodeGenerationStackD *d_nlmpcmoveCodeGenerationStackD = NULL;
  d_nlmpcmoveCodeGenerationStackD =
      (c_nlmpcmoveCodeGenerationStackD *)emlrtMxCalloc(
          (size_t)1, (size_t)1U * sizeof(c_nlmpcmoveCodeGenerationStackD));
  mexAtExit(&nlmpcmoveCodeGeneration_atexit);
  /* Module initialization. */
  nlmpcmoveCodeGeneration_initialize();
  /* Dispatch the entry-point. */
  nlmpcmoveCodeGeneration_mexFunction(d_nlmpcmoveCodeGenerationStackD, nlhs,
                                      plhs, nrhs, prhs);
  /* Module termination. */
  nlmpcmoveCodeGeneration_terminate();
  emlrtMxFree(d_nlmpcmoveCodeGenerationStackD);
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, (const char_T *)"GBK", true);
  return emlrtRootTLSGlobal;
}

void nlmpcmoveCodeGeneration_mexFunction(c_nlmpcmoveCodeGenerationStackD *SD,
                                         int32_T nlhs, mxArray *plhs[3],
                                         int32_T nrhs, const mxArray *prhs[4])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs[3];
  int32_T b_nlhs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs < 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooFewInputsConstants", 9, 4, 23,
                        "nlmpcmoveCodeGeneration", 4, 23,
                        "nlmpcmoveCodeGeneration", 4, 23,
                        "nlmpcmoveCodeGeneration");
  }
  if (nrhs != 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 4, 4,
                        23, "nlmpcmoveCodeGeneration");
  }
  if (nlhs > 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 23,
                        "nlmpcmoveCodeGeneration");
  }
  /* Call the function. */
  nlmpcmoveCodeGeneration_api(SD, prhs, nlhs, outputs);
  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }
  emlrtReturnArrays(b_nlhs, &plhs[0], &outputs[0]);
}

/* End of code generation (_coder_nlmpcmoveCodeGeneration_mex.c) */
