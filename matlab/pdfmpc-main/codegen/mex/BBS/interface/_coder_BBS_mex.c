/*
 * _coder_BBS_mex.c
 *
 * Code generation for function '_coder_BBS_mex'
 *
 */

/* Include files */
#include "_coder_BBS_mex.h"
#include "BBS_data.h"
#include "BBS_initialize.h"
#include "BBS_terminate.h"
#include "_coder_BBS_api.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void BBS_mexFunction(int32_T nlhs, mxArray *plhs[2], int32_T nrhs,
                     const mxArray *prhs[2])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs[2];
  int32_T b_nlhs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 2, 4,
                        3, "BBS");
  }
  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 3,
                        "BBS");
  }
  /* Call the function. */
  BBS_api(prhs, nlhs, outputs);
  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }
  emlrtReturnArrays(b_nlhs, &plhs[0], &outputs[0]);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&BBS_atexit);
  /* Module initialization. */
  BBS_initialize();
  /* Dispatch the entry-point. */
  BBS_mexFunction(nlhs, plhs, nrhs, prhs);
  /* Module termination. */
  BBS_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, (const char_T *)"GBK", true);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_BBS_mex.c) */
