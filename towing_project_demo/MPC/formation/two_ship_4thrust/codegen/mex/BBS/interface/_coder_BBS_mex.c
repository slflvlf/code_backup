/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_BBS_mex.c
 *
 * Code generation for function '_coder_BBS_mex'
 *
 */

/* Include files */
#include "_coder_BBS_mex.h"
#include "BBS.h"
#include "BBS_data.h"
#include "BBS_initialize.h"
#include "BBS_terminate.h"
#include "_coder_BBS_api.h"

/* Function Declarations */
MEXFUNCTION_LINKAGE void BBS_mexFunction(BBSStackData *SD, int32_T nlhs, mxArray
  *plhs[2], int32_T nrhs, const mxArray *prhs[2]);

/* Function Definitions */
void BBS_mexFunction(BBSStackData *SD, int32_T nlhs, mxArray *plhs[2], int32_T
                     nrhs, const mxArray *prhs[2])
{
  const mxArray *outputs[2];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 2, 4, 3,
                        "BBS");
  }

  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 3,
                        "BBS");
  }

  /* Call the function. */
  BBS_api(SD, prhs, nlhs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  BBSStackData *BBSStackDataGlobal = NULL;
  BBSStackDataGlobal = (BBSStackData *)emlrtMxCalloc(1, (size_t)1U * sizeof
    (BBSStackData));
  mexAtExit(&BBS_atexit);

  /* Module initialization. */
  BBS_initialize();

  /* Dispatch the entry-point. */
  BBS_mexFunction(BBSStackDataGlobal, nlhs, plhs, nrhs, prhs);

  /* Module termination. */
  BBS_terminate();
  emlrtMxFree(BBSStackDataGlobal);
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_BBS_mex.c) */
