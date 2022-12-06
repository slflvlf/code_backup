/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_update_mv_mex.c
 *
 * Code generation for function '_coder_update_mv_mex'
 *
 */

/* Include files */
#include "_coder_update_mv_mex.h"
#include "_coder_update_mv_api.h"
#include "update_mv.h"
#include "update_mv_data.h"
#include "update_mv_initialize.h"
#include "update_mv_terminate.h"

/* Function Declarations */
MEXFUNCTION_LINKAGE void update_mv_mexFunction(int32_T nlhs, mxArray *plhs[3],
  int32_T nrhs, const mxArray *prhs[3]);

/* Function Definitions */
void update_mv_mexFunction(int32_T nlhs, mxArray *plhs[3], int32_T nrhs, const
  mxArray *prhs[3])
{
  const mxArray *outputs[3];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 3, 4, 9,
                        "update_mv");
  }

  if (nlhs > 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 9,
                        "update_mv");
  }

  /* Call the function. */
  update_mv_api(prhs, nlhs, outputs);

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
  mexAtExit(&update_mv_atexit);

  /* Module initialization. */
  update_mv_initialize();

  /* Dispatch the entry-point. */
  update_mv_mexFunction(nlhs, plhs, nrhs, prhs);

  /* Module termination. */
  update_mv_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_update_mv_mex.c) */
