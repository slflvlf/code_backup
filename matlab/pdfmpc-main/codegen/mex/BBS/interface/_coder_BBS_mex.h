/*
 * _coder_BBS_mex.h
 *
 * Code generation for function '_coder_BBS_mex'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void BBS_mexFunction(int32_T nlhs, mxArray *plhs[2], int32_T nrhs,
                     const mxArray *prhs[2]);

MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS(void);

/* End of code generation (_coder_BBS_mex.h) */
