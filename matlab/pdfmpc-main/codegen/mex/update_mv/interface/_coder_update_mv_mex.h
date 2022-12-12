/*
 * _coder_update_mv_mex.h
 *
 * Code generation for function '_coder_update_mv_mex'
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
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS(void);

void update_mv_mexFunction(int32_T nlhs, mxArray *plhs[3], int32_T nrhs,
                           const mxArray *prhs[3]);

/* End of code generation (_coder_update_mv_mex.h) */
