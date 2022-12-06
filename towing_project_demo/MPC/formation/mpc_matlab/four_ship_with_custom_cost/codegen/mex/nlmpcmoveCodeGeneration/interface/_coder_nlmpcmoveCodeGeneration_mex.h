/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_nlmpcmoveCodeGeneration_mex.h
 *
 * Code generation for function '_coder_nlmpcmoveCodeGeneration_mex'
 *
 */

#pragma once

/* Include files */
#include "nlmpcmoveCodeGeneration_types.h"
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

void nlmpcmoveCodeGeneration_mexFunction(c_nlmpcmoveCodeGenerationStackD *SD,
                                         int32_T nlhs, mxArray *plhs[3],
                                         int32_T nrhs, const mxArray *prhs[4]);

/* End of code generation (_coder_nlmpcmoveCodeGeneration_mex.h) */
