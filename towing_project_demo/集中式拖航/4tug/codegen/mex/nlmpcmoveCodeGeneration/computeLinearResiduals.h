/*
 * computeLinearResiduals.h
 *
 * Code generation for function 'computeLinearResiduals'
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
void b_computeLinearResiduals(const real_T x[85], int32_T nVar,
                              real_T workspaceIneq_data[],
                              const int32_T *workspaceIneq_size,
                              int32_T mLinIneq, const real_T AineqT_data[],
                              const real_T bineq_data[], int32_T ldAi);

void computeLinearResiduals(const real_T x[85], real_T workspaceIneq_data[],
                            const int32_T *workspaceIneq_size, int32_T mLinIneq,
                            const real_T AineqT_data[],
                            const real_T bineq_data[], int32_T ldAi);

/* End of code generation (computeLinearResiduals.h) */
