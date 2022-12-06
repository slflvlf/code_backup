/*
 * checkNonlinearInputs.h
 *
 * Code generation for function 'checkNonlinearInputs'
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
int32_T checkNonlinearInputs(c_nlmpcmoveCodeGenerationStackD *SD,
                             const emlrtStack *sp, const real_T x0[85],
                             const real_T nonlcon_workspace_runtimedata_x[6],
                             const real_T c_nonlcon_workspace_runtimedata[60],
                             const real_T d_nonlcon_workspace_runtimedata[60]);

/* End of code generation (checkNonlinearInputs.h) */
