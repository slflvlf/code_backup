/*
 * fmincon.h
 *
 * Code generation for function 'fmincon'
 *
 */

#pragma once

/* Include files */
#include "nlmpcmoveCodeGeneration_internal_types.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void fmincon(c_nlmpcmoveCodeGenerationStackD *SD, const emlrtStack *sp,
             const l_struct_T *fun_workspace_runtimedata,
             const k_struct_T *fun_workspace_userdata, const real_T x0[85],
             const real_T Aineq_data[], const int32_T Aineq_size[2],
             const real_T bineq_data[], int32_T bineq_size, const real_T lb[85],
             const real_T ub[85],
             const l_struct_T *nonlcon_workspace_runtimedata,
             const k_struct_T *nonlcon_workspace_userdata, real_T x[85],
             real_T *fval, real_T *exitflag, c_struct_T *output);

/* End of code generation (fmincon.h) */
