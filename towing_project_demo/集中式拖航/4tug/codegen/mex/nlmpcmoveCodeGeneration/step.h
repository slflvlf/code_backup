/*
 * step.h
 *
 * Code generation for function 'step'
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
boolean_T step(c_nlmpcmoveCodeGenerationStackD *SD, const emlrtStack *sp,
               int32_T *STEP_TYPE, real_T Hessian[7225], const real_T lb[85],
               e_struct_T *TrialState, struct_T *MeritFunction,
               g_struct_T *memspace, h_struct_T *WorkingSet,
               i_struct_T *QRManager, j_struct_T *CholManager,
               f_struct_T *QPObjective, d_struct_T *qpoptions);

/* End of code generation (step.h) */
