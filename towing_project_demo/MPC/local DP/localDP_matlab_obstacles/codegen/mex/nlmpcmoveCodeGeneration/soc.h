/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * soc.h
 *
 * Code generation for function 'soc'
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
boolean_T soc(c_nlmpcmoveCodeGenerationStackD *SD, const emlrtStack *sp,
              const real_T Hessian[6241], const real_T grad_data[],
              int32_T grad_size, e_struct_T *TrialState, g_struct_T *memspace,
              h_struct_T *WorkingSet, i_struct_T *QRManager,
              j_struct_T *CholManager, f_struct_T *QPObjective,
              const d_struct_T *qpoptions);

/* End of code generation (soc.h) */
