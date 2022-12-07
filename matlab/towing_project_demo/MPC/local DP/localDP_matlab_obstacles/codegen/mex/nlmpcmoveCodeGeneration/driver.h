/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * driver.h
 *
 * Code generation for function 'driver'
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
void driver(c_nlmpcmoveCodeGenerationStackD *SD, const emlrtStack *sp,
            const real_T bineq_data[], const real_T lb[79],
            e_struct_T *TrialState, struct_T *MeritFunction,
            const n_struct_T *FcnEvaluator, g_struct_T *memspace,
            h_struct_T *WorkingSet, i_struct_T *QRManager,
            j_struct_T *CholManager, f_struct_T *QPObjective,
            int32_T fscales_lineq_constraint_size,
            int32_T fscales_cineq_constraint_size, real_T Hessian[6241]);

/* End of code generation (driver.h) */
