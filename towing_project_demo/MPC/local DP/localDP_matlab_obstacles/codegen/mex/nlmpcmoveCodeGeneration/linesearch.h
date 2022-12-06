/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * linesearch.h
 *
 * Code generation for function 'linesearch'
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
void linesearch(const emlrtStack *sp, boolean_T *evalWellDefined,
                const real_T bineq_data[], int32_T WorkingSet_nVar,
                int32_T WorkingSet_ldA, const real_T WorkingSet_Aineq_data[],
                e_struct_T *TrialState, real_T MeritFunction_penaltyParam,
                real_T MeritFunction_phi, real_T MeritFunction_phiPrimePlus,
                real_T MeritFunction_phiFullStep,
                const l_struct_T *c_FcnEvaluator_objfun_workspace,
                const l_struct_T *c_FcnEvaluator_nonlcon_workspac,
                int32_T FcnEvaluator_mCineq, boolean_T socTaken, real_T *alpha,
                int32_T *exitflag);

/* End of code generation (linesearch.h) */
