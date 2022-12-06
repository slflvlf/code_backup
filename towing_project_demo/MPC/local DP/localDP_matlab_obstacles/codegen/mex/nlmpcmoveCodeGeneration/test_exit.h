/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * test_exit.h
 *
 * Code generation for function 'test_exit'
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
void b_test_exit(const emlrtStack *sp, b_struct_T *Flags, g_struct_T *memspace,
                 struct_T *MeritFunction, int32_T fscales_lineq_constraint_size,
                 int32_T fscales_cineq_constraint_size, h_struct_T *WorkingSet,
                 e_struct_T *TrialState, i_struct_T *QRManager,
                 const real_T lb[79]);

void test_exit(const emlrtStack *sp, struct_T *MeritFunction,
               int32_T fscales_lineq_constraint_size,
               int32_T fscales_cineq_constraint_size,
               const h_struct_T *WorkingSet, e_struct_T *TrialState,
               const real_T lb[79], boolean_T *Flags_gradOK,
               boolean_T *Flags_fevalOK, boolean_T *Flags_done,
               boolean_T *Flags_stepAccepted, boolean_T *Flags_failedLineSearch,
               int32_T *Flags_stepType);

/* End of code generation (test_exit.h) */
