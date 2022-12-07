/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * checkLinearInputs.h
 *
 * Code generation for function 'checkLinearInputs'
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
void checkLinearInputs(c_nlmpcmoveCodeGenerationStackD *SD,
                       const emlrtStack *sp, const real_T Aineq_data[],
                       const int32_T Aineq_size[2], const real_T bineq_data[],
                       int32_T bineq_size, const real_T lb[265],
                       const real_T ub[265]);

/* End of code generation (checkLinearInputs.h) */
