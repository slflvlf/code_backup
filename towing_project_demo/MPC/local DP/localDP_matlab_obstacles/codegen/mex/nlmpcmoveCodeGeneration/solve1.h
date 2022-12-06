/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * solve1.h
 *
 * Code generation for function 'solve1'
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
void b_solve(const emlrtStack *sp, const j_struct_T *obj, real_T rhs_data[],
             const int32_T *rhs_size);

void c_solve(const emlrtStack *sp, const j_struct_T *obj, real_T rhs_data[],
             const int32_T rhs_size[2]);

/* End of code generation (solve1.h) */
