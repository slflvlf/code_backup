/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * phaseone.h
 *
 * Code generation for function 'phaseone'
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
void phaseone(c_nlmpcmoveCodeGenerationStackD *SD, const emlrtStack *sp,
              const real_T H[6241], const real_T f_data[], int32_T f_size,
              e_struct_T *solution, g_struct_T *memspace,
              h_struct_T *workingset, i_struct_T *qrmanager,
              j_struct_T *cholmanager, f_struct_T *objective,
              d_struct_T *options, const d_struct_T *runTimeOptions);

/* End of code generation (phaseone.h) */
