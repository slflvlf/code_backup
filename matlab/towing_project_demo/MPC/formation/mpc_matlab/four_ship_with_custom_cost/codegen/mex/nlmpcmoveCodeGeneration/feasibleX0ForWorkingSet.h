/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * feasibleX0ForWorkingSet.h
 *
 * Code generation for function 'feasibleX0ForWorkingSet'
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
boolean_T feasibleX0ForWorkingSet(c_nlmpcmoveCodeGenerationStackD *SD,
                                  const emlrtStack *sp, real_T workspace_data[],
                                  const int32_T workspace_size[2],
                                  real_T xCurrent_data[],
                                  h_struct_T *workingset,
                                  i_struct_T *qrmanager);

/* End of code generation (feasibleX0ForWorkingSet.h) */
