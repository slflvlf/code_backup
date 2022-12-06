/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * RemoveDependentIneq_.h
 *
 * Code generation for function 'RemoveDependentIneq_'
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
void RemoveDependentIneq_(const emlrtStack *sp, h_struct_T *workingset,
                          i_struct_T *qrmanager, g_struct_T *memspace);

void b_RemoveDependentIneq_(const emlrtStack *sp, h_struct_T *workingset,
                            i_struct_T *qrmanager, g_struct_T *memspace);

/* End of code generation (RemoveDependentIneq_.h) */
