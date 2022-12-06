/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compute_deltax.h
 *
 * Code generation for function 'compute_deltax'
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
void compute_deltax(const emlrtStack *sp, const real_T H[70225],
                    e_struct_T *solution, g_struct_T *memspace,
                    const i_struct_T *qrmanager, j_struct_T *cholmanager,
                    const f_struct_T *objective, boolean_T alwaysPositiveDef);

/* End of code generation (compute_deltax.h) */
