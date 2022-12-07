/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeDeltaLag.h
 *
 * Code generation for function 'computeDeltaLag'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void computeDeltaLag(const emlrtStack *sp, int32_T nVar, int32_T ldJ,
                     int32_T mNonlinIneq, real_T workspace_data[],
                     const int32_T *workspace_size, const real_T grad_data[],
                     int32_T grad_size, const real_T JacIneqTrans_data[],
                     int32_T ineqJ0, const real_T JacEqTrans_data[],
                     const real_T grad_old_data[],
                     const real_T JacIneqTrans_old_data[],
                     const real_T JacEqTrans_old_data[],
                     const real_T lambda_data[], int32_T ineqL0, int32_T eqL0);

/* End of code generation (computeDeltaLag.h) */
