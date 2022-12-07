/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * update_mv.h
 *
 * Code generation for function 'update_mv'
 *
 */

#pragma once

/* Include files */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mex.h"
#include "emlrt.h"
#include "rtwtypes.h"
#include "update_mv_types.h"

/* Function Declarations */
void update_mv(const emlrtStack *sp, struct0_T mv[32], const real_T subset_data[],
               const int32_T subset_size[1], struct1_T *param, emxArray_real_T
               *u_sol, real_T p_sol[32], emxArray_real_T *lesp);

/* End of code generation (update_mv.h) */
