/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * update_sc.h
 *
 * Code generation for function 'update_sc'
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
void update_sc(const emlrtStack *sp, struct0_T *sc_past, const struct1_T *param,
               real_T Niter);

/* End of code generation (update_sc.h) */
