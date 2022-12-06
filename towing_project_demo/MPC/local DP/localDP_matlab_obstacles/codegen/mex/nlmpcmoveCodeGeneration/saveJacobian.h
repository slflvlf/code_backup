/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * saveJacobian.h
 *
 * Code generation for function 'saveJacobian'
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
void saveJacobian(c_nlmpcmoveCodeGenerationStackD *SD, const emlrtStack *sp,
                  e_struct_T *obj, int32_T nVar, int32_T mIneq,
                  const real_T JacCineqTrans_data[], int32_T ineqCol0,
                  const real_T JacCeqTrans_data[], int32_T ldJ);

/* End of code generation (saveJacobian.h) */
