/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * checkNonlinearInputs.h
 *
 * Code generation for function 'checkNonlinearInputs'
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
int32_T checkNonlinearInputs(c_nlmpcmoveCodeGenerationStackD *SD,
                             const emlrtStack *sp, const real_T x0[265],
                             const real_T nonlcon_workspace_runtimedata_x[24],
                             const real_T c_nonlcon_workspace_runtimedata[192],
                             const real_T d_nonlcon_workspace_runtimedata[192]);

/* End of code generation (checkNonlinearInputs.h) */
