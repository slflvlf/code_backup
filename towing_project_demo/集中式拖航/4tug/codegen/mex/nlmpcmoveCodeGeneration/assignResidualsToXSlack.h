/*
 * assignResidualsToXSlack.h
 *
 * Code generation for function 'assignResidualsToXSlack'
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
void assignResidualsToXSlack(const emlrtStack *sp, int32_T nVarOrig,
                             h_struct_T *WorkingSet, e_struct_T *TrialState,
                             g_struct_T *memspace);

/* End of code generation (assignResidualsToXSlack.h) */
