/*
 * PresolveWorkingSet.h
 *
 * Code generation for function 'PresolveWorkingSet'
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
void PresolveWorkingSet(c_nlmpcmoveCodeGenerationStackD *SD,
                        const emlrtStack *sp, e_struct_T *solution,
                        g_struct_T *memspace, h_struct_T *workingset,
                        i_struct_T *qrmanager);

/* End of code generation (PresolveWorkingSet.h) */
