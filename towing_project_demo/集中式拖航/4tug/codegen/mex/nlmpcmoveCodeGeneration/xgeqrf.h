/*
 * xgeqrf.h
 *
 * Code generation for function 'xgeqrf'
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
void xgeqrf(const emlrtStack *sp, real_T A_data[], const int32_T A_size[2],
            int32_T m, int32_T n, real_T tau_data[], int32_T *tau_size);

/* End of code generation (xgeqrf.h) */
