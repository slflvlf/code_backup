/*
 * xaxpy.h
 *
 * Code generation for function 'xaxpy'
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
void b_xaxpy(int32_T n, const real_T x_data[], real_T y_data[]);

void xaxpy(int32_T n, real_T a, const real_T x_data[], real_T y_data[]);

/* End of code generation (xaxpy.h) */
