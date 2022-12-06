/*
 * xcopy.h
 *
 * Code generation for function 'xcopy'
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
void b_xcopy(int32_T n, const real_T x_data[], real_T y_data[]);

void c_xcopy(int32_T n, const real_T x_data[], real_T y_data[]);

void d_xcopy(int32_T n, const real_T x_data[], int32_T ix0, real_T y_data[]);

void e_xcopy(const emlrtStack *sp, int32_T n, const real_T x[7225], int32_T ix0,
             real_T y_data[], int32_T iy0);

void f_xcopy(const emlrtStack *sp, int32_T n, real_T y_data[]);

void g_xcopy(const emlrtStack *sp, int32_T n, real_T y[7225], int32_T iy0);

void xcopy(const emlrtStack *sp, int32_T n, const real_T x[85],
           real_T y_data[]);

/* End of code generation (xcopy.h) */
