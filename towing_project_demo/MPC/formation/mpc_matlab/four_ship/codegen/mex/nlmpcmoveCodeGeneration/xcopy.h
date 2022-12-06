/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
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
void b_xcopy(int32_T n, const real_T x_data[], real_T y_data[], int32_T iy0);

void c_xcopy(int32_T n, const real_T x_data[], real_T y_data[]);

void d_xcopy(int32_T n, const real_T x_data[], real_T y_data[]);

void e_xcopy(int32_T n, const real_T x_data[], int32_T ix0, real_T y_data[]);

void f_xcopy(const emlrtStack *sp, int32_T n, const real_T x[70225],
             int32_T ix0, real_T y_data[], int32_T iy0);

void g_xcopy(const emlrtStack *sp, int32_T n, real_T y_data[]);

void h_xcopy(const emlrtStack *sp, int32_T n, real_T y[70225], int32_T iy0);

void xcopy(const emlrtStack *sp, int32_T n, const real_T x[265],
           real_T y_data[]);

/* End of code generation (xcopy.h) */
