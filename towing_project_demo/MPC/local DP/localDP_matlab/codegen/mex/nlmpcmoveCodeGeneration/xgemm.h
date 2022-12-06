/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xgemm.h
 *
 * Code generation for function 'xgemm'
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
void b_xgemm(int32_T m, int32_T k, const real_T A_data[], int32_T lda,
             const real_T B_data[], int32_T ldb, real_T C_data[], int32_T ldc);

void c_xgemm(int32_T m, int32_T n, int32_T k, const real_T A[6241], int32_T lda,
             const real_T B_data[], int32_T ib0, int32_T ldb, real_T C_data[],
             int32_T ldc);

void d_xgemm(int32_T m, int32_T n, int32_T k, const real_T A_data[],
             int32_T ia0, int32_T lda, const real_T B_data[], int32_T ldb,
             real_T C_data[], int32_T ldc);

void xgemm(int32_T m, int32_T k, const real_T A_data[], int32_T lda,
           const real_T B_data[], int32_T ldb, real_T C_data[], int32_T ldc);

/* End of code generation (xgemm.h) */
