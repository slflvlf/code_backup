/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xtrsm.h
 *
 * Code generation for function 'xtrsm'
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
void b_xtrsm(int32_T m, const real_T A_data[], int32_T lda, real_T B_data[],
             int32_T ldb);

void xtrsm(int32_T m, const real_T A_data[], int32_T lda, real_T B_data[],
           int32_T ldb);

/* End of code generation (xtrsm.h) */
