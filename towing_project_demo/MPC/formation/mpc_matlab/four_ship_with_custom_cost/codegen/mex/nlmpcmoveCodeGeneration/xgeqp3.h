/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xgeqp3.h
 *
 * Code generation for function 'xgeqp3'
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
void xgeqp3(const emlrtStack *sp, real_T A_data[], const int32_T A_size[2],
            int32_T m, int32_T n, int32_T jpvt_data[], const int32_T *jpvt_size,
            real_T tau_data[], int32_T *tau_size);

/* End of code generation (xgeqp3.h) */
