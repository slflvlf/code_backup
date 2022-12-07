/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * BFGSUpdate.h
 *
 * Code generation for function 'BFGSUpdate'
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
boolean_T BFGSUpdate(int32_T nvar, real_T Bk[6241], const real_T sk_data[],
                     real_T yk_data[], real_T workspace_data[]);

/* End of code generation (BFGSUpdate.h) */
