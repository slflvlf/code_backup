/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * BBS_mexutil.c
 *
 * Code generation for function 'BBS_mexutil'
 *
 */

/* Include files */
#include "BBS_mexutil.h"
#include "BBS.h"
#include "rt_nonfinite.h"

/* Function Definitions */
const mxArray *emlrt_marshallOut(const real_T u)
{
  const mxArray *y;
  const mxArray *m;
  y = NULL;
  m = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m);
  return y;
}

/* End of code generation (BBS_mexutil.c) */
