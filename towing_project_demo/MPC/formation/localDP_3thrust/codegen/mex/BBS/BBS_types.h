/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * BBS_types.h
 *
 * Code generation for function 'BBS_types'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_real_T*/

#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T

typedef struct emxArray_real_T emxArray_real_T;

#endif                                 /*typedef_emxArray_real_T*/

#ifndef typedef_struct4_T
#define typedef_struct4_T

typedef struct {
  real_T pmin;
  real_T pmax;
  real_T alpha;
  real_T p;
  real_T beta_plus;
  real_T beta_moins;
  real_T alpha_min;
  real_T pbarmin;
  real_T pbarmax;
  real_T pc;
  real_T beta;
  real_T Jpbarmin;
  real_T gpbarmin;
  real_T Jpc;
  real_T gpc;
  real_T Jpbarmax;
  real_T gpbarmax;
  real_T aJ;
  real_T bJ;
  real_T cJ;
  real_T ag;
  real_T bg;
  real_T cg;
  real_T ps_J;
  real_T ps_J_star;
  real_T qJstar;
  real_T qJmin;
  real_T pJmin;
  real_T qJmax;
  real_T pJmax;
  real_T ps_g;
  real_T ps_g_star;
  real_T qgstar;
  real_T qgmin;
  real_T pgmin;
  real_T qgmax;
  real_T pgmax;
  real_T Z[4];
  real_T ngz;
} struct4_T;

#endif                                 /*typedef_struct4_T*/

#ifndef typedef_struct2_T
#define typedef_struct2_T

typedef struct {
  real_T tau;
  real_T rk_order;
  real_T x0[6];
  real_T u0[6];
  real_T M[9];
  real_T D[9];
  real_T thrust_config[6];
} struct2_T;

#endif                                 /*typedef_struct2_T*/

#ifndef typedef_struct1_T
#define typedef_struct1_T

typedef struct {
  real_T nu;
  real_T Np;
  real_T Ifree[4];
  real_T R[2880];
  real_T np;
  real_T p[24];
  real_T pmin[24];
  real_T pmax[24];
} struct1_T;

#endif                                 /*typedef_struct1_T*/

#ifndef typedef_struct3_T
#define typedef_struct3_T

typedef struct {
  real_T Q[36];
  real_T R[36];
  real_T Rdu[36];
  real_T rd[3];
  real_T dF_max;
  real_T da_max;
} struct3_T;

#endif                                 /*typedef_struct3_T*/

#ifndef typedef_struct0_T
#define typedef_struct0_T

typedef struct {
  real_T pmin[24];
  real_T pmax[24];
  real_T p[24];
  real_T alpha[24];
  real_T ell;
  struct1_T uparam;
  struct2_T ode;
  struct3_T ocp;
  real_T x0[6];
  struct4_T mv[24];
  real_T compiled;
  real_T Nev;
  real_T Niter;
  real_T np;
} struct0_T;

#endif                                 /*typedef_struct0_T*/

/* End of code generation (BBS_types.h) */
