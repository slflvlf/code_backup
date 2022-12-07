/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * nlmpcmoveCodeGeneration_types.h
 *
 * Code generation for function 'nlmpcmoveCodeGeneration'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"

/* Type Definitions */
#ifndef typedef_struct1_T
#define typedef_struct1_T
typedef struct {
  real_T ref[60];
  real_T MVTarget[6];
  real_T X0[60];
  real_T MV0[60];
  real_T Slack0;
} struct1_T;
#endif /* typedef_struct1_T */

#ifndef typedef_struct2_T
#define typedef_struct2_T
typedef struct {
  real_T MVopt[66];
  real_T Xopt[66];
  real_T Yopt[66];
  real_T Topt[11];
  real_T Slack;
  real_T ExitFlag;
  real_T Iterations;
  real_T Cost;
} struct2_T;
#endif /* typedef_struct2_T */

#ifndef struct_emxArray_real_T_360
#define struct_emxArray_real_T_360
struct emxArray_real_T_360 {
  real_T data[360];
  int32_T size[1];
};
#endif /* struct_emxArray_real_T_360 */
#ifndef typedef_emxArray_real_T_360
#define typedef_emxArray_real_T_360
typedef struct emxArray_real_T_360 emxArray_real_T_360;
#endif /* typedef_emxArray_real_T_360 */

#ifndef struct_emxArray_real_T_560
#define struct_emxArray_real_T_560
struct emxArray_real_T_560 {
  real_T data[560];
  int32_T size[1];
};
#endif /* struct_emxArray_real_T_560 */
#ifndef typedef_emxArray_real_T_560
#define typedef_emxArray_real_T_560
typedef struct emxArray_real_T_560 emxArray_real_T_560;
#endif /* typedef_emxArray_real_T_560 */

#ifndef struct_emxArray_real_T_1059
#define struct_emxArray_real_T_1059
struct emxArray_real_T_1059 {
  real_T data[1059];
  int32_T size[1];
};
#endif /* struct_emxArray_real_T_1059 */
#ifndef typedef_emxArray_real_T_1059
#define typedef_emxArray_real_T_1059
typedef struct emxArray_real_T_1059 emxArray_real_T_1059;
#endif /* typedef_emxArray_real_T_1059 */

#ifndef struct_emxArray_int32_T_1059
#define struct_emxArray_int32_T_1059
struct emxArray_int32_T_1059 {
  int32_T data[1059];
  int32_T size[1];
};
#endif /* struct_emxArray_int32_T_1059 */
#ifndef typedef_emxArray_int32_T_1059
#define typedef_emxArray_int32_T_1059
typedef struct emxArray_int32_T_1059 emxArray_int32_T_1059;
#endif /* typedef_emxArray_int32_T_1059 */

#ifndef struct_emxArray_real_T_560x120
#define struct_emxArray_real_T_560x120
struct emxArray_real_T_560x120 {
  real_T data[67200];
  int32_T size[2];
};
#endif /* struct_emxArray_real_T_560x120 */
#ifndef typedef_emxArray_real_T_560x120
#define typedef_emxArray_real_T_560x120
typedef struct emxArray_real_T_560x120 emxArray_real_T_560x120;
#endif /* typedef_emxArray_real_T_560x120 */

#ifndef struct_emxArray_real_T_560x60
#define struct_emxArray_real_T_560x60
struct emxArray_real_T_560x60 {
  real_T data[33600];
  int32_T size[2];
};
#endif /* struct_emxArray_real_T_560x60 */
#ifndef typedef_emxArray_real_T_560x60
#define typedef_emxArray_real_T_560x60
typedef struct emxArray_real_T_560x60 emxArray_real_T_560x60;
#endif /* typedef_emxArray_real_T_560x60 */

#ifndef typedef_e_struct_T
#define typedef_e_struct_T
typedef struct {
  int32_T nVarMax;
  int32_T mNonlinIneq;
  int32_T mNonlinEq;
  int32_T mIneq;
  int32_T mEq;
  int32_T iNonIneq0;
  int32_T iNonEq0;
  real_T sqpFval;
  real_T sqpFval_old;
  real_T xstarsqp[79];
  real_T xstarsqp_old[79];
  emxArray_real_T_360 cIneq;
  emxArray_real_T_360 cIneq_old;
  real_T cEq[60];
  real_T cEq_old[60];
  emxArray_real_T_560 grad;
  emxArray_real_T_560 grad_old;
  int32_T FunctionEvaluations;
  int32_T sqpIterations;
  int32_T sqpExitFlag;
  emxArray_real_T_1059 lambdasqp;
  emxArray_real_T_1059 lambdaStopTest;
  emxArray_real_T_1059 lambdaStopTestPrev;
  real_T steplength;
  emxArray_real_T_560 delta_x;
  emxArray_real_T_560 socDirection;
  emxArray_int32_T_1059 workingset_old;
  emxArray_real_T_560x120 JacCineqTrans_old;
  emxArray_real_T_560x60 JacCeqTrans_old;
  emxArray_real_T_560 gradLag;
  emxArray_real_T_560 delta_gradLag;
  emxArray_real_T_560 xstar;
  real_T fstar;
  real_T firstorderopt;
  emxArray_real_T_1059 lambda;
  int32_T state;
  real_T maxConstr;
  int32_T iterations;
  emxArray_real_T_560 searchDir;
} e_struct_T;
#endif /* typedef_e_struct_T */

#ifndef struct_emxArray_real_T_1059x560
#define struct_emxArray_real_T_1059x560
struct emxArray_real_T_1059x560 {
  real_T data[593040];
  int32_T size[2];
};
#endif /* struct_emxArray_real_T_1059x560 */
#ifndef typedef_emxArray_real_T_1059x560
#define typedef_emxArray_real_T_1059x560
typedef struct emxArray_real_T_1059x560 emxArray_real_T_1059x560;
#endif /* typedef_emxArray_real_T_1059x560 */

#ifndef typedef_g_struct_T
#define typedef_g_struct_T
typedef struct {
  emxArray_real_T_1059x560 workspace_double;
  emxArray_int32_T_1059 workspace_int;
  emxArray_int32_T_1059 workspace_sort;
} g_struct_T;
#endif /* typedef_g_struct_T */

#ifndef struct_emxArray_real_T_201600
#define struct_emxArray_real_T_201600
struct emxArray_real_T_201600 {
  real_T data[201600];
  int32_T size[1];
};
#endif /* struct_emxArray_real_T_201600 */
#ifndef typedef_emxArray_real_T_201600
#define typedef_emxArray_real_T_201600
typedef struct emxArray_real_T_201600 emxArray_real_T_201600;
#endif /* typedef_emxArray_real_T_201600 */

#ifndef struct_emxArray_real_T_33600
#define struct_emxArray_real_T_33600
struct emxArray_real_T_33600 {
  real_T data[33600];
  int32_T size[1];
};
#endif /* struct_emxArray_real_T_33600 */
#ifndef typedef_emxArray_real_T_33600
#define typedef_emxArray_real_T_33600
typedef struct emxArray_real_T_33600 emxArray_real_T_33600;
#endif /* typedef_emxArray_real_T_33600 */

#ifndef struct_emxArray_int32_T_560
#define struct_emxArray_int32_T_560
struct emxArray_int32_T_560 {
  int32_T data[560];
  int32_T size[1];
};
#endif /* struct_emxArray_int32_T_560 */
#ifndef typedef_emxArray_int32_T_560
#define typedef_emxArray_int32_T_560
typedef struct emxArray_int32_T_560 emxArray_int32_T_560;
#endif /* typedef_emxArray_int32_T_560 */

#ifndef struct_emxArray_real_T_593040
#define struct_emxArray_real_T_593040
struct emxArray_real_T_593040 {
  real_T data[593040];
  int32_T size[1];
};
#endif /* struct_emxArray_real_T_593040 */
#ifndef typedef_emxArray_real_T_593040
#define typedef_emxArray_real_T_593040
typedef struct emxArray_real_T_593040 emxArray_real_T_593040;
#endif /* typedef_emxArray_real_T_593040 */

#ifndef struct_emxArray_boolean_T_1059
#define struct_emxArray_boolean_T_1059
struct emxArray_boolean_T_1059 {
  boolean_T data[1059];
  int32_T size[1];
};
#endif /* struct_emxArray_boolean_T_1059 */
#ifndef typedef_emxArray_boolean_T_1059
#define typedef_emxArray_boolean_T_1059
typedef struct emxArray_boolean_T_1059 emxArray_boolean_T_1059;
#endif /* typedef_emxArray_boolean_T_1059 */

#ifndef typedef_h_struct_T
#define typedef_h_struct_T
typedef struct {
  int32_T mConstr;
  int32_T mConstrOrig;
  int32_T mConstrMax;
  int32_T nVar;
  int32_T nVarOrig;
  int32_T nVarMax;
  int32_T ldA;
  emxArray_real_T_201600 Aineq;
  emxArray_real_T_360 bineq;
  emxArray_real_T_33600 Aeq;
  real_T beq[60];
  emxArray_real_T_560 lb;
  emxArray_real_T_560 ub;
  emxArray_int32_T_560 indexLB;
  emxArray_int32_T_560 indexUB;
  emxArray_int32_T_560 indexFixed;
  int32_T mEqRemoved;
  int32_T indexEqRemoved[60];
  emxArray_real_T_593040 ATwset;
  emxArray_real_T_1059 bwset;
  int32_T nActiveConstr;
  emxArray_real_T_1059 maxConstrWorkspace;
  int32_T sizes[5];
  int32_T sizesNormal[5];
  int32_T sizesPhaseOne[5];
  int32_T sizesRegularized[5];
  int32_T sizesRegPhaseOne[5];
  int32_T isActiveIdx[6];
  int32_T isActiveIdxNormal[6];
  int32_T isActiveIdxPhaseOne[6];
  int32_T isActiveIdxRegularized[6];
  int32_T isActiveIdxRegPhaseOne[6];
  emxArray_boolean_T_1059 isActiveConstr;
  emxArray_int32_T_1059 Wid;
  emxArray_int32_T_1059 Wlocalidx;
  int32_T nWConstr[5];
  int32_T probType;
  real_T SLACK0;
} h_struct_T;
#endif /* typedef_h_struct_T */

#ifndef struct_emxArray_real_T_1059x1059
#define struct_emxArray_real_T_1059x1059
struct emxArray_real_T_1059x1059 {
  real_T data[1121481];
  int32_T size[2];
};
#endif /* struct_emxArray_real_T_1059x1059 */
#ifndef typedef_emxArray_real_T_1059x1059
#define typedef_emxArray_real_T_1059x1059
typedef struct emxArray_real_T_1059x1059 emxArray_real_T_1059x1059;
#endif /* typedef_emxArray_real_T_1059x1059 */

#ifndef typedef_i_struct_T
#define typedef_i_struct_T
typedef struct {
  int32_T ldq;
  emxArray_real_T_1059x1059 QR;
  emxArray_real_T_1059x1059 Q;
  emxArray_int32_T_1059 jpvt;
  int32_T mrows;
  int32_T ncols;
  emxArray_real_T_1059 tau;
  int32_T minRowCol;
  boolean_T usedPivoting;
} i_struct_T;
#endif /* typedef_i_struct_T */

#ifndef typedef_j_struct_T
#define typedef_j_struct_T
typedef struct {
  emxArray_real_T_1059x1059 FMat;
  int32_T ldm;
  int32_T ndims;
  int32_T info;
  real_T scaleFactor;
  boolean_T ConvexCheck;
  real_T regTol_;
  real_T workspace_;
  real_T workspace2_;
} j_struct_T;
#endif /* typedef_j_struct_T */

#ifndef typedef_b_saveJacobian
#define typedef_b_saveJacobian
typedef struct {
  real_T y_data[1121481];
} b_saveJacobian;
#endif /* typedef_b_saveJacobian */

#ifndef typedef_b_outputBounds
#define typedef_b_outputBounds
typedef struct {
  real_T tmp_data[14400];
  real_T y_data[4320];
} b_outputBounds;
#endif /* typedef_b_outputBounds */

#ifndef typedef_b_feasibleX0ForWorkingSet
#define typedef_b_feasibleX0ForWorkingSet
typedef struct {
  real_T workspace_data[593040];
} b_feasibleX0ForWorkingSet;
#endif /* typedef_b_feasibleX0ForWorkingSet */

#ifndef typedef_d_nlmpcmoveCodeGeneration_anonF
#define typedef_d_nlmpcmoveCodeGeneration_anonF
typedef struct {
  real_T Jc_data[9480];
} d_nlmpcmoveCodeGeneration_anonF;
#endif /* typedef_d_nlmpcmoveCodeGeneration_anonF */

#ifndef typedef_b_soc
#define typedef_b_soc
typedef struct {
  h_struct_T WorkingSet;
} b_soc;
#endif /* typedef_b_soc */

#ifndef typedef_b_evalObjAndConstrAndDerivative
#define typedef_b_evalObjAndConstrAndDerivative
typedef struct {
  real_T varargout_3_data[9480];
  real_T varargout_4[4740];
} b_evalObjAndConstrAndDerivative;
#endif /* typedef_b_evalObjAndConstrAndDerivative */

#ifndef typedef_b_checkNonlinearInputs
#define typedef_b_checkNonlinearInputs
typedef struct {
  real_T varargout_3_data[9480];
  real_T varargout_4[4740];
} b_checkNonlinearInputs;
#endif /* typedef_b_checkNonlinearInputs */

#ifndef typedef_b_fmincon
#define typedef_b_fmincon
typedef struct {
  i_struct_T obj;
  j_struct_T b_obj;
  h_struct_T WorkingSet;
  g_struct_T memspace;
  e_struct_T TrialState;
  real_T unusedExpr[6241];
} b_fmincon;
#endif /* typedef_b_fmincon */

#ifndef typedef_b_nlmpcmoveCodeGeneration
#define typedef_b_nlmpcmoveCodeGeneration
typedef struct {
  real_T A_data[29040];
  real_T Au[14400];
  real_T Auf_data[14400];
  real_T y_data[4320];
} b_nlmpcmoveCodeGeneration;
#endif /* typedef_b_nlmpcmoveCodeGeneration */

#ifndef typedef_c_nlmpcmoveCodeGenerationStackD
#define typedef_c_nlmpcmoveCodeGenerationStackD
typedef struct {
  union {
    b_saveJacobian f0;
    b_outputBounds f1;
    b_feasibleX0ForWorkingSet f2;
  } u1;
  union {
    d_nlmpcmoveCodeGeneration_anonF f3;
    b_soc f4;
  } u2;
  union {
    b_evalObjAndConstrAndDerivative f5;
    b_checkNonlinearInputs f6;
  } u3;
  b_fmincon f7;
  b_nlmpcmoveCodeGeneration f8;
} c_nlmpcmoveCodeGenerationStackD;
#endif /* typedef_c_nlmpcmoveCodeGenerationStackD */

/* End of code generation (nlmpcmoveCodeGeneration_types.h) */
