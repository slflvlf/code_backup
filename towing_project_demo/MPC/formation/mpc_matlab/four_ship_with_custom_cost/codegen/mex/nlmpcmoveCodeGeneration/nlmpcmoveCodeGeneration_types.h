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
  real_T ref[192];
  real_T MVTarget[24];
  real_T X0[192];
  real_T MV0[192];
  real_T Slack0;
} struct1_T;
#endif /* typedef_struct1_T */

#ifndef typedef_struct2_T
#define typedef_struct2_T
typedef struct {
  real_T MVopt[216];
  real_T Xopt[216];
  real_T Yopt[216];
  real_T Topt[9];
  real_T Slack;
  real_T ExitFlag;
  real_T Iterations;
  real_T Cost;
} struct2_T;
#endif /* typedef_struct2_T */

#ifndef struct_emxArray_real_T_1152
#define struct_emxArray_real_T_1152
struct emxArray_real_T_1152 {
  real_T data[1152];
  int32_T size[1];
};
#endif /* struct_emxArray_real_T_1152 */
#ifndef typedef_emxArray_real_T_1152
#define typedef_emxArray_real_T_1152
typedef struct emxArray_real_T_1152 emxArray_real_T_1152;
#endif /* typedef_emxArray_real_T_1152 */

#ifndef struct_emxArray_real_T_1802
#define struct_emxArray_real_T_1802
struct emxArray_real_T_1802 {
  real_T data[1802];
  int32_T size[1];
};
#endif /* struct_emxArray_real_T_1802 */
#ifndef typedef_emxArray_real_T_1802
#define typedef_emxArray_real_T_1802
typedef struct emxArray_real_T_1802 emxArray_real_T_1802;
#endif /* typedef_emxArray_real_T_1802 */

#ifndef struct_emxArray_real_T_3411
#define struct_emxArray_real_T_3411
struct emxArray_real_T_3411 {
  real_T data[3411];
  int32_T size[1];
};
#endif /* struct_emxArray_real_T_3411 */
#ifndef typedef_emxArray_real_T_3411
#define typedef_emxArray_real_T_3411
typedef struct emxArray_real_T_3411 emxArray_real_T_3411;
#endif /* typedef_emxArray_real_T_3411 */

#ifndef struct_emxArray_int32_T_3411
#define struct_emxArray_int32_T_3411
struct emxArray_int32_T_3411 {
  int32_T data[3411];
  int32_T size[1];
};
#endif /* struct_emxArray_int32_T_3411 */
#ifndef typedef_emxArray_int32_T_3411
#define typedef_emxArray_int32_T_3411
typedef struct emxArray_int32_T_3411 emxArray_int32_T_3411;
#endif /* typedef_emxArray_int32_T_3411 */

#ifndef struct_emxArray_real_T_1802x384
#define struct_emxArray_real_T_1802x384
struct emxArray_real_T_1802x384 {
  real_T data[691968];
  int32_T size[2];
};
#endif /* struct_emxArray_real_T_1802x384 */
#ifndef typedef_emxArray_real_T_1802x384
#define typedef_emxArray_real_T_1802x384
typedef struct emxArray_real_T_1802x384 emxArray_real_T_1802x384;
#endif /* typedef_emxArray_real_T_1802x384 */

#ifndef struct_emxArray_real_T_1802x192
#define struct_emxArray_real_T_1802x192
struct emxArray_real_T_1802x192 {
  real_T data[345984];
  int32_T size[2];
};
#endif /* struct_emxArray_real_T_1802x192 */
#ifndef typedef_emxArray_real_T_1802x192
#define typedef_emxArray_real_T_1802x192
typedef struct emxArray_real_T_1802x192 emxArray_real_T_1802x192;
#endif /* typedef_emxArray_real_T_1802x192 */

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
  real_T xstarsqp[265];
  real_T xstarsqp_old[265];
  emxArray_real_T_1152 cIneq;
  emxArray_real_T_1152 cIneq_old;
  real_T cEq[192];
  real_T cEq_old[192];
  emxArray_real_T_1802 grad;
  emxArray_real_T_1802 grad_old;
  int32_T FunctionEvaluations;
  int32_T sqpIterations;
  int32_T sqpExitFlag;
  emxArray_real_T_3411 lambdasqp;
  emxArray_real_T_3411 lambdaStopTest;
  emxArray_real_T_3411 lambdaStopTestPrev;
  real_T steplength;
  emxArray_real_T_1802 delta_x;
  emxArray_real_T_1802 socDirection;
  emxArray_int32_T_3411 workingset_old;
  emxArray_real_T_1802x384 JacCineqTrans_old;
  emxArray_real_T_1802x192 JacCeqTrans_old;
  emxArray_real_T_1802 gradLag;
  emxArray_real_T_1802 delta_gradLag;
  emxArray_real_T_1802 xstar;
  real_T fstar;
  real_T firstorderopt;
  emxArray_real_T_3411 lambda;
  int32_T state;
  real_T maxConstr;
  int32_T iterations;
  emxArray_real_T_1802 searchDir;
} e_struct_T;
#endif /* typedef_e_struct_T */

#ifndef struct_emxArray_real_T_1801
#define struct_emxArray_real_T_1801
struct emxArray_real_T_1801 {
  real_T data[1801];
  int32_T size[1];
};
#endif /* struct_emxArray_real_T_1801 */
#ifndef typedef_emxArray_real_T_1801
#define typedef_emxArray_real_T_1801
typedef struct emxArray_real_T_1801 emxArray_real_T_1801;
#endif /* typedef_emxArray_real_T_1801 */

#ifndef typedef_f_struct_T
#define typedef_f_struct_T
typedef struct {
  emxArray_real_T_1802 grad;
  emxArray_real_T_1801 Hx;
  boolean_T hasLinear;
  int32_T nvar;
  int32_T maxVar;
  real_T beta;
  real_T rho;
  int32_T objtype;
  int32_T prev_objtype;
  int32_T prev_nvar;
  boolean_T prev_hasLinear;
  real_T gammaScalar;
} f_struct_T;
#endif /* typedef_f_struct_T */

#ifndef struct_emxArray_real_T_3411x1802
#define struct_emxArray_real_T_3411x1802
struct emxArray_real_T_3411x1802 {
  real_T data[6146622];
  int32_T size[2];
};
#endif /* struct_emxArray_real_T_3411x1802 */
#ifndef typedef_emxArray_real_T_3411x1802
#define typedef_emxArray_real_T_3411x1802
typedef struct emxArray_real_T_3411x1802 emxArray_real_T_3411x1802;
#endif /* typedef_emxArray_real_T_3411x1802 */

#ifndef typedef_g_struct_T
#define typedef_g_struct_T
typedef struct {
  emxArray_real_T_3411x1802 workspace_double;
  emxArray_int32_T_3411 workspace_int;
  emxArray_int32_T_3411 workspace_sort;
} g_struct_T;
#endif /* typedef_g_struct_T */

#ifndef struct_emxArray_real_T_2075904
#define struct_emxArray_real_T_2075904
struct emxArray_real_T_2075904 {
  real_T data[2075904];
  int32_T size[1];
};
#endif /* struct_emxArray_real_T_2075904 */
#ifndef typedef_emxArray_real_T_2075904
#define typedef_emxArray_real_T_2075904
typedef struct emxArray_real_T_2075904 emxArray_real_T_2075904;
#endif /* typedef_emxArray_real_T_2075904 */

#ifndef struct_emxArray_real_T_345984
#define struct_emxArray_real_T_345984
struct emxArray_real_T_345984 {
  real_T data[345984];
  int32_T size[1];
};
#endif /* struct_emxArray_real_T_345984 */
#ifndef typedef_emxArray_real_T_345984
#define typedef_emxArray_real_T_345984
typedef struct emxArray_real_T_345984 emxArray_real_T_345984;
#endif /* typedef_emxArray_real_T_345984 */

#ifndef struct_emxArray_int32_T_1802
#define struct_emxArray_int32_T_1802
struct emxArray_int32_T_1802 {
  int32_T data[1802];
  int32_T size[1];
};
#endif /* struct_emxArray_int32_T_1802 */
#ifndef typedef_emxArray_int32_T_1802
#define typedef_emxArray_int32_T_1802
typedef struct emxArray_int32_T_1802 emxArray_int32_T_1802;
#endif /* typedef_emxArray_int32_T_1802 */

#ifndef struct_emxArray_real_T_6146622
#define struct_emxArray_real_T_6146622
struct emxArray_real_T_6146622 {
  real_T data[6146622];
  int32_T size[1];
};
#endif /* struct_emxArray_real_T_6146622 */
#ifndef typedef_emxArray_real_T_6146622
#define typedef_emxArray_real_T_6146622
typedef struct emxArray_real_T_6146622 emxArray_real_T_6146622;
#endif /* typedef_emxArray_real_T_6146622 */

#ifndef struct_emxArray_boolean_T_3411
#define struct_emxArray_boolean_T_3411
struct emxArray_boolean_T_3411 {
  boolean_T data[3411];
  int32_T size[1];
};
#endif /* struct_emxArray_boolean_T_3411 */
#ifndef typedef_emxArray_boolean_T_3411
#define typedef_emxArray_boolean_T_3411
typedef struct emxArray_boolean_T_3411 emxArray_boolean_T_3411;
#endif /* typedef_emxArray_boolean_T_3411 */

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
  emxArray_real_T_2075904 Aineq;
  emxArray_real_T_1152 bineq;
  emxArray_real_T_345984 Aeq;
  real_T beq[192];
  emxArray_real_T_1802 lb;
  emxArray_real_T_1802 ub;
  emxArray_int32_T_1802 indexLB;
  emxArray_int32_T_1802 indexUB;
  emxArray_int32_T_1802 indexFixed;
  int32_T mEqRemoved;
  int32_T indexEqRemoved[192];
  emxArray_real_T_6146622 ATwset;
  emxArray_real_T_3411 bwset;
  int32_T nActiveConstr;
  emxArray_real_T_3411 maxConstrWorkspace;
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
  emxArray_boolean_T_3411 isActiveConstr;
  emxArray_int32_T_3411 Wid;
  emxArray_int32_T_3411 Wlocalidx;
  int32_T nWConstr[5];
  int32_T probType;
  real_T SLACK0;
} h_struct_T;
#endif /* typedef_h_struct_T */

#ifndef struct_emxArray_real_T_3411x3411
#define struct_emxArray_real_T_3411x3411
struct emxArray_real_T_3411x3411 {
  real_T data[11634921];
  int32_T size[2];
};
#endif /* struct_emxArray_real_T_3411x3411 */
#ifndef typedef_emxArray_real_T_3411x3411
#define typedef_emxArray_real_T_3411x3411
typedef struct emxArray_real_T_3411x3411 emxArray_real_T_3411x3411;
#endif /* typedef_emxArray_real_T_3411x3411 */

#ifndef typedef_i_struct_T
#define typedef_i_struct_T
typedef struct {
  int32_T ldq;
  emxArray_real_T_3411x3411 QR;
  emxArray_real_T_3411x3411 Q;
  emxArray_int32_T_3411 jpvt;
  int32_T mrows;
  int32_T ncols;
  emxArray_real_T_3411 tau;
  int32_T minRowCol;
  boolean_T usedPivoting;
} i_struct_T;
#endif /* typedef_i_struct_T */

#ifndef typedef_j_struct_T
#define typedef_j_struct_T
typedef struct {
  emxArray_real_T_3411x3411 FMat;
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

#ifndef typedef_k_struct_T
#define typedef_k_struct_T
typedef struct {
  real_T Ts;
  real_T CurrentStates[24];
  real_T LastMV[24];
  real_T References[192];
  real_T MVTarget[192];
  real_T PredictionHorizon;
  real_T NumOfStates;
  real_T NumOfOutputs;
  real_T NumOfInputs;
  real_T MVIndex[24];
} k_struct_T;
#endif /* typedef_k_struct_T */

#ifndef typedef_l_struct_T
#define typedef_l_struct_T
typedef struct {
  real_T x[24];
  real_T lastMV[24];
  real_T ref[192];
  real_T OutputWeights[192];
  real_T MVWeights[192];
  real_T MVRateWeights[192];
  real_T ECRWeight;
  real_T OutputMin[192];
  real_T OutputMax[192];
  real_T StateMin[192];
  real_T StateMax[192];
  real_T MVMin[192];
  real_T MVMax[192];
  real_T MVRateMin[192];
  real_T MVRateMax[192];
  real_T MVScaledTarget[192];
} l_struct_T;
#endif /* typedef_l_struct_T */

#ifndef typedef_m_struct_T
#define typedef_m_struct_T
typedef struct {
  l_struct_T runtimedata;
  k_struct_T userdata;
} m_struct_T;
#endif /* typedef_m_struct_T */

#ifndef typedef_anonymous_function
#define typedef_anonymous_function
typedef struct {
  m_struct_T workspace;
} anonymous_function;
#endif /* typedef_anonymous_function */

#ifndef typedef_n_struct_T
#define typedef_n_struct_T
typedef struct {
  anonymous_function objfun;
  anonymous_function nonlcon;
  int32_T nVar;
  int32_T mCineq;
  int32_T mCeq;
  boolean_T NonFiniteSupport;
  boolean_T SpecifyObjectiveGradient;
  boolean_T SpecifyConstraintGradient;
  boolean_T ScaleProblem;
} n_struct_T;
#endif /* typedef_n_struct_T */

#ifndef typedef_b_saveJacobian
#define typedef_b_saveJacobian
typedef struct {
  real_T y_data[11634921];
} b_saveJacobian;
#endif /* typedef_b_saveJacobian */

#ifndef typedef_b_znlmpc_reformJacobian
#define typedef_b_znlmpc_reformJacobian
typedef struct {
  real_T varargin_1_data[73728];
  real_T y_data[55296];
  real_T varargin_2_data[27648];
} b_znlmpc_reformJacobian;
#endif /* typedef_b_znlmpc_reformJacobian */

#ifndef typedef_b_stateEvolution
#define typedef_b_stateEvolution
typedef struct {
  real_T Jx[36864];
  real_T Jmv[36864];
  real_T y[13824];
} b_stateEvolution;
#endif /* typedef_b_stateEvolution */

#ifndef typedef_b_feasibleX0ForWorkingSet
#define typedef_b_feasibleX0ForWorkingSet
typedef struct {
  real_T workspace_data[6146622];
} b_feasibleX0ForWorkingSet;
#endif /* typedef_b_feasibleX0ForWorkingSet */

#ifndef typedef_b_checkLinearInputs
#define typedef_b_checkLinearInputs
typedef struct {
  boolean_T tmp_data[295680];
  boolean_T b_tmp_data[295680];
} b_checkLinearInputs;
#endif /* typedef_b_checkLinearInputs */

#ifndef typedef_b_znlmpc_getUBounds
#define typedef_b_znlmpc_getUBounds
typedef struct {
  real_T Au[147456];
  real_T Auf_data[147456];
  real_T y_data[55296];
} b_znlmpc_getUBounds;
#endif /* typedef_b_znlmpc_getUBounds */

#ifndef typedef_b_outputBounds
#define typedef_b_outputBounds
typedef struct {
  real_T Jx_data[73728];
  real_T tmp_data[73728];
} b_outputBounds;
#endif /* typedef_b_outputBounds */

#ifndef typedef_b_soc
#define typedef_b_soc
typedef struct {
  h_struct_T WorkingSet;
} b_soc;
#endif /* typedef_b_soc */

#ifndef typedef_d_nlmpcmoveCodeGeneration_anonF
#define typedef_d_nlmpcmoveCodeGeneration_anonF
typedef struct {
  real_T Jc_data[101760];
} d_nlmpcmoveCodeGeneration_anonF;
#endif /* typedef_d_nlmpcmoveCodeGeneration_anonF */

#ifndef typedef_d_computeConstraintsAndUserJaco
#define typedef_d_computeConstraintsAndUserJaco
typedef struct {
  real_T varargout_3_data[101760];
  real_T varargout_4[50880];
} d_computeConstraintsAndUserJaco;
#endif /* typedef_d_computeConstraintsAndUserJaco */

#ifndef typedef_b_checkNonlinearInputs
#define typedef_b_checkNonlinearInputs
typedef struct {
  real_T varargout_3_data[101760];
  real_T varargout_4[50880];
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
  real_T Hessian[70225];
  n_struct_T FcnEvaluator;
  f_struct_T QPObjective;
} b_fmincon;
#endif /* typedef_b_fmincon */

#ifndef typedef_b_nlmpcmoveCodeGeneration
#define typedef_b_nlmpcmoveCodeGeneration
typedef struct {
  real_T A_data[295680];
} b_nlmpcmoveCodeGeneration;
#endif /* typedef_b_nlmpcmoveCodeGeneration */

#ifndef typedef_c_nlmpcmoveCodeGenerationStackD
#define typedef_c_nlmpcmoveCodeGenerationStackD
typedef struct {
  union {
    b_saveJacobian f0;
    b_znlmpc_reformJacobian f1;
    b_stateEvolution f2;
    b_feasibleX0ForWorkingSet f3;
    b_checkLinearInputs f4;
    b_znlmpc_getUBounds f5;
  } u1;
  union {
    b_outputBounds f6;
    b_soc f7;
  } u2;
  d_nlmpcmoveCodeGeneration_anonF f8;
  union {
    d_computeConstraintsAndUserJaco f9;
    b_checkNonlinearInputs f10;
  } u4;
  b_fmincon f11;
  b_nlmpcmoveCodeGeneration f12;
} c_nlmpcmoveCodeGenerationStackD;
#endif /* typedef_c_nlmpcmoveCodeGenerationStackD */

/* End of code generation (nlmpcmoveCodeGeneration_types.h) */
