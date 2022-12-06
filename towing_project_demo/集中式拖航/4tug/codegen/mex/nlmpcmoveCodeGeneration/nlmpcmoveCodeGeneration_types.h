/*
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
  real_T ref[6];
  real_T MVTarget[8];
  real_T X0[60];
  real_T MV0[80];
  real_T Slack0;
} struct1_T;
#endif /* typedef_struct1_T */

#ifndef typedef_struct2_T
#define typedef_struct2_T
typedef struct {
  real_T MVopt[88];
  real_T Xopt[66];
  real_T Yopt[66];
  real_T Topt[11];
  real_T Slack;
  real_T ExitFlag;
  real_T Iterations;
  real_T Cost;
} struct2_T;
#endif /* typedef_struct2_T */

#ifndef struct_emxArray_real_T_440
#define struct_emxArray_real_T_440
struct emxArray_real_T_440 {
  real_T data[440];
  int32_T size[1];
};
#endif /* struct_emxArray_real_T_440 */
#ifndef typedef_emxArray_real_T_440
#define typedef_emxArray_real_T_440
typedef struct emxArray_real_T_440 emxArray_real_T_440;
#endif /* typedef_emxArray_real_T_440 */

#ifndef struct_emxArray_real_T_646
#define struct_emxArray_real_T_646
struct emxArray_real_T_646 {
  real_T data[646];
  int32_T size[1];
};
#endif /* struct_emxArray_real_T_646 */
#ifndef typedef_emxArray_real_T_646
#define typedef_emxArray_real_T_646
typedef struct emxArray_real_T_646 emxArray_real_T_646;
#endif /* typedef_emxArray_real_T_646 */

#ifndef struct_emxArray_real_T_1231
#define struct_emxArray_real_T_1231
struct emxArray_real_T_1231 {
  real_T data[1231];
  int32_T size[1];
};
#endif /* struct_emxArray_real_T_1231 */
#ifndef typedef_emxArray_real_T_1231
#define typedef_emxArray_real_T_1231
typedef struct emxArray_real_T_1231 emxArray_real_T_1231;
#endif /* typedef_emxArray_real_T_1231 */

#ifndef struct_emxArray_int32_T_1231
#define struct_emxArray_int32_T_1231
struct emxArray_int32_T_1231 {
  int32_T data[1231];
  int32_T size[1];
};
#endif /* struct_emxArray_int32_T_1231 */
#ifndef typedef_emxArray_int32_T_1231
#define typedef_emxArray_int32_T_1231
typedef struct emxArray_int32_T_1231 emxArray_int32_T_1231;
#endif /* typedef_emxArray_int32_T_1231 */

#ifndef struct_emxArray_real_T_646x120
#define struct_emxArray_real_T_646x120
struct emxArray_real_T_646x120 {
  real_T data[77520];
  int32_T size[2];
};
#endif /* struct_emxArray_real_T_646x120 */
#ifndef typedef_emxArray_real_T_646x120
#define typedef_emxArray_real_T_646x120
typedef struct emxArray_real_T_646x120 emxArray_real_T_646x120;
#endif /* typedef_emxArray_real_T_646x120 */

#ifndef struct_emxArray_real_T_646x60
#define struct_emxArray_real_T_646x60
struct emxArray_real_T_646x60 {
  real_T data[38760];
  int32_T size[2];
};
#endif /* struct_emxArray_real_T_646x60 */
#ifndef typedef_emxArray_real_T_646x60
#define typedef_emxArray_real_T_646x60
typedef struct emxArray_real_T_646x60 emxArray_real_T_646x60;
#endif /* typedef_emxArray_real_T_646x60 */

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
  real_T xstarsqp[85];
  real_T xstarsqp_old[85];
  emxArray_real_T_440 cIneq;
  emxArray_real_T_440 cIneq_old;
  real_T cEq[60];
  real_T cEq_old[60];
  emxArray_real_T_646 grad;
  emxArray_real_T_646 grad_old;
  int32_T FunctionEvaluations;
  int32_T sqpIterations;
  int32_T sqpExitFlag;
  emxArray_real_T_1231 lambdasqp;
  emxArray_real_T_1231 lambdaStopTest;
  emxArray_real_T_1231 lambdaStopTestPrev;
  real_T steplength;
  emxArray_real_T_646 delta_x;
  emxArray_real_T_646 socDirection;
  emxArray_int32_T_1231 workingset_old;
  emxArray_real_T_646x120 JacCineqTrans_old;
  emxArray_real_T_646x60 JacCeqTrans_old;
  emxArray_real_T_646 gradLag;
  emxArray_real_T_646 delta_gradLag;
  emxArray_real_T_646 xstar;
  real_T fstar;
  real_T firstorderopt;
  emxArray_real_T_1231 lambda;
  int32_T state;
  real_T maxConstr;
  int32_T iterations;
  emxArray_real_T_646 searchDir;
} e_struct_T;
#endif /* typedef_e_struct_T */

#ifndef struct_emxArray_real_T_1231x646
#define struct_emxArray_real_T_1231x646
struct emxArray_real_T_1231x646 {
  real_T data[795226];
  int32_T size[2];
};
#endif /* struct_emxArray_real_T_1231x646 */
#ifndef typedef_emxArray_real_T_1231x646
#define typedef_emxArray_real_T_1231x646
typedef struct emxArray_real_T_1231x646 emxArray_real_T_1231x646;
#endif /* typedef_emxArray_real_T_1231x646 */

#ifndef typedef_g_struct_T
#define typedef_g_struct_T
typedef struct {
  emxArray_real_T_1231x646 workspace_double;
  emxArray_int32_T_1231 workspace_int;
  emxArray_int32_T_1231 workspace_sort;
} g_struct_T;
#endif /* typedef_g_struct_T */

#ifndef struct_emxArray_real_T_284240
#define struct_emxArray_real_T_284240
struct emxArray_real_T_284240 {
  real_T data[284240];
  int32_T size[1];
};
#endif /* struct_emxArray_real_T_284240 */
#ifndef typedef_emxArray_real_T_284240
#define typedef_emxArray_real_T_284240
typedef struct emxArray_real_T_284240 emxArray_real_T_284240;
#endif /* typedef_emxArray_real_T_284240 */

#ifndef struct_emxArray_real_T_38760
#define struct_emxArray_real_T_38760
struct emxArray_real_T_38760 {
  real_T data[38760];
  int32_T size[1];
};
#endif /* struct_emxArray_real_T_38760 */
#ifndef typedef_emxArray_real_T_38760
#define typedef_emxArray_real_T_38760
typedef struct emxArray_real_T_38760 emxArray_real_T_38760;
#endif /* typedef_emxArray_real_T_38760 */

#ifndef struct_emxArray_int32_T_646
#define struct_emxArray_int32_T_646
struct emxArray_int32_T_646 {
  int32_T data[646];
  int32_T size[1];
};
#endif /* struct_emxArray_int32_T_646 */
#ifndef typedef_emxArray_int32_T_646
#define typedef_emxArray_int32_T_646
typedef struct emxArray_int32_T_646 emxArray_int32_T_646;
#endif /* typedef_emxArray_int32_T_646 */

#ifndef struct_emxArray_real_T_795226
#define struct_emxArray_real_T_795226
struct emxArray_real_T_795226 {
  real_T data[795226];
  int32_T size[1];
};
#endif /* struct_emxArray_real_T_795226 */
#ifndef typedef_emxArray_real_T_795226
#define typedef_emxArray_real_T_795226
typedef struct emxArray_real_T_795226 emxArray_real_T_795226;
#endif /* typedef_emxArray_real_T_795226 */

#ifndef struct_emxArray_boolean_T_1231
#define struct_emxArray_boolean_T_1231
struct emxArray_boolean_T_1231 {
  boolean_T data[1231];
  int32_T size[1];
};
#endif /* struct_emxArray_boolean_T_1231 */
#ifndef typedef_emxArray_boolean_T_1231
#define typedef_emxArray_boolean_T_1231
typedef struct emxArray_boolean_T_1231 emxArray_boolean_T_1231;
#endif /* typedef_emxArray_boolean_T_1231 */

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
  emxArray_real_T_284240 Aineq;
  emxArray_real_T_440 bineq;
  emxArray_real_T_38760 Aeq;
  real_T beq[60];
  emxArray_real_T_646 lb;
  emxArray_real_T_646 ub;
  emxArray_int32_T_646 indexLB;
  emxArray_int32_T_646 indexUB;
  emxArray_int32_T_646 indexFixed;
  int32_T mEqRemoved;
  int32_T indexEqRemoved[60];
  emxArray_real_T_795226 ATwset;
  emxArray_real_T_1231 bwset;
  int32_T nActiveConstr;
  emxArray_real_T_1231 maxConstrWorkspace;
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
  emxArray_boolean_T_1231 isActiveConstr;
  emxArray_int32_T_1231 Wid;
  emxArray_int32_T_1231 Wlocalidx;
  int32_T nWConstr[5];
  int32_T probType;
  real_T SLACK0;
} h_struct_T;
#endif /* typedef_h_struct_T */

#ifndef struct_emxArray_real_T_1231x1231
#define struct_emxArray_real_T_1231x1231
struct emxArray_real_T_1231x1231 {
  real_T data[1515361];
  int32_T size[2];
};
#endif /* struct_emxArray_real_T_1231x1231 */
#ifndef typedef_emxArray_real_T_1231x1231
#define typedef_emxArray_real_T_1231x1231
typedef struct emxArray_real_T_1231x1231 emxArray_real_T_1231x1231;
#endif /* typedef_emxArray_real_T_1231x1231 */

#ifndef typedef_i_struct_T
#define typedef_i_struct_T
typedef struct {
  int32_T ldq;
  emxArray_real_T_1231x1231 QR;
  emxArray_real_T_1231x1231 Q;
  emxArray_int32_T_1231 jpvt;
  int32_T mrows;
  int32_T ncols;
  emxArray_real_T_1231 tau;
  int32_T minRowCol;
  boolean_T usedPivoting;
} i_struct_T;
#endif /* typedef_i_struct_T */

#ifndef typedef_j_struct_T
#define typedef_j_struct_T
typedef struct {
  emxArray_real_T_1231x1231 FMat;
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
  real_T y_data[1515361];
} b_saveJacobian;
#endif /* typedef_b_saveJacobian */

#ifndef typedef_b_outputBounds
#define typedef_b_outputBounds
typedef struct {
  real_T tmp_data[25600];
  real_T y_data[7680];
} b_outputBounds;
#endif /* typedef_b_outputBounds */

#ifndef typedef_b_feasibleX0ForWorkingSet
#define typedef_b_feasibleX0ForWorkingSet
typedef struct {
  real_T workspace_data[795226];
} b_feasibleX0ForWorkingSet;
#endif /* typedef_b_feasibleX0ForWorkingSet */

#ifndef typedef_d_nlmpcmoveCodeGeneration_anonF
#define typedef_d_nlmpcmoveCodeGeneration_anonF
typedef struct {
  real_T Jc_data[10200];
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
  real_T varargout_3_data[10200];
  real_T varargout_4[5100];
} b_evalObjAndConstrAndDerivative;
#endif /* typedef_b_evalObjAndConstrAndDerivative */

#ifndef typedef_b_checkNonlinearInputs
#define typedef_b_checkNonlinearInputs
typedef struct {
  real_T varargout_3_data[10200];
  real_T varargout_4[5100];
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
  real_T unusedExpr[7225];
} b_fmincon;
#endif /* typedef_b_fmincon */

#ifndef typedef_b_nlmpcmoveCodeGeneration
#define typedef_b_nlmpcmoveCodeGeneration
typedef struct {
  real_T A_data[45120];
  real_T Au[25600];
  real_T Auf_data[25600];
  real_T y_data[7680];
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
