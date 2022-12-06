/*
 * nlmpcmoveCodeGeneration_internal_types.h
 *
 * Code generation for function 'nlmpcmoveCodeGeneration'
 *
 */

#pragma once

/* Include files */
#include "nlmpcmoveCodeGeneration_types.h"
#include "rtwtypes.h"
#include "emlrt.h"

/* Type Definitions */
#ifndef typedef_struct_T
#define typedef_struct_T
typedef struct {
  real_T penaltyParam;
  real_T threshold;
  int32_T nPenaltyDecreases;
  real_T linearizedConstrViol;
  real_T initFval;
  real_T initConstrViolationEq;
  real_T initConstrViolationIneq;
  real_T phi;
  real_T phiPrimePlus;
  real_T phiFullStep;
  real_T feasRelativeFactor;
  real_T nlpPrimalFeasError;
  real_T nlpDualFeasError;
  real_T nlpComplError;
  real_T firstOrderOpt;
  boolean_T hasObjective;
} struct_T;
#endif /* typedef_struct_T */

#ifndef typedef_b_struct_T
#define typedef_b_struct_T
typedef struct {
  boolean_T gradOK;
  boolean_T fevalOK;
  boolean_T done;
  boolean_T stepAccepted;
  boolean_T failedLineSearch;
  int32_T stepType;
} b_struct_T;
#endif /* typedef_b_struct_T */

#ifndef typedef_c_struct_T
#define typedef_c_struct_T
typedef struct {
  real_T iterations;
  real_T funcCount;
  char_T algorithm[3];
  real_T constrviolation;
  real_T stepsize;
  real_T lssteplength;
  real_T firstorderopt;
} c_struct_T;
#endif /* typedef_c_struct_T */

#ifndef typedef_d_struct_T
#define typedef_d_struct_T
typedef struct {
  char_T SolverName[7];
  int32_T MaxIterations;
  real_T StepTolerance;
  real_T OptimalityTolerance;
  real_T ConstraintTolerance;
  real_T ObjectiveLimit;
  real_T PricingTolerance;
  real_T ConstrRelTolFactor;
  real_T ProbRelTolFactor;
  boolean_T RemainFeasible;
  boolean_T IterDisplayQP;
} d_struct_T;
#endif /* typedef_d_struct_T */

#ifndef struct_emxArray_real_T_645
#define struct_emxArray_real_T_645
struct emxArray_real_T_645 {
  real_T data[645];
  int32_T size[1];
};
#endif /* struct_emxArray_real_T_645 */
#ifndef typedef_emxArray_real_T_645
#define typedef_emxArray_real_T_645
typedef struct emxArray_real_T_645 emxArray_real_T_645;
#endif /* typedef_emxArray_real_T_645 */

#ifndef typedef_f_struct_T
#define typedef_f_struct_T
typedef struct {
  emxArray_real_T_646 grad;
  emxArray_real_T_645 Hx;
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

#ifndef typedef_k_struct_T
#define typedef_k_struct_T
typedef struct {
  real_T Ts;
  real_T CurrentStates[6];
  real_T LastMV[8];
  real_T References[60];
  real_T MVTarget[80];
  real_T PredictionHorizon;
  real_T NumOfStates;
  real_T NumOfOutputs;
  real_T NumOfInputs;
  real_T MVIndex[8];
} k_struct_T;
#endif /* typedef_k_struct_T */

#ifndef typedef_l_struct_T
#define typedef_l_struct_T
typedef struct {
  real_T x[6];
  real_T lastMV[8];
  real_T ref[60];
  real_T OutputWeights[60];
  real_T MVWeights[80];
  real_T MVRateWeights[80];
  real_T ECRWeight;
  real_T OutputMin[60];
  real_T OutputMax[60];
  real_T StateMin[60];
  real_T StateMax[60];
  real_T MVMin[80];
  real_T MVMax[80];
  real_T MVRateMin[80];
  real_T MVRateMax[80];
  real_T MVScaledTarget[80];
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

#ifndef typedef_rtDesignRangeCheckInfo
#define typedef_rtDesignRangeCheckInfo
typedef struct {
  int32_T lineNo;
  int32_T colNo;
  const char_T *fName;
  const char_T *pName;
} rtDesignRangeCheckInfo;
#endif /* typedef_rtDesignRangeCheckInfo */

#ifndef typedef_rtDoubleCheckInfo
#define typedef_rtDoubleCheckInfo
typedef struct {
  int32_T lineNo;
  int32_T colNo;
  const char_T *fName;
  const char_T *pName;
  int32_T checkKind;
} rtDoubleCheckInfo;
#endif /* typedef_rtDoubleCheckInfo */

#ifndef typedef_rtRunTimeErrorInfo
#define typedef_rtRunTimeErrorInfo
typedef struct {
  int32_T lineNo;
  int32_T colNo;
  const char_T *fName;
  const char_T *pName;
} rtRunTimeErrorInfo;
#endif /* typedef_rtRunTimeErrorInfo */

/* End of code generation (nlmpcmoveCodeGeneration_internal_types.h) */
