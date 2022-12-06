/*
 * OneShipStateJacobianFcn.c
 *
 * Code generation for function 'OneShipStateJacobianFcn'
 *
 */

/* Include files */
#include "OneShipStateJacobianFcn.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void OneShipStateJacobianFcn(const real_T in1[6], const real_T in2[8],
                             real_T A[36], real_T B[48])
{
  real_T t10;
  real_T t11;
  real_T t2;
  real_T t3;
  real_T t4;
  real_T t5;
  real_T t6;
  real_T t7;
  real_T t8;
  real_T t9;
  /* OneShipStateJacobianFcn */
  /*     [A,B] = OneShipStateJacobianFcn(IN1,IN2) */
  /*     This function was generated by the Symbolic Math Toolbox version 9.1.
   */
  /*     22-Nov-2022 20:29:28 */
  t2 = muDoubleScalarCos(in2[4]);
  t3 = muDoubleScalarCos(in2[5]);
  t4 = muDoubleScalarCos(in2[6]);
  t5 = muDoubleScalarCos(in2[7]);
  t6 = muDoubleScalarCos(in1[2]);
  t7 = muDoubleScalarSin(in2[4]);
  t8 = muDoubleScalarSin(in2[5]);
  t9 = muDoubleScalarSin(in2[6]);
  t10 = muDoubleScalarSin(in2[7]);
  t11 = muDoubleScalarSin(in1[2]);
  memset(&A[0], 0, 12U * sizeof(real_T));
  A[12] = -t11 * in1[3] - t6 * in1[4];
  A[13] = t6 * in1[3] - t11 * in1[4];
  A[14] = 0.0;
  A[15] = 0.0;
  A[16] = 0.0;
  A[17] = 0.0;
  A[18] = t6;
  A[19] = t11;
  A[20] = 0.0;
  A[21] = -0.04;
  A[22] = in1[5] * -0.70930232558139539;
  A[23] = in1[4] * -0.0002259131409155808;
  A[24] = -t11;
  A[25] = t6;
  A[26] = 0.0;
  A[27] = in1[5] * 1.40983606557377;
  A[28] = -0.05674418604651163;
  A[29] = in1[3] * -0.0002259131409155808;
  A[30] = 0.0;
  A[31] = 0.0;
  A[32] = 1.0;
  A[33] = in1[4] * 1.40983606557377;
  A[34] = in1[3] * -0.70930232558139539;
  A[35] = -0.070304169452928736;
  B[0] = 0.0;
  B[1] = 0.0;
  B[2] = 0.0;
  B[3] = t2 * 1.6393442622950821E-5;
  B[4] = t7 * 1.162790697674419E-5;
  B[5] = t2 * -4.5182628183116151E-7 + t7 * 6.7773942274674229E-7;
  B[6] = 0.0;
  B[7] = 0.0;
  B[8] = 0.0;
  B[9] = t3 * 1.6393442622950821E-5;
  B[10] = t8 * 1.162790697674419E-5;
  B[11] = t3 * -4.5182628183116151E-7 - t8 * 6.7773942274674229E-7;
  B[12] = 0.0;
  B[13] = 0.0;
  B[14] = 0.0;
  B[15] = t4 * 1.6393442622950821E-5;
  B[16] = t9 * 1.162790697674419E-5;
  B[17] = t4 * 4.5182628183116151E-7 - t9 * 6.7773942274674229E-7;
  B[18] = 0.0;
  B[19] = 0.0;
  B[20] = 0.0;
  B[21] = t5 * 1.6393442622950821E-5;
  B[22] = t10 * 1.162790697674419E-5;
  B[23] = t5 * 4.5182628183116151E-7 + t10 * 6.7773942274674229E-7;
  B[24] = 0.0;
  B[25] = 0.0;
  B[26] = 0.0;
  B[27] = in2[0] * t7 * -1.6393442622950821E-5;
  B[28] = in2[0] * t2 * 1.162790697674419E-5;
  B[29] = in2[0] * (t2 * 75.0 + t7 * 50.0) * 9.03652563662323E-9;
  B[30] = 0.0;
  B[31] = 0.0;
  B[32] = 0.0;
  B[33] = in2[1] * t8 * -1.6393442622950821E-5;
  B[34] = in2[1] * t3 * 1.162790697674419E-5;
  B[35] = in2[1] * (t3 * 75.0 - t8 * 50.0) * -9.03652563662323E-9;
  B[36] = 0.0;
  B[37] = 0.0;
  B[38] = 0.0;
  B[39] = in2[2] * t9 * -1.6393442622950821E-5;
  B[40] = in2[2] * t4 * 1.162790697674419E-5;
  B[41] = in2[2] * (t4 * 75.0 + t9 * 50.0) * -9.03652563662323E-9;
  B[42] = 0.0;
  B[43] = 0.0;
  B[44] = 0.0;
  B[45] = in2[3] * t10 * -1.6393442622950821E-5;
  B[46] = in2[3] * t5 * 1.162790697674419E-5;
  B[47] = in2[3] * (t5 * 75.0 - t10 * 50.0) * 9.03652563662323E-9;
}

/* End of code generation (OneShipStateJacobianFcn.c) */
