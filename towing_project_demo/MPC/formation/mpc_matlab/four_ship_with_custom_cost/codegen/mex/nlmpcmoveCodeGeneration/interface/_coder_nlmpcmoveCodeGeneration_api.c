/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_nlmpcmoveCodeGeneration_api.c
 *
 * Code generation for function '_coder_nlmpcmoveCodeGeneration_api'
 *
 */

/* Include files */
#include "_coder_nlmpcmoveCodeGeneration_api.h"
#include "nlmpcmoveCodeGeneration.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *x,
                                   const char_T *identifier))[24];

static const mxArray *b_emlrt_marshallOut(const struct1_T *u);

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[24];

static const mxArray *c_emlrt_marshallOut(const real_T u[192]);

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *onlinedata,
                               const char_T *identifier, struct1_T *y);

static const mxArray *d_emlrt_marshallOut(const struct2_T *u);

static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct1_T *y);

static const mxArray *e_emlrt_marshallOut(const real_T u[216]);

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId);

static const mxArray *emlrt_marshallOut(const real_T u[24]);

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[192]);

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[24]);

static real_T h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static real_T (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[24];

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real_T ret[192]);

static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[24]);

/* Function Definitions */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *x,
                                   const char_T *identifier))[24]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[24];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = c_emlrt_marshallIn(sp, emlrtAlias(x), &thisId);
  emlrtDestroyArray(&x);
  return y;
}

static const mxArray *b_emlrt_marshallOut(const struct1_T *u)
{
  static const int32_T iv[2] = {1, 24};
  static const char_T *sv[5] = {"ref", "MVTarget", "X0", "MV0", "Slack0"};
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T i;
  y = NULL;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 5, (const char_T **)&sv[0]));
  emlrtSetFieldR2017b(y, 0, (const char_T *)"ref", c_emlrt_marshallOut(u->ref),
                      0);
  b_y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  for (i = 0; i < 24; i++) {
    pData[i] = u->MVTarget[i];
  }
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"MVTarget", b_y, 1);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"X0", c_emlrt_marshallOut(u->X0),
                      2);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"MV0", c_emlrt_marshallOut(u->MV0),
                      3);
  c_y = NULL;
  m = emlrtCreateDoubleScalar(u->Slack0);
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Slack0", c_y, 4);
  return y;
}

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[24]
{
  real_T(*y)[24];
  y = i_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *c_emlrt_marshallOut(const real_T u[192])
{
  static const int32_T iv[2] = {8, 24};
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T b_i;
  int32_T c_i;
  int32_T i;
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (b_i = 0; b_i < 24; b_i++) {
    for (c_i = 0; c_i < 8; c_i++) {
      pData[i + c_i] = u[c_i + (b_i << 3)];
    }
    i += 8;
  }
  emlrtAssign(&y, m);
  return y;
}

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *onlinedata,
                               const char_T *identifier, struct1_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  e_emlrt_marshallIn(sp, emlrtAlias(onlinedata), &thisId, y);
  emlrtDestroyArray(&onlinedata);
}

static const mxArray *d_emlrt_marshallOut(const struct2_T *u)
{
  static const int32_T i = 9;
  static const char_T *sv[8] = {"MVopt", "Xopt",     "Yopt",       "Topt",
                                "Slack", "ExitFlag", "Iterations", "Cost"};
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T b_i;
  y = NULL;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 8, (const char_T **)&sv[0]));
  emlrtSetFieldR2017b(y, 0, (const char_T *)"MVopt",
                      e_emlrt_marshallOut(u->MVopt), 0);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Xopt",
                      e_emlrt_marshallOut(u->Xopt), 1);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Yopt",
                      e_emlrt_marshallOut(u->Yopt), 2);
  b_y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  for (b_i = 0; b_i < 9; b_i++) {
    pData[b_i] = u->Topt[b_i];
  }
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Topt", b_y, 3);
  c_y = NULL;
  m = emlrtCreateDoubleScalar(u->Slack);
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Slack", c_y, 4);
  d_y = NULL;
  m = emlrtCreateDoubleScalar(u->ExitFlag);
  emlrtAssign(&d_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"ExitFlag", d_y, 5);
  e_y = NULL;
  m = emlrtCreateDoubleScalar(u->Iterations);
  emlrtAssign(&e_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Iterations", e_y, 6);
  f_y = NULL;
  m = emlrtCreateDoubleScalar(u->Cost);
  emlrtAssign(&f_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Cost", f_y, 7);
  return y;
}

static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, struct1_T *y)
{
  static const int32_T dims = 0;
  static const char_T *fieldNames[5] = {"ref", "MVTarget", "X0", "MV0",
                                        "Slack0"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtCTX)sp, parentId, u, 5,
                         (const char_T **)&fieldNames[0], 0U, (void *)&dims);
  thisId.fIdentifier = "ref";
  f_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 0,
                                                    (const char_T *)"ref")),
                     &thisId, y->ref);
  thisId.fIdentifier = "MVTarget";
  g_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b(
                         (emlrtCTX)sp, u, 0, 1, (const char_T *)"MVTarget")),
                     &thisId, y->MVTarget);
  thisId.fIdentifier = "X0";
  f_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 2,
                                                    (const char_T *)"X0")),
                     &thisId, y->X0);
  thisId.fIdentifier = "MV0";
  f_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 3,
                                                    (const char_T *)"MV0")),
                     &thisId, y->MV0);
  thisId.fIdentifier = "Slack0";
  y->Slack0 =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b(
                           (emlrtCTX)sp, u, 0, 4, (const char_T *)"Slack0")),
                       &thisId);
  emlrtDestroyArray(&u);
}

static const mxArray *e_emlrt_marshallOut(const real_T u[216])
{
  static const int32_T iv[2] = {9, 24};
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T b_i;
  int32_T c_i;
  int32_T i;
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (b_i = 0; b_i < 24; b_i++) {
    for (c_i = 0; c_i < 9; c_i++) {
      pData[i + c_i] = u[c_i + 9 * b_i];
    }
    i += 9;
  }
  emlrtAssign(&y, m);
  return y;
}

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = h_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *emlrt_marshallOut(const real_T u[24])
{
  static const int32_T i = 0;
  static const int32_T i1 = 24;
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[192])
{
  j_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[24])
{
  k_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static real_T h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 0U, (void *)&dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[24]
{
  static const int32_T dims = 24;
  real_T(*ret)[24];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 1U, (void *)&dims);
  ret = (real_T(*)[24])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[192])
{
  static const int32_T dims[2] = {8, 24};
  real_T(*r)[192];
  int32_T i;
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)&dims[0]);
  r = (real_T(*)[192])emlrtMxGetData(src);
  for (i = 0; i < 192; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[24])
{
  static const int32_T dims[2] = {1, 24};
  real_T(*r)[24];
  int32_T i;
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)&dims[0]);
  r = (real_T(*)[24])emlrtMxGetData(src);
  for (i = 0; i < 24; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

void nlmpcmoveCodeGeneration_api(c_nlmpcmoveCodeGenerationStackD *SD,
                                 const mxArray *const prhs[4], int32_T nlhs,
                                 const mxArray *plhs[3])
{
  static const uint32_T uv[4] = {1333214118U, 648558870U, 3980707717U,
                                 1508077202U};
  static const char_T *s = "coredata";
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  struct1_T onlinedata;
  struct2_T info;
  real_T(*lastMV)[24];
  real_T(*mv)[24];
  real_T(*x)[24];
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  mv = (real_T(*)[24])mxMalloc(sizeof(real_T[24]));
  /* Check constant function inputs */
  i = 4;
  emlrtCheckArrayChecksumR2018b(&st, prhs[0], false, &i, (const char_T **)&s,
                                &uv[0]);
  /* Marshall function inputs */
  x = b_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "x");
  lastMV = b_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "lastMV");
  d_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "onlinedata", &onlinedata);
  /* Invoke the target function */
  nlmpcmoveCodeGeneration(SD, &st, *x, *lastMV, &onlinedata, *mv, &info);
  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*mv);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(&onlinedata);
  }
  if (nlhs > 2) {
    plhs[2] = d_emlrt_marshallOut(&info);
  }
}

/* End of code generation (_coder_nlmpcmoveCodeGeneration_api.c) */
