/*
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
                                   const char_T *identifier))[6];

static const mxArray *b_emlrt_marshallOut(const struct1_T *u);

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[6];

static const mxArray *c_emlrt_marshallOut(const struct2_T *u);

static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *lastMV,
                                   const char_T *identifier))[8];

static const mxArray *d_emlrt_marshallOut(const real_T u[66]);

static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[8];

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId);

static const mxArray *emlrt_marshallOut(const real_T u[8]);

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *onlinedata,
                               const char_T *identifier, struct1_T *y);

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct1_T *y);

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[6]);

static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[8]);

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[60]);

static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[80]);

static real_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static real_T (*m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[6];

static real_T (*n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[8];

static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[6]);

static void p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[8]);

static void q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[60]);

static void r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[80]);

/* Function Definitions */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *x,
                                   const char_T *identifier))[6]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[6];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = c_emlrt_marshallIn(sp, emlrtAlias(x), &thisId);
  emlrtDestroyArray(&x);
  return y;
}

static const mxArray *b_emlrt_marshallOut(const struct1_T *u)
{
  static const int32_T iv[2] = {1, 6};
  static const int32_T iv1[2] = {1, 8};
  static const int32_T iv2[2] = {10, 6};
  static const int32_T iv3[2] = {10, 8};
  static const char_T *sv[5] = {"ref", "MVTarget", "X0", "MV0", "Slack0"};
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T b_i;
  int32_T c_i;
  int32_T i;
  y = NULL;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 5, (const char_T **)&sv[0]));
  b_y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  for (i = 0; i < 6; i++) {
    pData[i] = u->ref[i];
  }
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"ref", b_y, 0);
  c_y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv1[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  for (i = 0; i < 8; i++) {
    pData[i] = u->MVTarget[i];
  }
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"MVTarget", c_y, 1);
  d_y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv2[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  b_i = 0;
  for (i = 0; i < 6; i++) {
    for (c_i = 0; c_i < 10; c_i++) {
      pData[b_i + c_i] = u->X0[c_i + 10 * i];
    }
    b_i += 10;
  }
  emlrtAssign(&d_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"X0", d_y, 2);
  e_y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv3[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  b_i = 0;
  for (i = 0; i < 8; i++) {
    for (c_i = 0; c_i < 10; c_i++) {
      pData[b_i + c_i] = u->MV0[c_i + 10 * i];
    }
    b_i += 10;
  }
  emlrtAssign(&e_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"MV0", e_y, 3);
  f_y = NULL;
  m = emlrtCreateDoubleScalar(u->Slack0);
  emlrtAssign(&f_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Slack0", f_y, 4);
  return y;
}

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[6]
{
  real_T(*y)[6];
  y = m_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *c_emlrt_marshallOut(const struct2_T *u)
{
  static const int32_T iv[2] = {11, 8};
  static const int32_T i1 = 11;
  static const char_T *sv[8] = {"MVopt", "Xopt",     "Yopt",       "Topt",
                                "Slack", "ExitFlag", "Iterations", "Cost"};
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *g_y;
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T b_i;
  int32_T c_i;
  int32_T i;
  y = NULL;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 8, (const char_T **)&sv[0]));
  b_y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (b_i = 0; b_i < 8; b_i++) {
    for (c_i = 0; c_i < 11; c_i++) {
      pData[i + c_i] = u->MVopt[c_i + 11 * b_i];
    }
    i += 11;
  }
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"MVopt", b_y, 0);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Xopt",
                      d_emlrt_marshallOut(u->Xopt), 1);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Yopt",
                      d_emlrt_marshallOut(u->Yopt), 2);
  c_y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i1, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  for (b_i = 0; b_i < 11; b_i++) {
    pData[b_i] = u->Topt[b_i];
  }
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Topt", c_y, 3);
  d_y = NULL;
  m = emlrtCreateDoubleScalar(u->Slack);
  emlrtAssign(&d_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Slack", d_y, 4);
  e_y = NULL;
  m = emlrtCreateDoubleScalar(u->ExitFlag);
  emlrtAssign(&e_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"ExitFlag", e_y, 5);
  f_y = NULL;
  m = emlrtCreateDoubleScalar(u->Iterations);
  emlrtAssign(&f_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Iterations", f_y, 6);
  g_y = NULL;
  m = emlrtCreateDoubleScalar(u->Cost);
  emlrtAssign(&g_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Cost", g_y, 7);
  return y;
}

static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *lastMV,
                                   const char_T *identifier))[8]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[8];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = e_emlrt_marshallIn(sp, emlrtAlias(lastMV), &thisId);
  emlrtDestroyArray(&lastMV);
  return y;
}

static const mxArray *d_emlrt_marshallOut(const real_T u[66])
{
  static const int32_T iv[2] = {11, 6};
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
  for (b_i = 0; b_i < 6; b_i++) {
    for (c_i = 0; c_i < 11; c_i++) {
      pData[i + c_i] = u[c_i + 11 * b_i];
    }
    i += 11;
  }
  emlrtAssign(&y, m);
  return y;
}

static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[8]
{
  real_T(*y)[8];
  y = n_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = l_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *emlrt_marshallOut(const real_T u[8])
{
  static const int32_T i = 0;
  static const int32_T i1 = 8;
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *onlinedata,
                               const char_T *identifier, struct1_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  g_emlrt_marshallIn(sp, emlrtAlias(onlinedata), &thisId, y);
  emlrtDestroyArray(&onlinedata);
}

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
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
  h_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 0,
                                                    (const char_T *)"ref")),
                     &thisId, y->ref);
  thisId.fIdentifier = "MVTarget";
  i_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b(
                         (emlrtCTX)sp, u, 0, 1, (const char_T *)"MVTarget")),
                     &thisId, y->MVTarget);
  thisId.fIdentifier = "X0";
  j_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 2,
                                                    (const char_T *)"X0")),
                     &thisId, y->X0);
  thisId.fIdentifier = "MV0";
  k_emlrt_marshallIn(sp,
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

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[6])
{
  o_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[8])
{
  p_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[60])
{
  q_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[80])
{
  r_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static real_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
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

static real_T (*m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[6]
{
  static const int32_T dims = 6;
  real_T(*ret)[6];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 1U, (void *)&dims);
  ret = (real_T(*)[6])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[8]
{
  static const int32_T dims = 8;
  real_T(*ret)[8];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 1U, (void *)&dims);
  ret = (real_T(*)[8])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[6])
{
  static const int32_T dims[2] = {1, 6};
  real_T(*r)[6];
  int32_T i;
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)&dims[0]);
  r = (real_T(*)[6])emlrtMxGetData(src);
  for (i = 0; i < 6; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

static void p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[8])
{
  static const int32_T dims[2] = {1, 8};
  real_T(*r)[8];
  int32_T i;
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)&dims[0]);
  r = (real_T(*)[8])emlrtMxGetData(src);
  for (i = 0; i < 8; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

static void q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[60])
{
  static const int32_T dims[2] = {10, 6};
  real_T(*r)[60];
  int32_T i;
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)&dims[0]);
  r = (real_T(*)[60])emlrtMxGetData(src);
  for (i = 0; i < 60; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

static void r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[80])
{
  static const int32_T dims[2] = {10, 8};
  real_T(*r)[80];
  int32_T i;
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)&dims[0]);
  r = (real_T(*)[80])emlrtMxGetData(src);
  for (i = 0; i < 80; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

void nlmpcmoveCodeGeneration_api(c_nlmpcmoveCodeGenerationStackD *SD,
                                 const mxArray *const prhs[4], int32_T nlhs,
                                 const mxArray *plhs[3])
{
  static const uint32_T uv[4] = {2577099784U, 62879347U, 1283356758U,
                                 4103295441U};
  static const char_T *s = "coredata";
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  struct1_T onlinedata;
  struct2_T info;
  real_T(*lastMV)[8];
  real_T(*mv)[8];
  real_T(*x)[6];
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  mv = (real_T(*)[8])mxMalloc(sizeof(real_T[8]));
  /* Check constant function inputs */
  i = 4;
  emlrtCheckArrayChecksumR2018b(&st, prhs[0], false, &i, (const char_T **)&s,
                                &uv[0]);
  /* Marshall function inputs */
  x = b_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "x");
  lastMV = d_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "lastMV");
  f_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "onlinedata", &onlinedata);
  /* Invoke the target function */
  nlmpcmoveCodeGeneration(SD, &st, *x, *lastMV, &onlinedata, *mv, &info);
  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*mv);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(&onlinedata);
  }
  if (nlhs > 2) {
    plhs[2] = c_emlrt_marshallOut(&info);
  }
}

/* End of code generation (_coder_nlmpcmoveCodeGeneration_api.c) */
