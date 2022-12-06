/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_update_mv_api.c
 *
 * Code generation for function '_coder_update_mv_api'
 *
 */

/* Include files */
#include "_coder_update_mv_api.h"
#include "rt_nonfinite.h"
#include "update_mv.h"
#include "update_mv_data.h"
#include "update_mv_emxutil.h"

/* Variable Definitions */
static emlrtRTEInfo qb_emlrtRTEI = { 1,/* lineNo */
  1,                                   /* colNo */
  "_coder_update_mv_api",              /* fName */
  ""                                   /* pName */
};

/* Function Declarations */
static void ab_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T **ret_data, int32_T ret_size[1]);
static const mxArray *b_emlrt_marshallOut(const real_T u[48]);
static void bb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[48]);
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *mv, const
  char_T *identifier, struct0_T y[48]);
static void cb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[11520]);
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct0_T y[48]);
static void db_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[12]);
static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static void eb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[9]);
static const mxArray *emlrt_marshallOut(const emxArray_real_T *u);
static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static void fb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[6]);
static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[4]);
static void gb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[144]);
static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *subset,
  const char_T *identifier, real_T **y_data, int32_T y_size[1]);
static void hb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[240]);
static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T **y_data, int32_T y_size[1]);
static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *param, const
  char_T *identifier, struct1_T *y);
static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct1_T *y);
static void l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[48]);
static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct2_T *y);
static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[11520]);
static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct3_T *y);
static void p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[12]);
static void q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[9]);
static void r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[6]);
static void s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct4_T *y);
static void t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[144]);
static void u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[240]);
static real_T w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static void x_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static void y_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[4]);

/* Function Definitions */
static void ab_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T **ret_data, int32_T ret_size[1])
{
  static const int32_T dims[1] = { 48 };

  const boolean_T bv[1] = { true };

  int32_T iv[1];
  emlrtCheckVsBuiltInR2012b(sp, msgId, src, "double", false, 1U, dims, &bv[0],
    iv);
  ret_size[0] = iv[0];
  *ret_data = (real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
}

static const mxArray *b_emlrt_marshallOut(const real_T u[48])
{
  const mxArray *y;
  const mxArray *m;
  static const int32_T iv[1] = { 0 };

  static const int32_T iv1[1] = { 48 };

  y = NULL;
  m = emlrtCreateNumericArray(1, &iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, iv1, 1);
  emlrtAssign(&y, m);
  return y;
}

static void bb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[48])
{
  static const int32_T dims[1] = { 48 };

  real_T (*r)[48];
  int32_T i;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, dims);
  r = (real_T (*)[48])emlrtMxGetData(src);
  for (i = 0; i < 48; i++) {
    ret[i] = (*r)[i];
  }

  emlrtDestroyArray(&src);
}

static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *mv, const
  char_T *identifier, struct0_T y[48])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  d_emlrt_marshallIn(sp, emlrtAlias(mv), &thisId, y);
  emlrtDestroyArray(&mv);
}

static void cb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[11520])
{
  static const int32_T dims[2] = { 240, 48 };

  real_T (*r)[11520];
  int32_T i;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims);
  r = (real_T (*)[11520])emlrtMxGetData(src);
  for (i = 0; i < 11520; i++) {
    ret[i] = (*r)[i];
  }

  emlrtDestroyArray(&src);
}

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct0_T y[48])
{
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[40] = { "pmin", "pmax", "alpha", "p",
    "beta_plus", "beta_moins", "alpha_min", "param", "pbarmin", "pbarmax", "pc",
    "beta", "Jpbarmin", "gpbarmin", "Jpc", "gpc", "Jpbarmax", "gpbarmax", "aJ",
    "bJ", "cJ", "ag", "bg", "cg", "ps_J", "ps_J_star", "qJstar", "qJmin",
    "pJmin", "qJmax", "pJmax", "ps_g", "ps_g_star", "qgstar", "qgmin", "pgmin",
    "qgmax", "pgmax", "Z", "ngz" };

  static const int32_T dims[2] = { 1, 48 };

  int32_T i;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b(sp, parentId, u, 40, fieldNames, 2U, dims);
  for (i = 0; i < 48; i++) {
    thisId.fIdentifier = "pmin";
    y[i].pmin = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      0, "pmin")), &thisId);
    thisId.fIdentifier = "pmax";
    y[i].pmax = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      1, "pmax")), &thisId);
    thisId.fIdentifier = "alpha";
    y[i].alpha = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      2, "alpha")), &thisId);
    thisId.fIdentifier = "p";
    y[i].p = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 3,
      "p")), &thisId);
    thisId.fIdentifier = "beta_plus";
    y[i].beta_plus = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
      i, 4, "beta_plus")), &thisId);
    thisId.fIdentifier = "beta_moins";
    y[i].beta_moins = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp,
      u, i, 5, "beta_moins")), &thisId);
    thisId.fIdentifier = "alpha_min";
    y[i].alpha_min = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
      i, 6, "alpha_min")), &thisId);
    thisId.fIdentifier = "param";
    f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 7, "param")),
                       &thisId);
    thisId.fIdentifier = "pbarmin";
    y[i].pbarmin = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
      i, 8, "pbarmin")), &thisId);
    thisId.fIdentifier = "pbarmax";
    y[i].pbarmax = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
      i, 9, "pbarmax")), &thisId);
    thisId.fIdentifier = "pc";
    y[i].pc = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 10,
      "pc")), &thisId);
    thisId.fIdentifier = "beta";
    y[i].beta = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      11, "beta")), &thisId);
    thisId.fIdentifier = "Jpbarmin";
    y[i].Jpbarmin = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
      i, 12, "Jpbarmin")), &thisId);
    thisId.fIdentifier = "gpbarmin";
    y[i].gpbarmin = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
      i, 13, "gpbarmin")), &thisId);
    thisId.fIdentifier = "Jpc";
    y[i].Jpc = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      14, "Jpc")), &thisId);
    thisId.fIdentifier = "gpc";
    y[i].gpc = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      15, "gpc")), &thisId);
    thisId.fIdentifier = "Jpbarmax";
    y[i].Jpbarmax = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
      i, 16, "Jpbarmax")), &thisId);
    thisId.fIdentifier = "gpbarmax";
    y[i].gpbarmax = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
      i, 17, "gpbarmax")), &thisId);
    thisId.fIdentifier = "aJ";
    y[i].aJ = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 18,
      "aJ")), &thisId);
    thisId.fIdentifier = "bJ";
    y[i].bJ = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 19,
      "bJ")), &thisId);
    thisId.fIdentifier = "cJ";
    y[i].cJ = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 20,
      "cJ")), &thisId);
    thisId.fIdentifier = "ag";
    y[i].ag = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 21,
      "ag")), &thisId);
    thisId.fIdentifier = "bg";
    y[i].bg = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 22,
      "bg")), &thisId);
    thisId.fIdentifier = "cg";
    y[i].cg = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 23,
      "cg")), &thisId);
    thisId.fIdentifier = "ps_J";
    y[i].ps_J = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      24, "ps_J")), &thisId);
    thisId.fIdentifier = "ps_J_star";
    y[i].ps_J_star = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
      i, 25, "ps_J_star")), &thisId);
    thisId.fIdentifier = "qJstar";
    y[i].qJstar = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      26, "qJstar")), &thisId);
    thisId.fIdentifier = "qJmin";
    y[i].qJmin = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      27, "qJmin")), &thisId);
    thisId.fIdentifier = "pJmin";
    y[i].pJmin = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      28, "pJmin")), &thisId);
    thisId.fIdentifier = "qJmax";
    y[i].qJmax = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      29, "qJmax")), &thisId);
    thisId.fIdentifier = "pJmax";
    y[i].pJmax = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      30, "pJmax")), &thisId);
    thisId.fIdentifier = "ps_g";
    y[i].ps_g = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      31, "ps_g")), &thisId);
    thisId.fIdentifier = "ps_g_star";
    y[i].ps_g_star = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
      i, 32, "ps_g_star")), &thisId);
    thisId.fIdentifier = "qgstar";
    y[i].qgstar = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      33, "qgstar")), &thisId);
    thisId.fIdentifier = "qgmin";
    y[i].qgmin = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      34, "qgmin")), &thisId);
    thisId.fIdentifier = "pgmin";
    y[i].pgmin = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      35, "pgmin")), &thisId);
    thisId.fIdentifier = "qgmax";
    y[i].qgmax = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      36, "qgmax")), &thisId);
    thisId.fIdentifier = "pgmax";
    y[i].pgmax = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      37, "pgmax")), &thisId);
    thisId.fIdentifier = "Z";
    g_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 38, "Z")),
                       &thisId, y[i].Z);
    thisId.fIdentifier = "ngz";
    y[i].ngz = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      39, "ngz")), &thisId);
  }

  emlrtDestroyArray(&u);
}

static void db_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[12])
{
  static const int32_T dims[1] = { 12 };

  real_T (*r)[12];
  int32_T i;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, dims);
  r = (real_T (*)[12])emlrtMxGetData(src);
  for (i = 0; i < 12; i++) {
    ret[i] = (*r)[i];
  }

  emlrtDestroyArray(&src);
}

static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = w_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void eb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[9])
{
  static const int32_T dims[2] = { 3, 3 };

  real_T (*r)[9];
  int32_T i;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims);
  r = (real_T (*)[9])emlrtMxGetData(src);
  for (i = 0; i < 9; i++) {
    ret[i] = (*r)[i];
  }

  emlrtDestroyArray(&src);
}

static const mxArray *emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *y;
  const mxArray *m;
  static const int32_T iv[2] = { 0, 0 };

  y = NULL;
  m = emlrtCreateNumericArray(2, &iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, &u->data[0]);
  emlrtSetDimensions((mxArray *)m, u->size, 2);
  emlrtAssign(&y, m);
  return y;
}

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  x_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
}

static void fb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[6])
{
  static const int32_T dims[2] = { 3, 2 };

  real_T (*r)[6];
  int32_T i;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims);
  r = (real_T (*)[6])emlrtMxGetData(src);
  for (i = 0; i < 6; i++) {
    ret[i] = (*r)[i];
  }

  emlrtDestroyArray(&src);
}

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[4])
{
  y_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void gb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[144])
{
  static const int32_T dims[2] = { 12, 12 };

  real_T (*r)[144];
  int32_T i;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims);
  r = (real_T (*)[144])emlrtMxGetData(src);
  for (i = 0; i < 144; i++) {
    ret[i] = (*r)[i];
  }

  emlrtDestroyArray(&src);
}

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *subset,
  const char_T *identifier, real_T **y_data, int32_T y_size[1])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  i_emlrt_marshallIn(sp, emlrtAlias(subset), &thisId, y_data, y_size);
  emlrtDestroyArray(&subset);
}

static void hb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[240])
{
  static const int32_T dims[2] = { 20, 12 };

  real_T (*r)[240];
  int32_T i;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims);
  r = (real_T (*)[240])emlrtMxGetData(src);
  for (i = 0; i < 240; i++) {
    ret[i] = (*r)[i];
  }

  emlrtDestroyArray(&src);
}

static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T **y_data, int32_T y_size[1])
{
  ab_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y_data, y_size);
  emlrtDestroyArray(&u);
}

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *param, const
  char_T *identifier, struct1_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  k_emlrt_marshallIn(sp, emlrtAlias(param), &thisId, y);
  emlrtDestroyArray(&param);
}

static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct1_T *y)
{
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[14] = { "pmin", "pmax", "p", "alpha", "ell",
    "uparam", "ode", "ocp", "x0", "mv", "compiled", "Nev", "Niter", "np" };

  static const int32_T dims = 0;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b(sp, parentId, u, 14, fieldNames, 0U, &dims);
  thisId.fIdentifier = "pmin";
  l_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 0, "pmin")),
                     &thisId, y->pmin);
  thisId.fIdentifier = "pmax";
  l_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 1, "pmax")),
                     &thisId, y->pmax);
  thisId.fIdentifier = "p";
  l_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 2, "p")),
                     &thisId, y->p);
  thisId.fIdentifier = "alpha";
  l_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 3, "alpha")),
                     &thisId, y->alpha);
  thisId.fIdentifier = "ell";
  y->ell = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 4,
    "ell")), &thisId);
  thisId.fIdentifier = "uparam";
  m_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 5, "uparam")),
                     &thisId, &y->uparam);
  thisId.fIdentifier = "ode";
  o_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 6, "ode")),
                     &thisId, &y->ode);
  thisId.fIdentifier = "ocp";
  s_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 7, "ocp")),
                     &thisId, &y->ocp);
  thisId.fIdentifier = "x0";
  p_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 8, "x0")),
                     &thisId, y->x0);
  thisId.fIdentifier = "mv";
  d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 9, "mv")),
                     &thisId, y->mv);
  thisId.fIdentifier = "compiled";
  y->compiled = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0,
    10, "compiled")), &thisId);
  thisId.fIdentifier = "Nev";
  y->Nev = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 11,
    "Nev")), &thisId);
  thisId.fIdentifier = "Niter";
  y->Niter = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 12,
    "Niter")), &thisId);
  thisId.fIdentifier = "np";
  y->np = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 13,
    "np")), &thisId);
  emlrtDestroyArray(&u);
}

static void l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[48])
{
  bb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct2_T *y)
{
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[8] = { "nu", "Np", "Ifree", "R", "np", "p",
    "pmin", "pmax" };

  static const int32_T dims = 0;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b(sp, parentId, u, 8, fieldNames, 0U, &dims);
  thisId.fIdentifier = "nu";
  y->nu = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 0,
    "nu")), &thisId);
  thisId.fIdentifier = "Np";
  y->Np = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 1,
    "Np")), &thisId);
  thisId.fIdentifier = "Ifree";
  g_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 2, "Ifree")),
                     &thisId, y->Ifree);
  thisId.fIdentifier = "R";
  n_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 3, "R")),
                     &thisId, y->R);
  thisId.fIdentifier = "np";
  y->np = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 4,
    "np")), &thisId);
  thisId.fIdentifier = "p";
  l_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 5, "p")),
                     &thisId, y->p);
  thisId.fIdentifier = "pmin";
  l_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 6, "pmin")),
                     &thisId, y->pmin);
  thisId.fIdentifier = "pmax";
  l_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 7, "pmax")),
                     &thisId, y->pmax);
  emlrtDestroyArray(&u);
}

static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[11520])
{
  cb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct3_T *y)
{
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[9] = { "Nship", "tau", "rk_order", "x0", "u0",
    "u_last", "M", "D", "thrust_config" };

  static const int32_T dims = 0;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b(sp, parentId, u, 9, fieldNames, 0U, &dims);
  thisId.fIdentifier = "Nship";
  y->Nship = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 0,
    "Nship")), &thisId);
  thisId.fIdentifier = "tau";
  y->tau = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 1,
    "tau")), &thisId);
  thisId.fIdentifier = "rk_order";
  y->rk_order = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0,
    2, "rk_order")), &thisId);
  thisId.fIdentifier = "x0";
  p_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 3, "x0")),
                     &thisId, y->x0);
  thisId.fIdentifier = "u0";
  p_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 4, "u0")),
                     &thisId, y->u0);
  thisId.fIdentifier = "u_last";
  p_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 5, "u_last")),
                     &thisId, y->u_last);
  thisId.fIdentifier = "M";
  q_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 6, "M")),
                     &thisId, y->M);
  thisId.fIdentifier = "D";
  q_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 7, "D")),
                     &thisId, y->D);
  thisId.fIdentifier = "thrust_config";
  r_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 8,
    "thrust_config")), &thisId, y->thrust_config);
  emlrtDestroyArray(&u);
}

static void p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[12])
{
  db_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[9])
{
  eb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[6])
{
  fb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct4_T *y)
{
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[7] = { "Q", "R", "Rdu", "xd_his", "F_max",
    "a_max", "u_last" };

  static const int32_T dims = 0;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b(sp, parentId, u, 7, fieldNames, 0U, &dims);
  thisId.fIdentifier = "Q";
  t_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 0, "Q")),
                     &thisId, y->Q);
  thisId.fIdentifier = "R";
  t_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 1, "R")),
                     &thisId, y->R);
  thisId.fIdentifier = "Rdu";
  t_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 2, "Rdu")),
                     &thisId, y->Rdu);
  thisId.fIdentifier = "xd_his";
  u_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 3, "xd_his")),
                     &thisId, y->xd_his);
  thisId.fIdentifier = "F_max";
  y->F_max = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 4,
    "F_max")), &thisId);
  thisId.fIdentifier = "a_max";
  y->a_max = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 5,
    "a_max")), &thisId);
  thisId.fIdentifier = "u_last";
  p_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 6, "u_last")),
                     &thisId, y->u_last);
  emlrtDestroyArray(&u);
}

static void t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[144])
{
  gb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[240])
{
  hb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static real_T w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  real_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, &dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static void x_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  static const int32_T dims[2] = { 0, 0 };

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims);
  emlrtMxGetData(src);
  emlrtDestroyArray(&src);
}

static void y_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[4])
{
  static const int32_T dims[1] = { 4 };

  real_T (*r)[4];
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, dims);
  r = (real_T (*)[4])emlrtMxGetData(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  ret[2] = (*r)[2];
  ret[3] = (*r)[3];
  emlrtDestroyArray(&src);
}

void update_mv_api(const mxArray * const prhs[3], int32_T nlhs, const mxArray
                   *plhs[3])
{
  real_T (*p_sol)[48];
  emxArray_real_T *u_sol;
  emxArray_real_T *lesp;
  struct0_T mv[48];
  real_T (*subset_data)[48];
  int32_T subset_size[1];
  struct1_T param;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  p_sol = (real_T (*)[48])mxMalloc(sizeof(real_T [48]));
  emlrtHeapReferenceStackEnterFcnR2012b(&st);
  emxInit_real_T(&st, &u_sol, 2, &qb_emlrtRTEI, true);
  emxInit_real_T(&st, &lesp, 2, &qb_emlrtRTEI, true);

  /* Marshall function inputs */
  c_emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "mv", mv);
  h_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "subset", (real_T **)&subset_data,
                     subset_size);
  j_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "param", &param);

  /* Invoke the target function */
  update_mv(&st, mv, *subset_data, subset_size, &param, u_sol, *p_sol, lesp);

  /* Marshall function outputs */
  u_sol->canFreeData = false;
  plhs[0] = emlrt_marshallOut(u_sol);
  emxFree_real_T(&u_sol);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(*p_sol);
  }

  if (nlhs > 2) {
    lesp->canFreeData = false;
    plhs[2] = emlrt_marshallOut(lesp);
  }

  emxFree_real_T(&lesp);
  emlrtHeapReferenceStackLeaveFcnR2012b(&st);
}

/* End of code generation (_coder_update_mv_api.c) */
