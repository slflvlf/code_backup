/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_BBS_api.c
 *
 * Code generation for function '_coder_BBS_api'
 *
 */

/* Include files */
#include "_coder_BBS_api.h"
#include "BBS.h"
#include "BBS_data.h"
#include "BBS_mexutil.h"
#include "inv.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void ab_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[9]);
static void bb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[6]);
static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *eta, const
  char_T *identifier);
static void cb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[144]);
static real_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static void db_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[240]);
static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *param, const
  char_T *identifier, struct0_T *y);
static void eb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct0_T *y);
static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[48]);
static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct1_T *y);
static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[4]);
static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[11520]);
static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct2_T *y);
static void l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[12]);
static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[9]);
static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[6]);
static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct3_T *y);
static void p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[144]);
static void q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[240]);
static void r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct4_T y[48]);
static void s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static real_T u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static void v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[48]);
static void w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[4]);
static void x_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[11520]);
static void y_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[12]);

/* Function Definitions */
static void ab_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
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

static void bb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
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

static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *eta, const
  char_T *identifier)
{
  real_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(eta), &thisId);
  emlrtDestroyArray(&eta);
  return y;
}

static void cb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
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

static real_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = u_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void db_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
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

static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *param, const
  char_T *identifier, struct0_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  f_emlrt_marshallIn(sp, emlrtAlias(param), &thisId, y);
  emlrtDestroyArray(&param);
}

static void eb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  static const int32_T dims[2] = { 0, 0 };

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims);
  emlrtMxGetData(src);
  emlrtDestroyArray(&src);
}

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct0_T *y)
{
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[14] = { "pmin", "pmax", "p", "alpha", "ell",
    "uparam", "ode", "ocp", "x0", "mv", "compiled", "Nev", "Niter", "np" };

  static const int32_T dims = 0;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b(sp, parentId, u, 14, fieldNames, 0U, &dims);
  thisId.fIdentifier = "pmin";
  g_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 0, "pmin")),
                     &thisId, y->pmin);
  thisId.fIdentifier = "pmax";
  g_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 1, "pmax")),
                     &thisId, y->pmax);
  thisId.fIdentifier = "p";
  g_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 2, "p")),
                     &thisId, y->p);
  thisId.fIdentifier = "alpha";
  g_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 3, "alpha")),
                     &thisId, y->alpha);
  thisId.fIdentifier = "ell";
  y->ell = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 4,
    "ell")), &thisId);
  thisId.fIdentifier = "uparam";
  h_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 5, "uparam")),
                     &thisId, &y->uparam);
  thisId.fIdentifier = "ode";
  k_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 6, "ode")),
                     &thisId, &y->ode);
  thisId.fIdentifier = "ocp";
  o_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 7, "ocp")),
                     &thisId, &y->ocp);
  thisId.fIdentifier = "x0";
  l_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 8, "x0")),
                     &thisId, y->x0);
  thisId.fIdentifier = "mv";
  r_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 9, "mv")),
                     &thisId, y->mv);
  thisId.fIdentifier = "compiled";
  y->compiled = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0,
    10, "compiled")), &thisId);
  thisId.fIdentifier = "Nev";
  y->Nev = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 11,
    "Nev")), &thisId);
  thisId.fIdentifier = "Niter";
  y->Niter = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 12,
    "Niter")), &thisId);
  thisId.fIdentifier = "np";
  y->np = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 13,
    "np")), &thisId);
  emlrtDestroyArray(&u);
}

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[48])
{
  v_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct1_T *y)
{
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[8] = { "nu", "Np", "Ifree", "R", "np", "p",
    "pmin", "pmax" };

  static const int32_T dims = 0;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b(sp, parentId, u, 8, fieldNames, 0U, &dims);
  thisId.fIdentifier = "nu";
  y->nu = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 0,
    "nu")), &thisId);
  thisId.fIdentifier = "Np";
  y->Np = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 1,
    "Np")), &thisId);
  thisId.fIdentifier = "Ifree";
  i_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 2, "Ifree")),
                     &thisId, y->Ifree);
  thisId.fIdentifier = "R";
  j_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 3, "R")),
                     &thisId, y->R);
  thisId.fIdentifier = "np";
  y->np = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 4,
    "np")), &thisId);
  thisId.fIdentifier = "p";
  g_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 5, "p")),
                     &thisId, y->p);
  thisId.fIdentifier = "pmin";
  g_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 6, "pmin")),
                     &thisId, y->pmin);
  thisId.fIdentifier = "pmax";
  g_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 7, "pmax")),
                     &thisId, y->pmax);
  emlrtDestroyArray(&u);
}

static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[4])
{
  w_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[11520])
{
  x_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct2_T *y)
{
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[8] = { "Nship", "tau", "rk_order", "x0", "u0",
    "M", "D", "thrust_config" };

  static const int32_T dims = 0;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b(sp, parentId, u, 8, fieldNames, 0U, &dims);
  thisId.fIdentifier = "Nship";
  y->Nship = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 0,
    "Nship")), &thisId);
  thisId.fIdentifier = "tau";
  y->tau = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 1,
    "tau")), &thisId);
  thisId.fIdentifier = "rk_order";
  y->rk_order = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0,
    2, "rk_order")), &thisId);
  thisId.fIdentifier = "x0";
  l_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 3, "x0")),
                     &thisId, y->x0);
  thisId.fIdentifier = "u0";
  l_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 4, "u0")),
                     &thisId, y->u0);
  thisId.fIdentifier = "M";
  m_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 5, "M")),
                     &thisId, y->M);
  thisId.fIdentifier = "D";
  m_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 6, "D")),
                     &thisId, y->D);
  thisId.fIdentifier = "thrust_config";
  n_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 7,
    "thrust_config")), &thisId, y->thrust_config);
  emlrtDestroyArray(&u);
}

static void l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[12])
{
  y_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[9])
{
  ab_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[6])
{
  bb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct3_T *y)
{
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[7] = { "Q", "R", "Rdu", "xd_his", "dF_max",
    "da_max", "u_last" };

  static const int32_T dims = 0;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b(sp, parentId, u, 7, fieldNames, 0U, &dims);
  thisId.fIdentifier = "Q";
  p_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 0, "Q")),
                     &thisId, y->Q);
  thisId.fIdentifier = "R";
  p_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 1, "R")),
                     &thisId, y->R);
  thisId.fIdentifier = "Rdu";
  p_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 2, "Rdu")),
                     &thisId, y->Rdu);
  thisId.fIdentifier = "xd_his";
  q_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 3, "xd_his")),
                     &thisId, y->xd_his);
  thisId.fIdentifier = "dF_max";
  y->dF_max = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 4,
    "dF_max")), &thisId);
  thisId.fIdentifier = "da_max";
  y->da_max = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 5,
    "da_max")), &thisId);
  thisId.fIdentifier = "u_last";
  l_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 6, "u_last")),
                     &thisId, y->u_last);
  emlrtDestroyArray(&u);
}

static void p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[144])
{
  cb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[240])
{
  db_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct4_T y[48])
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
    y[i].pmin = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      0, "pmin")), &thisId);
    thisId.fIdentifier = "pmax";
    y[i].pmax = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      1, "pmax")), &thisId);
    thisId.fIdentifier = "alpha";
    y[i].alpha = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      2, "alpha")), &thisId);
    thisId.fIdentifier = "p";
    y[i].p = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 3,
      "p")), &thisId);
    thisId.fIdentifier = "beta_plus";
    y[i].beta_plus = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
      i, 4, "beta_plus")), &thisId);
    thisId.fIdentifier = "beta_moins";
    y[i].beta_moins = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp,
      u, i, 5, "beta_moins")), &thisId);
    thisId.fIdentifier = "alpha_min";
    y[i].alpha_min = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
      i, 6, "alpha_min")), &thisId);
    thisId.fIdentifier = "param";
    s_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 7, "param")),
                       &thisId);
    thisId.fIdentifier = "pbarmin";
    y[i].pbarmin = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
      i, 8, "pbarmin")), &thisId);
    thisId.fIdentifier = "pbarmax";
    y[i].pbarmax = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
      i, 9, "pbarmax")), &thisId);
    thisId.fIdentifier = "pc";
    y[i].pc = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 10,
      "pc")), &thisId);
    thisId.fIdentifier = "beta";
    y[i].beta = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      11, "beta")), &thisId);
    thisId.fIdentifier = "Jpbarmin";
    y[i].Jpbarmin = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
      i, 12, "Jpbarmin")), &thisId);
    thisId.fIdentifier = "gpbarmin";
    y[i].gpbarmin = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
      i, 13, "gpbarmin")), &thisId);
    thisId.fIdentifier = "Jpc";
    y[i].Jpc = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      14, "Jpc")), &thisId);
    thisId.fIdentifier = "gpc";
    y[i].gpc = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      15, "gpc")), &thisId);
    thisId.fIdentifier = "Jpbarmax";
    y[i].Jpbarmax = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
      i, 16, "Jpbarmax")), &thisId);
    thisId.fIdentifier = "gpbarmax";
    y[i].gpbarmax = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
      i, 17, "gpbarmax")), &thisId);
    thisId.fIdentifier = "aJ";
    y[i].aJ = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 18,
      "aJ")), &thisId);
    thisId.fIdentifier = "bJ";
    y[i].bJ = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 19,
      "bJ")), &thisId);
    thisId.fIdentifier = "cJ";
    y[i].cJ = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 20,
      "cJ")), &thisId);
    thisId.fIdentifier = "ag";
    y[i].ag = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 21,
      "ag")), &thisId);
    thisId.fIdentifier = "bg";
    y[i].bg = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 22,
      "bg")), &thisId);
    thisId.fIdentifier = "cg";
    y[i].cg = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 23,
      "cg")), &thisId);
    thisId.fIdentifier = "ps_J";
    y[i].ps_J = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      24, "ps_J")), &thisId);
    thisId.fIdentifier = "ps_J_star";
    y[i].ps_J_star = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
      i, 25, "ps_J_star")), &thisId);
    thisId.fIdentifier = "qJstar";
    y[i].qJstar = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      26, "qJstar")), &thisId);
    thisId.fIdentifier = "qJmin";
    y[i].qJmin = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      27, "qJmin")), &thisId);
    thisId.fIdentifier = "pJmin";
    y[i].pJmin = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      28, "pJmin")), &thisId);
    thisId.fIdentifier = "qJmax";
    y[i].qJmax = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      29, "qJmax")), &thisId);
    thisId.fIdentifier = "pJmax";
    y[i].pJmax = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      30, "pJmax")), &thisId);
    thisId.fIdentifier = "ps_g";
    y[i].ps_g = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      31, "ps_g")), &thisId);
    thisId.fIdentifier = "ps_g_star";
    y[i].ps_g_star = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
      i, 32, "ps_g_star")), &thisId);
    thisId.fIdentifier = "qgstar";
    y[i].qgstar = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      33, "qgstar")), &thisId);
    thisId.fIdentifier = "qgmin";
    y[i].qgmin = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      34, "qgmin")), &thisId);
    thisId.fIdentifier = "pgmin";
    y[i].pgmin = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      35, "pgmin")), &thisId);
    thisId.fIdentifier = "qgmax";
    y[i].qgmax = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      36, "qgmax")), &thisId);
    thisId.fIdentifier = "pgmax";
    y[i].pgmax = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      37, "pgmax")), &thisId);
    thisId.fIdentifier = "Z";
    i_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i, 38, "Z")),
                       &thisId, y[i].Z);
    thisId.fIdentifier = "ngz";
    y[i].ngz = d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, i,
      39, "ngz")), &thisId);
  }

  emlrtDestroyArray(&u);
}

static void s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  eb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
}

static real_T u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  real_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, &dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static void v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
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

static void w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
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

static void x_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
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

static void y_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
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

void BBS_api(const mxArray * const prhs[2], int32_T nlhs, const mxArray *plhs[2])
{
  real_T eta;
  struct0_T param;
  real_T J;
  real_T g;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Marshall function inputs */
  eta = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "eta");
  e_emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "param", &param);

  /* Invoke the target function */
  BBS(&st, eta, &param, &J, &g);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(J);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(g);
  }
}

/* End of code generation (_coder_BBS_api.c) */
