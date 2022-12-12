/*
 * _coder_BBS_api.c
 *
 * Code generation for function '_coder_BBS_api'
 *
 */

/* Include files */
#include "_coder_BBS_api.h"
#include "BBS.h"
#include "BBS_data.h"
#include "BBS_types.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *param,
                               const char_T *identifier, struct0_T *y);

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct0_T *y);

static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[4]);

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *eta,
                               const char_T *identifier);

static const mxArray *emlrt_marshallOut(const real_T u);

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct1_T *y);

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[80]);

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct2_T *y);

static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[3]);

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct3_T *y);

static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[16]);

static void l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct4_T y[4]);

static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId);

static real_T n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[4]);

static void p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[80]);

static void q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[3]);

static void r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[16]);

static void s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId);

/* Function Definitions */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = n_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *param,
                               const char_T *identifier, struct0_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  d_emlrt_marshallIn(sp, emlrtAlias(param), &thisId, y);
  emlrtDestroyArray(&param);
}

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, struct0_T *y)
{
  static const int32_T dims = 0;
  static const char_T *fieldNames[14] = {
      "pmin", "pmax", "p",  "alpha",    "ell", "uparam", "ode",
      "ocp",  "x0",   "mv", "compiled", "Nev", "Niter",  "np"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtCTX)sp, parentId, u, 14,
                         (const char_T **)&fieldNames[0], 0U, (void *)&dims);
  thisId.fIdentifier = "pmin";
  e_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 0,
                                                    (const char_T *)"pmin")),
                     &thisId, y->pmin);
  thisId.fIdentifier = "pmax";
  e_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 1,
                                                    (const char_T *)"pmax")),
                     &thisId, y->pmax);
  thisId.fIdentifier = "p";
  e_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 2,
                                                    (const char_T *)"p")),
                     &thisId, y->p);
  thisId.fIdentifier = "alpha";
  e_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 3,
                                                    (const char_T *)"alpha")),
                     &thisId, y->alpha);
  thisId.fIdentifier = "ell";
  y->ell =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 4,
                                                        (const char_T *)"ell")),
                         &thisId);
  thisId.fIdentifier = "uparam";
  f_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 5,
                                                    (const char_T *)"uparam")),
                     &thisId, &y->uparam);
  thisId.fIdentifier = "ode";
  h_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 6,
                                                    (const char_T *)"ode")),
                     &thisId, &y->ode);
  thisId.fIdentifier = "ocp";
  j_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 7,
                                                    (const char_T *)"ocp")),
                     &thisId, &y->ocp);
  thisId.fIdentifier = "x0";
  e_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 8,
                                                    (const char_T *)"x0")),
                     &thisId, y->x0);
  thisId.fIdentifier = "mv";
  l_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 9,
                                                    (const char_T *)"mv")),
                     &thisId, y->mv);
  thisId.fIdentifier = "compiled";
  y->compiled = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 10,
                                     (const char_T *)"compiled")),
      &thisId);
  thisId.fIdentifier = "Nev";
  y->Nev =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 11,
                                                        (const char_T *)"Nev")),
                         &thisId);
  thisId.fIdentifier = "Niter";
  y->Niter =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 12, (const char_T *)"Niter")),
                         &thisId);
  thisId.fIdentifier = "np";
  y->np = b_emlrt_marshallIn(sp,
                             emlrtAlias(emlrtGetFieldR2017b(
                                 (emlrtCTX)sp, u, 0, 13, (const char_T *)"np")),
                             &thisId);
  emlrtDestroyArray(&u);
}

static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[4])
{
  o_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *eta,
                               const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(eta), &thisId);
  emlrtDestroyArray(&eta);
  return y;
}

static const mxArray *emlrt_marshallOut(const real_T u)
{
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m);
  return y;
}

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, struct1_T *y)
{
  static const int32_T dims = 0;
  static const char_T *fieldNames[8] = {"nu", "Np", "Ifree", "R",
                                        "np", "p",  "pmin",  "pmax"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtCTX)sp, parentId, u, 8,
                         (const char_T **)&fieldNames[0], 0U, (void *)&dims);
  thisId.fIdentifier = "nu";
  y->nu = b_emlrt_marshallIn(sp,
                             emlrtAlias(emlrtGetFieldR2017b(
                                 (emlrtCTX)sp, u, 0, 0, (const char_T *)"nu")),
                             &thisId);
  thisId.fIdentifier = "Np";
  y->Np = b_emlrt_marshallIn(sp,
                             emlrtAlias(emlrtGetFieldR2017b(
                                 (emlrtCTX)sp, u, 0, 1, (const char_T *)"Np")),
                             &thisId);
  thisId.fIdentifier = "Ifree";
  e_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 2,
                                                    (const char_T *)"Ifree")),
                     &thisId, y->Ifree);
  thisId.fIdentifier = "R";
  g_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 3,
                                                    (const char_T *)"R")),
                     &thisId, y->R);
  thisId.fIdentifier = "np";
  y->np = b_emlrt_marshallIn(sp,
                             emlrtAlias(emlrtGetFieldR2017b(
                                 (emlrtCTX)sp, u, 0, 4, (const char_T *)"np")),
                             &thisId);
  thisId.fIdentifier = "p";
  e_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 5,
                                                    (const char_T *)"p")),
                     &thisId, y->p);
  thisId.fIdentifier = "pmin";
  e_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 6,
                                                    (const char_T *)"pmin")),
                     &thisId, y->pmin);
  thisId.fIdentifier = "pmax";
  e_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 7,
                                                    (const char_T *)"pmax")),
                     &thisId, y->pmax);
  emlrtDestroyArray(&u);
}

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[80])
{
  p_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, struct2_T *y)
{
  static const int32_T dims = 0;
  static const char_T *fieldNames[5] = {"tau", "rk_order", "x0", "u0", "w"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtCTX)sp, parentId, u, 5,
                         (const char_T **)&fieldNames[0], 0U, (void *)&dims);
  thisId.fIdentifier = "tau";
  y->tau =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 0,
                                                        (const char_T *)"tau")),
                         &thisId);
  thisId.fIdentifier = "rk_order";
  y->rk_order = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 1,
                                     (const char_T *)"rk_order")),
      &thisId);
  thisId.fIdentifier = "x0";
  e_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 2,
                                                    (const char_T *)"x0")),
                     &thisId, y->x0);
  thisId.fIdentifier = "u0";
  y->u0 = b_emlrt_marshallIn(sp,
                             emlrtAlias(emlrtGetFieldR2017b(
                                 (emlrtCTX)sp, u, 0, 3, (const char_T *)"u0")),
                             &thisId);
  thisId.fIdentifier = "w";
  i_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 4,
                                                    (const char_T *)"w")),
                     &thisId, y->w);
  emlrtDestroyArray(&u);
}

static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[3])
{
  q_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, struct3_T *y)
{
  static const int32_T dims = 0;
  static const char_T *fieldNames[6] = {"Q",  "R",         "M",
                                        "rd", "theta_max", "thetap_max"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtCTX)sp, parentId, u, 6,
                         (const char_T **)&fieldNames[0], 0U, (void *)&dims);
  thisId.fIdentifier = "Q";
  k_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 0,
                                                    (const char_T *)"Q")),
                     &thisId, y->Q);
  thisId.fIdentifier = "R";
  y->R = b_emlrt_marshallIn(sp,
                            emlrtAlias(emlrtGetFieldR2017b(
                                (emlrtCTX)sp, u, 0, 1, (const char_T *)"R")),
                            &thisId);
  thisId.fIdentifier = "M";
  y->M = b_emlrt_marshallIn(sp,
                            emlrtAlias(emlrtGetFieldR2017b(
                                (emlrtCTX)sp, u, 0, 2, (const char_T *)"M")),
                            &thisId);
  thisId.fIdentifier = "rd";
  y->rd = b_emlrt_marshallIn(sp,
                             emlrtAlias(emlrtGetFieldR2017b(
                                 (emlrtCTX)sp, u, 0, 3, (const char_T *)"rd")),
                             &thisId);
  thisId.fIdentifier = "theta_max";
  y->theta_max = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 4,
                                     (const char_T *)"theta_max")),
      &thisId);
  thisId.fIdentifier = "thetap_max";
  y->thetap_max = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 5,
                                     (const char_T *)"thetap_max")),
      &thisId);
  emlrtDestroyArray(&u);
}

static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[16])
{
  r_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct4_T y[4])
{
  static const int32_T dims[2] = {1, 4};
  static const char_T *fieldNames[40] = {
      "pmin",       "pmax",      "alpha",     "p",        "beta_plus",
      "beta_moins", "alpha_min", "param",     "pbarmin",  "pbarmax",
      "pc",         "beta",      "Jpbarmin",  "gpbarmin", "Jpc",
      "gpc",        "Jpbarmax",  "gpbarmax",  "aJ",       "bJ",
      "cJ",         "ag",        "bg",        "cg",       "ps_J",
      "ps_J_star",  "qJstar",    "qJmin",     "pJmin",    "qJmax",
      "pJmax",      "ps_g",      "ps_g_star", "qgstar",   "qgmin",
      "pgmin",      "qgmax",     "pgmax",     "Z",        "ngz"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtCTX)sp, parentId, u, 40,
                         (const char_T **)&fieldNames[0], 2U, (void *)&dims[0]);
  thisId.fIdentifier = "pmin";
  y[0].pmin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 0, (const char_T *)"pmin")),
                         &thisId);
  thisId.fIdentifier = "pmax";
  y[0].pmax =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 1, (const char_T *)"pmax")),
                         &thisId);
  thisId.fIdentifier = "alpha";
  y[0].alpha =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 2, (const char_T *)"alpha")),
                         &thisId);
  thisId.fIdentifier = "p";
  y[0].p = b_emlrt_marshallIn(sp,
                              emlrtAlias(emlrtGetFieldR2017b(
                                  (emlrtCTX)sp, u, 0, 3, (const char_T *)"p")),
                              &thisId);
  thisId.fIdentifier = "beta_plus";
  y[0].beta_plus = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 4,
                                     (const char_T *)"beta_plus")),
      &thisId);
  thisId.fIdentifier = "beta_moins";
  y[0].beta_moins = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 5,
                                     (const char_T *)"beta_moins")),
      &thisId);
  thisId.fIdentifier = "alpha_min";
  y[0].alpha_min = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 6,
                                     (const char_T *)"alpha_min")),
      &thisId);
  thisId.fIdentifier = "param";
  m_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 7,
                                                    (const char_T *)"param")),
                     &thisId);
  thisId.fIdentifier = "pbarmin";
  y[0].pbarmin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 8, (const char_T *)"pbarmin")),
                         &thisId);
  thisId.fIdentifier = "pbarmax";
  y[0].pbarmax =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 9, (const char_T *)"pbarmax")),
                         &thisId);
  thisId.fIdentifier = "pc";
  y[0].pc =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 10,
                                                        (const char_T *)"pc")),
                         &thisId);
  thisId.fIdentifier = "beta";
  y[0].beta =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 11, (const char_T *)"beta")),
                         &thisId);
  thisId.fIdentifier = "Jpbarmin";
  y[0].Jpbarmin = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 12,
                                     (const char_T *)"Jpbarmin")),
      &thisId);
  thisId.fIdentifier = "gpbarmin";
  y[0].gpbarmin = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 13,
                                     (const char_T *)"gpbarmin")),
      &thisId);
  thisId.fIdentifier = "Jpc";
  y[0].Jpc =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 14,
                                                        (const char_T *)"Jpc")),
                         &thisId);
  thisId.fIdentifier = "gpc";
  y[0].gpc =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 15,
                                                        (const char_T *)"gpc")),
                         &thisId);
  thisId.fIdentifier = "Jpbarmax";
  y[0].Jpbarmax = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 16,
                                     (const char_T *)"Jpbarmax")),
      &thisId);
  thisId.fIdentifier = "gpbarmax";
  y[0].gpbarmax = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 17,
                                     (const char_T *)"gpbarmax")),
      &thisId);
  thisId.fIdentifier = "aJ";
  y[0].aJ =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 18,
                                                        (const char_T *)"aJ")),
                         &thisId);
  thisId.fIdentifier = "bJ";
  y[0].bJ =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 19,
                                                        (const char_T *)"bJ")),
                         &thisId);
  thisId.fIdentifier = "cJ";
  y[0].cJ =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 20,
                                                        (const char_T *)"cJ")),
                         &thisId);
  thisId.fIdentifier = "ag";
  y[0].ag =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 21,
                                                        (const char_T *)"ag")),
                         &thisId);
  thisId.fIdentifier = "bg";
  y[0].bg =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 22,
                                                        (const char_T *)"bg")),
                         &thisId);
  thisId.fIdentifier = "cg";
  y[0].cg =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 23,
                                                        (const char_T *)"cg")),
                         &thisId);
  thisId.fIdentifier = "ps_J";
  y[0].ps_J =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 24, (const char_T *)"ps_J")),
                         &thisId);
  thisId.fIdentifier = "ps_J_star";
  y[0].ps_J_star = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 25,
                                     (const char_T *)"ps_J_star")),
      &thisId);
  thisId.fIdentifier = "qJstar";
  y[0].qJstar =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 26, (const char_T *)"qJstar")),
                         &thisId);
  thisId.fIdentifier = "qJmin";
  y[0].qJmin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 27, (const char_T *)"qJmin")),
                         &thisId);
  thisId.fIdentifier = "pJmin";
  y[0].pJmin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 28, (const char_T *)"pJmin")),
                         &thisId);
  thisId.fIdentifier = "qJmax";
  y[0].qJmax =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 29, (const char_T *)"qJmax")),
                         &thisId);
  thisId.fIdentifier = "pJmax";
  y[0].pJmax =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 30, (const char_T *)"pJmax")),
                         &thisId);
  thisId.fIdentifier = "ps_g";
  y[0].ps_g =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 31, (const char_T *)"ps_g")),
                         &thisId);
  thisId.fIdentifier = "ps_g_star";
  y[0].ps_g_star = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 32,
                                     (const char_T *)"ps_g_star")),
      &thisId);
  thisId.fIdentifier = "qgstar";
  y[0].qgstar =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 33, (const char_T *)"qgstar")),
                         &thisId);
  thisId.fIdentifier = "qgmin";
  y[0].qgmin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 34, (const char_T *)"qgmin")),
                         &thisId);
  thisId.fIdentifier = "pgmin";
  y[0].pgmin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 35, (const char_T *)"pgmin")),
                         &thisId);
  thisId.fIdentifier = "qgmax";
  y[0].qgmax =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 36, (const char_T *)"qgmax")),
                         &thisId);
  thisId.fIdentifier = "pgmax";
  y[0].pgmax =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 37, (const char_T *)"pgmax")),
                         &thisId);
  thisId.fIdentifier = "Z";
  e_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 38,
                                                    (const char_T *)"Z")),
                     &thisId, y[0].Z);
  thisId.fIdentifier = "ngz";
  y[0].ngz =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 39,
                                                        (const char_T *)"ngz")),
                         &thisId);
  thisId.fIdentifier = "pmin";
  y[1].pmin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 0, (const char_T *)"pmin")),
                         &thisId);
  thisId.fIdentifier = "pmax";
  y[1].pmax =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 1, (const char_T *)"pmax")),
                         &thisId);
  thisId.fIdentifier = "alpha";
  y[1].alpha =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 2, (const char_T *)"alpha")),
                         &thisId);
  thisId.fIdentifier = "p";
  y[1].p = b_emlrt_marshallIn(sp,
                              emlrtAlias(emlrtGetFieldR2017b(
                                  (emlrtCTX)sp, u, 1, 3, (const char_T *)"p")),
                              &thisId);
  thisId.fIdentifier = "beta_plus";
  y[1].beta_plus = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 4,
                                     (const char_T *)"beta_plus")),
      &thisId);
  thisId.fIdentifier = "beta_moins";
  y[1].beta_moins = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 5,
                                     (const char_T *)"beta_moins")),
      &thisId);
  thisId.fIdentifier = "alpha_min";
  y[1].alpha_min = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 6,
                                     (const char_T *)"alpha_min")),
      &thisId);
  thisId.fIdentifier = "param";
  m_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 7,
                                                    (const char_T *)"param")),
                     &thisId);
  thisId.fIdentifier = "pbarmin";
  y[1].pbarmin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 8, (const char_T *)"pbarmin")),
                         &thisId);
  thisId.fIdentifier = "pbarmax";
  y[1].pbarmax =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 9, (const char_T *)"pbarmax")),
                         &thisId);
  thisId.fIdentifier = "pc";
  y[1].pc =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 10,
                                                        (const char_T *)"pc")),
                         &thisId);
  thisId.fIdentifier = "beta";
  y[1].beta =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 11, (const char_T *)"beta")),
                         &thisId);
  thisId.fIdentifier = "Jpbarmin";
  y[1].Jpbarmin = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 12,
                                     (const char_T *)"Jpbarmin")),
      &thisId);
  thisId.fIdentifier = "gpbarmin";
  y[1].gpbarmin = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 13,
                                     (const char_T *)"gpbarmin")),
      &thisId);
  thisId.fIdentifier = "Jpc";
  y[1].Jpc =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 14,
                                                        (const char_T *)"Jpc")),
                         &thisId);
  thisId.fIdentifier = "gpc";
  y[1].gpc =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 15,
                                                        (const char_T *)"gpc")),
                         &thisId);
  thisId.fIdentifier = "Jpbarmax";
  y[1].Jpbarmax = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 16,
                                     (const char_T *)"Jpbarmax")),
      &thisId);
  thisId.fIdentifier = "gpbarmax";
  y[1].gpbarmax = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 17,
                                     (const char_T *)"gpbarmax")),
      &thisId);
  thisId.fIdentifier = "aJ";
  y[1].aJ =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 18,
                                                        (const char_T *)"aJ")),
                         &thisId);
  thisId.fIdentifier = "bJ";
  y[1].bJ =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 19,
                                                        (const char_T *)"bJ")),
                         &thisId);
  thisId.fIdentifier = "cJ";
  y[1].cJ =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 20,
                                                        (const char_T *)"cJ")),
                         &thisId);
  thisId.fIdentifier = "ag";
  y[1].ag =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 21,
                                                        (const char_T *)"ag")),
                         &thisId);
  thisId.fIdentifier = "bg";
  y[1].bg =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 22,
                                                        (const char_T *)"bg")),
                         &thisId);
  thisId.fIdentifier = "cg";
  y[1].cg =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 23,
                                                        (const char_T *)"cg")),
                         &thisId);
  thisId.fIdentifier = "ps_J";
  y[1].ps_J =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 24, (const char_T *)"ps_J")),
                         &thisId);
  thisId.fIdentifier = "ps_J_star";
  y[1].ps_J_star = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 25,
                                     (const char_T *)"ps_J_star")),
      &thisId);
  thisId.fIdentifier = "qJstar";
  y[1].qJstar =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 26, (const char_T *)"qJstar")),
                         &thisId);
  thisId.fIdentifier = "qJmin";
  y[1].qJmin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 27, (const char_T *)"qJmin")),
                         &thisId);
  thisId.fIdentifier = "pJmin";
  y[1].pJmin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 28, (const char_T *)"pJmin")),
                         &thisId);
  thisId.fIdentifier = "qJmax";
  y[1].qJmax =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 29, (const char_T *)"qJmax")),
                         &thisId);
  thisId.fIdentifier = "pJmax";
  y[1].pJmax =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 30, (const char_T *)"pJmax")),
                         &thisId);
  thisId.fIdentifier = "ps_g";
  y[1].ps_g =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 31, (const char_T *)"ps_g")),
                         &thisId);
  thisId.fIdentifier = "ps_g_star";
  y[1].ps_g_star = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 32,
                                     (const char_T *)"ps_g_star")),
      &thisId);
  thisId.fIdentifier = "qgstar";
  y[1].qgstar =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 33, (const char_T *)"qgstar")),
                         &thisId);
  thisId.fIdentifier = "qgmin";
  y[1].qgmin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 34, (const char_T *)"qgmin")),
                         &thisId);
  thisId.fIdentifier = "pgmin";
  y[1].pgmin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 35, (const char_T *)"pgmin")),
                         &thisId);
  thisId.fIdentifier = "qgmax";
  y[1].qgmax =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 36, (const char_T *)"qgmax")),
                         &thisId);
  thisId.fIdentifier = "pgmax";
  y[1].pgmax =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 37, (const char_T *)"pgmax")),
                         &thisId);
  thisId.fIdentifier = "Z";
  e_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 38,
                                                    (const char_T *)"Z")),
                     &thisId, y[1].Z);
  thisId.fIdentifier = "ngz";
  y[1].ngz =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 39,
                                                        (const char_T *)"ngz")),
                         &thisId);
  thisId.fIdentifier = "pmin";
  y[2].pmin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 0, (const char_T *)"pmin")),
                         &thisId);
  thisId.fIdentifier = "pmax";
  y[2].pmax =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 1, (const char_T *)"pmax")),
                         &thisId);
  thisId.fIdentifier = "alpha";
  y[2].alpha =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 2, (const char_T *)"alpha")),
                         &thisId);
  thisId.fIdentifier = "p";
  y[2].p = b_emlrt_marshallIn(sp,
                              emlrtAlias(emlrtGetFieldR2017b(
                                  (emlrtCTX)sp, u, 2, 3, (const char_T *)"p")),
                              &thisId);
  thisId.fIdentifier = "beta_plus";
  y[2].beta_plus = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 4,
                                     (const char_T *)"beta_plus")),
      &thisId);
  thisId.fIdentifier = "beta_moins";
  y[2].beta_moins = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 5,
                                     (const char_T *)"beta_moins")),
      &thisId);
  thisId.fIdentifier = "alpha_min";
  y[2].alpha_min = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 6,
                                     (const char_T *)"alpha_min")),
      &thisId);
  thisId.fIdentifier = "param";
  m_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 7,
                                                    (const char_T *)"param")),
                     &thisId);
  thisId.fIdentifier = "pbarmin";
  y[2].pbarmin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 8, (const char_T *)"pbarmin")),
                         &thisId);
  thisId.fIdentifier = "pbarmax";
  y[2].pbarmax =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 9, (const char_T *)"pbarmax")),
                         &thisId);
  thisId.fIdentifier = "pc";
  y[2].pc =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 10,
                                                        (const char_T *)"pc")),
                         &thisId);
  thisId.fIdentifier = "beta";
  y[2].beta =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 11, (const char_T *)"beta")),
                         &thisId);
  thisId.fIdentifier = "Jpbarmin";
  y[2].Jpbarmin = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 12,
                                     (const char_T *)"Jpbarmin")),
      &thisId);
  thisId.fIdentifier = "gpbarmin";
  y[2].gpbarmin = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 13,
                                     (const char_T *)"gpbarmin")),
      &thisId);
  thisId.fIdentifier = "Jpc";
  y[2].Jpc =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 14,
                                                        (const char_T *)"Jpc")),
                         &thisId);
  thisId.fIdentifier = "gpc";
  y[2].gpc =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 15,
                                                        (const char_T *)"gpc")),
                         &thisId);
  thisId.fIdentifier = "Jpbarmax";
  y[2].Jpbarmax = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 16,
                                     (const char_T *)"Jpbarmax")),
      &thisId);
  thisId.fIdentifier = "gpbarmax";
  y[2].gpbarmax = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 17,
                                     (const char_T *)"gpbarmax")),
      &thisId);
  thisId.fIdentifier = "aJ";
  y[2].aJ =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 18,
                                                        (const char_T *)"aJ")),
                         &thisId);
  thisId.fIdentifier = "bJ";
  y[2].bJ =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 19,
                                                        (const char_T *)"bJ")),
                         &thisId);
  thisId.fIdentifier = "cJ";
  y[2].cJ =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 20,
                                                        (const char_T *)"cJ")),
                         &thisId);
  thisId.fIdentifier = "ag";
  y[2].ag =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 21,
                                                        (const char_T *)"ag")),
                         &thisId);
  thisId.fIdentifier = "bg";
  y[2].bg =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 22,
                                                        (const char_T *)"bg")),
                         &thisId);
  thisId.fIdentifier = "cg";
  y[2].cg =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 23,
                                                        (const char_T *)"cg")),
                         &thisId);
  thisId.fIdentifier = "ps_J";
  y[2].ps_J =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 24, (const char_T *)"ps_J")),
                         &thisId);
  thisId.fIdentifier = "ps_J_star";
  y[2].ps_J_star = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 25,
                                     (const char_T *)"ps_J_star")),
      &thisId);
  thisId.fIdentifier = "qJstar";
  y[2].qJstar =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 26, (const char_T *)"qJstar")),
                         &thisId);
  thisId.fIdentifier = "qJmin";
  y[2].qJmin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 27, (const char_T *)"qJmin")),
                         &thisId);
  thisId.fIdentifier = "pJmin";
  y[2].pJmin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 28, (const char_T *)"pJmin")),
                         &thisId);
  thisId.fIdentifier = "qJmax";
  y[2].qJmax =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 29, (const char_T *)"qJmax")),
                         &thisId);
  thisId.fIdentifier = "pJmax";
  y[2].pJmax =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 30, (const char_T *)"pJmax")),
                         &thisId);
  thisId.fIdentifier = "ps_g";
  y[2].ps_g =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 31, (const char_T *)"ps_g")),
                         &thisId);
  thisId.fIdentifier = "ps_g_star";
  y[2].ps_g_star = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 32,
                                     (const char_T *)"ps_g_star")),
      &thisId);
  thisId.fIdentifier = "qgstar";
  y[2].qgstar =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 33, (const char_T *)"qgstar")),
                         &thisId);
  thisId.fIdentifier = "qgmin";
  y[2].qgmin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 34, (const char_T *)"qgmin")),
                         &thisId);
  thisId.fIdentifier = "pgmin";
  y[2].pgmin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 35, (const char_T *)"pgmin")),
                         &thisId);
  thisId.fIdentifier = "qgmax";
  y[2].qgmax =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 36, (const char_T *)"qgmax")),
                         &thisId);
  thisId.fIdentifier = "pgmax";
  y[2].pgmax =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 37, (const char_T *)"pgmax")),
                         &thisId);
  thisId.fIdentifier = "Z";
  e_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 38,
                                                    (const char_T *)"Z")),
                     &thisId, y[2].Z);
  thisId.fIdentifier = "ngz";
  y[2].ngz =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 39,
                                                        (const char_T *)"ngz")),
                         &thisId);
  thisId.fIdentifier = "pmin";
  y[3].pmin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 0, (const char_T *)"pmin")),
                         &thisId);
  thisId.fIdentifier = "pmax";
  y[3].pmax =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 1, (const char_T *)"pmax")),
                         &thisId);
  thisId.fIdentifier = "alpha";
  y[3].alpha =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 2, (const char_T *)"alpha")),
                         &thisId);
  thisId.fIdentifier = "p";
  y[3].p = b_emlrt_marshallIn(sp,
                              emlrtAlias(emlrtGetFieldR2017b(
                                  (emlrtCTX)sp, u, 3, 3, (const char_T *)"p")),
                              &thisId);
  thisId.fIdentifier = "beta_plus";
  y[3].beta_plus = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 4,
                                     (const char_T *)"beta_plus")),
      &thisId);
  thisId.fIdentifier = "beta_moins";
  y[3].beta_moins = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 5,
                                     (const char_T *)"beta_moins")),
      &thisId);
  thisId.fIdentifier = "alpha_min";
  y[3].alpha_min = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 6,
                                     (const char_T *)"alpha_min")),
      &thisId);
  thisId.fIdentifier = "param";
  m_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 7,
                                                    (const char_T *)"param")),
                     &thisId);
  thisId.fIdentifier = "pbarmin";
  y[3].pbarmin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 8, (const char_T *)"pbarmin")),
                         &thisId);
  thisId.fIdentifier = "pbarmax";
  y[3].pbarmax =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 9, (const char_T *)"pbarmax")),
                         &thisId);
  thisId.fIdentifier = "pc";
  y[3].pc =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 10,
                                                        (const char_T *)"pc")),
                         &thisId);
  thisId.fIdentifier = "beta";
  y[3].beta =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 11, (const char_T *)"beta")),
                         &thisId);
  thisId.fIdentifier = "Jpbarmin";
  y[3].Jpbarmin = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 12,
                                     (const char_T *)"Jpbarmin")),
      &thisId);
  thisId.fIdentifier = "gpbarmin";
  y[3].gpbarmin = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 13,
                                     (const char_T *)"gpbarmin")),
      &thisId);
  thisId.fIdentifier = "Jpc";
  y[3].Jpc =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 14,
                                                        (const char_T *)"Jpc")),
                         &thisId);
  thisId.fIdentifier = "gpc";
  y[3].gpc =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 15,
                                                        (const char_T *)"gpc")),
                         &thisId);
  thisId.fIdentifier = "Jpbarmax";
  y[3].Jpbarmax = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 16,
                                     (const char_T *)"Jpbarmax")),
      &thisId);
  thisId.fIdentifier = "gpbarmax";
  y[3].gpbarmax = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 17,
                                     (const char_T *)"gpbarmax")),
      &thisId);
  thisId.fIdentifier = "aJ";
  y[3].aJ =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 18,
                                                        (const char_T *)"aJ")),
                         &thisId);
  thisId.fIdentifier = "bJ";
  y[3].bJ =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 19,
                                                        (const char_T *)"bJ")),
                         &thisId);
  thisId.fIdentifier = "cJ";
  y[3].cJ =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 20,
                                                        (const char_T *)"cJ")),
                         &thisId);
  thisId.fIdentifier = "ag";
  y[3].ag =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 21,
                                                        (const char_T *)"ag")),
                         &thisId);
  thisId.fIdentifier = "bg";
  y[3].bg =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 22,
                                                        (const char_T *)"bg")),
                         &thisId);
  thisId.fIdentifier = "cg";
  y[3].cg =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 23,
                                                        (const char_T *)"cg")),
                         &thisId);
  thisId.fIdentifier = "ps_J";
  y[3].ps_J =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 24, (const char_T *)"ps_J")),
                         &thisId);
  thisId.fIdentifier = "ps_J_star";
  y[3].ps_J_star = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 25,
                                     (const char_T *)"ps_J_star")),
      &thisId);
  thisId.fIdentifier = "qJstar";
  y[3].qJstar =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 26, (const char_T *)"qJstar")),
                         &thisId);
  thisId.fIdentifier = "qJmin";
  y[3].qJmin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 27, (const char_T *)"qJmin")),
                         &thisId);
  thisId.fIdentifier = "pJmin";
  y[3].pJmin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 28, (const char_T *)"pJmin")),
                         &thisId);
  thisId.fIdentifier = "qJmax";
  y[3].qJmax =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 29, (const char_T *)"qJmax")),
                         &thisId);
  thisId.fIdentifier = "pJmax";
  y[3].pJmax =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 30, (const char_T *)"pJmax")),
                         &thisId);
  thisId.fIdentifier = "ps_g";
  y[3].ps_g =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 31, (const char_T *)"ps_g")),
                         &thisId);
  thisId.fIdentifier = "ps_g_star";
  y[3].ps_g_star = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 32,
                                     (const char_T *)"ps_g_star")),
      &thisId);
  thisId.fIdentifier = "qgstar";
  y[3].qgstar =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 33, (const char_T *)"qgstar")),
                         &thisId);
  thisId.fIdentifier = "qgmin";
  y[3].qgmin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 34, (const char_T *)"qgmin")),
                         &thisId);
  thisId.fIdentifier = "pgmin";
  y[3].pgmin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 35, (const char_T *)"pgmin")),
                         &thisId);
  thisId.fIdentifier = "qgmax";
  y[3].qgmax =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 36, (const char_T *)"qgmax")),
                         &thisId);
  thisId.fIdentifier = "pgmax";
  y[3].pgmax =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 37, (const char_T *)"pgmax")),
                         &thisId);
  thisId.fIdentifier = "Z";
  e_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 38,
                                                    (const char_T *)"Z")),
                     &thisId, y[3].Z);
  thisId.fIdentifier = "ngz";
  y[3].ngz =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 39,
                                                        (const char_T *)"ngz")),
                         &thisId);
  emlrtDestroyArray(&u);
}

static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId)
{
  s_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
}

static real_T n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
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

static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[4])
{
  static const int32_T dims = 4;
  real_T(*r)[4];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 1U, (void *)&dims);
  r = (real_T(*)[4])emlrtMxGetData(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  ret[2] = (*r)[2];
  ret[3] = (*r)[3];
  emlrtDestroyArray(&src);
}

static void p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[80])
{
  static const int32_T dims[2] = {20, 4};
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

static void q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[3])
{
  static const int32_T dims = 3;
  real_T(*r)[3];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 1U, (void *)&dims);
  r = (real_T(*)[3])emlrtMxGetData(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  ret[2] = (*r)[2];
  emlrtDestroyArray(&src);
}

static void r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[16])
{
  static const int32_T dims[2] = {4, 4};
  real_T(*r)[16];
  int32_T i;
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)&dims[0]);
  r = (real_T(*)[16])emlrtMxGetData(src);
  for (i = 0; i < 16; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

static void s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims[2] = {0, 0};
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)&dims[0]);
  emlrtMxGetData(src);
  emlrtDestroyArray(&src);
}

void BBS_api(const mxArray *const prhs[2], int32_T nlhs, const mxArray *plhs[2])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  struct0_T param;
  real_T J;
  real_T eta;
  real_T g;
  st.tls = emlrtRootTLSGlobal;
  /* Marshall function inputs */
  eta = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "eta");
  c_emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "param", &param);
  /* Invoke the target function */
  BBS(&st, eta, &param, &J, &g);
  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(J);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(g);
  }
}

/* End of code generation (_coder_BBS_api.c) */
