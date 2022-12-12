/*
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
#include "update_mv_types.h"

/* Variable Definitions */
static emlrtRTEInfo v_emlrtRTEI = {
    1,                      /* lineNo */
    1,                      /* colNo */
    "_coder_update_mv_api", /* fName */
    ""                      /* pName */
};

/* Function Declarations */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct0_T y[4]);

static const mxArray *b_emlrt_marshallOut(const real_T u[4]);

static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static const mxArray *c_emlrt_marshallOut(const emxArray_real_T *u);

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId);

static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[4]);

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *mv,
                             const char_T *identifier, struct0_T y[4]);

static const mxArray *emlrt_marshallOut(const real_T u_data[],
                                        const int32_T u_size[2]);

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *subset,
                               const char_T *identifier, real_T **y_data,
                               int32_T *y_size);

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T **y_data, int32_T *y_size);

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *param,
                               const char_T *identifier, struct1_T *y);

static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct1_T *y);

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct2_T *y);

static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[80]);

static void l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct3_T *y);

static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[3]);

static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct4_T *y);

static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[16]);

static real_T p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static void q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId);

static void r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[4]);

static void s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real_T **ret_data, int32_T *ret_size);

static void t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[80]);

static void u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[3]);

static void v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[16]);

/* Function Definitions */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct0_T y[4])
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
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 0, (const char_T *)"pmin")),
                         &thisId);
  thisId.fIdentifier = "pmax";
  y[0].pmax =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 1, (const char_T *)"pmax")),
                         &thisId);
  thisId.fIdentifier = "alpha";
  y[0].alpha =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 2, (const char_T *)"alpha")),
                         &thisId);
  thisId.fIdentifier = "p";
  y[0].p = c_emlrt_marshallIn(sp,
                              emlrtAlias(emlrtGetFieldR2017b(
                                  (emlrtCTX)sp, u, 0, 3, (const char_T *)"p")),
                              &thisId);
  thisId.fIdentifier = "beta_plus";
  y[0].beta_plus = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 4,
                                     (const char_T *)"beta_plus")),
      &thisId);
  thisId.fIdentifier = "beta_moins";
  y[0].beta_moins = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 5,
                                     (const char_T *)"beta_moins")),
      &thisId);
  thisId.fIdentifier = "alpha_min";
  y[0].alpha_min = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 6,
                                     (const char_T *)"alpha_min")),
      &thisId);
  thisId.fIdentifier = "param";
  d_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 7,
                                                    (const char_T *)"param")),
                     &thisId);
  thisId.fIdentifier = "pbarmin";
  y[0].pbarmin =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 8, (const char_T *)"pbarmin")),
                         &thisId);
  thisId.fIdentifier = "pbarmax";
  y[0].pbarmax =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 9, (const char_T *)"pbarmax")),
                         &thisId);
  thisId.fIdentifier = "pc";
  y[0].pc =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 10,
                                                        (const char_T *)"pc")),
                         &thisId);
  thisId.fIdentifier = "beta";
  y[0].beta =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 11, (const char_T *)"beta")),
                         &thisId);
  thisId.fIdentifier = "Jpbarmin";
  y[0].Jpbarmin = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 12,
                                     (const char_T *)"Jpbarmin")),
      &thisId);
  thisId.fIdentifier = "gpbarmin";
  y[0].gpbarmin = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 13,
                                     (const char_T *)"gpbarmin")),
      &thisId);
  thisId.fIdentifier = "Jpc";
  y[0].Jpc =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 14,
                                                        (const char_T *)"Jpc")),
                         &thisId);
  thisId.fIdentifier = "gpc";
  y[0].gpc =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 15,
                                                        (const char_T *)"gpc")),
                         &thisId);
  thisId.fIdentifier = "Jpbarmax";
  y[0].Jpbarmax = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 16,
                                     (const char_T *)"Jpbarmax")),
      &thisId);
  thisId.fIdentifier = "gpbarmax";
  y[0].gpbarmax = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 17,
                                     (const char_T *)"gpbarmax")),
      &thisId);
  thisId.fIdentifier = "aJ";
  y[0].aJ =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 18,
                                                        (const char_T *)"aJ")),
                         &thisId);
  thisId.fIdentifier = "bJ";
  y[0].bJ =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 19,
                                                        (const char_T *)"bJ")),
                         &thisId);
  thisId.fIdentifier = "cJ";
  y[0].cJ =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 20,
                                                        (const char_T *)"cJ")),
                         &thisId);
  thisId.fIdentifier = "ag";
  y[0].ag =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 21,
                                                        (const char_T *)"ag")),
                         &thisId);
  thisId.fIdentifier = "bg";
  y[0].bg =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 22,
                                                        (const char_T *)"bg")),
                         &thisId);
  thisId.fIdentifier = "cg";
  y[0].cg =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 23,
                                                        (const char_T *)"cg")),
                         &thisId);
  thisId.fIdentifier = "ps_J";
  y[0].ps_J =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 24, (const char_T *)"ps_J")),
                         &thisId);
  thisId.fIdentifier = "ps_J_star";
  y[0].ps_J_star = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 25,
                                     (const char_T *)"ps_J_star")),
      &thisId);
  thisId.fIdentifier = "qJstar";
  y[0].qJstar =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 26, (const char_T *)"qJstar")),
                         &thisId);
  thisId.fIdentifier = "qJmin";
  y[0].qJmin =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 27, (const char_T *)"qJmin")),
                         &thisId);
  thisId.fIdentifier = "pJmin";
  y[0].pJmin =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 28, (const char_T *)"pJmin")),
                         &thisId);
  thisId.fIdentifier = "qJmax";
  y[0].qJmax =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 29, (const char_T *)"qJmax")),
                         &thisId);
  thisId.fIdentifier = "pJmax";
  y[0].pJmax =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 30, (const char_T *)"pJmax")),
                         &thisId);
  thisId.fIdentifier = "ps_g";
  y[0].ps_g =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 31, (const char_T *)"ps_g")),
                         &thisId);
  thisId.fIdentifier = "ps_g_star";
  y[0].ps_g_star = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 32,
                                     (const char_T *)"ps_g_star")),
      &thisId);
  thisId.fIdentifier = "qgstar";
  y[0].qgstar =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 33, (const char_T *)"qgstar")),
                         &thisId);
  thisId.fIdentifier = "qgmin";
  y[0].qgmin =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 34, (const char_T *)"qgmin")),
                         &thisId);
  thisId.fIdentifier = "pgmin";
  y[0].pgmin =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 35, (const char_T *)"pgmin")),
                         &thisId);
  thisId.fIdentifier = "qgmax";
  y[0].qgmax =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 36, (const char_T *)"qgmax")),
                         &thisId);
  thisId.fIdentifier = "pgmax";
  y[0].pgmax =
      c_emlrt_marshallIn(sp,
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
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 39,
                                                        (const char_T *)"ngz")),
                         &thisId);
  thisId.fIdentifier = "pmin";
  y[1].pmin =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 0, (const char_T *)"pmin")),
                         &thisId);
  thisId.fIdentifier = "pmax";
  y[1].pmax =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 1, (const char_T *)"pmax")),
                         &thisId);
  thisId.fIdentifier = "alpha";
  y[1].alpha =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 2, (const char_T *)"alpha")),
                         &thisId);
  thisId.fIdentifier = "p";
  y[1].p = c_emlrt_marshallIn(sp,
                              emlrtAlias(emlrtGetFieldR2017b(
                                  (emlrtCTX)sp, u, 1, 3, (const char_T *)"p")),
                              &thisId);
  thisId.fIdentifier = "beta_plus";
  y[1].beta_plus = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 4,
                                     (const char_T *)"beta_plus")),
      &thisId);
  thisId.fIdentifier = "beta_moins";
  y[1].beta_moins = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 5,
                                     (const char_T *)"beta_moins")),
      &thisId);
  thisId.fIdentifier = "alpha_min";
  y[1].alpha_min = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 6,
                                     (const char_T *)"alpha_min")),
      &thisId);
  thisId.fIdentifier = "param";
  d_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 7,
                                                    (const char_T *)"param")),
                     &thisId);
  thisId.fIdentifier = "pbarmin";
  y[1].pbarmin =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 8, (const char_T *)"pbarmin")),
                         &thisId);
  thisId.fIdentifier = "pbarmax";
  y[1].pbarmax =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 9, (const char_T *)"pbarmax")),
                         &thisId);
  thisId.fIdentifier = "pc";
  y[1].pc =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 10,
                                                        (const char_T *)"pc")),
                         &thisId);
  thisId.fIdentifier = "beta";
  y[1].beta =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 11, (const char_T *)"beta")),
                         &thisId);
  thisId.fIdentifier = "Jpbarmin";
  y[1].Jpbarmin = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 12,
                                     (const char_T *)"Jpbarmin")),
      &thisId);
  thisId.fIdentifier = "gpbarmin";
  y[1].gpbarmin = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 13,
                                     (const char_T *)"gpbarmin")),
      &thisId);
  thisId.fIdentifier = "Jpc";
  y[1].Jpc =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 14,
                                                        (const char_T *)"Jpc")),
                         &thisId);
  thisId.fIdentifier = "gpc";
  y[1].gpc =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 15,
                                                        (const char_T *)"gpc")),
                         &thisId);
  thisId.fIdentifier = "Jpbarmax";
  y[1].Jpbarmax = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 16,
                                     (const char_T *)"Jpbarmax")),
      &thisId);
  thisId.fIdentifier = "gpbarmax";
  y[1].gpbarmax = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 17,
                                     (const char_T *)"gpbarmax")),
      &thisId);
  thisId.fIdentifier = "aJ";
  y[1].aJ =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 18,
                                                        (const char_T *)"aJ")),
                         &thisId);
  thisId.fIdentifier = "bJ";
  y[1].bJ =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 19,
                                                        (const char_T *)"bJ")),
                         &thisId);
  thisId.fIdentifier = "cJ";
  y[1].cJ =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 20,
                                                        (const char_T *)"cJ")),
                         &thisId);
  thisId.fIdentifier = "ag";
  y[1].ag =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 21,
                                                        (const char_T *)"ag")),
                         &thisId);
  thisId.fIdentifier = "bg";
  y[1].bg =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 22,
                                                        (const char_T *)"bg")),
                         &thisId);
  thisId.fIdentifier = "cg";
  y[1].cg =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 23,
                                                        (const char_T *)"cg")),
                         &thisId);
  thisId.fIdentifier = "ps_J";
  y[1].ps_J =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 24, (const char_T *)"ps_J")),
                         &thisId);
  thisId.fIdentifier = "ps_J_star";
  y[1].ps_J_star = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 25,
                                     (const char_T *)"ps_J_star")),
      &thisId);
  thisId.fIdentifier = "qJstar";
  y[1].qJstar =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 26, (const char_T *)"qJstar")),
                         &thisId);
  thisId.fIdentifier = "qJmin";
  y[1].qJmin =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 27, (const char_T *)"qJmin")),
                         &thisId);
  thisId.fIdentifier = "pJmin";
  y[1].pJmin =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 28, (const char_T *)"pJmin")),
                         &thisId);
  thisId.fIdentifier = "qJmax";
  y[1].qJmax =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 29, (const char_T *)"qJmax")),
                         &thisId);
  thisId.fIdentifier = "pJmax";
  y[1].pJmax =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 30, (const char_T *)"pJmax")),
                         &thisId);
  thisId.fIdentifier = "ps_g";
  y[1].ps_g =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 31, (const char_T *)"ps_g")),
                         &thisId);
  thisId.fIdentifier = "ps_g_star";
  y[1].ps_g_star = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 32,
                                     (const char_T *)"ps_g_star")),
      &thisId);
  thisId.fIdentifier = "qgstar";
  y[1].qgstar =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 33, (const char_T *)"qgstar")),
                         &thisId);
  thisId.fIdentifier = "qgmin";
  y[1].qgmin =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 34, (const char_T *)"qgmin")),
                         &thisId);
  thisId.fIdentifier = "pgmin";
  y[1].pgmin =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 35, (const char_T *)"pgmin")),
                         &thisId);
  thisId.fIdentifier = "qgmax";
  y[1].qgmax =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 1, 36, (const char_T *)"qgmax")),
                         &thisId);
  thisId.fIdentifier = "pgmax";
  y[1].pgmax =
      c_emlrt_marshallIn(sp,
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
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 1, 39,
                                                        (const char_T *)"ngz")),
                         &thisId);
  thisId.fIdentifier = "pmin";
  y[2].pmin =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 0, (const char_T *)"pmin")),
                         &thisId);
  thisId.fIdentifier = "pmax";
  y[2].pmax =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 1, (const char_T *)"pmax")),
                         &thisId);
  thisId.fIdentifier = "alpha";
  y[2].alpha =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 2, (const char_T *)"alpha")),
                         &thisId);
  thisId.fIdentifier = "p";
  y[2].p = c_emlrt_marshallIn(sp,
                              emlrtAlias(emlrtGetFieldR2017b(
                                  (emlrtCTX)sp, u, 2, 3, (const char_T *)"p")),
                              &thisId);
  thisId.fIdentifier = "beta_plus";
  y[2].beta_plus = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 4,
                                     (const char_T *)"beta_plus")),
      &thisId);
  thisId.fIdentifier = "beta_moins";
  y[2].beta_moins = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 5,
                                     (const char_T *)"beta_moins")),
      &thisId);
  thisId.fIdentifier = "alpha_min";
  y[2].alpha_min = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 6,
                                     (const char_T *)"alpha_min")),
      &thisId);
  thisId.fIdentifier = "param";
  d_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 7,
                                                    (const char_T *)"param")),
                     &thisId);
  thisId.fIdentifier = "pbarmin";
  y[2].pbarmin =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 8, (const char_T *)"pbarmin")),
                         &thisId);
  thisId.fIdentifier = "pbarmax";
  y[2].pbarmax =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 9, (const char_T *)"pbarmax")),
                         &thisId);
  thisId.fIdentifier = "pc";
  y[2].pc =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 10,
                                                        (const char_T *)"pc")),
                         &thisId);
  thisId.fIdentifier = "beta";
  y[2].beta =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 11, (const char_T *)"beta")),
                         &thisId);
  thisId.fIdentifier = "Jpbarmin";
  y[2].Jpbarmin = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 12,
                                     (const char_T *)"Jpbarmin")),
      &thisId);
  thisId.fIdentifier = "gpbarmin";
  y[2].gpbarmin = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 13,
                                     (const char_T *)"gpbarmin")),
      &thisId);
  thisId.fIdentifier = "Jpc";
  y[2].Jpc =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 14,
                                                        (const char_T *)"Jpc")),
                         &thisId);
  thisId.fIdentifier = "gpc";
  y[2].gpc =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 15,
                                                        (const char_T *)"gpc")),
                         &thisId);
  thisId.fIdentifier = "Jpbarmax";
  y[2].Jpbarmax = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 16,
                                     (const char_T *)"Jpbarmax")),
      &thisId);
  thisId.fIdentifier = "gpbarmax";
  y[2].gpbarmax = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 17,
                                     (const char_T *)"gpbarmax")),
      &thisId);
  thisId.fIdentifier = "aJ";
  y[2].aJ =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 18,
                                                        (const char_T *)"aJ")),
                         &thisId);
  thisId.fIdentifier = "bJ";
  y[2].bJ =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 19,
                                                        (const char_T *)"bJ")),
                         &thisId);
  thisId.fIdentifier = "cJ";
  y[2].cJ =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 20,
                                                        (const char_T *)"cJ")),
                         &thisId);
  thisId.fIdentifier = "ag";
  y[2].ag =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 21,
                                                        (const char_T *)"ag")),
                         &thisId);
  thisId.fIdentifier = "bg";
  y[2].bg =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 22,
                                                        (const char_T *)"bg")),
                         &thisId);
  thisId.fIdentifier = "cg";
  y[2].cg =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 23,
                                                        (const char_T *)"cg")),
                         &thisId);
  thisId.fIdentifier = "ps_J";
  y[2].ps_J =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 24, (const char_T *)"ps_J")),
                         &thisId);
  thisId.fIdentifier = "ps_J_star";
  y[2].ps_J_star = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 25,
                                     (const char_T *)"ps_J_star")),
      &thisId);
  thisId.fIdentifier = "qJstar";
  y[2].qJstar =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 26, (const char_T *)"qJstar")),
                         &thisId);
  thisId.fIdentifier = "qJmin";
  y[2].qJmin =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 27, (const char_T *)"qJmin")),
                         &thisId);
  thisId.fIdentifier = "pJmin";
  y[2].pJmin =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 28, (const char_T *)"pJmin")),
                         &thisId);
  thisId.fIdentifier = "qJmax";
  y[2].qJmax =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 29, (const char_T *)"qJmax")),
                         &thisId);
  thisId.fIdentifier = "pJmax";
  y[2].pJmax =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 30, (const char_T *)"pJmax")),
                         &thisId);
  thisId.fIdentifier = "ps_g";
  y[2].ps_g =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 31, (const char_T *)"ps_g")),
                         &thisId);
  thisId.fIdentifier = "ps_g_star";
  y[2].ps_g_star = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 32,
                                     (const char_T *)"ps_g_star")),
      &thisId);
  thisId.fIdentifier = "qgstar";
  y[2].qgstar =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 33, (const char_T *)"qgstar")),
                         &thisId);
  thisId.fIdentifier = "qgmin";
  y[2].qgmin =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 34, (const char_T *)"qgmin")),
                         &thisId);
  thisId.fIdentifier = "pgmin";
  y[2].pgmin =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 35, (const char_T *)"pgmin")),
                         &thisId);
  thisId.fIdentifier = "qgmax";
  y[2].qgmax =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 2, 36, (const char_T *)"qgmax")),
                         &thisId);
  thisId.fIdentifier = "pgmax";
  y[2].pgmax =
      c_emlrt_marshallIn(sp,
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
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 2, 39,
                                                        (const char_T *)"ngz")),
                         &thisId);
  thisId.fIdentifier = "pmin";
  y[3].pmin =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 0, (const char_T *)"pmin")),
                         &thisId);
  thisId.fIdentifier = "pmax";
  y[3].pmax =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 1, (const char_T *)"pmax")),
                         &thisId);
  thisId.fIdentifier = "alpha";
  y[3].alpha =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 2, (const char_T *)"alpha")),
                         &thisId);
  thisId.fIdentifier = "p";
  y[3].p = c_emlrt_marshallIn(sp,
                              emlrtAlias(emlrtGetFieldR2017b(
                                  (emlrtCTX)sp, u, 3, 3, (const char_T *)"p")),
                              &thisId);
  thisId.fIdentifier = "beta_plus";
  y[3].beta_plus = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 4,
                                     (const char_T *)"beta_plus")),
      &thisId);
  thisId.fIdentifier = "beta_moins";
  y[3].beta_moins = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 5,
                                     (const char_T *)"beta_moins")),
      &thisId);
  thisId.fIdentifier = "alpha_min";
  y[3].alpha_min = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 6,
                                     (const char_T *)"alpha_min")),
      &thisId);
  thisId.fIdentifier = "param";
  d_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 7,
                                                    (const char_T *)"param")),
                     &thisId);
  thisId.fIdentifier = "pbarmin";
  y[3].pbarmin =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 8, (const char_T *)"pbarmin")),
                         &thisId);
  thisId.fIdentifier = "pbarmax";
  y[3].pbarmax =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 9, (const char_T *)"pbarmax")),
                         &thisId);
  thisId.fIdentifier = "pc";
  y[3].pc =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 10,
                                                        (const char_T *)"pc")),
                         &thisId);
  thisId.fIdentifier = "beta";
  y[3].beta =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 11, (const char_T *)"beta")),
                         &thisId);
  thisId.fIdentifier = "Jpbarmin";
  y[3].Jpbarmin = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 12,
                                     (const char_T *)"Jpbarmin")),
      &thisId);
  thisId.fIdentifier = "gpbarmin";
  y[3].gpbarmin = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 13,
                                     (const char_T *)"gpbarmin")),
      &thisId);
  thisId.fIdentifier = "Jpc";
  y[3].Jpc =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 14,
                                                        (const char_T *)"Jpc")),
                         &thisId);
  thisId.fIdentifier = "gpc";
  y[3].gpc =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 15,
                                                        (const char_T *)"gpc")),
                         &thisId);
  thisId.fIdentifier = "Jpbarmax";
  y[3].Jpbarmax = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 16,
                                     (const char_T *)"Jpbarmax")),
      &thisId);
  thisId.fIdentifier = "gpbarmax";
  y[3].gpbarmax = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 17,
                                     (const char_T *)"gpbarmax")),
      &thisId);
  thisId.fIdentifier = "aJ";
  y[3].aJ =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 18,
                                                        (const char_T *)"aJ")),
                         &thisId);
  thisId.fIdentifier = "bJ";
  y[3].bJ =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 19,
                                                        (const char_T *)"bJ")),
                         &thisId);
  thisId.fIdentifier = "cJ";
  y[3].cJ =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 20,
                                                        (const char_T *)"cJ")),
                         &thisId);
  thisId.fIdentifier = "ag";
  y[3].ag =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 21,
                                                        (const char_T *)"ag")),
                         &thisId);
  thisId.fIdentifier = "bg";
  y[3].bg =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 22,
                                                        (const char_T *)"bg")),
                         &thisId);
  thisId.fIdentifier = "cg";
  y[3].cg =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 23,
                                                        (const char_T *)"cg")),
                         &thisId);
  thisId.fIdentifier = "ps_J";
  y[3].ps_J =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 24, (const char_T *)"ps_J")),
                         &thisId);
  thisId.fIdentifier = "ps_J_star";
  y[3].ps_J_star = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 25,
                                     (const char_T *)"ps_J_star")),
      &thisId);
  thisId.fIdentifier = "qJstar";
  y[3].qJstar =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 26, (const char_T *)"qJstar")),
                         &thisId);
  thisId.fIdentifier = "qJmin";
  y[3].qJmin =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 27, (const char_T *)"qJmin")),
                         &thisId);
  thisId.fIdentifier = "pJmin";
  y[3].pJmin =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 28, (const char_T *)"pJmin")),
                         &thisId);
  thisId.fIdentifier = "qJmax";
  y[3].qJmax =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 29, (const char_T *)"qJmax")),
                         &thisId);
  thisId.fIdentifier = "pJmax";
  y[3].pJmax =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 30, (const char_T *)"pJmax")),
                         &thisId);
  thisId.fIdentifier = "ps_g";
  y[3].ps_g =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 31, (const char_T *)"ps_g")),
                         &thisId);
  thisId.fIdentifier = "ps_g_star";
  y[3].ps_g_star = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 32,
                                     (const char_T *)"ps_g_star")),
      &thisId);
  thisId.fIdentifier = "qgstar";
  y[3].qgstar =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 33, (const char_T *)"qgstar")),
                         &thisId);
  thisId.fIdentifier = "qgmin";
  y[3].qgmin =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 34, (const char_T *)"qgmin")),
                         &thisId);
  thisId.fIdentifier = "pgmin";
  y[3].pgmin =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 35, (const char_T *)"pgmin")),
                         &thisId);
  thisId.fIdentifier = "qgmax";
  y[3].qgmax =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 3, 36, (const char_T *)"qgmax")),
                         &thisId);
  thisId.fIdentifier = "pgmax";
  y[3].pgmax =
      c_emlrt_marshallIn(sp,
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
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 3, 39,
                                                        (const char_T *)"ngz")),
                         &thisId);
  emlrtDestroyArray(&u);
}

static const mxArray *b_emlrt_marshallOut(const real_T u[4])
{
  static const int32_T i = 0;
  static const int32_T i1 = 4;
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = p_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *c_emlrt_marshallOut(const emxArray_real_T *u)
{
  static const int32_T iv[2] = {0, 0};
  const mxArray *m;
  const mxArray *y;
  const real_T *u_data;
  u_data = u->data;
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u_data[0]);
  emlrtSetDimensions((mxArray *)m, &u->size[0], 2);
  emlrtAssign(&y, m);
  return y;
}

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId)
{
  q_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
}

static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[4])
{
  r_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *mv,
                             const char_T *identifier, struct0_T y[4])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  b_emlrt_marshallIn(sp, emlrtAlias(mv), &thisId, y);
  emlrtDestroyArray(&mv);
}

static const mxArray *emlrt_marshallOut(const real_T u_data[],
                                        const int32_T u_size[2])
{
  static const int32_T iv[2] = {0, 0};
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u_data[0]);
  emlrtSetDimensions((mxArray *)m, &u_size[0], 2);
  emlrtAssign(&y, m);
  return y;
}

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *subset,
                               const char_T *identifier, real_T **y_data,
                               int32_T *y_size)
{
  emlrtMsgIdentifier thisId;
  real_T *r;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  g_emlrt_marshallIn(sp, emlrtAlias(subset), &thisId, &r, y_size);
  *y_data = r;
  emlrtDestroyArray(&subset);
}

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T **y_data, int32_T *y_size)
{
  s_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y_data, y_size);
  emlrtDestroyArray(&u);
}

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *param,
                               const char_T *identifier, struct1_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  i_emlrt_marshallIn(sp, emlrtAlias(param), &thisId, y);
  emlrtDestroyArray(&param);
}

static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, struct1_T *y)
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
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 4,
                                                        (const char_T *)"ell")),
                         &thisId);
  thisId.fIdentifier = "uparam";
  j_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 5,
                                                    (const char_T *)"uparam")),
                     &thisId, &y->uparam);
  thisId.fIdentifier = "ode";
  l_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 6,
                                                    (const char_T *)"ode")),
                     &thisId, &y->ode);
  thisId.fIdentifier = "ocp";
  n_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 7,
                                                    (const char_T *)"ocp")),
                     &thisId, &y->ocp);
  thisId.fIdentifier = "x0";
  e_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 8,
                                                    (const char_T *)"x0")),
                     &thisId, y->x0);
  thisId.fIdentifier = "mv";
  b_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 9,
                                                    (const char_T *)"mv")),
                     &thisId, y->mv);
  thisId.fIdentifier = "compiled";
  y->compiled = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 10,
                                     (const char_T *)"compiled")),
      &thisId);
  thisId.fIdentifier = "Nev";
  y->Nev =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 11,
                                                        (const char_T *)"Nev")),
                         &thisId);
  thisId.fIdentifier = "Niter";
  y->Niter =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 12, (const char_T *)"Niter")),
                         &thisId);
  thisId.fIdentifier = "np";
  y->np = c_emlrt_marshallIn(sp,
                             emlrtAlias(emlrtGetFieldR2017b(
                                 (emlrtCTX)sp, u, 0, 13, (const char_T *)"np")),
                             &thisId);
  emlrtDestroyArray(&u);
}

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, struct2_T *y)
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
  y->nu = c_emlrt_marshallIn(sp,
                             emlrtAlias(emlrtGetFieldR2017b(
                                 (emlrtCTX)sp, u, 0, 0, (const char_T *)"nu")),
                             &thisId);
  thisId.fIdentifier = "Np";
  y->Np = c_emlrt_marshallIn(sp,
                             emlrtAlias(emlrtGetFieldR2017b(
                                 (emlrtCTX)sp, u, 0, 1, (const char_T *)"Np")),
                             &thisId);
  thisId.fIdentifier = "Ifree";
  e_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 2,
                                                    (const char_T *)"Ifree")),
                     &thisId, y->Ifree);
  thisId.fIdentifier = "R";
  k_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 3,
                                                    (const char_T *)"R")),
                     &thisId, y->R);
  thisId.fIdentifier = "np";
  y->np = c_emlrt_marshallIn(sp,
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

static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[80])
{
  t_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, struct3_T *y)
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
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 0,
                                                        (const char_T *)"tau")),
                         &thisId);
  thisId.fIdentifier = "rk_order";
  y->rk_order = c_emlrt_marshallIn(
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
  y->u0 = c_emlrt_marshallIn(sp,
                             emlrtAlias(emlrtGetFieldR2017b(
                                 (emlrtCTX)sp, u, 0, 3, (const char_T *)"u0")),
                             &thisId);
  thisId.fIdentifier = "w";
  m_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 4,
                                                    (const char_T *)"w")),
                     &thisId, y->w);
  emlrtDestroyArray(&u);
}

static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[3])
{
  u_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, struct4_T *y)
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
  o_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 0,
                                                    (const char_T *)"Q")),
                     &thisId, y->Q);
  thisId.fIdentifier = "R";
  y->R = c_emlrt_marshallIn(sp,
                            emlrtAlias(emlrtGetFieldR2017b(
                                (emlrtCTX)sp, u, 0, 1, (const char_T *)"R")),
                            &thisId);
  thisId.fIdentifier = "M";
  y->M = c_emlrt_marshallIn(sp,
                            emlrtAlias(emlrtGetFieldR2017b(
                                (emlrtCTX)sp, u, 0, 2, (const char_T *)"M")),
                            &thisId);
  thisId.fIdentifier = "rd";
  y->rd = c_emlrt_marshallIn(sp,
                             emlrtAlias(emlrtGetFieldR2017b(
                                 (emlrtCTX)sp, u, 0, 3, (const char_T *)"rd")),
                             &thisId);
  thisId.fIdentifier = "theta_max";
  y->theta_max = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 4,
                                     (const char_T *)"theta_max")),
      &thisId);
  thisId.fIdentifier = "thetap_max";
  y->thetap_max = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 5,
                                     (const char_T *)"thetap_max")),
      &thisId);
  emlrtDestroyArray(&u);
}

static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[16])
{
  v_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static real_T p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
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

static void q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims[2] = {0, 0};
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)&dims[0]);
  emlrtMxGetData(src);
  emlrtDestroyArray(&src);
}

static void r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
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

static void s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real_T **ret_data, int32_T *ret_size)
{
  static const int32_T dims = 4;
  const boolean_T b = true;
  emlrtCheckVsBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                            false, 1U, (void *)&dims, &b, ret_size);
  *ret_data = (real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
}

static void t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
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

static void u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
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

static void v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
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

void update_mv_api(const mxArray *const prhs[3], int32_T nlhs,
                   const mxArray *plhs[3])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  emxArray_real_T *lesp;
  struct0_T mv[4];
  struct1_T param;
  real_T(*u_sol_data)[400];
  real_T(*p_sol)[4];
  real_T(*subset_data)[4];
  int32_T u_sol_size[2];
  int32_T subset_size;
  st.tls = emlrtRootTLSGlobal;
  u_sol_data = (real_T(*)[400])mxMalloc(sizeof(real_T[400]));
  p_sol = (real_T(*)[4])mxMalloc(sizeof(real_T[4]));
  emlrtHeapReferenceStackEnterFcnR2012b(&st);
  emxInit_real_T(&st, &lesp, 2, &v_emlrtRTEI);
  /* Marshall function inputs */
  emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "mv", mv);
  f_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "subset",
                     (real_T **)&subset_data, &subset_size);
  h_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "param", &param);
  /* Invoke the target function */
  update_mv(&st, mv, *subset_data, *(int32_T(*)[1]) & subset_size, &param,
            *u_sol_data, u_sol_size, *p_sol, lesp);
  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*u_sol_data, u_sol_size);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(*p_sol);
  }
  if (nlhs > 2) {
    lesp->canFreeData = false;
    plhs[2] = c_emlrt_marshallOut(lesp);
  }
  emxFree_real_T(&st, &lesp);
  emlrtHeapReferenceStackLeaveFcnR2012b(&st);
}

/* End of code generation (_coder_update_mv_api.c) */
