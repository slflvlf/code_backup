/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compute_parabola.c
 *
 * Code generation for function 'compute_parabola'
 *
 */

/* Include files */
#include "compute_parabola.h"
#include "BBS.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "update_mv.h"

/* Variable Definitions */
static emlrtRSInfo kc_emlrtRSI = { 15, /* lineNo */
  "compute_parabola",                  /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\compute_parabola.m"/* pathName */
};

static emlrtRSInfo lc_emlrtRSI = { 16, /* lineNo */
  "compute_parabola",                  /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\compute_parabola.m"/* pathName */
};

static emlrtRSInfo mc_emlrtRSI = { 17, /* lineNo */
  "compute_parabola",                  /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\compute_parabola.m"/* pathName */
};

/* Function Definitions */
void compute_parabola(const emlrtStack *sp, const struct0_T *sc_past, const
                      struct1_T *param, struct0_T *sc)
{
  real_T inter;
  real_T lesJ[3];
  real_T lesp[3];
  int32_T idx;
  int32_T k;
  boolean_T exitg1;
  int32_T i;
  real_T d;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;

  /* --------------------------------------------------------- */
  /*  Author : Mazen Alamir  */
  /*  CNRS, UNiversity of Grenoble-Alpes */
  /*  Gipsa Lab. Last modification March 2017 */
  /* --------------------------------------------------------- */
  *sc = *sc_past;
  sc->pbarmin = muDoubleScalarMax(sc_past->pmin, sc_past->p - sc_past->alpha);
  sc->pbarmax = muDoubleScalarMin(sc_past->pmax, sc_past->p + sc_past->alpha);
  sc->pc = (sc->pbarmin + sc->pbarmax) / 2.0;
  sc->beta = (sc->pbarmax - sc->pbarmin) / 2.0;

  /* -------------------------------------------------------- */
  st.site = &kc_emlrtRSI;
  BBS(&st, sc->pbarmin, param->p, param->ell, param->uparam.nu, param->uparam.Np,
      param->uparam.R, param->ode.tau, param->ode.rk_order, param->ode.u0,
      param->ode.M, param->ode.D, param->ode.thrust_config, &param->ocp,
      param->x0, &sc->Jpbarmin, &sc->gpbarmin);
  st.site = &lc_emlrtRSI;
  BBS(&st, sc->pc, param->p, param->ell, param->uparam.nu, param->uparam.Np,
      param->uparam.R, param->ode.tau, param->ode.rk_order, param->ode.u0,
      param->ode.M, param->ode.D, param->ode.thrust_config, &param->ocp,
      param->x0, &sc->Jpc, &sc->gpc);
  st.site = &mc_emlrtRSI;
  BBS(&st, sc->pbarmax, param->p, param->ell, param->uparam.nu, param->uparam.Np,
      param->uparam.R, param->ode.tau, param->ode.rk_order, param->ode.u0,
      param->ode.M, param->ode.D, param->ode.thrust_config, &param->ocp,
      param->x0, &sc->Jpbarmax, &sc->gpbarmax);
  sc->aJ = (sc->Jpbarmin + sc->Jpbarmax) / 2.0 - sc->Jpc;
  sc->bJ = (sc->Jpbarmax - sc->Jpbarmin) / 2.0;
  sc->cJ = sc->Jpc;
  sc->ag = (sc->gpbarmin + sc->gpbarmax) / 2.0 - sc->gpc;
  sc->bg = (sc->gpbarmax - sc->gpbarmin) / 2.0;
  sc->cg = sc->gpc;

  /* -------------------------------------------------------- */
  if (muDoubleScalarAbs(sc->aJ) > 2.2204460492503131E-15) {
    sc->ps_J = sc->pc - sc->beta * sc->bJ / (2.0 * sc->aJ);
  } else if (sc->bJ <= 0.0) {
    sc->ps_J = rtInf;
  } else {
    sc->ps_J = rtMinusInf;
  }

  sc->ps_J_star = muDoubleScalarMin(sc->pbarmax, muDoubleScalarMax(sc->pbarmin,
    sc->ps_J));
  inter = (sc->ps_J_star - sc->pc) / sc->beta;
  sc->qJstar = (sc->aJ * (inter * inter) + sc->bJ * inter) + sc->cJ;
  lesJ[0] = sc->Jpbarmin;
  lesJ[1] = sc->Jpbarmax;
  lesJ[2] = sc->qJstar;
  lesp[0] = sc->pbarmin;
  lesp[1] = sc->pbarmax;
  lesp[2] = sc->ps_J_star;
  if (!muDoubleScalarIsNaN(sc->Jpbarmin)) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 4)) {
      if (!muDoubleScalarIsNaN(lesJ[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    sc->qJmin = sc->Jpbarmin;
    idx = 1;
  } else {
    inter = lesJ[idx - 1];
    i = idx + 1;
    for (k = i; k < 4; k++) {
      d = lesJ[k - 1];
      if (inter > d) {
        inter = d;
        idx = k;
      }
    }

    sc->qJmin = inter;
  }

  sc->pJmin = lesp[idx - 1];
  if (!muDoubleScalarIsNaN(sc->Jpbarmin)) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 4)) {
      if (!muDoubleScalarIsNaN(lesJ[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    sc->qJmax = sc->Jpbarmin;
    idx = 1;
  } else {
    inter = lesJ[idx - 1];
    i = idx + 1;
    for (k = i; k < 4; k++) {
      d = lesJ[k - 1];
      if (inter < d) {
        inter = d;
        idx = k;
      }
    }

    sc->qJmax = inter;
  }

  sc->pJmax = lesp[idx - 1];

  /* -------------------------------------------------------- */
  if (muDoubleScalarAbs(sc->ag) > 2.2204460492503131E-15) {
    sc->ps_g = sc->pc - sc->beta * sc->bg / (2.0 * sc->ag);
  } else if (sc->bg <= 0.0) {
    sc->ps_g = rtInf;
  } else {
    sc->ps_g = rtMinusInf;
  }

  sc->ps_g_star = muDoubleScalarMin(sc->pbarmax, muDoubleScalarMax(sc->pbarmin,
    sc->ps_g));
  inter = (sc->ps_g_star - sc->pc) / sc->beta;
  sc->qgstar = (sc->ag * (inter * inter) + sc->bg * inter) + sc->cg;
  lesJ[0] = sc->gpbarmin;
  lesJ[1] = sc->gpbarmax;
  lesJ[2] = sc->qgstar;
  lesp[0] = sc->pbarmin;
  lesp[1] = sc->pbarmax;
  lesp[2] = sc->ps_g_star;
  if (!muDoubleScalarIsNaN(sc->gpbarmin)) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 4)) {
      if (!muDoubleScalarIsNaN(lesJ[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    sc->qgmin = sc->gpbarmin;
    idx = 1;
  } else {
    inter = lesJ[idx - 1];
    i = idx + 1;
    for (k = i; k < 4; k++) {
      d = lesJ[k - 1];
      if (inter > d) {
        inter = d;
        idx = k;
      }
    }

    sc->qgmin = inter;
  }

  sc->pgmin = lesp[idx - 1];
  if (!muDoubleScalarIsNaN(sc->gpbarmin)) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 4)) {
      if (!muDoubleScalarIsNaN(lesJ[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    sc->qgmax = sc->gpbarmin;
    idx = 1;
  } else {
    inter = lesJ[idx - 1];
    i = idx + 1;
    for (k = i; k < 4; k++) {
      d = lesJ[k - 1];
      if (inter < d) {
        inter = d;
        idx = k;
      }
    }

    sc->qgmax = inter;
  }

  sc->pgmax = lesp[idx - 1];

  /* -------------------------------------------------------- */
}

/* End of code generation (compute_parabola.c) */
