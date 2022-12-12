/*
 * update_sc.c
 *
 * Code generation for function 'update_sc'
 *
 */

/* Include files */
#include "update_sc.h"
#include "BBS.h"
#include "compute_Zg.h"
#include "compute_parabola.h"
#include "qJ.h"
#include "qg.h"
#include "rt_nonfinite.h"
#include "update_mv_data.h"
#include "update_mv_emxutil.h"
#include "update_mv_types.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo c_emlrtRSI = {
    9,                                                    /* lineNo */
    "update_sc",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_sc.m" /* pathName */
};

static emlrtRSInfo d_emlrtRSI = {
    13,                                                   /* lineNo */
    "update_sc",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_sc.m" /* pathName */
};

static emlrtRSInfo e_emlrtRSI = {
    15,                                                   /* lineNo */
    "update_sc",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_sc.m" /* pathName */
};

static emlrtRSInfo f_emlrtRSI = {
    19,                                                   /* lineNo */
    "update_sc",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_sc.m" /* pathName */
};

static emlrtRSInfo g_emlrtRSI = {
    22,                                                   /* lineNo */
    "update_sc",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_sc.m" /* pathName */
};

static emlrtRSInfo m_emlrtRSI = {
    34,                                                   /* lineNo */
    "update_sc",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_sc.m" /* pathName */
};

static emlrtRSInfo t_emlrtRSI = {
    53,                                                   /* lineNo */
    "update_sc",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_sc.m" /* pathName */
};

static emlrtRTEInfo m_emlrtRTEI = {
    11,                                                   /* lineNo */
    7,                                                    /* colNo */
    "update_sc",                                          /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_sc.m" /* pName */
};

static emlrtDCInfo i_emlrtDCI = {
    10,                                                    /* lineNo */
    12,                                                    /* colNo */
    "update_sc",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_sc.m", /* pName */
    1                                                      /* checkKind */
};

static emlrtDCInfo j_emlrtDCI = {
    10,                                                    /* lineNo */
    12,                                                    /* colNo */
    "update_sc",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_sc.m", /* pName */
    4                                                      /* checkKind */
};

static emlrtBCInfo u_emlrtBCI = {
    -1,                                                    /* iFirst */
    -1,                                                    /* iLast */
    12,                                                    /* lineNo */
    10,                                                    /* colNo */
    "lesp",                                                /* aName */
    "update_sc",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_sc.m", /* pName */
    0                                                      /* checkKind */
};

static emlrtBCInfo v_emlrtBCI = {
    -1,                                                    /* iFirst */
    -1,                                                    /* iLast */
    12,                                                    /* lineNo */
    27,                                                    /* colNo */
    "lesalpha",                                            /* aName */
    "update_sc",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_sc.m", /* pName */
    0                                                      /* checkKind */
};

static emlrtRTEInfo w_emlrtRTEI = {
    10,                                                   /* lineNo */
    21,                                                   /* colNo */
    "update_sc",                                          /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_sc.m" /* pName */
};

static emlrtRTEInfo x_emlrtRTEI = {
    10,                                                   /* lineNo */
    6,                                                    /* colNo */
    "update_sc",                                          /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_sc.m" /* pName */
};

static emlrtRTEInfo y_emlrtRTEI = {
    7,                                                    /* lineNo */
    14,                                                   /* colNo */
    "update_sc",                                          /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_sc.m" /* pName */
};

static emlrtRTEInfo ab_emlrtRTEI = {
    7,                                                    /* lineNo */
    19,                                                   /* colNo */
    "update_sc",                                          /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_sc.m" /* pName */
};

/* Function Definitions */
void update_sc(const emlrtStack *sp, struct0_T *sc_past, const struct1_T *param,
               real_T Niter)
{
  emlrtStack st;
  emxArray_real_T *lesalpha;
  emxArray_real_T *lesp;
  struct0_T b_sc_past;
  real_T vJ[3];
  real_T vg[3];
  real_T vp[3];
  real_T Jcand;
  real_T Jcurrent;
  real_T gcand;
  real_T gcurrent;
  real_T *lesalpha_data;
  real_T *lesp_data;
  int32_T b_i;
  int32_T i;
  int32_T j;
  int32_T k;
  int32_T loop_ub_tmp;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtCTX)sp);
  /* --------------------------------------------------------- */
  /*  Author : Mazen Alamir  */
  /*  CNRS, UNiversity of Grenoble-Alpes */
  /*  Gipsa Lab. Last modification March 2017 */
  /* --------------------------------------------------------- */
  st.site = &c_emlrtRSI;
  BBS(&st, sc_past->p, param->p, param->ell, param->uparam.nu, param->uparam.Np,
      param->uparam.R, param->ode.tau, param->ode.rk_order, param->ode.u0,
      param->ode.w, &param->ocp, param->x0, &Jcurrent, &gcurrent);
  if (!(Niter >= 0.0)) {
    emlrtNonNegativeCheckR2012b(Niter, &j_emlrtDCI, (emlrtCTX)sp);
  }
  if (Niter != (int32_T)muDoubleScalarFloor(Niter)) {
    emlrtIntegerCheckR2012b(Niter, &i_emlrtDCI, (emlrtCTX)sp);
  }
  emxInit_real_T(sp, &lesalpha, 1, &ab_emlrtRTEI);
  loop_ub_tmp = (int32_T)Niter;
  i = lesalpha->size[0];
  lesalpha->size[0] = (int32_T)Niter;
  emxEnsureCapacity_real_T(sp, lesalpha, i, &w_emlrtRTEI);
  lesalpha_data = lesalpha->data;
  for (i = 0; i < loop_ub_tmp; i++) {
    lesalpha_data[i] = 0.0;
  }
  emxInit_real_T(sp, &lesp, 1, &y_emlrtRTEI);
  emlrtForLoopVectorCheckR2021a(1.0, 1.0, Niter, mxDOUBLE_CLASS, (int32_T)Niter,
                                &m_emlrtRTEI, (emlrtCTX)sp);
  i = lesp->size[0];
  lesp->size[0] = (int32_T)Niter;
  emxEnsureCapacity_real_T(sp, lesp, i, &x_emlrtRTEI);
  lesp_data = lesp->data;
  for (b_i = 0; b_i < loop_ub_tmp; b_i++) {
    real_T pcand;
    boolean_T Cond;
    if (((int32_T)(b_i + 1U) < 1) || ((int32_T)(b_i + 1U) > lesp->size[0])) {
      emlrtDynamicBoundsCheckR2012b((int32_T)(b_i + 1U), 1, lesp->size[0],
                                    &u_emlrtBCI, (emlrtCTX)sp);
    }
    lesp_data[b_i] = sc_past->p;
    if (((int32_T)(b_i + 1U) < 1) ||
        ((int32_T)(b_i + 1U) > lesalpha->size[0])) {
      emlrtDynamicBoundsCheckR2012b((int32_T)(b_i + 1U), 1, lesalpha->size[0],
                                    &v_emlrtBCI, (emlrtCTX)sp);
    }
    lesalpha_data[b_i] = sc_past->alpha;
    b_sc_past = *sc_past;
    st.site = &d_emlrtRSI;
    compute_parabola(&st, &b_sc_past, param, sc_past);
    if (sc_past->qgmax <= 0.0) {
      pcand = sc_past->pJmin;
      st.site = &e_emlrtRSI;
      BBS(&st, sc_past->pJmin, param->p, param->ell, param->uparam.nu,
          param->uparam.Np, param->uparam.R, param->ode.tau,
          param->ode.rk_order, param->ode.u0, param->ode.w, &param->ocp,
          param->x0, &Jcand, &gcand);
      if (((Jcand < Jcurrent) && (gcand <= 0.0)) ||
          ((Jcand <= Jcurrent) && (gcand < 0.0))) {
        Cond = true;
      } else {
        Cond = false;
      }
    } else if (sc_past->qgmin > 0.0) {
      pcand = sc_past->pgmin;
      st.site = &f_emlrtRSI;
      BBS(&st, sc_past->pgmin, param->p, param->ell, param->uparam.nu,
          param->uparam.Np, param->uparam.R, param->ode.tau,
          param->ode.rk_order, param->ode.u0, param->ode.w, &param->ocp,
          param->x0, &Jcand, &gcand);
      Cond = (gcand < gcurrent);
    } else {
      b_sc_past = *sc_past;
      st.site = &g_emlrtRSI;
      compute_Zg(&st, &b_sc_past, sc_past);
      if (sc_past->ngz == 1.0) {
        real_T d;
        real_T ex;
        int32_T idx;
        boolean_T exitg1;
        vp[0] = sc_past->Z[0];
        vp[1] = sc_past->Z[1];
        vp[2] = muDoubleScalarMin(
            sc_past->Z[1], muDoubleScalarMax(sc_past->Z[0], sc_past->pJmin));
        vJ[0] = qJ(sc_past->pc, sc_past->beta, sc_past->aJ, sc_past->bJ,
                   sc_past->cJ, vp[0]);
        vJ[1] = qJ(sc_past->pc, sc_past->beta, sc_past->aJ, sc_past->bJ,
                   sc_past->cJ, vp[1]);
        vJ[2] = qJ(sc_past->pc, sc_past->beta, sc_past->aJ, sc_past->bJ,
                   sc_past->cJ, vp[2]);
        Cond = muDoubleScalarIsNaN(vJ[0]);
        if (!Cond) {
          idx = 1;
        } else {
          idx = 0;
          k = 2;
          exitg1 = false;
          while ((!exitg1) && (k < 4)) {
            if (!muDoubleScalarIsNaN(vJ[k - 1])) {
              idx = k;
              exitg1 = true;
            } else {
              k++;
            }
          }
        }
        if (idx == 0) {
          gcand = vJ[0];
        } else {
          gcand = vJ[idx - 1];
          i = idx + 1;
          for (k = i; k < 4; k++) {
            d = vJ[k - 1];
            if (gcand > d) {
              gcand = d;
            }
          }
        }
        if (!Cond) {
          idx = 1;
        } else {
          idx = 0;
          k = 2;
          exitg1 = false;
          while ((!exitg1) && (k < 4)) {
            if (!muDoubleScalarIsNaN(vJ[k - 1])) {
              idx = k;
              exitg1 = true;
            } else {
              k++;
            }
          }
        }
        if (idx == 0) {
          ex = vJ[0];
        } else {
          ex = vJ[idx - 1];
          i = idx + 1;
          for (k = i; k < 4; k++) {
            d = vJ[k - 1];
            if (ex < d) {
              ex = d;
            }
          }
        }
        if (muDoubleScalarAbs(gcand - ex) < 2.2204460492503131E-15) {
          vg[0] = qg(sc_past->pc, sc_past->beta, sc_past->ag, sc_past->bg,
                     sc_past->cg, vp[0]);
          vg[1] = qg(sc_past->pc, sc_past->beta, sc_past->ag, sc_past->bg,
                     sc_past->cg, vp[1]);
          vg[2] = qg(sc_past->pc, sc_past->beta, sc_past->ag, sc_past->bg,
                     sc_past->cg, vp[2]);
          if (!muDoubleScalarIsNaN(vg[0])) {
            idx = 1;
          } else {
            idx = 0;
            k = 2;
            exitg1 = false;
            while ((!exitg1) && (k < 4)) {
              if (!muDoubleScalarIsNaN(vg[k - 1])) {
                idx = k;
                exitg1 = true;
              } else {
                k++;
              }
            }
          }
          if (idx == 0) {
            idx = 1;
          } else {
            gcand = vg[idx - 1];
            i = idx + 1;
            for (k = i; k < 4; k++) {
              d = vg[k - 1];
              if (gcand > d) {
                gcand = d;
                idx = k;
              }
            }
          }
          pcand = vp[idx - 1];
        } else {
          if (!Cond) {
            idx = 1;
          } else {
            idx = 0;
            k = 2;
            exitg1 = false;
            while ((!exitg1) && (k < 4)) {
              if (!muDoubleScalarIsNaN(vJ[k - 1])) {
                idx = k;
                exitg1 = true;
              } else {
                k++;
              }
            }
          }
          if (idx == 0) {
            idx = 1;
          } else {
            gcand = vJ[idx - 1];
            i = idx + 1;
            for (k = i; k < 4; k++) {
              d = vJ[k - 1];
              if (gcand > d) {
                gcand = d;
                idx = k;
              }
            }
          }
          pcand = vp[idx - 1];
        }
        st.site = &m_emlrtRSI;
        BBS(&st, pcand, param->p, param->ell, param->uparam.nu,
            param->uparam.Np, param->uparam.R, param->ode.tau,
            param->ode.rk_order, param->ode.u0, param->ode.w, &param->ocp,
            param->x0, &Jcand, &gcand);
      } else {
        real_T b_Jcand[2];
        real_T b_pcand[2];
        int32_T idx;
        for (j = 0; j < 2; j++) {
          real_T d;
          real_T ex;
          boolean_T exitg1;
          pcand = sc_past->Z[j << 1];
          vp[0] = pcand;
          Jcand = sc_past->Z[((j + 1) << 1) - 1];
          vp[1] = Jcand;
          vp[2] = muDoubleScalarMin(Jcand,
                                    muDoubleScalarMax(pcand, sc_past->pJmin));
          vJ[0] = qJ(sc_past->pc, sc_past->beta, sc_past->aJ, sc_past->bJ,
                     sc_past->cJ, pcand);
          vJ[1] = qJ(sc_past->pc, sc_past->beta, sc_past->aJ, sc_past->bJ,
                     sc_past->cJ, Jcand);
          vJ[2] = qJ(sc_past->pc, sc_past->beta, sc_past->aJ, sc_past->bJ,
                     sc_past->cJ, vp[2]);
          Cond = muDoubleScalarIsNaN(vJ[0]);
          if (!Cond) {
            idx = 1;
          } else {
            idx = 0;
            k = 2;
            exitg1 = false;
            while ((!exitg1) && (k < 4)) {
              if (!muDoubleScalarIsNaN(vJ[k - 1])) {
                idx = k;
                exitg1 = true;
              } else {
                k++;
              }
            }
          }
          if (idx == 0) {
            gcand = vJ[0];
          } else {
            gcand = vJ[idx - 1];
            i = idx + 1;
            for (k = i; k < 4; k++) {
              d = vJ[k - 1];
              if (gcand > d) {
                gcand = d;
              }
            }
          }
          if (!Cond) {
            idx = 1;
          } else {
            idx = 0;
            k = 2;
            exitg1 = false;
            while ((!exitg1) && (k < 4)) {
              if (!muDoubleScalarIsNaN(vJ[k - 1])) {
                idx = k;
                exitg1 = true;
              } else {
                k++;
              }
            }
          }
          if (idx == 0) {
            ex = vJ[0];
          } else {
            ex = vJ[idx - 1];
            i = idx + 1;
            for (k = i; k < 4; k++) {
              d = vJ[k - 1];
              if (ex < d) {
                ex = d;
              }
            }
          }
          if (muDoubleScalarAbs(gcand - ex) < 2.2204460492503131E-13) {
            vg[0] = qg(sc_past->pc, sc_past->beta, sc_past->ag, sc_past->bg,
                       sc_past->cg, pcand);
            vg[1] = qg(sc_past->pc, sc_past->beta, sc_past->ag, sc_past->bg,
                       sc_past->cg, Jcand);
            vg[2] = qg(sc_past->pc, sc_past->beta, sc_past->ag, sc_past->bg,
                       sc_past->cg, vp[2]);
            if (!muDoubleScalarIsNaN(vg[0])) {
              idx = 1;
            } else {
              idx = 0;
              k = 2;
              exitg1 = false;
              while ((!exitg1) && (k < 4)) {
                if (!muDoubleScalarIsNaN(vg[k - 1])) {
                  idx = k;
                  exitg1 = true;
                } else {
                  k++;
                }
              }
            }
            if (idx == 0) {
              idx = 1;
            } else {
              gcand = vg[idx - 1];
              i = idx + 1;
              for (k = i; k < 4; k++) {
                d = vg[k - 1];
                if (gcand > d) {
                  gcand = d;
                  idx = k;
                }
              }
            }
            d = vp[idx - 1];
            b_pcand[j] = d;
          } else {
            if (!Cond) {
              idx = 1;
            } else {
              idx = 0;
              k = 2;
              exitg1 = false;
              while ((!exitg1) && (k < 4)) {
                if (!muDoubleScalarIsNaN(vJ[k - 1])) {
                  idx = k;
                  exitg1 = true;
                } else {
                  k++;
                }
              }
            }
            if (idx == 0) {
              idx = 1;
            } else {
              gcand = vJ[idx - 1];
              i = idx + 1;
              for (k = i; k < 4; k++) {
                d = vJ[k - 1];
                if (gcand > d) {
                  gcand = d;
                  idx = k;
                }
              }
            }
            d = vp[idx - 1];
            b_pcand[j] = d;
          }
          b_Jcand[j] = qJ(sc_past->pc, sc_past->beta, sc_past->aJ, sc_past->bJ,
                          sc_past->cJ, d);
          if (*emlrtBreakCheckR2012bFlagVar != 0) {
            emlrtBreakCheckR2012b((emlrtCTX)sp);
          }
        }
        if ((b_Jcand[0] > b_Jcand[1]) || (muDoubleScalarIsNaN(b_Jcand[0]) &&
                                          (!muDoubleScalarIsNaN(b_Jcand[1])))) {
          idx = 1;
        } else {
          idx = 0;
        }
        pcand = b_pcand[idx];
        st.site = &t_emlrtRSI;
        BBS(&st, b_pcand[idx], param->p, param->ell, param->uparam.nu,
            param->uparam.Np, param->uparam.R, param->ode.tau,
            param->ode.rk_order, param->ode.u0, param->ode.w, &param->ocp,
            param->x0, &Jcand, &gcand);
      }
      if (gcurrent <= 0.0) {
        if (((Jcand < Jcurrent) && (gcand <= 0.0)) ||
            ((Jcand <= Jcurrent) && (gcand < 0.0))) {
          Cond = true;
        } else {
          Cond = false;
        }
      } else {
        Cond = (gcand < gcurrent);
      }
    }
    if (Cond) {
      sc_past->p = pcand;
      sc_past->alpha *= sc_past->beta_plus;
      Jcurrent = Jcand;
      gcurrent = gcand;
    } else {
      sc_past->alpha = muDoubleScalarMax(sc_past->alpha_min,
                                         sc_past->beta_moins * sc_past->alpha);
    }
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtCTX)sp);
    }
  }
  emxFree_real_T(sp, &lesalpha);
  emxFree_real_T(sp, &lesp);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtCTX)sp);
}

/* End of code generation (update_sc.c) */
