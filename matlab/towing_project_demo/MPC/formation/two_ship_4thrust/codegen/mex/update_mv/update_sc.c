/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
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
#include "mwmathutil.h"
#include "qJ.h"
#include "qg.h"
#include "rt_nonfinite.h"
#include "update_mv.h"
#include "update_mv_data.h"
#include "update_mv_emxutil.h"

/* Variable Definitions */
static emlrtRSInfo c_emlrtRSI = { 9,   /* lineNo */
  "update_sc",                         /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m"/* pathName */
};

static emlrtRSInfo d_emlrtRSI = { 13,  /* lineNo */
  "update_sc",                         /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m"/* pathName */
};

static emlrtRSInfo e_emlrtRSI = { 15,  /* lineNo */
  "update_sc",                         /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m"/* pathName */
};

static emlrtRSInfo f_emlrtRSI = { 19,  /* lineNo */
  "update_sc",                         /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m"/* pathName */
};

static emlrtRSInfo g_emlrtRSI = { 22,  /* lineNo */
  "update_sc",                         /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m"/* pathName */
};

static emlrtRSInfo m_emlrtRSI = { 34,  /* lineNo */
  "update_sc",                         /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m"/* pathName */
};

static emlrtRSInfo t_emlrtRSI = { 53,  /* lineNo */
  "update_sc",                         /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m"/* pathName */
};

static emlrtRTEInfo q_emlrtRTEI = { 11,/* lineNo */
  7,                                   /* colNo */
  "update_sc",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m"/* pName */
};

static emlrtDCInfo p_emlrtDCI = { 10,  /* lineNo */
  12,                                  /* colNo */
  "update_sc",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo q_emlrtDCI = { 10,  /* lineNo */
  12,                                  /* colNo */
  "update_sc",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m",/* pName */
  4                                    /* checkKind */
};

static emlrtBCInfo fb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  12,                                  /* lineNo */
  5,                                   /* colNo */
  "lesp",                              /* aName */
  "update_sc",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo gb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  12,                                  /* lineNo */
  18,                                  /* colNo */
  "lesalpha",                          /* aName */
  "update_sc",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo qb_emlrtRTEI = { 10,/* lineNo */
  21,                                  /* colNo */
  "update_sc",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m"/* pName */
};

static emlrtRTEInfo rb_emlrtRTEI = { 10,/* lineNo */
  6,                                   /* colNo */
  "update_sc",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m"/* pName */
};

static emlrtRTEInfo sb_emlrtRTEI = { 7,/* lineNo */
  14,                                  /* colNo */
  "update_sc",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m"/* pName */
};

static emlrtRTEInfo tb_emlrtRTEI = { 7,/* lineNo */
  19,                                  /* colNo */
  "update_sc",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m"/* pName */
};

/* Function Definitions */
void update_sc(const emlrtStack *sp, struct0_T *sc_past, const struct1_T *param,
               real_T Niter)
{
  real_T Jcurrent;
  real_T gcurrent;
  emxArray_real_T *lesalpha;
  int32_T loop_ub;
  int32_T i;
  emxArray_real_T *lesp;
  int32_T b_i;
  struct0_T b_sc_past;
  real_T Jcand_idx_1;
  real_T Jcand;
  real_T gcand;
  boolean_T Cond;
  real_T vp[3];
  real_T vJ[3];
  int32_T idx;
  int32_T k;
  boolean_T exitg1;
  real_T vg[3];
  real_T pcand[2];
  real_T Jcand_idx_0;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);

  /* --------------------------------------------------------- */
  /*  Author : Mazen Alamir  */
  /*  CNRS, UNiversity of Grenoble-Alpes */
  /*  Gipsa Lab. Last modification March 2017 */
  /* --------------------------------------------------------- */
  st.site = &c_emlrtRSI;
  BBS(&st, sc_past->p, param->p, param->ell, param->uparam.nu, param->uparam.Np,
      param->uparam.R, param->ode.Nship, param->ode.tau, param->ode.rk_order,
      param->ode.M, param->ode.D, param->ode.thrust_config, param->ode.u0,
      &param->ocp, param->x0, &Jcurrent, &gcurrent);
  if (!(Niter >= 0.0)) {
    emlrtNonNegativeCheckR2012b(Niter, &q_emlrtDCI, sp);
  }

  if (Niter != (int32_T)muDoubleScalarFloor(Niter)) {
    emlrtIntegerCheckR2012b(Niter, &p_emlrtDCI, sp);
  }

  emxInit_real_T(sp, &lesalpha, 1, &tb_emlrtRTEI, true);
  loop_ub = (int32_T)Niter;
  i = lesalpha->size[0];
  lesalpha->size[0] = loop_ub;
  emxEnsureCapacity_real_T(sp, lesalpha, i, &qb_emlrtRTEI);
  for (i = 0; i < loop_ub; i++) {
    lesalpha->data[i] = 0.0;
  }

  emxInit_real_T(sp, &lesp, 1, &sb_emlrtRTEI, true);
  emlrtForLoopVectorCheckR2012b(1.0, 1.0, Niter, mxDOUBLE_CLASS, (int32_T)Niter,
    &q_emlrtRTEI, sp);
  i = lesp->size[0];
  lesp->size[0] = loop_ub;
  emxEnsureCapacity_real_T(sp, lesp, i, &rb_emlrtRTEI);
  for (b_i = 0; b_i < loop_ub; b_i++) {
    i = (int32_T)(b_i + 1U);
    if ((i < 1) || (i > lesp->size[0])) {
      emlrtDynamicBoundsCheckR2012b(i, 1, lesp->size[0], &fb_emlrtBCI, sp);
    }

    lesp->data[i - 1] = sc_past->p;
    if (i > lesalpha->size[0]) {
      emlrtDynamicBoundsCheckR2012b(i, 1, lesalpha->size[0], &gb_emlrtBCI, sp);
    }

    lesalpha->data[i - 1] = sc_past->alpha;
    b_sc_past = *sc_past;
    st.site = &d_emlrtRSI;
    compute_parabola(&st, &b_sc_past, param, sc_past);
    if (sc_past->qgmax <= 0.0) {
      Jcand_idx_1 = sc_past->pJmin;
      st.site = &e_emlrtRSI;
      BBS(&st, sc_past->pJmin, param->p, param->ell, param->uparam.nu,
          param->uparam.Np, param->uparam.R, param->ode.Nship, param->ode.tau,
          param->ode.rk_order, param->ode.M, param->ode.D,
          param->ode.thrust_config, param->ode.u0, &param->ocp, param->x0,
          &Jcand, &gcand);
      if (((Jcand < Jcurrent) && (gcand <= 0.0)) || ((Jcand <= Jcurrent) &&
           (gcand < 0.0))) {
        Cond = true;
      } else {
        Cond = false;
      }
    } else if (sc_past->qgmin > 0.0) {
      Jcand_idx_1 = sc_past->pgmin;
      st.site = &f_emlrtRSI;
      BBS(&st, sc_past->pgmin, param->p, param->ell, param->uparam.nu,
          param->uparam.Np, param->uparam.R, param->ode.Nship, param->ode.tau,
          param->ode.rk_order, param->ode.M, param->ode.D,
          param->ode.thrust_config, param->ode.u0, &param->ocp, param->x0,
          &Jcand, &gcand);
      Cond = (gcand < gcurrent);
    } else {
      b_sc_past = *sc_past;
      st.site = &g_emlrtRSI;
      compute_Zg(&st, &b_sc_past, sc_past);
      if (sc_past->ngz == 1.0) {
        vp[0] = sc_past->Z[0];
        vp[1] = sc_past->Z[1];
        vp[2] = muDoubleScalarMin(sc_past->Z[1], muDoubleScalarMax(sc_past->Z[0],
          sc_past->pJmin));
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
          while ((!exitg1) && (k <= 3)) {
            if (!muDoubleScalarIsNaN(vJ[k - 1])) {
              idx = k;
              exitg1 = true;
            } else {
              k++;
            }
          }
        }

        if (idx == 0) {
          Jcand_idx_1 = vJ[0];
        } else {
          Jcand_idx_1 = vJ[idx - 1];
          i = idx + 1;
          for (k = i; k < 4; k++) {
            Jcand = vJ[k - 1];
            if (Jcand_idx_1 > Jcand) {
              Jcand_idx_1 = Jcand;
            }
          }
        }

        if (!Cond) {
          idx = 1;
        } else {
          idx = 0;
          k = 2;
          exitg1 = false;
          while ((!exitg1) && (k <= 3)) {
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
          idx++;
          for (k = idx; k < 4; k++) {
            Jcand = vJ[k - 1];
            if (gcand < Jcand) {
              gcand = Jcand;
            }
          }
        }

        if (muDoubleScalarAbs(Jcand_idx_1 - gcand) < 2.2204460492503131E-15) {
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
            Jcand_idx_1 = vg[idx - 1];
            i = idx + 1;
            for (k = i; k < 4; k++) {
              Jcand = vg[k - 1];
              if (Jcand_idx_1 > Jcand) {
                Jcand_idx_1 = Jcand;
                idx = k;
              }
            }
          }

          Jcand_idx_1 = vp[idx - 1];
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
            Jcand_idx_1 = vJ[idx - 1];
            i = idx + 1;
            for (k = i; k < 4; k++) {
              Jcand = vJ[k - 1];
              if (Jcand_idx_1 > Jcand) {
                Jcand_idx_1 = Jcand;
                idx = k;
              }
            }
          }

          Jcand_idx_1 = vp[idx - 1];
        }

        st.site = &m_emlrtRSI;
        BBS(&st, Jcand_idx_1, param->p, param->ell, param->uparam.nu,
            param->uparam.Np, param->uparam.R, param->ode.Nship, param->ode.tau,
            param->ode.rk_order, param->ode.M, param->ode.D,
            param->ode.thrust_config, param->ode.u0, &param->ocp, param->x0,
            &Jcand, &gcand);
      } else {
        vp[0] = sc_past->Z[0];
        vp[1] = sc_past->Z[1];
        vp[2] = muDoubleScalarMin(sc_past->Z[1], muDoubleScalarMax(sc_past->Z[0],
          sc_past->pJmin));
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
          while ((!exitg1) && (k <= 3)) {
            if (!muDoubleScalarIsNaN(vJ[k - 1])) {
              idx = k;
              exitg1 = true;
            } else {
              k++;
            }
          }
        }

        if (idx == 0) {
          Jcand_idx_1 = vJ[0];
        } else {
          Jcand_idx_1 = vJ[idx - 1];
          i = idx + 1;
          for (k = i; k < 4; k++) {
            Jcand = vJ[k - 1];
            if (Jcand_idx_1 > Jcand) {
              Jcand_idx_1 = Jcand;
            }
          }
        }

        if (!Cond) {
          idx = 1;
        } else {
          idx = 0;
          k = 2;
          exitg1 = false;
          while ((!exitg1) && (k <= 3)) {
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
          idx++;
          for (k = idx; k < 4; k++) {
            Jcand = vJ[k - 1];
            if (gcand < Jcand) {
              gcand = Jcand;
            }
          }
        }

        if (muDoubleScalarAbs(Jcand_idx_1 - gcand) < 2.2204460492503131E-13) {
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
            Jcand_idx_1 = vg[idx - 1];
            i = idx + 1;
            for (k = i; k < 4; k++) {
              Jcand = vg[k - 1];
              if (Jcand_idx_1 > Jcand) {
                Jcand_idx_1 = Jcand;
                idx = k;
              }
            }
          }

          Jcand = vp[idx - 1];
          pcand[0] = Jcand;
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
            Jcand_idx_1 = vJ[idx - 1];
            i = idx + 1;
            for (k = i; k < 4; k++) {
              Jcand = vJ[k - 1];
              if (Jcand_idx_1 > Jcand) {
                Jcand_idx_1 = Jcand;
                idx = k;
              }
            }
          }

          Jcand = vp[idx - 1];
          pcand[0] = Jcand;
        }

        Jcand_idx_0 = qJ(sc_past->pc, sc_past->beta, sc_past->aJ, sc_past->bJ,
                         sc_past->cJ, Jcand);
        if (*emlrtBreakCheckR2012bFlagVar != 0) {
          emlrtBreakCheckR2012b(sp);
        }

        vp[0] = sc_past->Z[2];
        vp[1] = sc_past->Z[3];
        vp[2] = muDoubleScalarMin(sc_past->Z[3], muDoubleScalarMax(sc_past->Z[2],
          sc_past->pJmin));
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
          while ((!exitg1) && (k <= 3)) {
            if (!muDoubleScalarIsNaN(vJ[k - 1])) {
              idx = k;
              exitg1 = true;
            } else {
              k++;
            }
          }
        }

        if (idx == 0) {
          Jcand_idx_1 = vJ[0];
        } else {
          Jcand_idx_1 = vJ[idx - 1];
          i = idx + 1;
          for (k = i; k < 4; k++) {
            Jcand = vJ[k - 1];
            if (Jcand_idx_1 > Jcand) {
              Jcand_idx_1 = Jcand;
            }
          }
        }

        if (!Cond) {
          idx = 1;
        } else {
          idx = 0;
          k = 2;
          exitg1 = false;
          while ((!exitg1) && (k <= 3)) {
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
          idx++;
          for (k = idx; k < 4; k++) {
            Jcand = vJ[k - 1];
            if (gcand < Jcand) {
              gcand = Jcand;
            }
          }
        }

        if (muDoubleScalarAbs(Jcand_idx_1 - gcand) < 2.2204460492503131E-13) {
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
            Jcand_idx_1 = vg[idx - 1];
            i = idx + 1;
            for (k = i; k < 4; k++) {
              Jcand = vg[k - 1];
              if (Jcand_idx_1 > Jcand) {
                Jcand_idx_1 = Jcand;
                idx = k;
              }
            }
          }

          Jcand = vp[idx - 1];
          pcand[1] = Jcand;
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
            Jcand_idx_1 = vJ[idx - 1];
            i = idx + 1;
            for (k = i; k < 4; k++) {
              Jcand = vJ[k - 1];
              if (Jcand_idx_1 > Jcand) {
                Jcand_idx_1 = Jcand;
                idx = k;
              }
            }
          }

          Jcand = vp[idx - 1];
          pcand[1] = Jcand;
        }

        Jcand_idx_1 = qJ(sc_past->pc, sc_past->beta, sc_past->aJ, sc_past->bJ,
                         sc_past->cJ, Jcand);
        if (*emlrtBreakCheckR2012bFlagVar != 0) {
          emlrtBreakCheckR2012b(sp);
        }

        if ((Jcand_idx_0 > Jcand_idx_1) || (muDoubleScalarIsNaN(Jcand_idx_0) &&
             (!muDoubleScalarIsNaN(Jcand_idx_1)))) {
          idx = 1;
        } else {
          idx = 0;
        }

        Jcand_idx_1 = pcand[idx];
        st.site = &t_emlrtRSI;
        BBS(&st, pcand[idx], param->p, param->ell, param->uparam.nu,
            param->uparam.Np, param->uparam.R, param->ode.Nship, param->ode.tau,
            param->ode.rk_order, param->ode.M, param->ode.D,
            param->ode.thrust_config, param->ode.u0, &param->ocp, param->x0,
            &Jcand, &gcand);
      }

      if (gcurrent <= 0.0) {
        if (((Jcand < Jcurrent) && (gcand <= 0.0)) || ((Jcand <= Jcurrent) &&
             (gcand < 0.0))) {
          Cond = true;
        } else {
          Cond = false;
        }
      } else {
        Cond = (gcand < gcurrent);
      }
    }

    if (Cond) {
      sc_past->p = Jcand_idx_1;
      sc_past->alpha *= sc_past->beta_plus;
      Jcurrent = Jcand;
      gcurrent = gcand;
    } else {
      sc_past->alpha = muDoubleScalarMax(sc_past->alpha_min, sc_past->beta_moins
        * sc_past->alpha);
    }

    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }

  emxFree_real_T(&lesalpha);
  emxFree_real_T(&lesp);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (update_sc.c) */
