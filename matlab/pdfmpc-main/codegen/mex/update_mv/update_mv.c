/*
 * update_mv.c
 *
 * Code generation for function 'update_mv'
 *
 */

/* Include files */
#include "update_mv.h"
#include "rt_nonfinite.h"
#include "update_mv_data.h"
#include "update_mv_emxutil.h"
#include "update_mv_types.h"
#include "update_sc.h"
#include "user_control_profile.h"
#include "mwmathutil.h"
#include <math.h>

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = {
    27,                                                   /* lineNo */
    "update_mv",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_mv.m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    39,                                                   /* lineNo */
    "update_mv",                                          /* fcnName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_mv.m" /* pathName */
};

static emlrtBCInfo emlrtBCI = {
    -1,                                                    /* iFirst */
    -1,                                                    /* iLast */
    21,                                                    /* lineNo */
    12,                                                    /* colNo */
    "subset",                                              /* aName */
    "update_mv",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_mv.m", /* pName */
    0                                                      /* checkKind */
};

static emlrtRTEInfo emlrtRTEI = {
    22,                                                   /* lineNo */
    7,                                                    /* colNo */
    "update_mv",                                          /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_mv.m" /* pName */
};

static emlrtBCInfo b_emlrtBCI = {
    -1,                                                    /* iFirst */
    -1,                                                    /* iLast */
    24,                                                    /* lineNo */
    24,                                                    /* colNo */
    "lesp",                                                /* aName */
    "update_mv",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_mv.m", /* pName */
    0                                                      /* checkKind */
};

static emlrtBCInfo c_emlrtBCI = {
    -1,                                                    /* iFirst */
    -1,                                                    /* iLast */
    28,                                                    /* lineNo */
    30,                                                    /* colNo */
    "lesp",                                                /* aName */
    "update_mv",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_mv.m", /* pName */
    0                                                      /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = {
    -1,                                                    /* iFirst */
    -1,                                                    /* iLast */
    28,                                                    /* lineNo */
    16,                                                    /* colNo */
    "lesp",                                                /* aName */
    "update_mv",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_mv.m", /* pName */
    0                                                      /* checkKind */
};

static emlrtBCInfo e_emlrtBCI = {
    -1,                                                    /* iFirst */
    -1,                                                    /* iLast */
    37,                                                    /* lineNo */
    15,                                                    /* colNo */
    "lesp",                                                /* aName */
    "update_mv",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_mv.m", /* pName */
    0                                                      /* checkKind */
};

static emlrtBCInfo f_emlrtBCI = {
    -1,                                                    /* iFirst */
    -1,                                                    /* iLast */
    38,                                                    /* lineNo */
    14,                                                    /* colNo */
    "lesp",                                                /* aName */
    "update_mv",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_mv.m", /* pName */
    0                                                      /* checkKind */
};

static emlrtDCInfo emlrtDCI = {
    11,                                                    /* lineNo */
    15,                                                    /* colNo */
    "update_mv",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_mv.m", /* pName */
    1                                                      /* checkKind */
};

static emlrtDCInfo b_emlrtDCI = {
    11,                                                    /* lineNo */
    1,                                                     /* colNo */
    "update_mv",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_mv.m", /* pName */
    1                                                      /* checkKind */
};

static emlrtBCInfo g_emlrtBCI = {
    -1,                                                    /* iFirst */
    -1,                                                    /* iLast */
    26,                                                    /* lineNo */
    28,                                                    /* colNo */
    "lesp",                                                /* aName */
    "update_mv",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_mv.m", /* pName */
    0                                                      /* checkKind */
};

static emlrtBCInfo h_emlrtBCI = {
    1,                                                     /* iFirst */
    4,                                                     /* iLast */
    26,                                                    /* lineNo */
    12,                                                    /* colNo */
    "mv",                                                  /* aName */
    "update_mv",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_mv.m", /* pName */
    3                                                      /* checkKind */
};

static emlrtDCInfo c_emlrtDCI = {
    26,                                                    /* lineNo */
    12,                                                    /* colNo */
    "update_mv",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_mv.m", /* pName */
    1                                                      /* checkKind */
};

static emlrtBCInfo i_emlrtBCI = {
    -1,                                                    /* iFirst */
    -1,                                                    /* iLast */
    28,                                                    /* lineNo */
    44,                                                    /* colNo */
    "lesp",                                                /* aName */
    "update_mv",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_mv.m", /* pName */
    0                                                      /* checkKind */
};

static emlrtBCInfo j_emlrtBCI = {
    -1,                                                    /* iFirst */
    -1,                                                    /* iLast */
    30,                                                    /* lineNo */
    24,                                                    /* colNo */
    "subset",                                              /* aName */
    "update_mv",                                           /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_mv.m", /* pName */
    0                                                      /* checkKind */
};

static emlrtRTEInfo n_emlrtRTEI = {
    11,                                                   /* lineNo */
    1,                                                    /* colNo */
    "update_mv",                                          /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_mv.m" /* pName */
};

static emlrtRTEInfo o_emlrtRTEI = {
    37,                                                   /* lineNo */
    1,                                                    /* colNo */
    "update_mv",                                          /* fName */
    "C:\\Download\\pdfmpc-main\\pdfmpc-main\\update_mv.m" /* pName */
};

/* Function Definitions */
void update_mv(const emlrtStack *sp, struct0_T mv[4],
               const real_T subset_data[], const int32_T subset_size[1],
               struct1_T *param, real_T u_sol_data[], int32_T u_sol_size[2],
               real_T p_sol[4], emxArray_real_T *lesp)
{
  emlrtStack st;
  real_T Nloop;
  real_T ell;
  real_T lesp_idx_1;
  real_T *lesp_data;
  int32_T b_i;
  int32_T i;
  int32_T i1;
  int32_T npr;
  int32_T sig;
  uint32_T ind;
  st.prev = sp;
  st.tls = sp->tls;
  /* --------------------------------------------------------- */
  /*  Author : Mazen Alamir  */
  /*  CNRS, UNiversity of Grenoble-Alpes */
  /*  Gipsa Lab. Last modification March 2017 */
  /* --------------------------------------------------------- */
  Nloop =
      muDoubleScalarMax(2.0, trunc((param->Nev - 1.0) / (16.0 * param->Niter)));
  i = lesp->size[0] * lesp->size[1];
  lesp->size[0] = 4;
  emxEnsureCapacity_real_T(sp, lesp, i, &n_emlrtRTEI);
  lesp_idx_1 = Nloop * 4.0;
  if (lesp_idx_1 != (int32_T)lesp_idx_1) {
    emlrtIntegerCheckR2012b(lesp_idx_1, &emlrtDCI, (emlrtCTX)sp);
  }
  i = lesp->size[0] * lesp->size[1];
  lesp->size[1] = (int32_T)lesp_idx_1;
  emxEnsureCapacity_real_T(sp, lesp, i, &n_emlrtRTEI);
  lesp_data = lesp->data;
  if (lesp_idx_1 != (int32_T)lesp_idx_1) {
    emlrtIntegerCheckR2012b(lesp_idx_1, &b_emlrtDCI, (emlrtCTX)sp);
  }
  npr = (int32_T)lesp_idx_1 << 2;
  for (i = 0; i < npr; i++) {
    lesp_data[i] = 0.0;
  }
  ind = 1U;
  lesp_data[0] = mv[0].p;
  param->pmin[0] = mv[0].pmin;
  param->pmax[0] = mv[0].pmax;
  if (*emlrtBreakCheckR2012bFlagVar != 0) {
    emlrtBreakCheckR2012b((emlrtCTX)sp);
  }
  lesp_data[1] = mv[1].p;
  param->pmin[1] = mv[1].pmin;
  param->pmax[1] = mv[1].pmax;
  if (*emlrtBreakCheckR2012bFlagVar != 0) {
    emlrtBreakCheckR2012b((emlrtCTX)sp);
  }
  lesp_data[2] = mv[2].p;
  param->pmin[2] = mv[2].pmin;
  param->pmax[2] = mv[2].pmax;
  if (*emlrtBreakCheckR2012bFlagVar != 0) {
    emlrtBreakCheckR2012b((emlrtCTX)sp);
  }
  lesp_data[3] = mv[3].p;
  param->pmin[3] = mv[3].pmin;
  param->pmax[3] = mv[3].pmax;
  if (*emlrtBreakCheckR2012bFlagVar != 0) {
    emlrtBreakCheckR2012b((emlrtCTX)sp);
  }
  npr = subset_size[0];
  if (subset_size[0] < 1) {
    emlrtDynamicBoundsCheckR2012b(1, 1, subset_size[0], &emlrtBCI,
                                  (emlrtCTX)sp);
  }
  ell = subset_data[0];
  i = (int32_T)(Nloop - 1.0);
  emlrtForLoopVectorCheckR2021a(1.0, 1.0, Nloop - 1.0, mxDOUBLE_CLASS,
                                (int32_T)(Nloop - 1.0), &emlrtRTEI,
                                (emlrtCTX)sp);
  for (b_i = 0; b_i < i; b_i++) {
    for (sig = 0; sig < npr; sig++) {
      real_T lesp_idx_3;
      int32_T i2;
      uint32_T u;
      u = ind + sig;
      if (((int32_T)u < 1) || ((int32_T)u > lesp->size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)u, 1, lesp->size[1], &b_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      i1 = 4 * ((int32_T)u - 1);
      param->p[0] = lesp_data[i1];
      param->p[1] = lesp_data[i1 + 1];
      param->p[2] = lesp_data[i1 + 2];
      param->p[3] = lesp_data[i1 + 3];
      param->ell = ell;
      if (((int32_T)u < 1) || ((int32_T)u > lesp->size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)u, 1, lesp->size[1], &g_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      if (ell != (int32_T)muDoubleScalarFloor(ell)) {
        emlrtIntegerCheckR2012b(ell, &c_emlrtDCI, (emlrtCTX)sp);
      }
      if (((int32_T)ell < 1) || ((int32_T)ell > 4)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)ell, 1, 4, &h_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      mv[(int32_T)ell - 1].p = lesp_data[((int32_T)ell + i1) - 1];
      st.site = &emlrtRSI;
      update_sc(&st, &mv[(int32_T)ell - 1], param, param->Niter);
      if (((int32_T)u < 1) || ((int32_T)u > lesp->size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)u, 1, lesp->size[1], &c_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      if (((int32_T)(u + 1U) < 1) || ((int32_T)(u + 1U) > lesp->size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)(u + 1U), 1, lesp->size[1],
                                      &d_emlrtBCI, (emlrtCTX)sp);
      }
      lesp_idx_1 = lesp_data[i1 + 1];
      Nloop = lesp_data[i1 + 2];
      lesp_idx_3 = lesp_data[i1 + 3];
      i2 = 4 * (int32_T)u;
      lesp_data[i2] = lesp_data[i1];
      lesp_data[i2 + 1] = lesp_idx_1;
      lesp_data[i2 + 2] = Nloop;
      lesp_data[i2 + 3] = lesp_idx_3;
      if (((int32_T)(u + 1U) < 1) || ((int32_T)(u + 1U) > lesp->size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)(u + 1U), 1, lesp->size[1],
                                      &i_emlrtBCI, (emlrtCTX)sp);
      }
      lesp_data[((int32_T)ell + i2) - 1] = mv[(int32_T)ell - 1].p;
      if ((int32_T)ell < npr) {
        if (sig + 2 > subset_size[0]) {
          emlrtDynamicBoundsCheckR2012b(sig + 2, 1, subset_size[0], &j_emlrtBCI,
                                        (emlrtCTX)sp);
        }
        ell = subset_data[sig + 1];
      } else {
        ell = subset_data[0];
      }
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b((emlrtCTX)sp);
      }
    }
    ind += npr;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtCTX)sp);
    }
  }
  if ((int32_T)(ind - 1U) < 1) {
    i = 0;
  } else {
    if (((int32_T)(ind - 1U) < 1) || ((int32_T)(ind - 1U) > lesp->size[1])) {
      emlrtDynamicBoundsCheckR2012b((int32_T)(ind - 1U), 1, lesp->size[1],
                                    &e_emlrtBCI, (emlrtCTX)sp);
    }
    i = (int32_T)(ind - 1U);
  }
  i1 = lesp->size[0] * lesp->size[1];
  lesp->size[0] = 4;
  lesp->size[1] = i;
  emxEnsureCapacity_real_T(sp, lesp, i1, &o_emlrtRTEI);
  lesp_data = lesp->data;
  if (i < 1) {
    emlrtDynamicBoundsCheckR2012b(i, 1, i, &f_emlrtBCI, (emlrtCTX)sp);
  }
  i1 = 4 * (i - 1);
  p_sol[0] = lesp_data[i1];
  p_sol[1] = lesp_data[i1 + 1];
  p_sol[2] = lesp_data[i1 + 2];
  p_sol[3] = lesp_data[i1 + 3];
  st.site = &b_emlrtRSI;
  user_control_profile(&st, *(real_T(*)[4]) & lesp_data[4 * (i - 1)],
                       param->uparam.nu, param->uparam.Np, param->uparam.R,
                       u_sol_data, u_sol_size);
}

/* End of code generation (update_mv.c) */
