/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * update_mv.c
 *
 * Code generation for function 'update_mv'
 *
 */

/* Include files */
#include "update_mv.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "update_mv_data.h"
#include "update_mv_emxutil.h"
#include "update_sc.h"
#include "user_control_profile.h"

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = { 27,    /* lineNo */
  "update_mv",                         /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_mv.m"/* pathName */
};

static emlrtRSInfo b_emlrtRSI = { 39,  /* lineNo */
  "update_mv",                         /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_mv.m"/* pathName */
};

static emlrtBCInfo emlrtBCI = { -1,    /* iFirst */
  -1,                                  /* iLast */
  21,                                  /* lineNo */
  12,                                  /* colNo */
  "subset",                            /* aName */
  "update_mv",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_mv.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo emlrtRTEI = { 22,  /* lineNo */
  7,                                   /* colNo */
  "update_mv",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_mv.m"/* pName */
};

static emlrtBCInfo b_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  24,                                  /* lineNo */
  24,                                  /* colNo */
  "lesp",                              /* aName */
  "update_mv",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_mv.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo c_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  28,                                  /* lineNo */
  30,                                  /* colNo */
  "lesp",                              /* aName */
  "update_mv",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_mv.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  28,                                  /* lineNo */
  16,                                  /* colNo */
  "lesp",                              /* aName */
  "update_mv",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_mv.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo e_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  37,                                  /* lineNo */
  15,                                  /* colNo */
  "lesp",                              /* aName */
  "update_mv",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_mv.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo f_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  38,                                  /* lineNo */
  14,                                  /* colNo */
  "lesp",                              /* aName */
  "update_mv",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_mv.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo emlrtDCI = { 11,    /* lineNo */
  15,                                  /* colNo */
  "update_mv",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_mv.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo b_emlrtDCI = { 11,  /* lineNo */
  1,                                   /* colNo */
  "update_mv",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_mv.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo g_emlrtBCI = { 1,   /* iFirst */
  24,                                  /* iLast */
  26,                                  /* lineNo */
  19,                                  /* colNo */
  "lesp",                              /* aName */
  "update_mv",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_mv.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo c_emlrtDCI = { 26,  /* lineNo */
  19,                                  /* colNo */
  "update_mv",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_mv.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo h_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  26,                                  /* lineNo */
  19,                                  /* colNo */
  "lesp",                              /* aName */
  "update_mv",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_mv.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo d_emlrtDCI = { 26,  /* lineNo */
  9,                                   /* colNo */
  "update_mv",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_mv.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo i_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  28,                                  /* lineNo */
  35,                                  /* colNo */
  "lesp",                              /* aName */
  "update_mv",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_mv.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo j_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  30,                                  /* lineNo */
  17,                                  /* colNo */
  "subset",                            /* aName */
  "update_mv",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_mv.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo q_emlrtRTEI = { 11,/* lineNo */
  1,                                   /* colNo */
  "update_mv",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_mv.m"/* pName */
};

static emlrtRTEInfo r_emlrtRTEI = { 37,/* lineNo */
  6,                                   /* colNo */
  "update_mv",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_mv.m"/* pName */
};

static emlrtRTEInfo s_emlrtRTEI = { 37,/* lineNo */
  1,                                   /* colNo */
  "update_mv",                         /* fName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_mv.m"/* pName */
};

/* Function Definitions */
void update_mv(const emlrtStack *sp, struct0_T mv[24], const real_T subset_data[],
               const int32_T subset_size[1], struct1_T *param, emxArray_real_T
               *u_sol, real_T p_sol[24], emxArray_real_T *lesp)
{
  real_T ell;
  real_T Nloop;
  int32_T i;
  int32_T b_i;
  int32_T loop_ub;
  uint32_T ind;
  int32_T c_i;
  real_T pmin[24];
  real_T pmax[24];
  int32_T sig;
  emxArray_real_T *b_lesp;
  int32_T i1;
  real_T c_lesp[24];
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);

  /* --------------------------------------------------------- */
  /*  Author : Mazen Alamir  */
  /*  CNRS, UNiversity of Grenoble-Alpes */
  /*  Gipsa Lab. Last modification March 2017 */
  /* --------------------------------------------------------- */
  ell = (param->Nev - 1.0) / (96.0 * param->Niter);
  if (ell < 0.0) {
    ell = muDoubleScalarCeil(ell);
  } else {
    ell = muDoubleScalarFloor(ell);
  }

  Nloop = muDoubleScalarMax(2.0, ell);
  i = lesp->size[0] * lesp->size[1];
  lesp->size[0] = 24;
  emxEnsureCapacity_real_T(sp, lesp, i, &q_emlrtRTEI);
  ell = Nloop * 24.0;
  i = (int32_T)ell;
  if (ell != i) {
    emlrtIntegerCheckR2012b(ell, &emlrtDCI, sp);
  }

  b_i = lesp->size[0] * lesp->size[1];
  lesp->size[1] = i;
  emxEnsureCapacity_real_T(sp, lesp, b_i, &q_emlrtRTEI);
  ell = Nloop * 24.0;
  i = (int32_T)ell;
  if (ell != i) {
    emlrtIntegerCheckR2012b(ell, &b_emlrtDCI, sp);
  }

  loop_ub = 24 * i;
  for (i = 0; i < loop_ub; i++) {
    lesp->data[i] = 0.0;
  }

  ind = 1U;
  for (c_i = 0; c_i < 24; c_i++) {
    lesp->data[c_i] = mv[c_i].p;
    pmin[c_i] = mv[c_i].pmin;
    pmax[c_i] = mv[c_i].pmax;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }

  loop_ub = subset_size[0];
  if (1 > subset_size[0]) {
    emlrtDynamicBoundsCheckR2012b(1, 1, subset_size[0], &emlrtBCI, sp);
  }

  ell = subset_data[0];
  i = (int32_T)(Nloop - 1.0);
  emlrtForLoopVectorCheckR2012b(1.0, 1.0, Nloop - 1.0, mxDOUBLE_CLASS, (int32_T)
    (Nloop - 1.0), &emlrtRTEI, sp);
  for (c_i = 0; c_i < i; c_i++) {
    for (sig = 0; sig < loop_ub; sig++) {
      if (((int32_T)ind < 1) || ((int32_T)ind > lesp->size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)ind, 1, lesp->size[1],
          &b_emlrtBCI, sp);
      }

      param->ell = ell;
      for (b_i = 0; b_i < 24; b_i++) {
        param->p[b_i] = lesp->data[b_i + 24 * ((int32_T)ind - 1)];
        param->pmin[b_i] = pmin[b_i];
        param->pmax[b_i] = pmax[b_i];
      }

      b_i = (int32_T)muDoubleScalarFloor(ell);
      if (ell != b_i) {
        emlrtIntegerCheckR2012b(ell, &c_emlrtDCI, sp);
      }

      if (((int32_T)ell < 1) || ((int32_T)ell > 24)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)ell, 1, 24, &g_emlrtBCI, sp);
      }

      if (((int32_T)ind < 1) || ((int32_T)ind > lesp->size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)ind, 1, lesp->size[1],
          &h_emlrtBCI, sp);
      }

      if ((int32_T)ell != b_i) {
        emlrtIntegerCheckR2012b(ell, &d_emlrtDCI, sp);
      }

      b_i = (int32_T)ind - 1;
      mv[(int32_T)ell - 1].p = lesp->data[((int32_T)ell + 24 * b_i) - 1];
      st.site = &emlrtRSI;
      update_sc(&st, &mv[(int32_T)ell - 1], param, param->Niter);
      if (((int32_T)ind < 1) || ((int32_T)ind > lesp->size[1])) {
        emlrtDynamicBoundsCheckR2012b((int32_T)ind, 1, lesp->size[1],
          &c_emlrtBCI, sp);
      }

      i1 = (int32_T)(ind + 1U);
      if ((i1 < 1) || (i1 > lesp->size[1])) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, lesp->size[1], &d_emlrtBCI, sp);
      }

      for (i1 = 0; i1 < 24; i1++) {
        c_lesp[i1] = lesp->data[i1 + 24 * b_i];
      }

      for (b_i = 0; b_i < 24; b_i++) {
        lesp->data[b_i + 24 * (int32_T)ind] = c_lesp[b_i];
      }

      b_i = (int32_T)(ind + 1U);
      if ((b_i < 1) || (b_i > lesp->size[1])) {
        emlrtDynamicBoundsCheckR2012b(b_i, 1, lesp->size[1], &i_emlrtBCI, sp);
      }

      lesp->data[((int32_T)ell + 24 * (b_i - 1)) - 1] = mv[(int32_T)ell - 1].p;
      if ((int32_T)ell < loop_ub) {
        b_i = sig + 2;
        if ((b_i < 1) || (b_i > subset_size[0])) {
          emlrtDynamicBoundsCheckR2012b(b_i, 1, subset_size[0], &j_emlrtBCI, sp);
        }

        ell = subset_data[b_i - 1];
      } else {
        ell = subset_data[0];
      }

      ind++;
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(sp);
      }
    }

    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }

  if (1 > (int32_T)(ind - 1U)) {
    loop_ub = 0;
  } else {
    loop_ub = (int32_T)(ind - 1U);
    if ((loop_ub < 1) || (loop_ub > lesp->size[1])) {
      emlrtDynamicBoundsCheckR2012b(loop_ub, 1, lesp->size[1], &e_emlrtBCI, sp);
    }
  }

  emxInit_real_T(sp, &b_lesp, 2, &r_emlrtRTEI, true);
  i = b_lesp->size[0] * b_lesp->size[1];
  b_lesp->size[0] = 24;
  b_lesp->size[1] = loop_ub;
  emxEnsureCapacity_real_T(sp, b_lesp, i, &r_emlrtRTEI);
  for (i = 0; i < loop_ub; i++) {
    for (b_i = 0; b_i < 24; b_i++) {
      i1 = b_i + 24 * i;
      b_lesp->data[i1] = lesp->data[i1];
    }
  }

  i = lesp->size[0] * lesp->size[1];
  lesp->size[0] = 24;
  lesp->size[1] = b_lesp->size[1];
  emxEnsureCapacity_real_T(sp, lesp, i, &s_emlrtRTEI);
  loop_ub = b_lesp->size[0] * b_lesp->size[1];
  for (i = 0; i < loop_ub; i++) {
    lesp->data[i] = b_lesp->data[i];
  }

  emxFree_real_T(&b_lesp);
  if (lesp->size[1] < 1) {
    emlrtDynamicBoundsCheckR2012b(lesp->size[1], 1, lesp->size[1], &f_emlrtBCI,
      sp);
  }

  for (i = 0; i < 24; i++) {
    p_sol[i] = lesp->data[i + 24 * (lesp->size[1] - 1)];
  }

  loop_ub = lesp->size[1];
  st.site = &b_emlrtRSI;
  user_control_profile(&st, *(real_T (*)[24])&lesp->data[24 * (loop_ub - 1)],
                       param->uparam.nu, param->uparam.Np, param->uparam.R,
                       u_sol);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (update_mv.c) */
