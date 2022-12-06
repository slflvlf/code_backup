/*
 * nlmpcmoveCodeGeneration.c
 *
 * Code generation for function 'nlmpcmoveCodeGeneration'
 *
 */

/* Include files */
#include "nlmpcmoveCodeGeneration.h"
#include "OneShipStateFcn.h"
#include "OneShipStateJacobianFcn.h"
#include "fmincon.h"
#include "indexShapeCheck.h"
#include "mtimes.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_internal_types.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "reshapeSizeChecks.h"
#include "rt_nonfinite.h"
#include "znlmpc_confun.h"
#include "znlmpc_generateRuntimeData.h"
#include "znlmpc_getXUe.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = {
    56,                        /* lineNo */
    "nlmpcmoveCodeGeneration", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpc\\nlmpcmoveCodeGeneration.m" /* pathName
                                                                           */
};

static emlrtRSInfo b_emlrtRSI = {
    58,                        /* lineNo */
    "nlmpcmoveCodeGeneration", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpc\\nlmpcmoveCodeGeneration.m" /* pathName
                                                                           */
};

static emlrtRSInfo c_emlrtRSI = {
    101,                       /* lineNo */
    "nlmpcmoveCodeGeneration", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpc\\nlmpcmoveCodeGeneration.m" /* pathName
                                                                           */
};

static emlrtRSInfo d_emlrtRSI = {
    122,                       /* lineNo */
    "nlmpcmoveCodeGeneration", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpc\\nlmpcmoveCodeGeneration.m" /* pathName
                                                                           */
};

static emlrtRSInfo e_emlrtRSI = {
    142,                       /* lineNo */
    "nlmpcmoveCodeGeneration", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpc\\nlmpcmoveCodeGeneration.m" /* pathName
                                                                           */
};

static emlrtRSInfo f_emlrtRSI = {
    149,                       /* lineNo */
    "nlmpcmoveCodeGeneration", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpc\\nlmpcmoveCodeGeneration.m" /* pathName
                                                                           */
};

static emlrtRSInfo u_emlrtRSI = {
    50,                  /* lineNo */
    "znlmpc_getUBounds", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m" /* pathName
                                                                          */
};

static emlrtRSInfo v_emlrtRSI = {
    65,                  /* lineNo */
    "znlmpc_getUBounds", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m" /* pathName
                                                                          */
};

static emlrtRSInfo w_emlrtRSI = {
    39,     /* lineNo */
    "find", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\eml\\lib\\matlab\\elmat\\find.m" /* pathName
                                                                       */
};

static emlrtRSInfo rb_emlrtRSI =
    {
        84,              /* lineNo */
        "znlmpc_confun", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_confun.m" /* pathName
                                                                          */
};

static emlrtRSInfo ub_emlrtRSI =
    {
        339,              /* lineNo */
        "stateEvolution", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_confun.m" /* pathName
                                                                          */
};

static emlrtBCInfo c_emlrtBCI = {
    1,                /* iFirst */
    60,               /* iLast */
    246,              /* lineNo */
    9,                /* colNo */
    "",               /* aName */
    "stateEvolution", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_confun.m", /* pName
                                                                       */
    3 /* checkKind */
};

static emlrtBCInfo t_emlrtBCI = {
    1,                   /* iFirst */
    320,                 /* iLast */
    22,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo u_emlrtBCI = {
    1,                   /* iFirst */
    320,                 /* iLast */
    23,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo v_emlrtBCI = {
    1,                   /* iFirst */
    320,                 /* iLast */
    24,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo w_emlrtBCI = {
    1,                   /* iFirst */
    320,                 /* iLast */
    25,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo x_emlrtBCI = {
    1,                   /* iFirst */
    320,                 /* iLast */
    28,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo y_emlrtBCI = {
    1,                   /* iFirst */
    320,                 /* iLast */
    29,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo ab_emlrtBCI = {
    1,                   /* iFirst */
    320,                 /* iLast */
    30,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo bb_emlrtBCI = {
    1,                   /* iFirst */
    320,                 /* iLast */
    33,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo cb_emlrtBCI = {
    1,                   /* iFirst */
    320,                 /* iLast */
    34,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo db_emlrtBCI = {
    1,                   /* iFirst */
    320,                 /* iLast */
    35,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo eb_emlrtBCI = {
    1,                   /* iFirst */
    320,                 /* iLast */
    37,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo fb_emlrtBCI = {
    1,                   /* iFirst */
    320,                 /* iLast */
    38,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo gb_emlrtBCI = {
    1,                   /* iFirst */
    320,                 /* iLast */
    39,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo hb_emlrtBCI = {
    1,                   /* iFirst */
    320,                 /* iLast */
    45,                  /* lineNo */
    9,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo ib_emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    60,                  /* lineNo */
    37,                  /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtBCInfo jb_emlrtBCI = {
    1,                   /* iFirst */
    320,                 /* iLast */
    42,                  /* lineNo */
    27,                  /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtBCInfo kb_emlrtBCI = {
    1,                   /* iFirst */
    320,                 /* iLast */
    42,                  /* lineNo */
    9,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

/* Function Definitions */
void c_nlmpcmoveCodeGeneration_anonF(
    c_nlmpcmoveCodeGenerationStackD *SD, const emlrtStack *sp,
    const real_T runtimedata_x[6], const real_T runtimedata_OutputMin[60],
    const real_T runtimedata_OutputMax[60], const real_T z[85],
    real_T varargout_1_data[], int32_T varargout_1_size[2],
    real_T varargout_2[60], real_T varargout_3_data[],
    int32_T varargout_3_size[2], real_T varargout_4[5100])
{
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  real_T Jmv[4800];
  real_T Jx[3600];
  real_T y[1440];
  real_T c_data[120];
  real_T U[88];
  real_T b_U[88];
  real_T X[66];
  real_T b_X[66];
  real_T alpha1;
  real_T beta1;
  real_T e;
  int32_T Jc_size[2];
  int32_T c_size[2];
  int32_T Jx_tmp;
  int32_T b_i;
  int32_T i;
  int32_T i1;
  int32_T k;
  char_T TRANSA1;
  char_T TRANSB1;
  int8_T ic[6];
  int8_T input_sizes_idx_0;
  int8_T sizes_idx_1;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &mb_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  b_st.site = &nb_emlrtRSI;
  znlmpc_getXUe(z, runtimedata_x, X, U, &e);
  b_st.site = &ob_emlrtRSI;
  memset(&Jx[0], 0, 3600U * sizeof(real_T));
  memset(&Jmv[0], 0, 4800U * sizeof(real_T));
  memset(&varargout_2[0], 0, 60U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    ic[i] = (int8_T)(i + 1);
  }
  for (i = 0; i < 11; i++) {
    for (i1 = 0; i1 < 8; i1++) {
      b_U[i1 + (i << 3)] = U[i + 11 * i1];
    }
    for (i1 = 0; i1 < 6; i1++) {
      b_X[i1 + 6 * i] = X[i + 11 * i1];
    }
  }
  for (b_i = 0; b_i < 10; b_i++) {
    real_T Bk1[48];
    real_T val[48];
    real_T Ak[36];
    real_T Ak1[36];
    real_T b_dv[6];
    real_T dv1[6];
    OneShipStateJacobianFcn(*(real_T(*)[6]) & b_X[6 * b_i],
                            *(real_T(*)[8]) & b_U[b_i << 3], Ak, val);
    OneShipStateJacobianFcn(*(real_T(*)[6]) & b_X[6 * (b_i + 1)],
                            *(real_T(*)[8]) & b_U[b_i << 3], Ak1, Bk1);
    i = b_i << 3;
    OneShipStateFcn(*(real_T(*)[6]) & b_X[6 * b_i], *(real_T(*)[8]) & b_U[i],
                    b_dv);
    i1 = 6 * (b_i + 1);
    OneShipStateFcn(*(real_T(*)[6]) & b_X[i1], *(real_T(*)[8]) & b_U[i], dv1);
    for (i = 0; i < 6; i++) {
      sizes_idx_1 = ic[i];
      if ((sizes_idx_1 < 1) || (sizes_idx_1 > 60)) {
        emlrtDynamicBoundsCheckR2012b(sizes_idx_1, 1, 60, &c_emlrtBCI, &b_st);
      }
      varargout_2[ic[i] - 1] =
          (b_X[i + 6 * b_i] + 0.5 * (b_dv[i] + dv1[i])) - b_X[i + i1];
    }
    if (b_i + 1 > 1) {
      for (k = 0; k < 6; k++) {
        for (i = 0; i < 6; i++) {
          Jx[((ic[i] + 60 * k) + 360 * (b_i - 1)) - 1] = 0.5 * Ak[i + 6 * k];
        }
        Jx_tmp = ((ic[k] + 60 * k) + 360 * (b_i - 1)) - 1;
        Jx[Jx_tmp]++;
      }
    }
    for (k = 0; k < 6; k++) {
      for (i = 0; i < 6; i++) {
        Jx[((ic[i] + 60 * k) + 360 * b_i) - 1] = 0.5 * Ak1[i + 6 * k];
      }
      Jx_tmp = ((ic[k] + 60 * k) + 360 * b_i) - 1;
      Jx[Jx_tmp]--;
    }
    for (i = 0; i < 48; i++) {
      val[i] = 0.5 * (val[i] + Bk1[i]);
    }
    for (k = 0; k < 8; k++) {
      for (i = 0; i < 6; i++) {
        Jmv[((ic[i] + 60 * k) + 480 * b_i) - 1] = val[i + 6 * k];
      }
    }
    for (i = 0; i < 6; i++) {
      ic[i] = (int8_T)(ic[i] + 6);
    }
  }
  c_st.site = &ub_emlrtRSI;
  TRANSB1 = 'N';
  TRANSA1 = 'N';
  alpha1 = 1.0;
  beta1 = 0.0;
  m_t = (ptrdiff_t)60;
  n_t = (ptrdiff_t)24;
  k_t = (ptrdiff_t)80;
  lda_t = (ptrdiff_t)60;
  ldb_t = (ptrdiff_t)80;
  ldc_t = (ptrdiff_t)60;
  dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &Jmv[0], &lda_t, &dv[0],
        &ldb_t, &beta1, &y[0], &ldc_t);
  b_st.site = &pb_emlrtRSI;
  outputBounds(SD, &b_st, runtimedata_OutputMin, runtimedata_OutputMax, X, e,
               c_data, c_size, SD->u2.f3.Jc_data, Jc_size);
  b_st.site = &qb_emlrtRSI;
  c_st.site = &eb_emlrtRSI;
  sizes_idx_1 = (int8_T)((c_size[0] != 0) && (c_size[1] != 0));
  d_st.site = &fb_emlrtRSI;
  if ((c_size[1] != sizes_idx_1) && ((c_size[0] != 0) && (c_size[1] != 0))) {
    emlrtErrorWithMessageIdR2018a(&d_st, &k_emlrtRTEI,
                                  "MATLAB:catenate:matrixDimensionMismatch",
                                  "MATLAB:catenate:matrixDimensionMismatch", 0);
  }
  if ((sizes_idx_1 == 0) || ((c_size[0] != 0) && (c_size[1] != 0))) {
    input_sizes_idx_0 = (int8_T)c_size[0];
  } else {
    input_sizes_idx_0 = 0;
  }
  varargout_1_size[0] = input_sizes_idx_0;
  varargout_1_size[1] = sizes_idx_1;
  Jx_tmp = sizes_idx_1;
  for (i = 0; i < Jx_tmp; i++) {
    k = input_sizes_idx_0;
    if (k - 1 >= 0) {
      memcpy(&varargout_1_data[0], &c_data[0], k * sizeof(real_T));
    }
  }
  b_st.site = &rb_emlrtRSI;
  c_st.site = &eb_emlrtRSI;
  if ((Jc_size[0] != 0) && (Jc_size[1] != 0)) {
    sizes_idx_1 = (int8_T)Jc_size[0];
  } else {
    sizes_idx_1 = 0;
  }
  d_st.site = &fb_emlrtRSI;
  if ((Jc_size[0] != sizes_idx_1) && ((Jc_size[0] != 0) && (Jc_size[1] != 0))) {
    emlrtErrorWithMessageIdR2018a(&d_st, &k_emlrtRTEI,
                                  "MATLAB:catenate:matrixDimensionMismatch",
                                  "MATLAB:catenate:matrixDimensionMismatch", 0);
  }
  if ((sizes_idx_1 == 0) || ((Jc_size[0] != 0) && (Jc_size[1] != 0))) {
    input_sizes_idx_0 = (int8_T)Jc_size[1];
  } else {
    input_sizes_idx_0 = 0;
  }
  varargout_3_size[0] = sizes_idx_1;
  varargout_3_size[1] = input_sizes_idx_0;
  Jx_tmp = input_sizes_idx_0;
  for (i = 0; i < Jx_tmp; i++) {
    k = sizes_idx_1;
    for (i1 = 0; i1 < k; i1++) {
      varargout_3_data[i1 + sizes_idx_1 * i] =
          SD->u2.f3.Jc_data[i1 + sizes_idx_1 * i];
    }
  }
  for (i = 0; i < 60; i++) {
    for (i1 = 0; i1 < 60; i1++) {
      varargout_4[i1 + 85 * i] = Jx[i + 60 * i1];
    }
    for (i1 = 0; i1 < 24; i1++) {
      varargout_4[(i1 + 85 * i) + 60] = y[i + 60 * i1];
    }
    varargout_4[85 * i + 84] = 0.0;
  }
}

void nlmpcmoveCodeGeneration(c_nlmpcmoveCodeGenerationStackD *SD,
                             const emlrtStack *sp, const real_T x[6],
                             const real_T lastMV[8], struct1_T *onlinedata,
                             real_T mv[8], struct2_T *info)
{
  static const int8_T iv[10] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 10};
  c_struct_T Out;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  k_struct_T CostFcn_workspace_userdata;
  l_struct_T b_expl_temp;
  l_struct_T expl_temp;
  l_struct_T runtimedata;
  real_T B_data[320];
  real_T Bu[320];
  real_T d_runtimedata[85];
  real_T e_runtimedata[85];
  real_T z[85];
  real_T z0[85];
  real_T b_runtimedata[60];
  real_T c_runtimedata[60];
  real_T ic[8];
  real_T ExitFlag;
  real_T e;
  int32_T A_size[2];
  int32_T b_i;
  int32_T c_i;
  int32_T i;
  int32_T idx;
  int32_T ii_size;
  int32_T k;
  int32_T loop_ub;
  int16_T ii_data[320];
  int8_T As[64];
  boolean_T icf[320];
  boolean_T exitg1;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  b_st.site = &g_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 6)) {
    if ((!muDoubleScalarIsInf(x[k])) && (!muDoubleScalarIsNaN(x[k]))) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &b_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:nlmpcmoveCodeGeneration:expectedFinite", 3, 4, 3, "\"x\"");
  }
  st.site = &b_emlrtRSI;
  b_st.site = &g_emlrtRSI;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 8)) {
    if ((!muDoubleScalarIsInf(lastMV[k])) &&
        (!muDoubleScalarIsNaN(lastMV[k]))) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &b_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:nlmpcmoveCodeGeneration:expectedFinite", 3, 4, 8, "\"lastMV\"");
  }
  st.site = &c_emlrtRSI;
  znlmpc_generateRuntimeData(&st, x, lastMV, onlinedata->ref,
                             onlinedata->MVTarget, onlinedata->X0,
                             onlinedata->MV0, onlinedata->Slack0, &runtimedata,
                             &CostFcn_workspace_userdata, z0);
  st.site = &d_emlrtRSI;
  memset(&SD->f8.Au[0], 0, 25600U * sizeof(real_T));
  memset(&Bu[0], 0, 320U * sizeof(real_T));
  memset(&icf[0], 0, 320U * sizeof(boolean_T));
  for (i = 0; i < 8; i++) {
    ic[i] = (real_T)i + 1.0;
  }
  memset(&As[0], 0, 64U * sizeof(int8_T));
  for (b_i = 0; b_i < 8; b_i++) {
    As[b_i + (b_i << 3)] = 1;
  }
  for (c_i = 0; c_i < 10; c_i++) {
    boolean_T bv[8];
    boolean_T bv1[8];
    for (i = 0; i < 8; i++) {
      ExitFlag = runtimedata.MVRateMin[c_i + 10 * i];
      bv[i] = muDoubleScalarIsInf(ExitFlag);
      bv1[i] = muDoubleScalarIsNaN(ExitFlag);
    }
    for (i = 0; i < 8; i++) {
      ExitFlag = ic[i];
      if (((int32_T)ExitFlag < 1) || ((int32_T)ExitFlag > 320)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)ExitFlag, 1, 320, &t_emlrtBCI,
                                      &st);
      }
      icf[(int32_T)ExitFlag - 1] = ((!bv[i]) && (!bv1[i]));
    }
    for (i = 0; i < 8; i++) {
      ExitFlag = runtimedata.MVRateMax[c_i + 10 * i];
      bv[i] = muDoubleScalarIsInf(ExitFlag);
      bv1[i] = muDoubleScalarIsNaN(ExitFlag);
    }
    for (i = 0; i < 8; i++) {
      idx = (int32_T)(ic[i] + 8.0);
      if ((idx < 1) || (idx > 320)) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, 320, &u_emlrtBCI, &st);
      }
      icf[idx - 1] = ((!bv[i]) && (!bv1[i]));
    }
    for (i = 0; i < 8; i++) {
      ExitFlag = runtimedata.MVMin[c_i + 10 * i];
      bv[i] = muDoubleScalarIsInf(ExitFlag);
      bv1[i] = muDoubleScalarIsNaN(ExitFlag);
    }
    for (i = 0; i < 8; i++) {
      idx = (int32_T)(ic[i] + 16.0);
      if ((idx < 1) || (idx > 320)) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, 320, &v_emlrtBCI, &st);
      }
      icf[idx - 1] = ((!bv[i]) && (!bv1[i]));
    }
    for (i = 0; i < 8; i++) {
      ExitFlag = runtimedata.MVMax[c_i + 10 * i];
      bv[i] = muDoubleScalarIsInf(ExitFlag);
      bv1[i] = muDoubleScalarIsNaN(ExitFlag);
    }
    for (i = 0; i < 8; i++) {
      idx = (int32_T)(ic[i] + 24.0);
      if ((idx < 1) || (idx > 320)) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, 320, &w_emlrtBCI, &st);
      }
      icf[idx - 1] = ((!bv[i]) && (!bv1[i]));
    }
    for (i = 0; i < 8; i++) {
      for (idx = 0; idx < 8; idx++) {
        SD->f8.Au[(((int32_T)ic[idx] + 320 * i) + 2560 * c_i) - 1] =
            -(real_T)As[idx + (i << 3)];
      }
    }
    for (i = 0; i < 8; i++) {
      for (idx = 0; idx < 8; idx++) {
        b_i = (int32_T)(ic[idx] + 8.0);
        if ((b_i < 1) || (b_i > 320)) {
          emlrtDynamicBoundsCheckR2012b(b_i, 1, 320, &x_emlrtBCI, &st);
        }
        SD->f8.Au[((b_i + 320 * i) + 2560 * c_i) - 1] = As[idx + (i << 3)];
      }
    }
    for (i = 0; i < 8; i++) {
      for (idx = 0; idx < 8; idx++) {
        b_i = (int32_T)(ic[idx] + 16.0);
        if ((b_i < 1) || (b_i > 320)) {
          emlrtDynamicBoundsCheckR2012b(b_i, 1, 320, &y_emlrtBCI, &st);
        }
        SD->f8.Au[((b_i + 320 * i) + 2560 * c_i) - 1] =
            -(real_T)As[idx + (i << 3)];
      }
    }
    for (i = 0; i < 8; i++) {
      for (idx = 0; idx < 8; idx++) {
        b_i = (int32_T)(ic[idx] + 24.0);
        if ((b_i < 1) || (b_i > 320)) {
          emlrtDynamicBoundsCheckR2012b(b_i, 1, 320, &ab_emlrtBCI, &st);
        }
        SD->f8.Au[((b_i + 320 * i) + 2560 * c_i) - 1] = As[idx + (i << 3)];
      }
    }
    for (i = 0; i < 8; i++) {
      idx = (int32_T)(ic[i] + 8.0);
      if ((idx < 1) || (idx > 320)) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, 320, &bb_emlrtBCI, &st);
      }
    }
    for (i = 0; i < 8; i++) {
      idx = (int32_T)(ic[i] + 16.0);
      if ((idx < 1) || (idx > 320)) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, 320, &cb_emlrtBCI, &st);
      }
    }
    for (i = 0; i < 8; i++) {
      idx = (int32_T)(ic[i] + 24.0);
      if ((idx < 1) || (idx > 320)) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, 320, &db_emlrtBCI, &st);
      }
    }
    for (i = 0; i < 8; i++) {
      Bu[(int32_T)ic[i] - 1] = -runtimedata.MVRateMin[c_i + 10 * i];
    }
    for (i = 0; i < 8; i++) {
      idx = (int32_T)(ic[i] + 8.0);
      if ((idx < 1) || (idx > 320)) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, 320, &eb_emlrtBCI, &st);
      }
      Bu[idx - 1] = runtimedata.MVRateMax[c_i + 10 * i];
    }
    for (i = 0; i < 8; i++) {
      idx = (int32_T)(ic[i] + 16.0);
      if ((idx < 1) || (idx > 320)) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, 320, &fb_emlrtBCI, &st);
      }
      Bu[idx - 1] = -runtimedata.MVMin[c_i + 10 * i];
    }
    for (i = 0; i < 8; i++) {
      idx = (int32_T)(ic[i] + 24.0);
      if ((idx < 1) || (idx > 320)) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, 320, &gb_emlrtBCI, &st);
      }
      Bu[idx - 1] = runtimedata.MVMax[c_i + 10 * i];
    }
    if (c_i + 1 == 1) {
      real_T b_Bu[8];
      real_T b_dv[8];
      for (b_i = 0; b_i < 8; b_i++) {
        ExitFlag = runtimedata.lastMV[b_i];
        b_dv[b_i] = ExitFlag;
        b_Bu[b_i] = Bu[(int32_T)ic[b_i] - 1] - ExitFlag;
      }
      for (i = 0; i < 8; i++) {
        Bu[(int32_T)ic[i] - 1] = b_Bu[i];
      }
      for (i = 0; i < 8; i++) {
        idx = (int32_T)(ic[i] + 8.0);
        if ((idx < 1) || (idx > 320)) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, 320, &jb_emlrtBCI, &st);
        }
        b_Bu[i] = Bu[idx - 1] + b_dv[i];
      }
      for (i = 0; i < 8; i++) {
        idx = (int32_T)(ic[i] + 8.0);
        if ((idx < 1) || (idx > 320)) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, 320, &kb_emlrtBCI, &st);
        }
        Bu[idx - 1] = b_Bu[i];
      }
    } else {
      for (i = 0; i < 8; i++) {
        for (idx = 0; idx < 8; idx++) {
          SD->f8.Au[(((int32_T)ic[idx] + 320 * i) + 2560 * (c_i - 1)) - 1] =
              As[idx + (i << 3)];
        }
      }
      for (i = 0; i < 8; i++) {
        for (idx = 0; idx < 8; idx++) {
          b_i = (int32_T)(ic[idx] + 8.0);
          if ((b_i < 1) || (b_i > 320)) {
            emlrtDynamicBoundsCheckR2012b(b_i, 1, 320, &hb_emlrtBCI, &st);
          }
          SD->f8.Au[((b_i + 320 * i) + 2560 * (c_i - 1)) - 1] =
              -(real_T)As[idx + (i << 3)];
        }
      }
    }
    for (i = 0; i < 8; i++) {
      ic[i] += 32.0;
    }
  }
  b_st.site = &u_emlrtRSI;
  c_st.site = &w_emlrtRSI;
  idx = 0;
  b_i = 0;
  exitg1 = false;
  while ((!exitg1) && (b_i < 320)) {
    if (icf[b_i]) {
      idx++;
      ii_data[idx - 1] = (int16_T)(b_i + 1);
      if (idx >= 320) {
        exitg1 = true;
      } else {
        b_i++;
      }
    } else {
      b_i++;
    }
  }
  if (idx < 1) {
    loop_ub = 0;
  } else {
    loop_ub = idx;
  }
  indexShapeCheck();
  if (idx < 1) {
    ii_size = 0;
  } else {
    ii_size = idx;
  }
  if (loop_ub > 0) {
    int32_T ii[2];
    int32_T y_size[2];
    for (i = 0; i < loop_ub; i++) {
      B_data[i] = Bu[ii_data[i] - 1];
    }
    for (b_i = 0; b_i < 8; b_i++) {
      for (k = 0; k < 10; k++) {
        for (c_i = 0; c_i < loop_ub; c_i++) {
          if (c_i + 1 > loop_ub) {
            emlrtDynamicBoundsCheckR2012b(c_i + 1, 1, loop_ub, &ib_emlrtBCI,
                                          &st);
          }
          SD->f8.Auf_data[(c_i + loop_ub * b_i) + loop_ub * 8 * k] =
              SD->f8.Au[((ii_data[c_i] + 320 * b_i) + 2560 * k) - 1];
        }
      }
    }
    b_st.site = &v_emlrtRSI;
    b_i = (loop_ub << 3) * 10;
    c_st.site = &bb_emlrtRSI;
    computeDimsData(&c_st, loop_ub);
    k = loop_ub;
    if (loop_ub < 8) {
      k = 8;
    }
    if (k < 10) {
      k = 10;
    }
    if (loop_ub > muIntScalarMax_sint32(b_i, k)) {
      emlrtErrorWithMessageIdR2018a(
          &b_st, &l_emlrtRTEI, "Coder:toolbox:reshape_emptyReshapeLimit",
          "Coder:toolbox:reshape_emptyReshapeLimit", 0);
    }
    if (loop_ub * 80 != b_i) {
      emlrtErrorWithMessageIdR2018a(
          &b_st, &m_emlrtRTEI, "Coder:MATLAB:getReshapeDims_notSameNumel",
          "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
    }
    b_st.site = &v_emlrtRSI;
    ii[0] = loop_ub;
    ii[1] = 80;
    c_st.site = &r_emlrtRSI;
    mtimes(SD->f8.Auf_data, ii, SD->f8.y_data, y_size);
    b_st.site = &v_emlrtRSI;
    c_st.site = &eb_emlrtRSI;
    d_st.site = &fb_emlrtRSI;
    if (y_size[0] != loop_ub) {
      emlrtErrorWithMessageIdR2018a(
          &d_st, &k_emlrtRTEI, "MATLAB:catenate:matrixDimensionMismatch",
          "MATLAB:catenate:matrixDimensionMismatch", 0);
    }
    if (idx < 1) {
      A_size[0] = 0;
    } else {
      A_size[0] = idx;
    }
    A_size[1] = 85;
    for (i = 0; i < 60; i++) {
      for (idx = 0; idx < loop_ub; idx++) {
        SD->f8.A_data[idx + ii_size * i] = 0.0;
      }
    }
    for (i = 0; i < 24; i++) {
      for (idx = 0; idx < loop_ub; idx++) {
        SD->f8.A_data[idx + ii_size * (i + 60)] =
            SD->f8.y_data[idx + loop_ub * i];
      }
    }
    memset(&SD->f8.A_data[ii_size * 84], 0,
           ((loop_ub + ii_size * 84) - ii_size * 84) * sizeof(real_T));
  } else {
    loop_ub = 0;
    A_size[0] = 0;
    A_size[1] = 141;
  }
  memcpy(&expl_temp.MVScaledTarget[0], &runtimedata.MVScaledTarget[0],
         80U * sizeof(real_T));
  memcpy(&expl_temp.MVRateMax[0], &runtimedata.MVRateMax[0],
         80U * sizeof(real_T));
  memcpy(&expl_temp.MVRateMin[0], &runtimedata.MVRateMin[0],
         80U * sizeof(real_T));
  memcpy(&expl_temp.MVMax[0], &runtimedata.MVMax[0], 80U * sizeof(real_T));
  memcpy(&expl_temp.MVMin[0], &runtimedata.MVMin[0], 80U * sizeof(real_T));
  memcpy(&expl_temp.StateMax[0], &runtimedata.StateMax[0],
         60U * sizeof(real_T));
  memcpy(&expl_temp.StateMin[0], &runtimedata.StateMin[0],
         60U * sizeof(real_T));
  memcpy(&expl_temp.OutputMax[0], &runtimedata.OutputMax[0],
         60U * sizeof(real_T));
  memcpy(&expl_temp.OutputMin[0], &runtimedata.OutputMin[0],
         60U * sizeof(real_T));
  expl_temp.ECRWeight = runtimedata.ECRWeight;
  memcpy(&expl_temp.MVRateWeights[0], &runtimedata.MVRateWeights[0],
         80U * sizeof(real_T));
  memcpy(&expl_temp.MVWeights[0], &runtimedata.MVWeights[0],
         80U * sizeof(real_T));
  memcpy(&expl_temp.OutputWeights[0], &runtimedata.OutputWeights[0],
         60U * sizeof(real_T));
  memcpy(&expl_temp.ref[0], &runtimedata.ref[0], 60U * sizeof(real_T));
  memcpy(&expl_temp.lastMV[0], &runtimedata.lastMV[0], 8U * sizeof(real_T));
  for (c_i = 0; c_i < 6; c_i++) {
    expl_temp.x[c_i] = runtimedata.x[c_i];
  }
  memcpy(&b_expl_temp.MVScaledTarget[0], &runtimedata.MVScaledTarget[0],
         80U * sizeof(real_T));
  memcpy(&b_expl_temp.MVRateMax[0], &runtimedata.MVRateMax[0],
         80U * sizeof(real_T));
  memcpy(&b_expl_temp.MVRateMin[0], &runtimedata.MVRateMin[0],
         80U * sizeof(real_T));
  memcpy(&b_expl_temp.MVMax[0], &runtimedata.MVMax[0], 80U * sizeof(real_T));
  memcpy(&b_expl_temp.MVMin[0], &runtimedata.MVMin[0], 80U * sizeof(real_T));
  memcpy(&b_expl_temp.StateMax[0], &runtimedata.StateMax[0],
         60U * sizeof(real_T));
  memcpy(&b_expl_temp.StateMin[0], &runtimedata.StateMin[0],
         60U * sizeof(real_T));
  memcpy(&b_expl_temp.OutputMax[0], &runtimedata.OutputMax[0],
         60U * sizeof(real_T));
  memcpy(&b_expl_temp.OutputMin[0], &runtimedata.OutputMin[0],
         60U * sizeof(real_T));
  b_expl_temp.ECRWeight = runtimedata.ECRWeight;
  memcpy(&b_expl_temp.MVRateWeights[0], &runtimedata.MVRateWeights[0],
         80U * sizeof(real_T));
  memcpy(&b_expl_temp.MVWeights[0], &runtimedata.MVWeights[0],
         80U * sizeof(real_T));
  memcpy(&b_expl_temp.OutputWeights[0], &runtimedata.OutputWeights[0],
         60U * sizeof(real_T));
  memcpy(&b_expl_temp.ref[0], &runtimedata.ref[0], 60U * sizeof(real_T));
  memcpy(&b_expl_temp.lastMV[0], &runtimedata.lastMV[0], 8U * sizeof(real_T));
  for (c_i = 0; c_i < 6; c_i++) {
    b_expl_temp.x[c_i] = runtimedata.x[c_i];
  }
  for (i = 0; i < 10; i++) {
    for (idx = 0; idx < 6; idx++) {
      b_i = i + 10 * idx;
      k = idx + 6 * i;
      b_runtimedata[k] = runtimedata.StateMin[b_i];
      c_runtimedata[k] = runtimedata.StateMax[b_i];
    }
  }
  memcpy(&d_runtimedata[0], &b_runtimedata[0], 60U * sizeof(real_T));
  for (i = 0; i < 24; i++) {
    d_runtimedata[i + 60] = rtMinusInf;
  }
  d_runtimedata[84] = 0.0;
  memcpy(&e_runtimedata[0], &c_runtimedata[0], 60U * sizeof(real_T));
  for (i = 0; i < 24; i++) {
    e_runtimedata[i + 60] = rtInf;
  }
  e_runtimedata[84] = rtInf;
  st.site = &e_emlrtRSI;
  fmincon(SD, &st, &expl_temp, &CostFcn_workspace_userdata, z0, SD->f8.A_data,
          A_size, B_data, loop_ub, d_runtimedata, e_runtimedata, &b_expl_temp,
          &CostFcn_workspace_userdata, z, &info->Cost, &ExitFlag, &Out);
  if ((ExitFlag == 0.0) && (Out.constrviolation > 1.0E-6)) {
    ExitFlag = -2.0;
  }
  st.site = &f_emlrtRSI;
  znlmpc_getXUe(z, x, info->Xopt, info->MVopt, &e);
  if (ExitFlag > 0.0) {
    for (i = 0; i < 8; i++) {
      mv[i] = info->MVopt[11 * i];
    }
  } else {
    memcpy(&mv[0], &lastMV[0], 8U * sizeof(real_T));
  }
  info->ExitFlag = ExitFlag;
  info->Iterations = Out.iterations;
  for (i = 0; i < 8; i++) {
    memcpy(&onlinedata->MV0[i * 10], &info->MVopt[i * 11 + 1],
           10U * sizeof(real_T));
  }
  for (i = 0; i < 6; i++) {
    for (idx = 0; idx < 10; idx++) {
      onlinedata->X0[idx + 10 * i] = info->Xopt[iv[idx] + 11 * i];
    }
  }
  onlinedata->Slack0 = muDoubleScalarMax(0.0, e);
  memcpy(&info->Yopt[0], &info->Xopt[0], 66U * sizeof(real_T));
  for (i = 0; i < 11; i++) {
    info->Topt[i] = i;
  }
  info->Slack = e;
}

/* End of code generation (nlmpcmoveCodeGeneration.c) */
