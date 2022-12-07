/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * znlmpc_getUBounds.c
 *
 * Code generation for function 'znlmpc_getUBounds'
 *
 */

/* Include files */
#include "znlmpc_getUBounds.h"
#include "indexShapeCheck.h"
#include "mtimes.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
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

static emlrtBCInfo emlrtBCI = {
    1,                   /* iFirst */
    768,                 /* iLast */
    22,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo b_emlrtBCI = {
    1,                   /* iFirst */
    768,                 /* iLast */
    23,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo c_emlrtBCI = {
    1,                   /* iFirst */
    768,                 /* iLast */
    24,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = {
    1,                   /* iFirst */
    768,                 /* iLast */
    25,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo e_emlrtBCI = {
    1,                   /* iFirst */
    768,                 /* iLast */
    28,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo f_emlrtBCI = {
    1,                   /* iFirst */
    768,                 /* iLast */
    29,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo g_emlrtBCI = {
    1,                   /* iFirst */
    768,                 /* iLast */
    30,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo h_emlrtBCI = {
    1,                   /* iFirst */
    768,                 /* iLast */
    33,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo i_emlrtBCI = {
    1,                   /* iFirst */
    768,                 /* iLast */
    34,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo j_emlrtBCI = {
    1,                   /* iFirst */
    768,                 /* iLast */
    35,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo k_emlrtBCI = {
    1,                   /* iFirst */
    768,                 /* iLast */
    37,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo l_emlrtBCI = {
    1,                   /* iFirst */
    768,                 /* iLast */
    38,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo m_emlrtBCI = {
    1,                   /* iFirst */
    768,                 /* iLast */
    39,                  /* lineNo */
    5,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo n_emlrtBCI = {
    1,                   /* iFirst */
    768,                 /* iLast */
    45,                  /* lineNo */
    9,                   /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    3 /* checkKind */
};

static emlrtBCInfo o_emlrtBCI = {
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

static emlrtBCInfo p_emlrtBCI = {
    1,                   /* iFirst */
    768,                 /* iLast */
    42,                  /* lineNo */
    27,                  /* colNo */
    "",                  /* aName */
    "znlmpc_getUBounds", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\mpc\\mpcutils\\znlmpc_getUBounds.m", /* pName
                                                                           */
    0 /* checkKind */
};

static emlrtBCInfo q_emlrtBCI = {
    1,                   /* iFirst */
    768,                 /* iLast */
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
void znlmpc_getUBounds(c_nlmpcmoveCodeGenerationStackD *SD,
                       const emlrtStack *sp, const l_struct_T *runtimedata,
                       real_T A_data[], int32_T A_size[2], real_T Bu_data[],
                       int32_T *Bu_size)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T Bu[768];
  real_T ic[24];
  int32_T b_i;
  int32_T c_i;
  int32_T i;
  int32_T idx;
  int32_T ii_size;
  int32_T j;
  int16_T ii_data[768];
  int8_T As[576];
  boolean_T icf[768];
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  memset(&SD->u1.f5.Au[0], 0, 147456U * sizeof(real_T));
  memset(&Bu[0], 0, 768U * sizeof(real_T));
  memset(&icf[0], 0, 768U * sizeof(boolean_T));
  for (i = 0; i < 24; i++) {
    ic[i] = (real_T)i + 1.0;
  }
  memset(&As[0], 0, 576U * sizeof(int8_T));
  for (j = 0; j < 24; j++) {
    As[j + 24 * j] = 1;
  }
  for (b_i = 0; b_i < 8; b_i++) {
    real_T d;
    boolean_T bv[24];
    boolean_T bv1[24];
    for (i = 0; i < 24; i++) {
      d = runtimedata->MVRateMin[b_i + (i << 3)];
      bv[i] = muDoubleScalarIsInf(d);
      bv1[i] = muDoubleScalarIsNaN(d);
    }
    for (i = 0; i < 24; i++) {
      d = ic[i];
      if (((int32_T)d < 1) || ((int32_T)d > 768)) {
        emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, 768, &emlrtBCI,
                                      (emlrtCTX)sp);
      }
      icf[(int32_T)d - 1] = ((!bv[i]) && (!bv1[i]));
    }
    for (i = 0; i < 24; i++) {
      d = runtimedata->MVRateMax[b_i + (i << 3)];
      bv[i] = muDoubleScalarIsInf(d);
      bv1[i] = muDoubleScalarIsNaN(d);
    }
    for (i = 0; i < 24; i++) {
      j = (int32_T)(ic[i] + 24.0);
      if ((j < 1) || (j > 768)) {
        emlrtDynamicBoundsCheckR2012b(j, 1, 768, &b_emlrtBCI, (emlrtCTX)sp);
      }
      icf[j - 1] = ((!bv[i]) && (!bv1[i]));
    }
    for (i = 0; i < 24; i++) {
      d = runtimedata->MVMin[b_i + (i << 3)];
      bv[i] = muDoubleScalarIsInf(d);
      bv1[i] = muDoubleScalarIsNaN(d);
    }
    for (i = 0; i < 24; i++) {
      j = (int32_T)(ic[i] + 48.0);
      if ((j < 1) || (j > 768)) {
        emlrtDynamicBoundsCheckR2012b(j, 1, 768, &c_emlrtBCI, (emlrtCTX)sp);
      }
      icf[j - 1] = ((!bv[i]) && (!bv1[i]));
    }
    for (i = 0; i < 24; i++) {
      d = runtimedata->MVMax[b_i + (i << 3)];
      bv[i] = muDoubleScalarIsInf(d);
      bv1[i] = muDoubleScalarIsNaN(d);
    }
    for (i = 0; i < 24; i++) {
      j = (int32_T)(ic[i] + 72.0);
      if ((j < 1) || (j > 768)) {
        emlrtDynamicBoundsCheckR2012b(j, 1, 768, &d_emlrtBCI, (emlrtCTX)sp);
      }
      icf[j - 1] = ((!bv[i]) && (!bv1[i]));
    }
    for (i = 0; i < 24; i++) {
      for (j = 0; j < 24; j++) {
        SD->u1.f5.Au[(((int32_T)ic[j] + 768 * i) + 18432 * b_i) - 1] =
            -(real_T)As[j + 24 * i];
      }
    }
    for (i = 0; i < 24; i++) {
      for (j = 0; j < 24; j++) {
        c_i = (int32_T)(ic[j] + 24.0);
        if ((c_i < 1) || (c_i > 768)) {
          emlrtDynamicBoundsCheckR2012b(c_i, 1, 768, &e_emlrtBCI, (emlrtCTX)sp);
        }
        SD->u1.f5.Au[((c_i + 768 * i) + 18432 * b_i) - 1] = As[j + 24 * i];
      }
    }
    for (i = 0; i < 24; i++) {
      for (j = 0; j < 24; j++) {
        c_i = (int32_T)(ic[j] + 48.0);
        if ((c_i < 1) || (c_i > 768)) {
          emlrtDynamicBoundsCheckR2012b(c_i, 1, 768, &f_emlrtBCI, (emlrtCTX)sp);
        }
        SD->u1.f5.Au[((c_i + 768 * i) + 18432 * b_i) - 1] =
            -(real_T)As[j + 24 * i];
      }
    }
    for (i = 0; i < 24; i++) {
      for (j = 0; j < 24; j++) {
        c_i = (int32_T)(ic[j] + 72.0);
        if ((c_i < 1) || (c_i > 768)) {
          emlrtDynamicBoundsCheckR2012b(c_i, 1, 768, &g_emlrtBCI, (emlrtCTX)sp);
        }
        SD->u1.f5.Au[((c_i + 768 * i) + 18432 * b_i) - 1] = As[j + 24 * i];
      }
    }
    for (i = 0; i < 24; i++) {
      j = (int32_T)(ic[i] + 24.0);
      if ((j < 1) || (j > 768)) {
        emlrtDynamicBoundsCheckR2012b(j, 1, 768, &h_emlrtBCI, (emlrtCTX)sp);
      }
    }
    for (i = 0; i < 24; i++) {
      j = (int32_T)(ic[i] + 48.0);
      if ((j < 1) || (j > 768)) {
        emlrtDynamicBoundsCheckR2012b(j, 1, 768, &i_emlrtBCI, (emlrtCTX)sp);
      }
    }
    for (i = 0; i < 24; i++) {
      j = (int32_T)(ic[i] + 72.0);
      if ((j < 1) || (j > 768)) {
        emlrtDynamicBoundsCheckR2012b(j, 1, 768, &j_emlrtBCI, (emlrtCTX)sp);
      }
    }
    for (i = 0; i < 24; i++) {
      Bu[(int32_T)ic[i] - 1] = -runtimedata->MVRateMin[b_i + (i << 3)];
    }
    for (i = 0; i < 24; i++) {
      j = (int32_T)(ic[i] + 24.0);
      if ((j < 1) || (j > 768)) {
        emlrtDynamicBoundsCheckR2012b(j, 1, 768, &k_emlrtBCI, (emlrtCTX)sp);
      }
      Bu[j - 1] = runtimedata->MVRateMax[b_i + (i << 3)];
    }
    for (i = 0; i < 24; i++) {
      j = (int32_T)(ic[i] + 48.0);
      if ((j < 1) || (j > 768)) {
        emlrtDynamicBoundsCheckR2012b(j, 1, 768, &l_emlrtBCI, (emlrtCTX)sp);
      }
      Bu[j - 1] = -runtimedata->MVMin[b_i + (i << 3)];
    }
    for (i = 0; i < 24; i++) {
      j = (int32_T)(ic[i] + 72.0);
      if ((j < 1) || (j > 768)) {
        emlrtDynamicBoundsCheckR2012b(j, 1, 768, &m_emlrtBCI, (emlrtCTX)sp);
      }
      Bu[j - 1] = runtimedata->MVMax[b_i + (i << 3)];
    }
    if (b_i + 1 == 1) {
      real_T b_Bu[24];
      real_T b_dv[24];
      for (c_i = 0; c_i < 24; c_i++) {
        d = runtimedata->lastMV[c_i];
        b_dv[c_i] = d;
        b_Bu[c_i] = Bu[(int32_T)ic[c_i] - 1] - d;
      }
      for (i = 0; i < 24; i++) {
        Bu[(int32_T)ic[i] - 1] = b_Bu[i];
      }
      for (i = 0; i < 24; i++) {
        j = (int32_T)(ic[i] + 24.0);
        if ((j < 1) || (j > 768)) {
          emlrtDynamicBoundsCheckR2012b(j, 1, 768, &p_emlrtBCI, (emlrtCTX)sp);
        }
        b_Bu[i] = Bu[j - 1] + b_dv[i];
      }
      for (i = 0; i < 24; i++) {
        j = (int32_T)(ic[i] + 24.0);
        if ((j < 1) || (j > 768)) {
          emlrtDynamicBoundsCheckR2012b(j, 1, 768, &q_emlrtBCI, (emlrtCTX)sp);
        }
        Bu[j - 1] = b_Bu[i];
      }
    } else {
      for (i = 0; i < 24; i++) {
        for (j = 0; j < 24; j++) {
          SD->u1.f5.Au[(((int32_T)ic[j] + 768 * i) + 18432 * (b_i - 1)) - 1] =
              As[j + 24 * i];
        }
      }
      for (i = 0; i < 24; i++) {
        for (j = 0; j < 24; j++) {
          c_i = (int32_T)(ic[j] + 24.0);
          if ((c_i < 1) || (c_i > 768)) {
            emlrtDynamicBoundsCheckR2012b(c_i, 1, 768, &n_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          SD->u1.f5.Au[((c_i + 768 * i) + 18432 * (b_i - 1)) - 1] =
              -(real_T)As[j + 24 * i];
        }
      }
    }
    for (i = 0; i < 24; i++) {
      ic[i] += 96.0;
    }
  }
  st.site = &u_emlrtRSI;
  b_st.site = &w_emlrtRSI;
  idx = 0;
  c_i = 0;
  exitg1 = false;
  while ((!exitg1) && (c_i < 768)) {
    if (icf[c_i]) {
      idx++;
      ii_data[idx - 1] = (int16_T)(c_i + 1);
      if (idx >= 768) {
        exitg1 = true;
      } else {
        c_i++;
      }
    } else {
      c_i++;
    }
  }
  if (idx < 1) {
    *Bu_size = 0;
  } else {
    *Bu_size = idx;
  }
  indexShapeCheck();
  if (idx < 1) {
    ii_size = 0;
  } else {
    ii_size = idx;
  }
  if (*Bu_size > 0) {
    int32_T ii[2];
    int32_T y_size[2];
    for (i = 0; i < *Bu_size; i++) {
      Bu_data[i] = Bu[ii_data[i] - 1];
    }
    for (j = 0; j < 24; j++) {
      for (c_i = 0; c_i < 8; c_i++) {
        for (b_i = 0; b_i < *Bu_size; b_i++) {
          if (b_i + 1 > *Bu_size) {
            emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, *Bu_size, &o_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          SD->u1.f5.Auf_data[(b_i + *Bu_size * j) + *Bu_size * 24 * c_i] =
              SD->u1.f5.Au[((ii_data[b_i] + 768 * j) + 18432 * c_i) - 1];
        }
      }
    }
    st.site = &v_emlrtRSI;
    c_i = (*Bu_size * 24) << 3;
    b_st.site = &bb_emlrtRSI;
    j = *Bu_size;
    if (*Bu_size < 24) {
      j = 24;
    }
    if (*Bu_size > muIntScalarMax_sint32(c_i, j)) {
      emlrtErrorWithMessageIdR2018a(
          &st, &f_emlrtRTEI, "Coder:toolbox:reshape_emptyReshapeLimit",
          "Coder:toolbox:reshape_emptyReshapeLimit", 0);
    }
    if (*Bu_size * 192 != c_i) {
      emlrtErrorWithMessageIdR2018a(
          &st, &e_emlrtRTEI, "Coder:MATLAB:getReshapeDims_notSameNumel",
          "Coder:MATLAB:getReshapeDims_notSameNumel", 0);
    }
    st.site = &v_emlrtRSI;
    ii[0] = *Bu_size;
    ii[1] = 192;
    b_st.site = &r_emlrtRSI;
    mtimes(SD->u1.f5.Auf_data, ii, SD->u1.f5.y_data, y_size);
    st.site = &v_emlrtRSI;
    b_st.site = &eb_emlrtRSI;
    c_st.site = &fb_emlrtRSI;
    if (y_size[0] != *Bu_size) {
      emlrtErrorWithMessageIdR2018a(
          &c_st, &d_emlrtRTEI, "MATLAB:catenate:matrixDimensionMismatch",
          "MATLAB:catenate:matrixDimensionMismatch", 0);
    }
    if (idx < 1) {
      A_size[0] = 0;
    } else {
      A_size[0] = idx;
    }
    A_size[1] = 265;
    for (i = 0; i < 192; i++) {
      for (j = 0; j < *Bu_size; j++) {
        A_data[j + ii_size * i] = 0.0;
      }
    }
    for (i = 0; i < 72; i++) {
      for (j = 0; j < *Bu_size; j++) {
        A_data[j + ii_size * (i + 192)] = SD->u1.f5.y_data[j + *Bu_size * i];
      }
    }
    memset(&A_data[ii_size * 264], 0,
           ((*Bu_size + ii_size * 264) - ii_size * 264) * sizeof(real_T));
  } else {
    *Bu_size = 0;
    A_size[0] = 0;
    A_size[1] = 385;
  }
}

/* End of code generation (znlmpc_getUBounds.c) */
