/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * update_mv_data.c
 *
 * Code generation for function 'update_mv_data'
 *
 */

/* Include files */
#include "update_mv_data.h"
#include "rt_nonfinite.h"
#include "update_mv.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
const volatile char_T *emlrtBreakCheckR2012bFlagVar = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131594U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "update_mv",                         /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

emlrtRSInfo h_emlrtRSI = { 26,         /* lineNo */
  "update_sc",                         /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m"/* pathName */
};

emlrtRSInfo i_emlrtRSI = { 27,         /* lineNo */
  "update_sc",                         /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m"/* pathName */
};

emlrtRSInfo j_emlrtRSI = { 29,         /* lineNo */
  "update_sc",                         /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m"/* pathName */
};

emlrtRSInfo k_emlrtRSI = { 30,         /* lineNo */
  "update_sc",                         /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m"/* pathName */
};

emlrtRSInfo l_emlrtRSI = { 32,         /* lineNo */
  "update_sc",                         /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m"/* pathName */
};

emlrtRSInfo n_emlrtRSI = { 41,         /* lineNo */
  "update_sc",                         /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m"/* pathName */
};

emlrtRSInfo o_emlrtRSI = { 42,         /* lineNo */
  "update_sc",                         /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m"/* pathName */
};

emlrtRSInfo p_emlrtRSI = { 44,         /* lineNo */
  "update_sc",                         /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m"/* pathName */
};

emlrtRSInfo q_emlrtRSI = { 45,         /* lineNo */
  "update_sc",                         /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m"/* pathName */
};

emlrtRSInfo r_emlrtRSI = { 47,         /* lineNo */
  "update_sc",                         /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m"/* pathName */
};

emlrtRSInfo s_emlrtRSI = { 49,         /* lineNo */
  "update_sc",                         /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\update_sc.m"/* pathName */
};

emlrtRSInfo bb_emlrtRSI = { 168,       /* lineNo */
  "mtimes",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\+blas\\mtimes.m"/* pathName */
};

emlrtRSInfo cb_emlrtRSI = { 164,       /* lineNo */
  "mtimes",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\+blas\\mtimes.m"/* pathName */
};

emlrtRSInfo nc_emlrtRSI = { 34,        /* lineNo */
  "compute_parabola",                  /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\compute_parabola.m"/* pathName */
};

emlrtRSInfo oc_emlrtRSI = { 37,        /* lineNo */
  "compute_parabola",                  /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\compute_parabola.m"/* pathName */
};

emlrtRSInfo pc_emlrtRSI = { 39,        /* lineNo */
  "compute_parabola",                  /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\compute_parabola.m"/* pathName */
};

emlrtRSInfo qc_emlrtRSI = { 51,        /* lineNo */
  "compute_parabola",                  /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\compute_parabola.m"/* pathName */
};

emlrtRSInfo rc_emlrtRSI = { 54,        /* lineNo */
  "compute_parabola",                  /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\compute_parabola.m"/* pathName */
};

emlrtRSInfo sc_emlrtRSI = { 56,        /* lineNo */
  "compute_parabola",                  /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\compute_parabola.m"/* pathName */
};

emlrtRSInfo tc_emlrtRSI = { 45,        /* lineNo */
  "mpower",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\ops\\mpower.m"/* pathName */
};

emlrtRSInfo uc_emlrtRSI = { 70,        /* lineNo */
  "power",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\ops\\power.m"/* pathName */
};

emlrtRSInfo vc_emlrtRSI = { 16,        /* lineNo */
  "min",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\datafun\\min.m"/* pathName */
};

emlrtRSInfo wc_emlrtRSI = { 40,        /* lineNo */
  "minOrMax",                          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax.m"/* pathName */
};

emlrtRSInfo xc_emlrtRSI = { 90,        /* lineNo */
  "minimum",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax.m"/* pathName */
};

emlrtRSInfo yc_emlrtRSI = { 131,       /* lineNo */
  "unaryMinOrMax",                     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

emlrtRSInfo ad_emlrtRSI = { 16,        /* lineNo */
  "max",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\datafun\\max.m"/* pathName */
};

emlrtRSInfo bd_emlrtRSI = { 38,        /* lineNo */
  "minOrMax",                          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax.m"/* pathName */
};

emlrtRSInfo cd_emlrtRSI = { 77,        /* lineNo */
  "maximum",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax.m"/* pathName */
};

emlrtRSInfo dd_emlrtRSI = { 15,        /* lineNo */
  "compute_Zg",                        /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\compute_Zg.m"/* pathName */
};

emlrtRSInfo ed_emlrtRSI = { 14,        /* lineNo */
  "compute_Zg",                        /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\compute_Zg.m"/* pathName */
};

emlrtRSInfo gd_emlrtRSI = { 14,        /* lineNo */
  "min",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\lib\\matlab\\datafun\\min.m"/* pathName */
};

emlrtRSInfo hd_emlrtRSI = { 46,        /* lineNo */
  "minOrMax",                          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax.m"/* pathName */
};

emlrtRSInfo id_emlrtRSI = { 92,        /* lineNo */
  "minimum",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax.m"/* pathName */
};

emlrtRSInfo jd_emlrtRSI = { 155,       /* lineNo */
  "unaryMinOrMax",                     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

emlrtRSInfo kd_emlrtRSI = { 1015,      /* lineNo */
  "minRealVectorOmitNaN",              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

emlrtRSInfo ld_emlrtRSI = { 9,         /* lineNo */
  "qJ",                                /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\qJ.m"/* pathName */
};

emlrtRSInfo md_emlrtRSI = { 10,        /* lineNo */
  "qg",                                /* fcnName */
  "C:\\\\Users\\\\jiang\\\\Desktop\\\\towing_project_demo\\\\\xe8\xae\xa1\xe7\xae\x97\xe7\xbb\x84\xe4\xbb\xb6\\\\pdfmpc\\\\qg.m"/* pathName */
};

emlrtRTEInfo j_emlrtRTEI = { 26,       /* lineNo */
  27,                                  /* colNo */
  "unaryMinOrMax",                     /* fName */
  "C:\\Program Files\\MATLAB\\R2020a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pName */
};

/* End of code generation (update_mv_data.c) */
