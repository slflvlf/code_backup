/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * feasibleX0ForWorkingSet.c
 *
 * Code generation for function 'feasibleX0ForWorkingSet'
 *
 */

/* Include files */
#include "feasibleX0ForWorkingSet.h"
#include "computeQ_.h"
#include "eml_int_forloop_overflow_check.h"
#include "factorQR.h"
#include "maxConstraintViolation.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include "xaxpy.h"
#include "xcopy.h"
#include "xgemm.h"
#include "xgemv.h"
#include "xgeqrf.h"
#include "xtrsm.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo we_emlrtRSI = {
    1,                         /* lineNo */
    "feasibleX0ForWorkingSet", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "initialize\\feasibleX0ForWorkingSet.p" /* pathName */
};

static emlrtRSInfo jf_emlrtRSI = {
    1,              /* lineNo */
    "computeTallQ", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+"
    "QRManager\\computeTallQ.p" /* pathName */
};

static emlrtBCInfo oc_emlrtBCI = {
    -1,                        /* iFirst */
    -1,                        /* iLast */
    1,                         /* lineNo */
    1,                         /* colNo */
    "",                        /* aName */
    "feasibleX0ForWorkingSet", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+qpactiveset\\+"
    "initialize\\feasibleX0ForWorkingSet.p", /* pName */
    0                                        /* checkKind */
};

/* Function Definitions */
boolean_T feasibleX0ForWorkingSet(c_nlmpcmoveCodeGenerationStackD *SD,
                                  const emlrtStack *sp, real_T workspace_data[],
                                  const int32_T workspace_size[2],
                                  real_T xCurrent_data[],
                                  h_struct_T *workingset, i_struct_T *qrmanager)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T idx_col;
  int32_T idx_row;
  int32_T ldw;
  int32_T mWConstr;
  int32_T nVar;
  boolean_T nonDegenerateWset;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  mWConstr = workingset->nActiveConstr;
  nVar = workingset->nVar;
  nonDegenerateWset = true;
  if (mWConstr != 0) {
    real_T constrViolation_minnormX;
    int32_T i;
    st.site = &we_emlrtRSI;
    if (mWConstr > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    for (ldw = 0; ldw < mWConstr; ldw++) {
      i = workingset->bwset.size[0];
      if ((ldw + 1 < 1) || (ldw + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(ldw + 1, 1, i, &oc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      i = workspace_size[0];
      if (ldw + 1 > i) {
        emlrtDynamicBoundsCheckR2012b(ldw + 1, 1, i, &oc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      constrViolation_minnormX = workingset->bwset.data[ldw];
      workspace_data[ldw] = constrViolation_minnormX;
      i = workingset->bwset.size[0];
      if (ldw + 1 > i) {
        emlrtDynamicBoundsCheckR2012b(ldw + 1, 1, i, &oc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      i = workspace_size[0];
      if (ldw + 1 > i) {
        emlrtDynamicBoundsCheckR2012b(ldw + 1, 1, i, &oc_emlrtBCI,
                                      (emlrtCTX)sp);
      }
      workspace_data[ldw + workspace_size[0]] = constrViolation_minnormX;
    }
    st.site = &we_emlrtRSI;
    c_xgemv(nVar, mWConstr, workingset->ATwset.data, workingset->ldA,
            xCurrent_data, workspace_data);
    if (mWConstr >= nVar) {
      int32_T loop_ub;
      st.site = &we_emlrtRSI;
      i = workingset->ATwset.size[0];
      for (idx_col = 0; idx_col < nVar; idx_col++) {
        ldw = qrmanager->ldq * idx_col + 1;
        st.site = &we_emlrtRSI;
        for (idx_row = 0; idx_row < mWConstr; idx_row++) {
          int32_T i1;
          int32_T i2;
          loop_ub = (idx_col + workingset->ldA * idx_row) + 1;
          if ((loop_ub < 1) || (loop_ub > i)) {
            emlrtDynamicBoundsCheckR2012b(loop_ub, 1, i, &oc_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          i1 = qrmanager->QR.size[0] * qrmanager->QR.size[1];
          i2 = idx_row + ldw;
          if ((i2 < 1) || (i2 > i1)) {
            emlrtDynamicBoundsCheckR2012b(i2, 1, i1, &oc_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          qrmanager->QR.data[i2 - 1] = workingset->ATwset.data[loop_ub - 1];
        }
      }
      st.site = &we_emlrtRSI;
      qrmanager->usedPivoting = false;
      qrmanager->mrows = mWConstr;
      qrmanager->ncols = nVar;
      for (ldw = 0; ldw < nVar; ldw++) {
        i = qrmanager->jpvt.size[0];
        if ((ldw + 1 < 1) || (ldw + 1 > i)) {
          emlrtDynamicBoundsCheckR2012b(ldw + 1, 1, i, &pc_emlrtBCI, &st);
        }
        qrmanager->jpvt.data[ldw] = ldw + 1;
      }
      qrmanager->minRowCol = muIntScalarMin_sint32(mWConstr, nVar);
      b_st.site = &xe_emlrtRSI;
      xgeqrf(&b_st, qrmanager->QR.data, qrmanager->QR.size, mWConstr, nVar,
             qrmanager->tau.data, &qrmanager->tau.size[0]);
      st.site = &we_emlrtRSI;
      b_st.site = &ae_emlrtRSI;
      computeQ_(&b_st, qrmanager, mWConstr);
      loop_ub = workspace_size[0] * workspace_size[1] - 1;
      if (loop_ub >= 0) {
        memcpy(&SD->u1.f3.workspace_data[0], &workspace_data[0],
               (loop_ub + 1) * sizeof(real_T));
      }
      st.site = &we_emlrtRSI;
      xgemm(nVar, mWConstr, qrmanager->Q.data, qrmanager->ldq,
            SD->u1.f3.workspace_data, workspace_size[0], workspace_data,
            workspace_size[0]);
      st.site = &we_emlrtRSI;
      xtrsm(nVar, qrmanager->QR.data, qrmanager->ldq, workspace_data,
            workspace_size[0]);
    } else {
      int32_T loop_ub;
      st.site = &we_emlrtRSI;
      factorQR(&st, qrmanager, workingset->ATwset.data, nVar, mWConstr,
               workingset->ldA);
      st.site = &we_emlrtRSI;
      b_st.site = &jf_emlrtRSI;
      computeQ_(&b_st, qrmanager, qrmanager->minRowCol);
      ldw = workspace_size[0];
      st.site = &we_emlrtRSI;
      b_xtrsm(mWConstr, qrmanager->QR.data, qrmanager->ldq, workspace_data,
              workspace_size[0]);
      loop_ub = workspace_size[0] * workspace_size[1] - 1;
      if (loop_ub >= 0) {
        memcpy(&SD->u1.f3.workspace_data[0], &workspace_data[0],
               (loop_ub + 1) * sizeof(real_T));
      }
      st.site = &we_emlrtRSI;
      b_xgemm(nVar, mWConstr, qrmanager->Q.data, qrmanager->ldq,
              SD->u1.f3.workspace_data, ldw, workspace_data, ldw);
    }
    st.site = &we_emlrtRSI;
    if (nVar > 2147483646) {
      b_st.site = &db_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }
    ldw = 0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (ldw <= nVar - 1) {
        i = workspace_size[0];
        if ((ldw + 1 < 1) || (ldw + 1 > i)) {
          emlrtDynamicBoundsCheckR2012b(ldw + 1, 1, i, &oc_emlrtBCI,
                                        (emlrtCTX)sp);
        }
        constrViolation_minnormX = workspace_data[ldw];
        if (muDoubleScalarIsInf(constrViolation_minnormX) ||
            muDoubleScalarIsNaN(constrViolation_minnormX)) {
          nonDegenerateWset = false;
          exitg1 = 1;
        } else {
          i = workspace_size[0];
          if ((ldw + 1 < 1) || (ldw + 1 > i)) {
            emlrtDynamicBoundsCheckR2012b(ldw + 1, 1, i, &oc_emlrtBCI,
                                          (emlrtCTX)sp);
          }
          constrViolation_minnormX = workspace_data[ldw + workspace_size[0]];
          if (muDoubleScalarIsInf(constrViolation_minnormX) ||
              muDoubleScalarIsNaN(constrViolation_minnormX)) {
            nonDegenerateWset = false;
            exitg1 = 1;
          } else {
            ldw++;
          }
        }
      } else {
        real_T constrViolation_basicX;
        st.site = &we_emlrtRSI;
        b_xaxpy(nVar, xCurrent_data, workspace_data);
        st.site = &we_emlrtRSI;
        constrViolation_minnormX =
            maxConstraintViolation(&st, workingset, workspace_data);
        st.site = &we_emlrtRSI;
        constrViolation_basicX = b_maxConstraintViolation(
            &st, workingset, workspace_data, workspace_size[0] + 1);
        if ((constrViolation_minnormX <= 2.2204460492503131E-16) ||
            (constrViolation_minnormX < constrViolation_basicX)) {
          st.site = &we_emlrtRSI;
          d_xcopy(nVar, workspace_data, xCurrent_data);
        } else {
          st.site = &we_emlrtRSI;
          e_xcopy(nVar, workspace_data, workspace_size[0] + 1, xCurrent_data);
        }
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return nonDegenerateWset;
}

/* End of code generation (feasibleX0ForWorkingSet.c) */
