/*
 * checkNonlinearInputs.c
 *
 * Code generation for function 'checkNonlinearInputs'
 *
 */

/* Include files */
#include "checkNonlinearInputs.h"
#include "nlmpcmoveCodeGeneration.h"
#include "nlmpcmoveCodeGeneration_data.h"
#include "nlmpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo kb_emlrtRSI = {
    1,                      /* lineNo */
    "checkNonlinearInputs", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "parseinput\\checkNonlinearInputs.p" /* pathName */
};

static emlrtRTEInfo j_emlrtRTEI = {
    1,                      /* lineNo */
    1,                      /* colNo */
    "checkNonlinearInputs", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2022a\\toolbox\\optim\\+optim\\+coder\\+fminconsqp\\+"
    "parseinput\\checkNonlinearInputs.p" /* pName */
};

/* Function Definitions */
int32_T checkNonlinearInputs(c_nlmpcmoveCodeGenerationStackD *SD,
                             const emlrtStack *sp, const real_T x0[85],
                             const real_T nonlcon_workspace_runtimedata_x[6],
                             const real_T c_nonlcon_workspace_runtimedata[60],
                             const real_T d_nonlcon_workspace_runtimedata[60])
{
  emlrtStack b_st;
  emlrtStack st;
  real_T varargout_1_data[120];
  real_T varargout_2[60];
  int32_T varargout_1_size[2];
  int32_T varargout_3_size[2];
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &kb_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  b_st.site = &lb_emlrtRSI;
  c_nlmpcmoveCodeGeneration_anonF(
      SD, &b_st, nonlcon_workspace_runtimedata_x,
      c_nonlcon_workspace_runtimedata, d_nonlcon_workspace_runtimedata, x0,
      varargout_1_data, varargout_1_size, varargout_2,
      SD->u3.f6.varargout_3_data, varargout_3_size, SD->u3.f6.varargout_4);
  if ((varargout_3_size[0] != 0) && (varargout_3_size[1] != 0) &&
      ((varargout_3_size[0] != 85) ||
       (varargout_3_size[1] != varargout_1_size[0] * varargout_1_size[1]))) {
    emlrtErrorWithMessageIdR2018a(
        sp, &j_emlrtRTEI, "optimlib:fmincon:WrongSizeGradNonlinIneq",
        "optimlib:fmincon:WrongSizeGradNonlinIneq", 4, 6, 85.0, 6,
        (real_T)(varargout_1_size[0] * varargout_1_size[1]));
  }
  return varargout_1_size[0] * varargout_1_size[1];
}

/* End of code generation (checkNonlinearInputs.c) */
