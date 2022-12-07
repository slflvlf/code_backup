/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * qJ.c
 *
 * Code generation for function 'qJ'
 *
 */

/* Include files */
#include "qJ.h"
#include "rt_nonfinite.h"
#include "update_mv.h"

/* Function Definitions */
real_T qJ(real_T sc_pc, real_T sc_beta, real_T sc_aJ, real_T sc_bJ, real_T sc_cJ,
          real_T p)
{
  real_T inter;

  /* --------------------------------------------------------- */
  /*  Author : Mazen Alamir  */
  /*  CNRS, UNiversity of Grenoble-Alpes */
  /*  Gipsa Lab. Last modification March 2017 */
  /* --------------------------------------------------------- */
  inter = (p - sc_pc) / sc_beta;
  return (sc_aJ * (inter * inter) + sc_bJ * inter) + sc_cJ;
}

/* End of code generation (qJ.c) */
