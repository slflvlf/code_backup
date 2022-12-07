/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * qg.c
 *
 * Code generation for function 'qg'
 *
 */

/* Include files */
#include "qg.h"
#include "rt_nonfinite.h"
#include "update_mv.h"

/* Function Definitions */
real_T qg(real_T sc_pc, real_T sc_beta, real_T sc_ag, real_T sc_bg, real_T sc_cg,
          real_T p)
{
  real_T inter;

  /* --------------------------------------------------------- */
  /*  Author : Mazen Alamir  */
  /*  CNRS, UNiversity of Grenoble-Alpes */
  /*  Gipsa Lab. Last modification March 2017 */
  /* --------------------------------------------------------- */
  inter = (p - sc_pc) / sc_beta;
  return (sc_ag * (inter * inter) + sc_bg * inter) + sc_cg;
}

/* End of code generation (qg.c) */
