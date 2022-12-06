/*
 * strcmp.c
 *
 * Code generation for function 'strcmp'
 *
 */

/* Include files */
#include "strcmp.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
boolean_T b_strcmp(const char_T a[7])
{
  static const char_T b[7] = {'f', 'm', 'i', 'n', 'c', 'o', 'n'};
  return memcmp(&a[0], &b[0], 7) == 0;
}

/* End of code generation (strcmp.c) */
