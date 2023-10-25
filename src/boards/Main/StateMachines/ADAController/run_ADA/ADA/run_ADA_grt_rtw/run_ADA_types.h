/*
 * run_ADA_types.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "run_ADA".
 *
 * Model version              : 1.25
 * Simulink Coder version : 9.9 (R2023a) 19-Nov-2022
 * C++ source code generated on : Wed Oct 25 17:58:39 2023
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_run_ADA_types_h_
#define RTW_HEADER_run_ADA_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_ADAState_
#define DEFINED_TYPEDEF_FOR_ADAState_

struct ADAState
{
  real_T timestamp;
  real_T mslAltitude;
  real_T aglAltitude;
  real_T verticalSpeed;
  real_T x0;
  real_T x1;
  real_T x2;
};

#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_run_ADA_T RT_MODEL_run_ADA_T;

#endif                                 /* RTW_HEADER_run_ADA_types_h_ */
