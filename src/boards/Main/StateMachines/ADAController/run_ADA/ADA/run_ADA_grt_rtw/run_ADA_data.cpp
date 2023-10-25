/*
 * run_ADA_data.cpp
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

#include "run_ADA.h"

/* Constant parameters (default storage) */
const ConstP_run_ADA_T run_ADA_ConstP{
  /* Expression: [1 0.02 0.5*0.02^2; 0 1 0.02; 0 0 1]
   * Referenced by: '<S1>/F_memory'
   */
  { 1.0, 0.0, 0.0, 0.02, 1.0, 0.0, 0.0002, 0.02, 1.0 },

  /* Pooled Parameter (Expression: eye(3))
   * Referenced by:
   *   '<S1>/P_memory'
   *   '<S2>/I'
   */
  { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 },

  /* Expression: [30 0 0; 0 10 0; 0 0 2.5]
   * Referenced by: '<S1>/Q_memory'
   */
  { 30.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 2.5 }
};
