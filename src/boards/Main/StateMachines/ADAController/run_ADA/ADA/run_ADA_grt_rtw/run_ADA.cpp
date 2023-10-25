/*
 * run_ADA.cpp
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
#include "rtwtypes.h"
#include <cstring>
#include <cmath>
#include <emmintrin.h>

/* System initialize for atomic system: '<Root>/run_ADA' */
void run_ADA::run_ADA_run_ADA_Init(DW_run_ADA_run_ADA_T *localDW)
{
  /* InitializeConditions for Memory: '<S1>/H_memory' */
  localDW->H_memory_PreviousInput[0] = 1.0;
  localDW->H_memory_PreviousInput[1] = 0.0;
  localDW->H_memory_PreviousInput[2] = 0.0;

  /* InitializeConditions for Memory: '<S1>/F_memory' */
  std::memcpy(&localDW->F_memory_PreviousInput[0],
              &run_ADA_ConstP.F_memory_InitialCondition[0], 9U * sizeof(real_T));

  /* InitializeConditions for Memory: '<S1>/P_memory' */
  std::memcpy(&localDW->P_memory_PreviousInput[0], &run_ADA_ConstP.pooled5[0],
              9U * sizeof(real_T));

  /* InitializeConditions for Memory: '<S1>/Q_memory' */
  std::memcpy(&localDW->Q_memory_PreviousInput[0],
              &run_ADA_ConstP.Q_memory_InitialCondition[0], 9U * sizeof(real_T));

  /* InitializeConditions for Memory: '<S1>/H_memory1' */
  localDW->H_memory1_PreviousInput = 4000.0;

  /* InitializeConditions for Memory: '<S1>/x_memory' */
  localDW->x_memory_PreviousInput[0] = 0.0;
  localDW->x_memory_PreviousInput[1] = 0.0;
  localDW->x_memory_PreviousInput[2] = 1.0;
}

/* Output and update for atomic system: '<Root>/run_ADA' */
void run_ADA::run_ADA_run_ADA(real_T rtu_Pressure, B_run_ADA_run_ADA_T *localB,
  DW_run_ADA_run_ADA_T *localDW) const
{
  __m128d tmp_0;
  __m128d tmp_1;
  __m128d tmp_2;
  real_T rtb_Add[9];
  real_T tmp[9];
  real_T rtb_K[3];
  real_T rtb_MatrixMultiply[3];
  real_T F_memory_PreviousInput;
  real_T F_memory_PreviousInput_0;
  real_T F_memory_PreviousInput_1;
  real_T P_memory_PreviousInput;
  real_T P_memory_PreviousInput_0;
  real_T rtb_MatrixMultiply_0;
  real_T rtb_Product_n;
  real_T x_memory_PreviousInput;
  int32_T i;
  int32_T i_0;
  int32_T rtb_Add_tmp;

  /* Product: '<S3>/Matrix Multiply1' incorporates:
   *  Math: '<S3>/Transpose'
   *  Memory: '<S1>/F_memory'
   *  Memory: '<S1>/P_memory'
   */
  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Product_n = localDW->P_memory_PreviousInput[i_0 + 3];
    P_memory_PreviousInput = localDW->P_memory_PreviousInput[i_0];
    P_memory_PreviousInput_0 = localDW->P_memory_PreviousInput[i_0 + 6];
    for (i = 0; i < 3; i++) {
      tmp[i_0 + 3 * i] = (localDW->F_memory_PreviousInput[i + 3] * rtb_Product_n
                          + P_memory_PreviousInput *
                          localDW->F_memory_PreviousInput[i]) +
        localDW->F_memory_PreviousInput[i + 6] * P_memory_PreviousInput_0;
    }
  }

  /* Product: '<S2>/Matrix Multiply' */
  rtb_Product_n = 0.0;
  for (i_0 = 0; i_0 < 3; i_0++) {
    /* Sum: '<S3>/Add' incorporates:
     *  Memory: '<S1>/F_memory'
     *  Product: '<S3>/Matrix Multiply1'
     */
    F_memory_PreviousInput = localDW->F_memory_PreviousInput[i_0 + 3];
    F_memory_PreviousInput_0 = localDW->F_memory_PreviousInput[i_0];
    F_memory_PreviousInput_1 = localDW->F_memory_PreviousInput[i_0 + 6];

    /* Product: '<S2>/Matrix Multiply' */
    P_memory_PreviousInput = 0.0;
    for (i = 0; i < 3; i++) {
      /* Sum: '<S3>/Add' incorporates:
       *  Memory: '<S1>/Q_memory'
       *  Product: '<S3>/Matrix Multiply1'
       */
      rtb_Add_tmp = 3 * i + i_0;
      P_memory_PreviousInput_0 = ((tmp[3 * i + 1] * F_memory_PreviousInput +
        tmp[3 * i] * F_memory_PreviousInput_0) + tmp[3 * i + 2] *
        F_memory_PreviousInput_1) + localDW->Q_memory_PreviousInput[rtb_Add_tmp];
      rtb_Add[rtb_Add_tmp] = P_memory_PreviousInput_0;

      /* Product: '<S2>/Matrix Multiply' incorporates:
       *  Memory: '<S1>/H_memory'
       */
      P_memory_PreviousInput += P_memory_PreviousInput_0 *
        localDW->H_memory_PreviousInput[i];
    }

    /* Product: '<S2>/Matrix Multiply' incorporates:
     *  Memory: '<S1>/H_memory'
     */
    rtb_Product_n += localDW->H_memory_PreviousInput[i_0] *
      P_memory_PreviousInput;
  }

  /* Sum: '<S2>/Add' incorporates:
   *  Memory: '<S1>/H_memory1'
   *  Product: '<S2>/Matrix Multiply'
   */
  rtb_Product_n += localDW->H_memory1_PreviousInput;

  /* Product: '<S2>/Matrix Multiply1' incorporates:
   *  Memory: '<S1>/H_memory'
   */
  P_memory_PreviousInput = localDW->H_memory_PreviousInput[0] / rtb_Product_n;
  P_memory_PreviousInput_0 = localDW->H_memory_PreviousInput[1] / rtb_Product_n;
  F_memory_PreviousInput = localDW->H_memory_PreviousInput[2] / rtb_Product_n;

  /* Product: '<S2>/Matrix Multiply4' */
  rtb_Product_n = 0.0;

  /* Memory: '<S1>/x_memory' incorporates:
   *  Product: '<S3>/Matrix Multiply'
   */
  F_memory_PreviousInput_0 = localDW->x_memory_PreviousInput[1];
  F_memory_PreviousInput_1 = localDW->x_memory_PreviousInput[0];
  x_memory_PreviousInput = localDW->x_memory_PreviousInput[2];
  for (i_0 = 0; i_0 < 3; i_0++) {
    /* Product: '<S3>/Matrix Multiply' incorporates:
     *  Memory: '<S1>/F_memory'
     *  Memory: '<S1>/x_memory'
     */
    rtb_MatrixMultiply_0 = (localDW->F_memory_PreviousInput[i_0 + 3] *
      F_memory_PreviousInput_0 + localDW->F_memory_PreviousInput[i_0] *
      F_memory_PreviousInput_1) + localDW->F_memory_PreviousInput[i_0 + 6] *
      x_memory_PreviousInput;
    rtb_MatrixMultiply[i_0] = rtb_MatrixMultiply_0;

    /* Product: '<S2>/Matrix Multiply1' incorporates:
     *  Sum: '<S3>/Add'
     */
    rtb_K[i_0] = (rtb_Add[i_0 + 3] * P_memory_PreviousInput_0 + rtb_Add[i_0] *
                  P_memory_PreviousInput) + rtb_Add[i_0 + 6] *
      F_memory_PreviousInput;

    /* Product: '<S2>/Matrix Multiply4' incorporates:
     *  Memory: '<S1>/H_memory'
     *  Product: '<S3>/Matrix Multiply'
     */
    rtb_Product_n += localDW->H_memory_PreviousInput[i_0] * rtb_MatrixMultiply_0;
  }

  /* Sum: '<S2>/Subtract' incorporates:
   *  Product: '<S2>/Matrix Multiply4'
   */
  rtb_Product_n = rtu_Pressure - rtb_Product_n;

  /* DigitalClock: '<S1>/Digital Clock' */
  localB->BusCreator.timestamp = ((((&run_ADA_M)->Timing.clockTick0+(&run_ADA_M
    )->Timing.clockTickH0* 4294967296.0)) * 0.02);
  for (i = 0; i < 3; i++) {
    /* Sum: '<S2>/Add2' incorporates:
     *  Product: '<S2>/Matrix Multiply1'
     *  Product: '<S2>/Matrix Multiply5'
     *  Product: '<S3>/Matrix Multiply'
     */
    localDW->x_memory_PreviousInput[i] = rtb_K[i] * rtb_Product_n +
      rtb_MatrixMultiply[i];

    /* Memory: '<S1>/H_memory' incorporates:
     *  Product: '<S2>/Matrix Multiply2'
     */
    P_memory_PreviousInput = localDW->H_memory_PreviousInput[i];

    /* Sum: '<S2>/Add1' incorporates:
     *  Constant: '<S2>/I'
     *  Memory: '<S1>/H_memory'
     *  Product: '<S2>/Matrix Multiply1'
     *  Product: '<S2>/Matrix Multiply2'
     */
    tmp[3 * i] = run_ADA_ConstP.pooled5[3 * i] - rtb_K[0] *
      P_memory_PreviousInput;
    i_0 = 3 * i + 1;
    tmp[i_0] = run_ADA_ConstP.pooled5[i_0] - rtb_K[1] * P_memory_PreviousInput;
    i_0 = 3 * i + 2;
    tmp[i_0] = run_ADA_ConstP.pooled5[i_0] - rtb_K[2] * P_memory_PreviousInput;
  }

  /* Math: '<S4>/Pow' incorporates:
   *  Constant: '<S4>/P_ref'
   *  Math: '<S5>/Pow'
   *  Product: '<S4>/Divide1'
   */
  P_memory_PreviousInput = std::pow(localDW->x_memory_PreviousInput[0] /
    101325.0, 0.19025441011522382);

  /* Product: '<S4>/Product' incorporates:
   *  Constant: '<S4>/Constant'
   *  Math: '<S4>/Pow'
   *  Sum: '<S4>/Subtract'
   */
  rtb_Product_n = (1.0 - P_memory_PreviousInput) * 44330.769230769227;

  /* Sum: '<S1>/Subtract' incorporates:
   *  Constant: '<S1>/Constant'
   */
  localB->BusCreator.aglAltitude = rtb_Product_n - 160.0;

  /* BusCreator: '<S1>/Bus Creator' incorporates:
   *  Constant: '<S5>/Constant1'
   *  Constant: '<S5>/T_ref'
   *  Constant: '<S5>/a'
   *  Gain: '<S5>/Gain'
   *  Product: '<S5>/Divide'
   *  Product: '<S5>/Product'
   *  Product: '<S5>/Product1'
   */
  localB->BusCreator.mslAltitude = rtb_Product_n;
  localB->BusCreator.verticalSpeed = -(1.0 / (0.0012366536657489548 *
    localDW->x_memory_PreviousInput[0]) * (288.15 *
    localDW->x_memory_PreviousInput[1] * P_memory_PreviousInput));
  localB->BusCreator.x0 = localDW->x_memory_PreviousInput[0];
  localB->BusCreator.x1 = localDW->x_memory_PreviousInput[1];
  localB->BusCreator.x2 = localDW->x_memory_PreviousInput[2];

  /* Update for Memory: '<S1>/P_memory' incorporates:
   *  Product: '<S2>/Matrix Multiply3'
   */
  for (i_0 = 0; i_0 < 3; i_0++) {
    /* Product: '<S2>/Matrix Multiply3' incorporates:
     *  Sum: '<S3>/Add'
     */
    P_memory_PreviousInput = rtb_Add[3 * i_0 + 1];
    P_memory_PreviousInput_0 = rtb_Add[3 * i_0];
    rtb_Product_n = rtb_Add[3 * i_0 + 2];
    for (i = 0; i <= 0; i += 2) {
      tmp_0 = _mm_loadu_pd(&tmp[i + 3]);
      tmp_1 = _mm_loadu_pd(&tmp[i]);
      tmp_2 = _mm_loadu_pd(&tmp[i + 6]);
      _mm_storeu_pd(&localDW->P_memory_PreviousInput[i + 3 * i_0], _mm_add_pd
                    (_mm_add_pd(_mm_mul_pd(_mm_set1_pd(P_memory_PreviousInput),
        tmp_0), _mm_mul_pd(_mm_set1_pd(P_memory_PreviousInput_0), tmp_1)),
                     _mm_mul_pd(_mm_set1_pd(rtb_Product_n), tmp_2)));
    }

    for (i = 2; i < 3; i++) {
      localDW->P_memory_PreviousInput[i + 3 * i_0] = (tmp[i + 3] *
        P_memory_PreviousInput + P_memory_PreviousInput_0 * tmp[i]) + tmp[i + 6]
        * rtb_Product_n;
    }
  }

  /* End of Update for Memory: '<S1>/P_memory' */
}

/* Model step function */
void run_ADA::step()
{
  /* Outputs for Atomic SubSystem: '<Root>/run_ADA' */

  /* Inport: '<Root>/Pressure' */
  run_ADA_run_ADA(run_ADA_U.Pressure, &run_ADA_B.run_ADA_e,
                  &run_ADA_DW.run_ADA_e);

  /* End of Outputs for SubSystem: '<Root>/run_ADA' */

  /* Outport: '<Root>/ADAState' */
  run_ADA_Y.ADAState_o = run_ADA_B.run_ADA_e.BusCreator;

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.02, which is the step size
   * of the task. Size of "clockTick0" ensures timer will not overflow during the
   * application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  (&run_ADA_M)->Timing.clockTick0++;
  if (!(&run_ADA_M)->Timing.clockTick0) {
    (&run_ADA_M)->Timing.clockTickH0++;
  }
}

/* Model initialize function */
void run_ADA::initialize()
{
  /* SystemInitialize for Atomic SubSystem: '<Root>/run_ADA' */
  run_ADA_run_ADA_Init(&run_ADA_DW.run_ADA_e);

  /* End of SystemInitialize for SubSystem: '<Root>/run_ADA' */
}

/* Model terminate function */
void run_ADA::terminate()
{
  /* (no terminate code required) */
}

/* Constructor */
run_ADA::run_ADA() :
  run_ADA_U(),
  run_ADA_Y(),
  run_ADA_B(),
  run_ADA_DW(),
  run_ADA_M()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
/* Currently there is no destructor body generated.*/
run_ADA::~run_ADA() = default;

/* Real-Time Model get method */
RT_MODEL_run_ADA_T * run_ADA::getRTM()
{
  return (&run_ADA_M);
}
