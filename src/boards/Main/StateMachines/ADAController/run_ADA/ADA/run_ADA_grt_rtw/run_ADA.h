/*
 * run_ADA.h
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

#ifndef RTW_HEADER_run_ADA_h_
#define RTW_HEADER_run_ADA_h_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "run_ADA_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals for system '<Root>/run_ADA' */
struct B_run_ADA_run_ADA_T {
  ADAState BusCreator;                 /* '<S1>/Bus Creator' */
};

/* Block states (default storage) for system '<Root>/run_ADA' */
struct DW_run_ADA_run_ADA_T {
  real_T H_memory_PreviousInput[3];    /* '<S1>/H_memory' */
  real_T F_memory_PreviousInput[9];    /* '<S1>/F_memory' */
  real_T P_memory_PreviousInput[9];    /* '<S1>/P_memory' */
  real_T Q_memory_PreviousInput[9];    /* '<S1>/Q_memory' */
  real_T H_memory1_PreviousInput;      /* '<S1>/H_memory1' */
  real_T MatrixMultiply1_DWORK4;       /* '<S2>/Matrix Multiply1' */
  real_T x_memory_PreviousInput[3];    /* '<S1>/x_memory' */
};

/* Block signals (default storage) */
struct B_run_ADA_T {
  B_run_ADA_run_ADA_T run_ADA_e;       /* '<Root>/run_ADA' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_run_ADA_T {
  DW_run_ADA_run_ADA_T run_ADA_e;      /* '<Root>/run_ADA' */
};

/* Constant parameters (default storage) */
struct ConstP_run_ADA_T {
  /* Expression: [1 0.02 0.5*0.02^2; 0 1 0.02; 0 0 1]
   * Referenced by: '<S1>/F_memory'
   */
  real_T F_memory_InitialCondition[9];

  /* Pooled Parameter (Expression: eye(3))
   * Referenced by:
   *   '<S1>/P_memory'
   *   '<S2>/I'
   */
  real_T pooled5[9];

  /* Expression: [30 0 0; 0 10 0; 0 0 2.5]
   * Referenced by: '<S1>/Q_memory'
   */
  real_T Q_memory_InitialCondition[9];
};

/* External inputs (root inport signals with default storage) */
struct ExtU_run_ADA_T {
  real_T Pressure;                     /* '<Root>/Pressure' */
};

/* External outputs (root outports fed by signals with default storage) */
struct ExtY_run_ADA_T {
  ADAState ADAState_o;                 /* '<Root>/ADAState' */
};

/* Real-time Model Data Structure */
struct tag_RTM_run_ADA_T {
  const char_T *errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
  } Timing;
};

/* Constant parameters (default storage) */
extern const ConstP_run_ADA_T run_ADA_ConstP;

/* Class declaration for model run_ADA */
class run_ADA final
{
  /* public data and function members */
 public:
  /* Copy Constructor */
  run_ADA(run_ADA const&) = delete;

  /* Assignment Operator */
  run_ADA& operator= (run_ADA const&) & = delete;

  /* Move Constructor */
  run_ADA(run_ADA &&) = delete;

  /* Move Assignment Operator */
  run_ADA& operator= (run_ADA &&) = delete;

  /* Real-Time Model get method */
  RT_MODEL_run_ADA_T * getRTM();

  /* Root inports set method */
  void setExternalInputs(const ExtU_run_ADA_T *pExtU_run_ADA_T)
  {
    run_ADA_U = *pExtU_run_ADA_T;
  }

  /* Root outports get method */
  const ExtY_run_ADA_T &getExternalOutputs() const
  {
    return run_ADA_Y;
  }

  /* Initial conditions function */
  void initialize();

  /* model step function */
  void step();

  /* model terminate function */
  static void terminate();

  /* Constructor */
  run_ADA();

  /* Destructor */
  ~run_ADA();

  /* private data and function members */
 private:
  /* External inputs */
  ExtU_run_ADA_T run_ADA_U;

  /* External outputs */
  ExtY_run_ADA_T run_ADA_Y;

  /* Block signals */
  B_run_ADA_T run_ADA_B;

  /* Block states */
  DW_run_ADA_T run_ADA_DW;

  /* private member function(s) for subsystem '<Root>/run_ADA'*/
  static void run_ADA_run_ADA_Init(DW_run_ADA_run_ADA_T *localDW);
  void run_ADA_run_ADA(real_T rtu_Pressure, B_run_ADA_run_ADA_T *localB,
                       DW_run_ADA_run_ADA_T *localDW) const;

  /* Real-Time Model */
  RT_MODEL_run_ADA_T run_ADA_M;
};

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S2>/Reshape' : Reshape block reduction
 * Block '<S2>/Reshape2' : Reshape block reduction
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('test_ada/run_ADA')    - opens subsystem test_ada/run_ADA
 * hilite_system('test_ada/run_ADA/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'test_ada'
 * '<S1>'   : 'test_ada/run_ADA'
 * '<S2>'   : 'test_ada/run_ADA/Correction'
 * '<S3>'   : 'test_ada/run_ADA/Prediction'
 * '<S4>'   : 'test_ada/run_ADA/getAltitude'
 * '<S5>'   : 'test_ada/run_ADA/getVelocity'
 */
#endif                                 /* RTW_HEADER_run_ADA_h_ */
