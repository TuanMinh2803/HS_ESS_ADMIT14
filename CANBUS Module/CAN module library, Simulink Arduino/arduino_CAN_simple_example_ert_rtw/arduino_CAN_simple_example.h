/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: arduino_CAN_simple_example.h
 *
 * Code generated for Simulink model 'arduino_CAN_simple_example'.
 *
 * Model version                  : 1.11
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Tue Mar 25 16:22:10 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Atmel->AVR
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef arduino_CAN_simple_example_h_
#define arduino_CAN_simple_example_h_
#ifndef arduino_CAN_simple_example_COMMON_INCLUDES_
#define arduino_CAN_simple_example_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "mcp2515.h"
#include "MW_arduino_digitalio.h"
#endif                         /* arduino_CAN_simple_example_COMMON_INCLUDES_ */

#include "arduino_CAN_simple_example_types.h"
#include <stddef.h>
#include <string.h>
#include "MW_target_hardware_resources.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
typedef struct {
  uint8_T CAN_RX_ID21_o1;              /* '<Root>/CAN_RX_ID21' */
  uint8_T CAN_RX_ID21_o2[8];           /* '<Root>/CAN_RX_ID21' */
  uint8_T Compare;                     /* '<S1>/Compare' */
  uint8_T Compare_c;                   /* '<S2>/Compare' */
  uint8_T CAN_RX_ID20_o1;              /* '<Root>/CAN_RX_ID20' */
  uint8_T CAN_RX_ID20_o2[8];           /* '<Root>/CAN_RX_ID20' */
  uint8_T CAN_RX_ID22_o1;              /* '<Root>/CAN_RX_ID22' */
  uint8_T CAN_RX_ID22_o2[8];           /* '<Root>/CAN_RX_ID22' */
  uint8_T Add[8];                      /* '<Root>/Add' */
  uint8_T TmpSignalConversionAtCAN_TX_ID1[8];
} B_arduino_CAN_simple_example_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  codertarget_arduinobase_block_T obj; /* '<Root>/Digital Output1' */
  codertarget_arduinobase_block_T obj_m;/* '<Root>/Digital Output' */
  uint8_T Output_DSTATE;               /* '<S4>/Output' */
} DW_arduino_CAN_simple_example_T;

/* Parameters (default storage) */
struct P_arduino_CAN_simple_example_T_ {
  uint16_T CompareToConstant_const;   /* Mask Parameter: CompareToConstant_const
                                       * Referenced by: '<S1>/Constant'
                                       */
  uint8_T RepeatingSequenceStair_OutValue[7];
                              /* Mask Parameter: RepeatingSequenceStair_OutValue
                               * Referenced by: '<S3>/Vector'
                               */
  uint8_T WrapToZero_Threshold;        /* Mask Parameter: WrapToZero_Threshold
                                        * Referenced by: '<S6>/FixPt Switch'
                                        */
  uint16_T CAN_RX_ID21_P1;             /* Expression: uint16(CANM_ID)
                                        * Referenced by: '<Root>/CAN_RX_ID21'
                                        */
  uint16_T CAN_RTR_ID55_P1;            /* Expression: uint16(CANM_ID)
                                        * Referenced by: '<Root>/CAN_RTR_ID55'
                                        */
  uint16_T CAN_RX_ID20_P1;             /* Expression: uint16(CANM_ID)
                                        * Referenced by: '<Root>/CAN_RX_ID20'
                                        */
  uint16_T CAN_RX_ID22_P1;             /* Expression: uint16(CANM_ID)
                                        * Referenced by: '<Root>/CAN_RX_ID22'
                                        */
  uint16_T CAN_TX_ID33_P1;             /* Expression: uint16(CANM_ID)
                                        * Referenced by: '<Root>/CAN_TX_ID33'
                                        */
  uint16_T CAN_TX_ID105_P1;            /* Expression: uint16(CANM_ID)
                                        * Referenced by: '<Root>/CAN_TX_ID105'
                                        */
  uint8_T Constant_Value;              /* Computed Parameter: Constant_Value
                                        * Referenced by: '<S6>/Constant'
                                        */
  uint8_T Constant_Value_m;            /* Computed Parameter: Constant_Value_m
                                        * Referenced by: '<S2>/Constant'
                                        */
  uint8_T CAN_RX_ID21_P2;              /* Expression: uint8(CANM_DNTYPE)
                                        * Referenced by: '<Root>/CAN_RX_ID21'
                                        */
  uint8_T CAN_RX_ID20_P2;              /* Expression: uint8(CANM_DNTYPE)
                                        * Referenced by: '<Root>/CAN_RX_ID20'
                                        */
  uint8_T CAN_RX_ID22_P2;              /* Expression: uint8(CANM_DNTYPE)
                                        * Referenced by: '<Root>/CAN_RX_ID22'
                                        */
  uint8_T CAN_TX_ID33_P2;              /* Expression: uint8(CANM_DNTYPE)
                                        * Referenced by: '<Root>/CAN_TX_ID33'
                                        */
  uint8_T ConstantShort2_Value;      /* Computed Parameter: ConstantShort2_Value
                                      * Referenced by: '<Root>/Constant Short2'
                                      */
  uint8_T ConstantShort_Value;        /* Computed Parameter: ConstantShort_Value
                                       * Referenced by: '<Root>/Constant Short'
                                       */
  uint8_T ConstantShort1_Value;      /* Computed Parameter: ConstantShort1_Value
                                      * Referenced by: '<Root>/Constant Short1'
                                      */
  uint8_T CAN_TX_ID105_P2;             /* Expression: uint8(CANM_DNTYPE)
                                        * Referenced by: '<Root>/CAN_TX_ID105'
                                        */
  uint8_T Output_InitialCondition;/* Computed Parameter: Output_InitialCondition
                                   * Referenced by: '<S4>/Output'
                                   */
  uint8_T FixPtConstant_Value;        /* Computed Parameter: FixPtConstant_Value
                                       * Referenced by: '<S5>/FixPt Constant'
                                       */
  uint8_T CAN_SETUP_P1;                /* Expression: uint8(CANSPEEDVALUE)
                                        * Referenced by: '<Root>/CAN_SETUP'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_arduino_CAN_simple_ex_T {
  const char_T * volatile errorStatus;
};

/* Block parameters (default storage) */
extern P_arduino_CAN_simple_example_T arduino_CAN_simple_example_P;

/* Block signals (default storage) */
extern B_arduino_CAN_simple_example_T arduino_CAN_simple_example_B;

/* Block states (default storage) */
extern DW_arduino_CAN_simple_example_T arduino_CAN_simple_example_DW;

/* Model entry point functions */
extern void arduino_CAN_simple_example_initialize(void);
extern void arduino_CAN_simple_example_step(void);
extern void arduino_CAN_simple_example_terminate(void);

/* Real-time Model object */
extern RT_MODEL_arduino_CAN_simple_e_T *const arduino_CAN_simple_example_M;
extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S4>/Data Type Propagation' : Unused code path elimination
 * Block '<S5>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S6>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S3>/Out' : Eliminate redundant signal conversion block
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'arduino_CAN_simple_example'
 * '<S1>'   : 'arduino_CAN_simple_example/Compare To Constant'
 * '<S2>'   : 'arduino_CAN_simple_example/Compare To Zero'
 * '<S3>'   : 'arduino_CAN_simple_example/Repeating Sequence Stair'
 * '<S4>'   : 'arduino_CAN_simple_example/Repeating Sequence Stair/LimitedCounter'
 * '<S5>'   : 'arduino_CAN_simple_example/Repeating Sequence Stair/LimitedCounter/Increment Real World'
 * '<S6>'   : 'arduino_CAN_simple_example/Repeating Sequence Stair/LimitedCounter/Wrap To Zero'
 */
#endif                                 /* arduino_CAN_simple_example_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
