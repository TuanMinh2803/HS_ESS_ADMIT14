/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: arduino_CAN_simple_example_data.c
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

#include "arduino_CAN_simple_example.h"

/* Block parameters (default storage) */
P_arduino_CAN_simple_example_T arduino_CAN_simple_example_P = {
  /* Mask Parameter: CompareToConstant_const
   * Referenced by: '<S1>/Constant'
   */
  5U,

  /* Mask Parameter: RepeatingSequenceStair_OutValue
   * Referenced by: '<S3>/Vector'
   */
  { 1U, 1U, 0U, 1U, 0U, 0U, 1U },

  /* Mask Parameter: WrapToZero_Threshold
   * Referenced by: '<S6>/FixPt Switch'
   */
  6U,

  /* Expression: uint16(CANM_ID)
   * Referenced by: '<Root>/CAN_RX_ID21'
   */
  21U,

  /* Expression: uint16(CANM_ID)
   * Referenced by: '<Root>/CAN_RTR_ID55'
   */
  55U,

  /* Expression: uint16(CANM_ID)
   * Referenced by: '<Root>/CAN_RX_ID20'
   */
  20U,

  /* Expression: uint16(CANM_ID)
   * Referenced by: '<Root>/CAN_RX_ID22'
   */
  22U,

  /* Expression: uint16(CANM_ID)
   * Referenced by: '<Root>/CAN_TX_ID33'
   */
  33U,

  /* Expression: uint16(CANM_ID)
   * Referenced by: '<Root>/CAN_TX_ID105'
   */
  105U,

  /* Computed Parameter: Constant_Value
   * Referenced by: '<S6>/Constant'
   */
  0U,

  /* Computed Parameter: Constant_Value_m
   * Referenced by: '<S2>/Constant'
   */
  0U,

  /* Expression: uint8(CANM_DNTYPE)
   * Referenced by: '<Root>/CAN_RX_ID21'
   */
  0U,

  /* Expression: uint8(CANM_DNTYPE)
   * Referenced by: '<Root>/CAN_RX_ID20'
   */
  0U,

  /* Expression: uint8(CANM_DNTYPE)
   * Referenced by: '<Root>/CAN_RX_ID22'
   */
  0U,

  /* Expression: uint8(CANM_DNTYPE)
   * Referenced by: '<Root>/CAN_TX_ID33'
   */
  0U,

  /* Computed Parameter: ConstantShort2_Value
   * Referenced by: '<Root>/Constant Short2'
   */
  1U,

  /* Computed Parameter: ConstantShort_Value
   * Referenced by: '<Root>/Constant Short'
   */
  2U,

  /* Computed Parameter: ConstantShort1_Value
   * Referenced by: '<Root>/Constant Short1'
   */
  3U,

  /* Expression: uint8(CANM_DNTYPE)
   * Referenced by: '<Root>/CAN_TX_ID105'
   */
  0U,

  /* Computed Parameter: Output_InitialCondition
   * Referenced by: '<S4>/Output'
   */
  0U,

  /* Computed Parameter: FixPtConstant_Value
   * Referenced by: '<S5>/FixPt Constant'
   */
  1U,

  /* Expression: uint8(CANSPEEDVALUE)
   * Referenced by: '<Root>/CAN_SETUP'
   */
  3U
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
