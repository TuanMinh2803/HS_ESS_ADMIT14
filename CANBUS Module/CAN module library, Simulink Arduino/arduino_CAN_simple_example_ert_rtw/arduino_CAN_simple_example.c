/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: arduino_CAN_simple_example.c
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
#include "rtwtypes.h"
#include <string.h>

uint8_t isReceived_CAN_RX_ID21;
tCAN RMsg_CAN_RX_ID21;
tCAN RTRMsg_CAN_RTR_ID55;
uint8_t isReceived_CAN_RX_ID20;
tCAN RMsg_CAN_RX_ID20;
uint8_t isReceived_CAN_RX_ID22;
tCAN RMsg_CAN_RX_ID22;
tCAN TMsg_CAN_TX_ID33;
tCAN TMsg_CAN_TX_ID105;

/* Block signals (default storage) */
B_arduino_CAN_simple_example_T arduino_CAN_simple_example_B;

/* Block states (default storage) */
DW_arduino_CAN_simple_example_T arduino_CAN_simple_example_DW;

/* Real-time model */
static RT_MODEL_arduino_CAN_simple_e_T arduino_CAN_simple_example_M_;
RT_MODEL_arduino_CAN_simple_e_T *const arduino_CAN_simple_example_M =
  &arduino_CAN_simple_example_M_;

/* Model step function */
void arduino_CAN_simple_example_step(void)
{
  int16_T i;
  uint16_T tmp;
  uint8_T rtb_FixPtSum1;

  /* S-Function (arduino_CAN_rx_msg): '<Root>/CAN_RX_ID21' */

  /* Retreive the relevant messages from CAN buffer (if any) */
  retreive_RxCAN();

  /* Get the CAN message */
  /* Prepare the output CAN message */
  arduino_CAN_simple_example_B.CAN_RX_ID21_o2[0] = RMsg_CAN_RX_ID21.data[0];

  /* Prepare the output CAN message */
  arduino_CAN_simple_example_B.CAN_RX_ID21_o2[1] = RMsg_CAN_RX_ID21.data[1];

  /* Prepare the output CAN message */
  arduino_CAN_simple_example_B.CAN_RX_ID21_o2[2] = RMsg_CAN_RX_ID21.data[2];

  /* Prepare the output CAN message */
  arduino_CAN_simple_example_B.CAN_RX_ID21_o2[3] = RMsg_CAN_RX_ID21.data[3];

  /* Prepare the output CAN message */
  arduino_CAN_simple_example_B.CAN_RX_ID21_o2[4] = RMsg_CAN_RX_ID21.data[4];

  /* Prepare the output CAN message */
  arduino_CAN_simple_example_B.CAN_RX_ID21_o2[5] = RMsg_CAN_RX_ID21.data[5];

  /* Prepare the output CAN message */
  arduino_CAN_simple_example_B.CAN_RX_ID21_o2[6] = RMsg_CAN_RX_ID21.data[6];

  /* Prepare the output CAN message */
  arduino_CAN_simple_example_B.CAN_RX_ID21_o2[7] = RMsg_CAN_RX_ID21.data[7];
  arduino_CAN_simple_example_B.CAN_RX_ID21_o1 = isReceived_CAN_RX_ID21;
  isReceived_CAN_RX_ID21 = 0;

  /* Sum: '<Root>/Sum of Elements' */
  tmp = 0U;
  for (i = 0; i < 8; i++) {
    tmp += arduino_CAN_simple_example_B.CAN_RX_ID21_o2[i];
  }

  /* RelationalOperator: '<S1>/Compare' incorporates:
   *  Constant: '<S1>/Constant'
   *  Product: '<Root>/Product'
   *  Sum: '<Root>/Sum of Elements'
   */
  arduino_CAN_simple_example_B.Compare = (uint8_T)((uint16_T)(uint8_T)tmp *
    arduino_CAN_simple_example_B.CAN_RX_ID21_o1 >
    arduino_CAN_simple_example_P.CompareToConstant_const);

  /* S-Function (arduino_CAN_rtr_msg): '<Root>/CAN_RTR_ID55' */
  if (arduino_CAN_simple_example_B.Compare > 0) {
    /* Send the Remote Request */
    mcp2515_send_message(&RTRMsg_CAN_RTR_ID55);
  }

  /* RelationalOperator: '<S2>/Compare' incorporates:
   *  Constant: '<S2>/Constant'
   *  Sum: '<Root>/Sum of Elements'
   */
  arduino_CAN_simple_example_B.Compare_c = (uint8_T)((uint8_T)tmp >
    arduino_CAN_simple_example_P.Constant_Value_m);

  /* S-Function (arduino_CAN_rx_msg): '<Root>/CAN_RX_ID20' */

  /* Get the CAN message */
  /* Prepare the output CAN message */
  arduino_CAN_simple_example_B.CAN_RX_ID20_o2[0] = RMsg_CAN_RX_ID20.data[0];

  /* Prepare the output CAN message */
  arduino_CAN_simple_example_B.CAN_RX_ID20_o2[1] = RMsg_CAN_RX_ID20.data[1];

  /* Prepare the output CAN message */
  arduino_CAN_simple_example_B.CAN_RX_ID20_o2[2] = RMsg_CAN_RX_ID20.data[2];

  /* Prepare the output CAN message */
  arduino_CAN_simple_example_B.CAN_RX_ID20_o2[3] = RMsg_CAN_RX_ID20.data[3];

  /* Prepare the output CAN message */
  arduino_CAN_simple_example_B.CAN_RX_ID20_o2[4] = RMsg_CAN_RX_ID20.data[4];

  /* Prepare the output CAN message */
  arduino_CAN_simple_example_B.CAN_RX_ID20_o2[5] = RMsg_CAN_RX_ID20.data[5];

  /* Prepare the output CAN message */
  arduino_CAN_simple_example_B.CAN_RX_ID20_o2[6] = RMsg_CAN_RX_ID20.data[6];

  /* Prepare the output CAN message */
  arduino_CAN_simple_example_B.CAN_RX_ID20_o2[7] = RMsg_CAN_RX_ID20.data[7];
  arduino_CAN_simple_example_B.CAN_RX_ID20_o1 = isReceived_CAN_RX_ID20;
  isReceived_CAN_RX_ID20 = 0;

  /* S-Function (arduino_CAN_rx_msg): '<Root>/CAN_RX_ID22' */

  /* Get the CAN message */
  /* Prepare the output CAN message */
  arduino_CAN_simple_example_B.CAN_RX_ID22_o2[0] = RMsg_CAN_RX_ID22.data[0];

  /* Prepare the output CAN message */
  arduino_CAN_simple_example_B.CAN_RX_ID22_o2[1] = RMsg_CAN_RX_ID22.data[1];

  /* Prepare the output CAN message */
  arduino_CAN_simple_example_B.CAN_RX_ID22_o2[2] = RMsg_CAN_RX_ID22.data[2];

  /* Prepare the output CAN message */
  arduino_CAN_simple_example_B.CAN_RX_ID22_o2[3] = RMsg_CAN_RX_ID22.data[3];

  /* Prepare the output CAN message */
  arduino_CAN_simple_example_B.CAN_RX_ID22_o2[4] = RMsg_CAN_RX_ID22.data[4];

  /* Prepare the output CAN message */
  arduino_CAN_simple_example_B.CAN_RX_ID22_o2[5] = RMsg_CAN_RX_ID22.data[5];

  /* Prepare the output CAN message */
  arduino_CAN_simple_example_B.CAN_RX_ID22_o2[6] = RMsg_CAN_RX_ID22.data[6];

  /* Prepare the output CAN message */
  arduino_CAN_simple_example_B.CAN_RX_ID22_o2[7] = RMsg_CAN_RX_ID22.data[7];
  arduino_CAN_simple_example_B.CAN_RX_ID22_o1 = isReceived_CAN_RX_ID22;
  isReceived_CAN_RX_ID22 = 0;
  for (i = 0; i < 8; i++) {
    /* Sum: '<Root>/Add' */
    arduino_CAN_simple_example_B.Add[i] = (uint8_T)((uint8_T)
      (arduino_CAN_simple_example_B.CAN_RX_ID20_o2[i] +
       arduino_CAN_simple_example_B.CAN_RX_ID21_o2[i]) +
      arduino_CAN_simple_example_B.CAN_RX_ID22_o2[i]);
  }

  /* S-Function (arduino_CAN_tx_msg): '<Root>/CAN_TX_ID33' */
  if (arduino_CAN_simple_example_B.Compare_c > 0) {
    /* Prepare the CAN message */
    TMsg_CAN_TX_ID33.id = 33U;
    TMsg_CAN_TX_ID33.header.rtr = 0;
    TMsg_CAN_TX_ID33.header.length = 8;
    TMsg_CAN_TX_ID33.data[0] = arduino_CAN_simple_example_B.Add[0];
    TMsg_CAN_TX_ID33.data[1] = arduino_CAN_simple_example_B.Add[1];
    TMsg_CAN_TX_ID33.data[2] = arduino_CAN_simple_example_B.Add[2];
    TMsg_CAN_TX_ID33.data[3] = arduino_CAN_simple_example_B.Add[3];
    TMsg_CAN_TX_ID33.data[4] = arduino_CAN_simple_example_B.Add[4];
    TMsg_CAN_TX_ID33.data[5] = arduino_CAN_simple_example_B.Add[5];
    TMsg_CAN_TX_ID33.data[6] = arduino_CAN_simple_example_B.Add[6];
    TMsg_CAN_TX_ID33.data[7] = arduino_CAN_simple_example_B.Add[7];

    /* Send the CAN message */
    mcp2515_send_message(&TMsg_CAN_TX_ID33);
  }

  /* MATLABSystem: '<Root>/Digital Output1' incorporates:
   *  DataTypeConversion: '<Root>/Conversion'
   *  Logic: '<Root>/Logical Operator'
   */
  writeDigitalPin(8, (uint8_T)((arduino_CAN_simple_example_B.CAN_RX_ID22_o1 != 0)
    || (arduino_CAN_simple_example_B.CAN_RX_ID21_o1 != 0) ||
    (arduino_CAN_simple_example_B.CAN_RX_ID20_o1 != 0)));

  /* SignalConversion generated from: '<Root>/CAN_TX_ID105' incorporates:
   *  Constant: '<Root>/Constant Short'
   *  Constant: '<Root>/Constant Short1'
   */
  arduino_CAN_simple_example_B.TmpSignalConversionAtCAN_TX_ID1[0] =
    arduino_CAN_simple_example_P.ConstantShort_Value;
  arduino_CAN_simple_example_B.TmpSignalConversionAtCAN_TX_ID1[1] =
    arduino_CAN_simple_example_P.ConstantShort_Value;
  arduino_CAN_simple_example_B.TmpSignalConversionAtCAN_TX_ID1[2] =
    arduino_CAN_simple_example_P.ConstantShort_Value;
  arduino_CAN_simple_example_B.TmpSignalConversionAtCAN_TX_ID1[3] =
    arduino_CAN_simple_example_P.ConstantShort_Value;
  arduino_CAN_simple_example_B.TmpSignalConversionAtCAN_TX_ID1[4] =
    arduino_CAN_simple_example_P.ConstantShort1_Value;
  arduino_CAN_simple_example_B.TmpSignalConversionAtCAN_TX_ID1[5] =
    arduino_CAN_simple_example_P.ConstantShort1_Value;
  arduino_CAN_simple_example_B.TmpSignalConversionAtCAN_TX_ID1[6] =
    arduino_CAN_simple_example_P.ConstantShort1_Value;
  arduino_CAN_simple_example_B.TmpSignalConversionAtCAN_TX_ID1[7] =
    arduino_CAN_simple_example_P.ConstantShort1_Value;

  /* S-Function (arduino_CAN_tx_msg): '<Root>/CAN_TX_ID105' incorporates:
   *  Constant: '<Root>/Constant Short2'
   */
  if (arduino_CAN_simple_example_P.ConstantShort2_Value > 0) {
    /* Prepare the CAN message */
    TMsg_CAN_TX_ID105.id = 105U;
    TMsg_CAN_TX_ID105.header.rtr = 0;
    TMsg_CAN_TX_ID105.header.length = 8;
    TMsg_CAN_TX_ID105.data[0] =
      arduino_CAN_simple_example_B.TmpSignalConversionAtCAN_TX_ID1[0];
    TMsg_CAN_TX_ID105.data[1] =
      arduino_CAN_simple_example_B.TmpSignalConversionAtCAN_TX_ID1[1];
    TMsg_CAN_TX_ID105.data[2] =
      arduino_CAN_simple_example_B.TmpSignalConversionAtCAN_TX_ID1[2];
    TMsg_CAN_TX_ID105.data[3] =
      arduino_CAN_simple_example_B.TmpSignalConversionAtCAN_TX_ID1[3];
    TMsg_CAN_TX_ID105.data[4] =
      arduino_CAN_simple_example_B.TmpSignalConversionAtCAN_TX_ID1[4];
    TMsg_CAN_TX_ID105.data[5] =
      arduino_CAN_simple_example_B.TmpSignalConversionAtCAN_TX_ID1[5];
    TMsg_CAN_TX_ID105.data[6] =
      arduino_CAN_simple_example_B.TmpSignalConversionAtCAN_TX_ID1[6];
    TMsg_CAN_TX_ID105.data[7] =
      arduino_CAN_simple_example_B.TmpSignalConversionAtCAN_TX_ID1[7];

    /* Send the CAN message */
    mcp2515_send_message(&TMsg_CAN_TX_ID105);
  }

  /* MATLABSystem: '<Root>/Digital Output' incorporates:
   *  Constant: '<S3>/Vector'
   *  MultiPortSwitch: '<S3>/Output'
   *  UnitDelay: '<S4>/Output'
   */
  writeDigitalPin(7,
                  arduino_CAN_simple_example_P.RepeatingSequenceStair_OutValue[arduino_CAN_simple_example_DW.Output_DSTATE]);

  /* Sum: '<S5>/FixPt Sum1' incorporates:
   *  Constant: '<S5>/FixPt Constant'
   *  UnitDelay: '<S4>/Output'
   */
  rtb_FixPtSum1 = (uint8_T)(arduino_CAN_simple_example_DW.Output_DSTATE +
    arduino_CAN_simple_example_P.FixPtConstant_Value);

  /* Switch: '<S6>/FixPt Switch' */
  if (rtb_FixPtSum1 > arduino_CAN_simple_example_P.WrapToZero_Threshold) {
    /* Update for UnitDelay: '<S4>/Output' incorporates:
     *  Constant: '<S6>/Constant'
     */
    arduino_CAN_simple_example_DW.Output_DSTATE =
      arduino_CAN_simple_example_P.Constant_Value;
  } else {
    /* Update for UnitDelay: '<S4>/Output' */
    arduino_CAN_simple_example_DW.Output_DSTATE = rtb_FixPtSum1;
  }

  /* End of Switch: '<S6>/FixPt Switch' */
}

/* Model initialize function */
void arduino_CAN_simple_example_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(arduino_CAN_simple_example_M, (NULL));

  /* block I/O */
  (void) memset(((void *) &arduino_CAN_simple_example_B), 0,
                sizeof(B_arduino_CAN_simple_example_T));

  /* states (dwork) */
  (void) memset((void *)&arduino_CAN_simple_example_DW, 0,
                sizeof(DW_arduino_CAN_simple_example_T));

  /* user code (Start function Trailer) */
  RMsg_CAN_RX_ID21.id = 21U;
  RMsg_CAN_RX_ID21.header.rtr = 0;
  RMsg_CAN_RX_ID21.header.length = 8;
  RMsg_CAN_RX_ID21.data[0] = 0U;
  RMsg_CAN_RX_ID21.data[1] = 0U;
  RMsg_CAN_RX_ID21.data[2] = 0U;
  RMsg_CAN_RX_ID21.data[3] = 0U;
  RMsg_CAN_RX_ID21.data[4] = 0U;
  RMsg_CAN_RX_ID21.data[5] = 0U;
  RMsg_CAN_RX_ID21.data[6] = 0U;
  RMsg_CAN_RX_ID21.data[7] = 0U;
  isReceived_CAN_RX_ID21 = 0;
  RTRMsg_CAN_RTR_ID55.id = 55U;
  RTRMsg_CAN_RTR_ID55.header.rtr = 1;
  RTRMsg_CAN_RTR_ID55.header.length = 0;
  RMsg_CAN_RX_ID20.id = 20U;
  RMsg_CAN_RX_ID20.header.rtr = 0;
  RMsg_CAN_RX_ID20.header.length = 8;
  RMsg_CAN_RX_ID20.data[0] = 0U;
  RMsg_CAN_RX_ID20.data[1] = 0U;
  RMsg_CAN_RX_ID20.data[2] = 0U;
  RMsg_CAN_RX_ID20.data[3] = 0U;
  RMsg_CAN_RX_ID20.data[4] = 0U;
  RMsg_CAN_RX_ID20.data[5] = 0U;
  RMsg_CAN_RX_ID20.data[6] = 0U;
  RMsg_CAN_RX_ID20.data[7] = 0U;
  isReceived_CAN_RX_ID20 = 0;
  RMsg_CAN_RX_ID22.id = 22U;
  RMsg_CAN_RX_ID22.header.rtr = 0;
  RMsg_CAN_RX_ID22.header.length = 8;
  RMsg_CAN_RX_ID22.data[0] = 0U;
  RMsg_CAN_RX_ID22.data[1] = 0U;
  RMsg_CAN_RX_ID22.data[2] = 0U;
  RMsg_CAN_RX_ID22.data[3] = 0U;
  RMsg_CAN_RX_ID22.data[4] = 0U;
  RMsg_CAN_RX_ID22.data[5] = 0U;
  RMsg_CAN_RX_ID22.data[6] = 0U;
  RMsg_CAN_RX_ID22.data[7] = 0U;
  isReceived_CAN_RX_ID22 = 0;
  mcp2515_init(3U);

  /* InitializeConditions for UnitDelay: '<S4>/Output' */
  arduino_CAN_simple_example_DW.Output_DSTATE =
    arduino_CAN_simple_example_P.Output_InitialCondition;

  /* Start for MATLABSystem: '<Root>/Digital Output1' */
  arduino_CAN_simple_example_DW.obj.matlabCodegenIsDeleted = false;
  arduino_CAN_simple_example_DW.obj.isInitialized = 1L;
  digitalIOSetup(8, 1);
  arduino_CAN_simple_example_DW.obj.isSetupComplete = true;

  /* Start for MATLABSystem: '<Root>/Digital Output' */
  arduino_CAN_simple_example_DW.obj_m.matlabCodegenIsDeleted = false;
  arduino_CAN_simple_example_DW.obj_m.isInitialized = 1L;
  digitalIOSetup(7, 1);
  arduino_CAN_simple_example_DW.obj_m.isSetupComplete = true;
}

/* Model terminate function */
void arduino_CAN_simple_example_terminate(void)
{
  /* Terminate for MATLABSystem: '<Root>/Digital Output1' */
  if (!arduino_CAN_simple_example_DW.obj.matlabCodegenIsDeleted) {
    arduino_CAN_simple_example_DW.obj.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<Root>/Digital Output1' */
  /* Terminate for MATLABSystem: '<Root>/Digital Output' */
  if (!arduino_CAN_simple_example_DW.obj_m.matlabCodegenIsDeleted) {
    arduino_CAN_simple_example_DW.obj_m.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<Root>/Digital Output' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
