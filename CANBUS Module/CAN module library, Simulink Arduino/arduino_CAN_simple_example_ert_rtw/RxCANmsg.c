/* Retreive the relevant messages from CAN buffer (if any) */
/* Function implementation */
#include "mcp2515.h"
#include "RxCANmsg.h"
#define LED3                           8U

void retreive_RxCAN()
{
  uint8_t isNotFinished;
  uint8_t i;
  tCAN tempCANmsg;

  /* Retreive CAN messages */
  if (mcp2515_check_message()) {
    isNotFinished = 0;

    /* Loop through CAN messages and assign it to the right place*/
    isNotFinished = mcp2515_get_message(&tempCANmsg);
    while (isNotFinished) {
      if (tempCANmsg.id == RMsg_CAN_RX_ID21.id) {
        RMsg_CAN_RX_ID21.data[0] = tempCANmsg.data[0];
        RMsg_CAN_RX_ID21.data[1] = tempCANmsg.data[1];
        RMsg_CAN_RX_ID21.data[2] = tempCANmsg.data[2];
        RMsg_CAN_RX_ID21.data[3] = tempCANmsg.data[3];
        RMsg_CAN_RX_ID21.data[4] = tempCANmsg.data[4];
        RMsg_CAN_RX_ID21.data[5] = tempCANmsg.data[5];
        RMsg_CAN_RX_ID21.data[6] = tempCANmsg.data[6];
        RMsg_CAN_RX_ID21.data[7] = tempCANmsg.data[7];
        isReceived_CAN_RX_ID21 = 1;
      } else {
      }

      if (tempCANmsg.id == RMsg_CAN_RX_ID20.id) {
        RMsg_CAN_RX_ID20.data[0] = tempCANmsg.data[0];
        RMsg_CAN_RX_ID20.data[1] = tempCANmsg.data[1];
        RMsg_CAN_RX_ID20.data[2] = tempCANmsg.data[2];
        RMsg_CAN_RX_ID20.data[3] = tempCANmsg.data[3];
        RMsg_CAN_RX_ID20.data[4] = tempCANmsg.data[4];
        RMsg_CAN_RX_ID20.data[5] = tempCANmsg.data[5];
        RMsg_CAN_RX_ID20.data[6] = tempCANmsg.data[6];
        RMsg_CAN_RX_ID20.data[7] = tempCANmsg.data[7];
        isReceived_CAN_RX_ID20 = 1;
      } else {
      }

      if (tempCANmsg.id == RMsg_CAN_RX_ID22.id) {
        RMsg_CAN_RX_ID22.data[0] = tempCANmsg.data[0];
        RMsg_CAN_RX_ID22.data[1] = tempCANmsg.data[1];
        RMsg_CAN_RX_ID22.data[2] = tempCANmsg.data[2];
        RMsg_CAN_RX_ID22.data[3] = tempCANmsg.data[3];
        RMsg_CAN_RX_ID22.data[4] = tempCANmsg.data[4];
        RMsg_CAN_RX_ID22.data[5] = tempCANmsg.data[5];
        RMsg_CAN_RX_ID22.data[6] = tempCANmsg.data[6];
        RMsg_CAN_RX_ID22.data[7] = tempCANmsg.data[7];
        isReceived_CAN_RX_ID22 = 1;
      } else {
      }

      /* Keep going */
      isNotFinished = mcp2515_get_message(&tempCANmsg);
    }
  } else {
  }
}
