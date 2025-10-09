/* Retreive the relevant messages from CAN buffer (if any) */
/* Function implementation */

#ifndef RXCANMSG_H_
#define RXCANMSG_H_

void retreive_RxCAN();
extern tCAN RMsg_CAN_RX_ID21;
extern uint8_t isReceived_CAN_RX_ID21;
extern tCAN RMsg_CAN_RX_ID20;
extern uint8_t isReceived_CAN_RX_ID20;
extern tCAN RMsg_CAN_RX_ID22;
extern uint8_t isReceived_CAN_RX_ID22;

#endif
