#ifndef CANBUS_MSGS_H_
#define CANBUS_MSGS_H_

#include "stm32f4xx_hal.h"
#include "canbus.h"
#include "car_state.h"

// MACROS
#define GET_MSB(__VAL__) ((__VAL__ >> 8) & 0xFF)
#define GET_LSB(__VAL__) ((__VAL__ & 0xFF))
#define JOIN_2_BYTES(__MSB__, __LSB__) (((__MSB__ << 8) | __LSB__))
#define JOIN_4_BYTES(__MSB__, __MLSB__, __LMSB__, __LSB__) (((__MSB__ << 24) | (__MLSB__ << 16) | (__LMSB__ << 8) | __LSB__))

#define GET_BITS(__VAL__, __BITS__, __POS__) ((__VAL__ >> __POS__) & __BITS__)
#define PLACE_BITS(__VAL__, __BITS__, __POS__) ((__VAL__ & __BITS__) << __POS__)

#define GET_BITS2(__DATA__, __STRUCT__, __BITS__, __POS__) (__DATA__ = ((__STRUCT__ >> __POS__) & __BITS__))

#define BIT_LEN_1 0x01
#define BIT_LEN_2 0x03
#define BIT_LEN_3 0x07
#define BIT_LEN_4 0x0F
#define BIT_LEN_5 0x1F
#define BIT_LEN_6 0x3F
#define BIT_LEN_7 0x7F
#define BIT_LEN_8 0xFF

#define BIT_POS_0 0
#define BIT_POS_1 1
#define BIT_POS_2 2
#define BIT_POS_3 3
#define BIT_POS_4 4
#define BIT_POS_5 5
#define BIT_POS_6 6
#define BIT_POS_7 7
#define BIT_POS_8 8
#define BIT_POS_9 9
#define BIT_POS_10 10
#define BIT_POS_11 11
#define BIT_POS_12 12
#define BIT_POS_13 13
#define BIT_POS_14 14
#define BIT_POS_15 15

/*************************************************************************/
/* 								MSG IDS 								 */
/*************************************************************************/
#define MSGID_SYNC_MSG 0x250

#define MSGID_INV_SETPOINT 0x220
#define MSGID_INV_STATUS 0x225
#define MSGID_INV_MTR_A_SETPOINT 0x230
#define MSGID_INV_MTR_A_STATUS 0x235
#define MSGID_INV_MTR_B_SETPOINT 0x240
#define MSGID_INV_MTR_B_STATUS 0x245

#define MSGID_ECU_STATE 0x300
#define MSGID_ECU_APPS 0x310
#define MSGID_ECU_SAS 0x320

#define MSGID_AMS_STATE 0x400
#define MSGID_AMS_VTIS 0x410

#define MSGID_HVB_STATE 0x500

#define MSGID_PDM_STATUS 0x600
#define MSGID_PDM_VIS 0x610

#define MSGID_UIM_STATE 0x700

#define MSGID_SWB_STATE 0x750

#define MSGID_AMS_MINMAX_VT 0x602 // example ID

/*************************************************************************/
/* 								BASE FUNC 								 */
/*************************************************************************/

void CANBUS_BASE_SEND_MSG(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, uint8_t *data);

/*************************************************************************/
/* 								  ECU 	   								 */
/*************************************************************************/
void CANBUS_SEND_ECU_STATE(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state);
void CANBUS_RECEIVE_ECU_STATE(Car_State_Handle *state, uint8_t dlc, uint8_t *data);
void CANBUS_SEND_ECU_APPS(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state);
void CANBUS_SEND_ECU_SAS(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state);

void CANBUS_SEND_ECU_INV_STATUS(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state);
void CANBUS_SEND_ECU_INV_SETPOINT(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state);
void CANBUS_SEND_ECU_INV_MOTOR_A_STATUS(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state);
void CANBUS_SEND_ECU_INV_MOTOR_A_SETPOINT(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state);
void CANBUS_SEND_ECU_INV_MOTOR_B_STATUS(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state);
void CANBUS_SEND_ECU_INV_MOTOR_B_SETPOINT(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state);

/*************************************************************************/
/* 								  AMS   								 */
/*************************************************************************/
void CANBUS_SEND_AMS_STATE(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state);
void CANBUS_RECEIVE_AMS_STATE(Car_State_Handle *state, uint8_t dlc, uint8_t *data);
void CANBUS_SEND_AMS_VTIS(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state);
void CANBUS_RECEIVE_AMS_VTIS(Car_State_Handle *state, uint8_t dlc, uint8_t *data);
void CANBUS_SEND_AMS_MINMAX_VT(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state);

/*************************************************************************/
/* 								    PDM 								 */
/*************************************************************************/
void CANBUS_SEND_PDM_VIS(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state);
void CANBUS_RECEIVE_PDM_VIS(Car_State_Handle *state, uint8_t dlc, uint8_t *data);
void CANBUS_SEND_PDM_STATE(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state);
void CANBUS_RECEIVE_PDM_STATE(Car_State_Handle *state, uint8_t dlc, uint8_t *data);

/*************************************************************************/
/* 								  HVB 	 								 */
/*************************************************************************/
void CANBUS_SEND_HVB_STATE(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state);
void CANBUS_RECEIVE_HVB_STATE(Car_State_Handle *state, uint8_t dlc, uint8_t *data);

/*************************************************************************/
/* 								  UIM 	 								 */
/*************************************************************************/
void CANBUS_SEND_UIM_STATE(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state);
void CANBUS_RECEIVE_UIM_STATE(Car_State_Handle *state, uint8_t dlc, uint8_t *data);

/*************************************************************************/
/* 								  SWB 	 								 */
/*************************************************************************/
void CANBUS_SEND_SWB_STATE(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state);
void CANBUS_RECEIVE_SWB_STATE(Car_State_Handle *state, uint8_t dlc, uint8_t *data);

#endif /* CANBUS_MSGS_H_ */
