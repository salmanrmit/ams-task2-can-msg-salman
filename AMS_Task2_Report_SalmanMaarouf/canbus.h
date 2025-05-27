#ifndef CANBUS_H_
#define CANBUS_H_

#include "stm32f4xx_hal.h"
#include "car_state.h"
#include "canbus_msgs.h"
#include <string.h>
typedef enum
{
	NODE_NONE,
	NODE_ECU,
	NODE_AMS,
	NODE_HVB,
	NODE_PDM,
	NODE_UIM,
	NODE_CHARGER,
	NODE_SWB
}CANBUS_Node;

// The max num of messages possible, update if more messages required
#define CANBUS_MAX_TRANSMIT_FUNC_COUNT 20
#define CANBUS_MAX_RECEIVE_FUNC_COUNT 20

#define CANBUS_MSG_SYNC_RATE 10

#define CANBUS_SLOT_COUNT 2000 // In slots/hz
#define CANBUS_CLOCK_RATE 16000 // In hz, this is the high res clock

// Delay represented as ticks, as clock is at CANBUS_CLOCK_RATE hz, 1 tick = 1/CANBUS_CLOCK_RATE secs
#define CANBUS_SYNC_MSG_DELAY 2

#define CANBUS_BUFFER_LENGTH 10

#define MAX_NODE_COUNT 8


#define MSG_RATE_10HZ 10
#define MSG_RATE_50HZ 50
#define MSG_RATE_100HZ 100
#define MSG_RATE_200HZ 200
#define MSG_RATE_250HZ 250

/**
 * @brief Structure for timed canbus function
 */
typedef struct
{
	// Function to be called
	void (*func)(CAN_HandleTypeDef*, CAN_TxHeaderTypeDef*, Car_State_Handle*);
	// Message offset
	uint16_t offset;
	// Rate in Hz
	uint16_t rate;
}CANBUS_TransmitFuncTypeDef;

typedef struct
{
	uint16_t reload;
	uint16_t count;
}CANBUS_Watchdog;

typedef struct
{
	// Function to be called
	void (*func)(Car_State_Handle*, uint8_t, uint8_t*);
	// Msg id to call when this is received.
	uint16_t msgID;
	CANBUS_Watchdog watchdog;
	CANBUS_Node dependantNodes[MAX_NODE_COUNT];
}CANBUS_ReceiveFuncTypeDef;

typedef struct
{
	uint16_t ID;
	uint8_t DLC;
	uint8_t data[8];
}CANBUS_Can_Msg;

typedef struct
{
	CANBUS_Can_Msg msgBuffer[CANBUS_BUFFER_LENGTH];
	uint8_t readPos;
	uint8_t writePos;
}CANBUS_Ring_Buffer;

typedef struct
{
	CANBUS_TransmitFuncTypeDef txFunctions[CANBUS_MAX_TRANSMIT_FUNC_COUNT];
	uint8_t txFunctionCount;

	CANBUS_ReceiveFuncTypeDef rxFunctions[CANBUS_MAX_RECEIVE_FUNC_COUNT];
	uint8_t rxFunctionCount;

	uint8_t isRunning; // If the module is active.
	uint16_t tick; // The current tick, counts up to CANBUS_CLOCK_RATE
	uint8_t isMaster; // If the devices is the master, one per network
	uint8_t waitingForSync; // If the device is waiting for a sync msg
	uint8_t isWatchdogRunning; // 1 if the watchdogs have been started

	CANBUS_Node self;

	Car_State_Handle *carState;

	CAN_HandleTypeDef *hcan;
	CAN_TxHeaderTypeDef txHeader;
	uint8_t txData[8];
	CAN_RxHeaderTypeDef rxHeader;
	uint8_t rxData[8];

	CANBUS_Ring_Buffer msgs;
}CANBUS_HandleTypeDef;

void CANBUS_Init(CANBUS_HandleTypeDef *cb, Car_State_Handle *state, CAN_HandleTypeDef *hcan, CANBUS_Node node);
void CANBUS_Start(CANBUS_HandleTypeDef *cb);
void CANBUS_Stop(CANBUS_HandleTypeDef *cb);
void CANBUS_Add_Transmit_Func(CANBUS_HandleTypeDef *cb, void (*func)(CAN_HandleTypeDef *, CAN_TxHeaderTypeDef *, Car_State_Handle *), uint16_t offset, uint16_t rate);
void CANBUS_Add_Receive_Func(CANBUS_HandleTypeDef *cb, void(*func)(Car_State_Handle*, uint8_t, uint8_t*), uint16_t msgID, uint16_t watchDogTime, CANBUS_Node nodes[MAX_NODE_COUNT], uint8_t nodeCount);
void CANBUS_Tick(CANBUS_HandleTypeDef *cb);
void CANBUS_Task_Handler(CANBUS_HandleTypeDef *cb);
void CANBUS_Apply_Catch_All_Filter(CAN_HandleTypeDef *hcan, uint32_t fifo);
void CANBUS_WatchDog_Elapsed(uint16_t msgid);
void __CANBUS_SEND_SYNC_MSG(CANBUS_HandleTypeDef *cb);
void CANBUS_MSG_Received(CANBUS_HandleTypeDef *cb);
void CANBUS_Enable_Watchdog(CANBUS_HandleTypeDef *cb);
void CANBUS_Disable_Watchdog(CANBUS_HandleTypeDef *cb);
#endif /* TIMED_CANBUS_H_ */
