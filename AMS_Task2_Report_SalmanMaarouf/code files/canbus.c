/**
  ******************************************************************************
  * @file    canbus.c
  * @author  Tyler Watkins
  * @brief   Timed Canbus module
  *			 This modules goal is to sync a struct across multiple devices,
  *			 allowing for the system to operate as a single device.
  *
  @verbatim
  ==============================================================================
                        ##### How to use this module #####
  ==============================================================================
    [..]
      (#) To use this module a timer interrupt at a tick rate equal to
          CANBUS_TICK_RATE_HIGH_RES (canbus.h) must call CANBUS_TICK()

      (#) Init the module by calling CANBUS_Init(), this configures the module
          as well as the given hcan (bxcan). This function will also configure
          the can modules receive filters.

      (#) To start the module CANBUS_Start() is called, this begins the
          transmission and reception of CANBUS messages.

      (#) CANBUS_Task_Handler() this function needs to be called about every 1ms.
          This is best done by calling this function at the beginning of
          the while loop.

      (#) CANBUS_MSG_Received() needs to be placed inside the canbus RX0 receive
          Interrupt, this will then handle the syncing of nodes.
          One thing to note is that this function only handles syncing,
          no canbus msgs are processed within this function except the sync msg.

      (#) CANBUS_Stop() will stop all canbus actions.

      (#) Implement the callback function CANBUS_WatchDog_Elapsed() to receive a
          notification when a message overruns its watchdog

      (#) The following functions are for internal use within the module.
         (++) CANBUS_Add_Transmit_Func() and CANBUS_Add_Receive_Func() are to be
              used to configure messages sent/received by the device. This should
              take place within CANBUS_Init() as it consumes some time.
 */

#include "canbus.h"

/**
 * @brief  Inits the CANBUS module, loading in msgs to be sent by this node
 * @param  cb pointer to a CANBUS_HandleTypeDef structure.
 * @param  hcan to a CAN_HandleTypeDef structure.
 * @param  CANBUS_Node, the node of the current device
 */
void CANBUS_Init(CANBUS_HandleTypeDef *cb, Car_State_Handle *state, CAN_HandleTypeDef *hcan, CANBUS_Node node)
{
    cb->hcan = hcan;
    cb->carState = state;
    cb->self = node;
    // Apply a catch all filter to the can hardware
    CANBUS_Apply_Catch_All_Filter(hcan, CAN_RX_FIFO0);

    // Add the receive messages to the module
    CANBUS_Add_Receive_Func(cb, &CANBUS_RECEIVE_AMS_STATE, MSGID_AMS_STATE, 10000, (CANBUS_Node[]){NODE_ECU, NODE_UIM},
                            2);
    CANBUS_Add_Receive_Func(cb, &CANBUS_RECEIVE_AMS_VTIS, MSGID_AMS_VTIS, 10000, (CANBUS_Node[]){NODE_ECU}, 1);
    CANBUS_Add_Receive_Func(cb, &CANBUS_RECEIVE_HVB_STATE, MSGID_HVB_STATE, 10000,
                            (CANBUS_Node[]){NODE_ECU, NODE_AMS, NODE_UIM}, 3);
    CANBUS_Add_Receive_Func(cb, &CANBUS_RECEIVE_UIM_STATE, MSGID_UIM_STATE, 10000, (CANBUS_Node[]){NODE_ECU}, 1);
    CANBUS_Add_Receive_Func(cb, &CANBUS_RECEIVE_ECU_STATE, MSGID_ECU_STATE, 10000,
                            (CANBUS_Node[]){NODE_AMS, NODE_UIM, NODE_PDM}, 3);
    CANBUS_Add_Receive_Func(cb, &CANBUS_RECEIVE_PDM_STATE, MSGID_PDM_STATUS, 10000, (CANBUS_Node[]){NODE_ECU}, 1);
    CANBUS_Add_Receive_Func(cb, &CANBUS_RECEIVE_PDM_VIS, MSGID_PDM_VIS, 10000, (CANBUS_Node[]){NODE_ECU}, 1);
    CANBUS_Add_Receive_Func(cb, &CANBUS_RECEIVE_SWB_STATE, MSGID_SWB_STATE, 10000, (CANBUS_Node[]){}, 0);
    // Add all messages to the transmit handler, depending on the node
    switch (node)
    {
    case NODE_NONE:
        break;
    case NODE_ECU:
        cb->isMaster = 1;
        CANBUS_Add_Transmit_Func(cb, &CANBUS_SEND_ECU_INV_SETPOINT, 1, MSG_RATE_50HZ);
        CANBUS_Add_Transmit_Func(cb, &CANBUS_SEND_ECU_INV_MOTOR_A_SETPOINT, 2, MSG_RATE_50HZ);
        CANBUS_Add_Transmit_Func(cb, &CANBUS_SEND_ECU_INV_MOTOR_B_SETPOINT, 3, MSG_RATE_50HZ);
        CANBUS_Add_Transmit_Func(cb, &CANBUS_SEND_ECU_INV_STATUS, 4, MSG_RATE_50HZ);
        CANBUS_Add_Transmit_Func(cb, &CANBUS_SEND_ECU_INV_MOTOR_A_STATUS, 5, MSG_RATE_50HZ);
        CANBUS_Add_Transmit_Func(cb, &CANBUS_SEND_ECU_INV_MOTOR_B_STATUS, 6, MSG_RATE_50HZ);

        CANBUS_Add_Transmit_Func(cb, &CANBUS_SEND_ECU_STATE, 17, MSG_RATE_50HZ);
        CANBUS_Add_Transmit_Func(cb, &CANBUS_SEND_ECU_APPS, 10, MSG_RATE_50HZ);

        CANBUS_Add_Transmit_Func(cb, &CANBUS_SEND_ECU_SAS, 28, MSG_RATE_50HZ);
        break;
    case NODE_AMS:
        CANBUS_Add_Transmit_Func(cb, &CANBUS_SEND_AMS_STATE, 13, MSG_RATE_50HZ);
        CANBUS_Add_Transmit_Func(cb, &CANBUS_SEND_AMS_VTIS, 14, MSG_RATE_50HZ);
        CANBUS_Add_Transmit_Func(cb, &CANBUS_SEND_AMS_MINMAX_VT, 16, MSG_RATE_50HZ);
        break;

    case NODE_HVB:
        CANBUS_Add_Transmit_Func(cb, &CANBUS_SEND_HVB_STATE, 18, MSG_RATE_50HZ);
        break;
    case NODE_PDM:
        CANBUS_Add_Transmit_Func(cb, &CANBUS_SEND_PDM_STATE, 15, MSG_RATE_50HZ);
        CANBUS_Add_Transmit_Func(cb, &CANBUS_SEND_PDM_VIS, 16, MSG_RATE_50HZ);
        break;
    case NODE_UIM:
        CANBUS_Add_Transmit_Func(cb, &CANBUS_SEND_UIM_STATE, 19, MSG_RATE_50HZ);
        break;
    case NODE_CHARGER:
        cb->isMaster = 1;
        CANBUS_Add_Transmit_Func(cb, &CANBUS_SEND_ECU_STATE, 1, MSG_RATE_50HZ);
        break;
    case NODE_SWB:
        CANBUS_Add_Transmit_Func(cb, &CANBUS_SEND_SWB_STATE, 30, MSG_RATE_50HZ);
        break;
    }

    // Fill in the canbus header information
    cb->txHeader.IDE = CAN_ID_STD;
    cb->txHeader.RTR = CAN_RTR_DATA;
    cb->txHeader.TransmitGlobalTime = DISABLE;

    cb->isWatchdogRunning = 0;
}

/**
 * @brief  Starts the timed canbus module.
 * @param  cb pointer to a CANBUS_HandleTypeDef structure.
 */
void CANBUS_Start(CANBUS_HandleTypeDef *cb)
{
    // Only start straight away if the device is the master.
    // All other devices will have isRunning set when the first sync arrives.
    cb->tick = 0;
    cb->waitingForSync = 1;

    if (cb->isMaster)
    {
        cb->isRunning = 1;
        cb->waitingForSync = 0;
    }
    HAL_CAN_Start(cb->hcan);
}

/**
 * @brief  Stops the timed canbus module.
 * @param  cb pointer to a CANBUS_HandleTypeDef structure.
 */
void CANBUS_Stop(CANBUS_HandleTypeDef *cb)
{
    cb->isRunning = 0;
    cb->waitingForSync = 0;
    HAL_CAN_Stop(cb->hcan);
}

/**
 * @brief  Determines if a function is scheduled for the current slot and calls it.
 * @param  cb pointer to a CANBUS_HandleTypeDef structure.
 */
void CANBUS_Tick(CANBUS_HandleTypeDef *cb)
{
    // Check the module is running
    if (!cb->isRunning)
    {
        return;
    }

    (cb->tick)++;

    if (cb->tick == CANBUS_CLOCK_RATE)
    {
        cb->tick = 0;
    }

    // Only allow past this point if the tick is a slot.
    if (cb->tick % (CANBUS_CLOCK_RATE / CANBUS_SLOT_COUNT) != 0)
    {
        return;
    }

    uint16_t slot = cb->tick / (CANBUS_CLOCK_RATE / CANBUS_SLOT_COUNT);

    // If this device is a master, then see if a sync msg should be sent.
    if (cb->isMaster && ((slot % (CANBUS_SLOT_COUNT / CANBUS_MSG_SYNC_RATE)) == 0))
    {
        // Send the sync msg.
        cb->txHeader.StdId = MSGID_SYNC_MSG;
        cb->txHeader.DLC = 2;
        cb->txData[0] = GET_MSB(cb->tick);
        cb->txData[1] = GET_LSB(cb->tick);

        CANBUS_BASE_SEND_MSG(cb->hcan, &(cb->txHeader), cb->txData);
    }
    else
    {
        // Determine the function to run.
        // This will NOT check if there are duplicate functions in slots
        // It will run the first valid slot, function it finds
        for (int i = 0; i < cb->txFunctionCount; i++)
        {
            if (((slot % (CANBUS_SLOT_COUNT / cb->txFunctions[i].rate))) - cb->txFunctions[i].offset == 0)
            {
                // Call the function
                cb->txFunctions[i].func(cb->hcan, &(cb->txHeader), cb->carState);
                break;
            }
        }
    }

    // If the device is waiting to be sync'ed don't worry about watchdogs
    if (cb->waitingForSync)
    {
        return;
    }

    if (cb->isWatchdogRunning)
    {
        // Perform a update on the watchdogs.
        for (int i = 0; i < cb->rxFunctionCount; i++)
        {
            // The watchdog has expired.
            if (cb->rxFunctions[i].watchdog.count == 0)
            {
                for (int d = 0; d < MAX_NODE_COUNT; d++)
                {
                    if (cb->rxFunctions[i].dependantNodes[d] == cb->self)
                    {
                        CANBUS_WatchDog_Elapsed(cb->rxFunctions[i].msgID);
                        cb->rxFunctions[i].watchdog.count = cb->rxFunctions[i].watchdog.reload;
                    }

                    if (cb->rxFunctions[i].dependantNodes[d] == NODE_NONE)
                    {
                        break;
                    }
                }
            }
            else
            {
                cb->rxFunctions[i].watchdog.count--;
            }
        }
    }
}

/**
 * @brief  When called checks if the msg received is a sync msg,
 * 		if the msg is a sync msg, the device syncs.
 * @param  cb pointer to a CANBUS_HandleTypeDef structure.
 */
void CANBUS_MSG_Received(CANBUS_HandleTypeDef *cb)
{
    // Get the message ID of the message without removing it from the fifo
    HAL_CAN_GetRxMessage(cb->hcan, CAN_RX_FIFO0, &(cb->rxHeader), cb->rxData);
    if (cb->rxHeader.StdId == MSGID_SYNC_MSG)
    {
        // Get the sync message
        cb->tick = JOIN_2_BYTES(cb->rxData[0], cb->rxData[1]);
        // If a device is waiting for the first sync msg,
        // Now that one has been received, update its status and start the module.
        if (cb->waitingForSync)
        {
            cb->isRunning = 1;
            cb->waitingForSync = 0;
        }
    }
    for (int i = 0; i < cb->rxFunctionCount; i++)
    {
        // Determine if there is a function to process this msgid
        if (cb->rxHeader.StdId == cb->rxFunctions[i].msgID)
        {
            // Reload the watchdog timer, as the message with the given ID has arrived.
            cb->rxFunctions[i].watchdog.count = cb->rxFunctions[i].watchdog.reload;

            // Store the message
            CANBUS_Can_Msg msg = {cb->rxHeader.StdId, cb->rxHeader.DLC};
            memcpy(msg.data, cb->rxData, cb->rxHeader.DLC);
            cb->msgs.msgBuffer[cb->msgs.writePos++] = msg;
            if (cb->msgs.writePos == CANBUS_BUFFER_LENGTH)
            {
                cb->msgs.writePos = 0;
            }
        }
    }
}

/**
 * @brief  Adds a function pointer to the canbus module.
 * @param  cb pointer to a CANBUS_HandleTypeDef structure.
 * @param  offset determines the initial delay before executing the function
 * @param	rate is the rate at which, in Hz the function is executed, after the offset
 */
void CANBUS_Add_Transmit_Func(CANBUS_HandleTypeDef *cb,
                              void (*func)(CAN_HandleTypeDef *, CAN_TxHeaderTypeDef *, Car_State_Handle *),
                              uint16_t offset, uint16_t rate)
{
    CANBUS_TransmitFuncTypeDef f = {func, offset, rate};
    cb->txFunctions[cb->txFunctionCount++] = f;
}

/**
 * @brief  Adds a function pointer to the canbus module, to handle received messages
 * @param  cb pointer to a CANBUS_HandleTypeDef structure.
 * @param	func pointer to function to be called on receive
 * @param	msgID, the canbus message id to call this function for
 * @param	watchDogTime, the max time allowed between receives of this message
 */
void CANBUS_Add_Receive_Func(CANBUS_HandleTypeDef *cb, void (*func)(Car_State_Handle *, uint8_t, uint8_t *),
                             uint16_t msgID, uint16_t watchDogTime, CANBUS_Node nodes[MAX_NODE_COUNT],
                             uint8_t nodeCount)
{
    CANBUS_ReceiveFuncTypeDef f = {func, msgID};
    f.watchdog.reload = watchDogTime;
    f.watchdog.count = f.watchdog.reload;
    memcpy(f.dependantNodes, nodes, sizeof(CANBUS_Node) * nodeCount);
    cb->rxFunctions[cb->rxFunctionCount++] = f;
}

/**
 * @brief  Processes messages in the can modules fifo
 * @param  CANBUS handle ptr
 * @param	hcan ptr to HAL can module
 * @param	FIFO to read from, either CAN_RX_FIFO0 or CAN_RX_FIFO1
 */
inline void CANBUS_Task_Handler(CANBUS_HandleTypeDef *cb)
{
    // Keep processing messages until a blank is found
    CANBUS_Can_Msg msg;
    while (cb->msgs.msgBuffer[cb->msgs.readPos].ID != 0)
    {
        // Copy the msg out the buffer,
        msg = cb->msgs.msgBuffer[cb->msgs.readPos];
        memset(&cb->msgs.msgBuffer[cb->msgs.readPos], 0, sizeof(CANBUS_Can_Msg));

        cb->msgs.readPos++;
        if (cb->msgs.readPos == CANBUS_BUFFER_LENGTH)
        {
            cb->msgs.readPos = 0;
        }

        // Process the message
        for (int i = 0; i < cb->rxFunctionCount; i++)
        {
            if (cb->rxFunctions[i].msgID == msg.ID)
            {
                cb->rxFunctions[i].func(cb->carState, msg.DLC, msg.data);
                break;
            }
        }
    }
}

/**
 * @brief  Processes messages in the can modules fifo
 * @param  msgid, the canbus id that has exceeded its watchdog time.
 */
__weak void CANBUS_WatchDog_Elapsed(uint16_t msgid)
{
    // Implement this function in order to receive the error
}

/**
 * @brief  Applies a catch all canbus filter to the given can handle
 * @param  can handle ptr
 * @param	FIFO to apply to, either CAN_RX_FIFO0 or CAN_RX_FIFO1
 */
void CANBUS_Apply_Catch_All_Filter(CAN_HandleTypeDef *hcan, uint32_t fifo)
{
    CAN_FilterTypeDef filter;
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x001F; // Only allow 11 bit std ids
    filter.FilterMaskIdLow = 0xFFFF;
    filter.FilterFIFOAssignment = fifo;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(hcan, &filter);
}

/**
 * @brief  Enables the canbus message watchdog
 * @param  CANBUS handle ptr
 */
void CANBUS_Enable_Watchdog(CANBUS_HandleTypeDef *cb) { cb->isWatchdogRunning = 1; }

/**
 * @brief  Disables the canbus message watchdog
 * @param  CANBUS handle ptr
 */
void CANBUS_Disable_Watchdog(CANBUS_HandleTypeDef *cb) { cb->isWatchdogRunning = 0; }
