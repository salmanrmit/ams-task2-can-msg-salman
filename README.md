# RMIT Motorsport R24 AMS - Task 2 Submission

## Overview
This submission covers **Task 2** of the RMIT R24 AMS project, which involves broadcasting minimum and maximum cell voltage and temperature values via CAN from the AMS node.

## Task Summary
- Added a new CAN message `MSGID_AMS_MINMAX_VT` (ID: 0x602)
- Implemented function `CANBUS_SEND_AMS_MINMAX_VT()` in `canbus_msgs.c`
- Registered the transmit function in `CANBUS_Init()` for the `NODE_AMS`
- Data includes:
  - Minimum cell voltage (mV)
  - Maximum cell voltage (mV)
  - Minimum cell temperature (0.1 °C)
  - Maximum cell temperature (0.1 °C)

## How to Build & Run
1. Open the STM32CubeIDE project `R24_AMS`.
2. Ensure the AMS firmware is selected.
3. Build the project.
4. Flash to target board using ST-Link.

## Notes
- Timer 3 generates 1ms ticks, which drive CANBUS and TICK logic.
- CANBUS is initialized for the AMS node via `CANBUS_Init()`.

## Author
Salman Maarouf
