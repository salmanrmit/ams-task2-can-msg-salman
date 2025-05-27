#include "canbus_msgs.h"
/*************************************************************************/
/* 								BASE FUNC 								 */
/*************************************************************************/

void CANBUS_BASE_SEND_MSG(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, uint8_t *data)
{
	uint32_t txMail;
	if (HAL_CAN_AddTxMessage(hcan, txHeader, data, &txMail) != HAL_OK)
	{
		// TODO Add Error handling for failed msg
	}
	// Clear out the transmit headers
	txHeader->StdId = 0;
	txHeader->DLC = 0;
	// memset(cb->txData, 0, 8);
}

/*************************************************************************/
/* 								   ECU 									 */
/*************************************************************************/
void CANBUS_SEND_ECU_STATE(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state)
{
	txHeader->StdId = MSGID_ECU_STATE;
	txHeader->DLC = 8;

	uint8_t data[8] = {0};

	data[0] |= PLACE_BITS(state->ECU.fault_flags, BIT_LEN_8, BIT_POS_0);

	data[1] |= PLACE_BITS(state->ECU.State, BIT_LEN_5, BIT_POS_0);
	data[1] |= PLACE_BITS(state->ECU.Board_ID, BIT_LEN_3, BIT_POS_5);

	data[3] |= PLACE_BITS(state->ECU.SDC_Present, BIT_LEN_1, BIT_POS_0);
	data[3] |= PLACE_BITS(state->ECU.TSMS_On, BIT_LEN_1, BIT_POS_1);
	data[3] |= PLACE_BITS(state->ECU.Node_Missing, BIT_LEN_1, BIT_POS_2);
	data[3] |= PLACE_BITS(state->ECU.Brake_Threshold_Meet, BIT_LEN_1, BIT_POS_3);
	data[3] |= PLACE_BITS(state->ECU.SDC_EN, BIT_LEN_1, BIT_POS_4);
	data[3] |= PLACE_BITS(state->ECU.INV_EN, BIT_LEN_1, BIT_POS_5);
	data[3] |= PLACE_BITS(state->ECU.APPS_Implausable, BIT_LEN_1, BIT_POS_6);
	data[3] |= PLACE_BITS(state->ECU.APPS_BSE_Implasuable, BIT_LEN_1, BIT_POS_7);

	data[2] |= PLACE_BITS(state->ECU.Power_Limit, BIT_LEN_1, BIT_POS_0);

	data[6] |= PLACE_BITS(state->ECU.sensor_status.APPS_1_Ok, BIT_LEN_1, BIT_POS_0);
	data[6] |= PLACE_BITS(state->ECU.sensor_status.APPS_2_Ok, BIT_LEN_1, BIT_POS_1);
	data[6] |= PLACE_BITS(state->ECU.sensor_status.BSE_FRONT_Ok, BIT_LEN_1, BIT_POS_2);
	data[6] |= PLACE_BITS(state->ECU.sensor_status.BSE_REAR_Ok, BIT_LEN_1, BIT_POS_3);

	data[7] = state->ECU.drs_state;

	CANBUS_BASE_SEND_MSG(hcan, txHeader, data);
}

void CANBUS_RECEIVE_ECU_STATE(Car_State_Handle *state, uint8_t dlc, uint8_t *data)
{
	state->ECU.fault_flags = PLACE_BITS(data[0], BIT_LEN_8, BIT_POS_0);

	state->ECU.State = GET_BITS(data[1], BIT_LEN_5, BIT_POS_0);
	state->ECU.Board_ID = GET_BITS(data[1], BIT_LEN_3, BIT_POS_5);

	state->ECU.SDC_Present = GET_BITS(data[3], BIT_LEN_1, BIT_POS_0);
	state->ECU.TSMS_On = GET_BITS(data[3], BIT_LEN_1, BIT_POS_1);
	state->ECU.Node_Missing = GET_BITS(data[3], BIT_LEN_1, BIT_POS_2);
	state->ECU.Brake_Threshold_Meet = GET_BITS(data[3], BIT_LEN_1, BIT_POS_3);
	state->ECU.SDC_EN = GET_BITS(data[3], BIT_LEN_1, BIT_POS_4);
	state->ECU.INV_EN = GET_BITS(data[3], BIT_LEN_1, BIT_POS_5);
	state->ECU.APPS_Implausable = GET_BITS(data[3], BIT_LEN_1, BIT_POS_6);
	state->ECU.APPS_BSE_Implasuable = GET_BITS(data[3], BIT_LEN_1, BIT_POS_7);

	state->ECU.Power_Limit = GET_BITS(data[2], BIT_LEN_1, BIT_POS_0);

	state->ECU.sensor_status.APPS_1_Ok = GET_BITS(data[6], BIT_LEN_1, BIT_POS_0);
	state->ECU.sensor_status.APPS_2_Ok = GET_BITS(data[6], BIT_LEN_1, BIT_POS_1);
	state->ECU.sensor_status.BSE_FRONT_Ok = GET_BITS(data[6], BIT_LEN_1, BIT_POS_2);
	state->ECU.sensor_status.BSE_REAR_Ok = GET_BITS(data[6], BIT_LEN_1, BIT_POS_3);

	state->ECU.drs_state = data[7];
}

void CANBUS_SEND_ECU_APPS(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state)
{
	txHeader->StdId = MSGID_ECU_APPS;
	txHeader->DLC = 8;

	uint8_t data[8] = {0};
	data[0] = GET_MSB(state->ECU.sensor_values.APPS1_Percent);
	data[1] = GET_LSB(state->ECU.sensor_values.APPS1_Percent);

	data[2] = GET_MSB(state->ECU.sensor_values.APPS2_Percent);
	data[3] = GET_LSB(state->ECU.sensor_values.APPS2_Percent);

	data[4] = GET_MSB(state->ECU.sensor_values.BSE_FRONT_Pressure);
	data[5] = GET_LSB(state->ECU.sensor_values.BSE_FRONT_Pressure);

	data[6] = GET_MSB(state->ECU.sensor_values.BSE_REAR_Pressure);
	data[7] = GET_LSB(state->ECU.sensor_values.BSE_REAR_Pressure);
	CANBUS_BASE_SEND_MSG(hcan, txHeader, data);
}

void CANBUS_SEND_ECU_SAS(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state)
{
	txHeader->StdId = MSGID_ECU_SAS;
	txHeader->DLC = 6;

	uint8_t data[6] = {0};
	data[0] = GET_MSB(state->ECU.sensor_values.SAS);
	data[1] = GET_LSB(state->ECU.sensor_values.SAS);

	data[2] = GET_MSB(state->ECU.sensor_values.gear_tempR);
	data[3] = GET_LSB(state->ECU.sensor_values.gear_tempR);

	data[4] = GET_MSB(state->ECU.sensor_values.gear_tempL);
	data[5] = GET_LSB(state->ECU.sensor_values.gear_tempL);

	CANBUS_BASE_SEND_MSG(hcan, txHeader, data);
}

void CANBUS_SEND_ECU_INV_STATUS(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state)
{
	txHeader->StdId = MSGID_INV_STATUS;
	txHeader->DLC = 8;

	CANBUS_BASE_SEND_MSG(hcan, txHeader, state->INV.inv_status);
}

void CANBUS_SEND_ECU_INV_SETPOINT(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state)
{
	txHeader->StdId = MSGID_INV_SETPOINT;
	txHeader->DLC = 8;

	CANBUS_BASE_SEND_MSG(hcan, txHeader, state->INV.inv_setpoint);
}

void CANBUS_SEND_ECU_INV_MOTOR_A_STATUS(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state)
{
	txHeader->StdId = MSGID_INV_MTR_A_STATUS;
	txHeader->DLC = 8;

	CANBUS_BASE_SEND_MSG(hcan, txHeader, state->INV.motor_a_status);
}

void CANBUS_SEND_ECU_INV_MOTOR_A_SETPOINT(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state)
{
	txHeader->StdId = MSGID_INV_MTR_A_SETPOINT;
	txHeader->DLC = 8;

	CANBUS_BASE_SEND_MSG(hcan, txHeader, state->INV.motor_a_setpoint);
}

void CANBUS_SEND_ECU_INV_MOTOR_B_STATUS(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state)
{
	txHeader->StdId = MSGID_INV_MTR_B_STATUS;
	txHeader->DLC = 8;

	CANBUS_BASE_SEND_MSG(hcan, txHeader, state->INV.motor_b_status);
}

void CANBUS_SEND_ECU_INV_MOTOR_B_SETPOINT(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state)
{
	txHeader->StdId = MSGID_INV_MTR_B_SETPOINT;
	txHeader->DLC = 8;

	CANBUS_BASE_SEND_MSG(hcan, txHeader, state->INV.motor_b_setpoint);
}

/*************************************************************************/
/* 									AMS 								 */
/*************************************************************************/
void CANBUS_SEND_AMS_STATE(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state)
{
	txHeader->StdId = MSGID_AMS_STATE;
	txHeader->DLC = 8;

	uint8_t data[8] = {0};

	data[0] |= PLACE_BITS(state->AMS.sdc_status.IMD_ok, BIT_LEN_1, BIT_POS_0);
	data[0] |= PLACE_BITS(state->AMS.sdc_status.AMS_ok, BIT_LEN_1, BIT_POS_1);
	data[0] |= PLACE_BITS(state->AMS.sdc_status.PDOC_ok, BIT_LEN_1, BIT_POS_2);
	data[0] |= PLACE_BITS(state->AMS.sdc_status.HV_IL_sen, BIT_LEN_1, BIT_POS_3);

	data[1] = state->AMS.board_id;

	data[2] = GET_MSB(state->AMS.ams_trip_flags);
	data[3] = GET_LSB(state->AMS.ams_trip_flags);

	data[4] |= PLACE_BITS(state->AMS.air_status.air_io.air2_closed, BIT_LEN_1, BIT_POS_0);
	data[4] |= PLACE_BITS(state->AMS.air_status.air_io.prech_en, BIT_LEN_1, BIT_POS_1);
	data[4] |= PLACE_BITS(state->AMS.air_status.air_io.prech_closed, BIT_LEN_1, BIT_POS_2);

	data[5] |= PLACE_BITS(state->AMS.air_status.air_state, BIT_LEN_4, BIT_POS_0);
	data[5] |= PLACE_BITS(state->AMS.air_status.air_io.sdc_ok, BIT_LEN_1, BIT_POS_4);
	data[5] |= PLACE_BITS(state->AMS.air_status.air_io.air1_en, BIT_LEN_1, BIT_POS_5);
	data[5] |= PLACE_BITS(state->AMS.air_status.air_io.air1_closed, BIT_LEN_1, BIT_POS_6);
	data[5] |= PLACE_BITS(state->AMS.air_status.air_io.air2_en, BIT_LEN_1, BIT_POS_7);

	data[6] = GET_MSB(state->AMS.air_status.air_fault);
	data[7] = GET_LSB(state->AMS.air_status.air_fault);

	CANBUS_BASE_SEND_MSG(hcan, txHeader, data);
}

void CANBUS_RECEIVE_AMS_STATE(Car_State_Handle *state, uint8_t dlc, uint8_t *data)
{
	state->AMS.sdc_status.IMD_ok = GET_BITS(data[0], BIT_LEN_1, BIT_POS_0);
	state->AMS.sdc_status.AMS_ok = GET_BITS(data[0], BIT_LEN_1, BIT_POS_1);
	state->AMS.sdc_status.PDOC_ok = GET_BITS(data[0], BIT_LEN_1, BIT_POS_2);
	state->AMS.sdc_status.HV_IL_sen = GET_BITS(data[0], BIT_LEN_1, BIT_POS_3);

	state->AMS.board_id = data[1];

	state->AMS.ams_trip_flags = JOIN_2_BYTES(data[2], data[3]);

	state->AMS.air_status.air_io.air2_closed = GET_BITS(data[4], BIT_LEN_1, BIT_POS_0);
	state->AMS.air_status.air_io.prech_en = GET_BITS(data[4], BIT_LEN_1, BIT_POS_1);
	state->AMS.air_status.air_io.prech_closed = GET_BITS(data[4], BIT_LEN_1, BIT_POS_2);

	state->AMS.air_status.air_state = GET_BITS(data[5], BIT_LEN_4, BIT_POS_0);
	state->AMS.air_status.air_io.sdc_ok = GET_BITS(data[5], BIT_LEN_1, BIT_POS_4);
	state->AMS.air_status.air_io.air1_en = GET_BITS(data[5], BIT_LEN_1, BIT_POS_5);
	state->AMS.air_status.air_io.air1_closed = GET_BITS(data[5], BIT_LEN_1, BIT_POS_6);
	state->AMS.air_status.air_io.air2_en = GET_BITS(data[5], BIT_LEN_1, BIT_POS_7);

	state->AMS.air_status.air_fault = JOIN_2_BYTES(data[6], data[7]);
}

void CANBUS_SEND_AMS_VTIS(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state)
{
	txHeader->StdId = MSGID_AMS_VTIS;
	txHeader->DLC = 8;

	uint8_t data[8] = {0};
	data[0] = GET_MSB(state->AMS.VTIS.voltage);
	data[1] = GET_LSB(state->AMS.VTIS.voltage);

	data[2] = GET_MSB(state->AMS.VTIS.temperature);
	data[3] = GET_LSB(state->AMS.VTIS.temperature);

	data[4] = GET_MSB(state->AMS.VTIS.current);
	data[5] = GET_LSB(state->AMS.VTIS.current);

	data[6] = GET_MSB(state->AMS.VTIS.soc);
	data[7] = GET_LSB(state->AMS.VTIS.soc);

	CANBUS_BASE_SEND_MSG(hcan, txHeader, data);
}

void CANBUS_RECEIVE_AMS_VTIS(Car_State_Handle *state, uint8_t dlc, uint8_t *data)
{
	state->AMS.VTIS.voltage = JOIN_2_BYTES(data[0], data[1]);
	state->AMS.VTIS.temperature = JOIN_2_BYTES(data[2], data[3]);
	state->AMS.VTIS.current = JOIN_2_BYTES(data[4], data[5]);
	state->AMS.VTIS.soc = JOIN_2_BYTES(data[6], data[7]);
}

void CANBUS_SEND_AMS_MINMAX_VT(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *TxHeader, Car_State_Handle *state)
{
	txHeader->StdId = MSGID_AMS_MINMAX_VT;
	txHeader->dlc = 8;

	bms_meas_t measurments = bms_get_measurements();

	float min_voltage = measurements.cell_voltage[0];
	float max_voltage = measurements.cell_voltage[0];

	for (int i = 1; i < NUM_CELLS; i++)
	{
		if (measurements.cell_voltage[i] < min_voltage)
			min_voltage = measurements.cell_voltage[i];
		if (measurements.cell_voltage[i] > max_voltage)
			max_voltage = measurements.cell_voltage[i];
	}

	float min_temp = measurements.cell_temperature[0];
	float max_temp = measurements.cell_temperature[0];
	for (int i = 1; i < NUM_TEMPS; i++)
	{
		if (measurements.cell_temperature[i] < min_temp)
			min_temp = measurements.cell_temperature[i];
		if (measurements.cell_temperature[i] > max_temp)
			max_temp = measurements.cell_temperature[i];
	}

	unit16_t min_voltage_mV = (unit16_t)(min_voltage * 1000.0f);
	unit16_t max_voltage_mV = (unit16_t)(max_voltage * 1000.0f);
	unit16_t min_temp_dC = (unit16_t)(min_temp * 10.0f);
	unit16_t max_temp_dC = (unit16_t)(max_temp * 10.0f);

	unit8_t data[8];
	data[0] = GET_MSB(min_voltage_mV);
	data[1] = GET_LSB(min_voltage_mV);
	data[2] = GET_MSB(max_voltage_mV);
	data[3] = GET_LSB(max_voltage_mV);
	data[4] = GET_MSB(min_temp_dC);
	data[5] = GET_LSB(min_temp_dC);
	data[6] = GET_MSB(max_temp_dC);
	data[7] = GET_LSB(max_temp_dC);

	CANBUS_BASE_SEND_MSG(hcan, txHeader, data);
}

/*************************************************************************/
/* 									HVB 								 */
/*************************************************************************/
void CANBUS_SEND_HVB_STATE(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state)
{
	txHeader->StdId = MSGID_HVB_STATE;
	txHeader->DLC = 4;

	uint8_t data[4] = {0};
	data[1] |= PLACE_BITS(state->HVB.State, BIT_LEN_3, BIT_POS_0);
	data[1] |= PLACE_BITS(state->HVB.Board_ID, BIT_LEN_3, BIT_POS_3);
	data[1] |= PLACE_BITS(state->HVB.HVD_IL_Present, BIT_LEN_1, BIT_POS_6);
	data[1] |= PLACE_BITS(state->HVB.BSPD_Tripped, BIT_LEN_1, BIT_POS_7);

	data[2] = GET_MSB(state->HVB.TS_Volts);
	data[3] = GET_LSB(state->HVB.TS_Volts);

	CANBUS_BASE_SEND_MSG(hcan, txHeader, data);
}

void CANBUS_RECEIVE_HVB_STATE(Car_State_Handle *state, uint8_t dlc, uint8_t *data)
{
	state->HVB.State = GET_BITS(data[1], BIT_LEN_3, BIT_POS_0);
	state->HVB.Board_ID = GET_BITS(data[1], BIT_LEN_3, BIT_POS_3);
	state->HVB.HVD_IL_Present = GET_BITS(data[1], BIT_LEN_1, BIT_POS_6);
	state->HVB.BSPD_Tripped = GET_BITS(data[1], BIT_LEN_1, BIT_POS_7);

	state->HVB.TS_Volts = JOIN_2_BYTES(data[2], data[3]);
}

/*************************************************************************/
/* 								    PDM 								 */
/*************************************************************************/
void CANBUS_SEND_PDM_VIS(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state)
{
	txHeader->StdId = MSGID_PDM_VIS;
	txHeader->DLC = 6;

	uint8_t data[6] = {0};
	data[0] = GET_MSB(state->PDM.LV_Volts);
	data[1] = GET_LSB(state->PDM.LV_Volts);

	data[2] = GET_MSB(state->PDM.LV_Current);
	data[3] = GET_LSB(state->PDM.LV_Current);

	data[4] = GET_MSB(state->PDM.LV_SOC);
	data[5] = GET_LSB(state->PDM.LV_SOC);

	CANBUS_BASE_SEND_MSG(hcan, txHeader, data);
}

void CANBUS_RECEIVE_PDM_VIS(Car_State_Handle *state, uint8_t dlc, uint8_t *data)
{
	state->PDM.LV_Volts = JOIN_2_BYTES(data[0], data[1]);
	state->PDM.LV_Current = JOIN_2_BYTES(data[2], data[3]);
	state->PDM.LV_SOC = JOIN_2_BYTES(data[4], data[5]);
}

void CANBUS_SEND_PDM_STATE(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state)
{
	txHeader->StdId = MSGID_PDM_STATUS;
	txHeader->DLC = 6;

	uint8_t data[6] = {0};
	data[1] |= PLACE_BITS(state->PDM.State, BIT_LEN_3, BIT_POS_0);
	data[1] |= PLACE_BITS(state->PDM.Board_ID, BIT_LEN_3, BIT_POS_3);
	data[1] |= PLACE_BITS(state->PDM.Battery_ID, BIT_LEN_2, BIT_POS_5);

	data[2] |= PLACE_BITS(state->PDM.channels[15], BIT_LEN_2, BIT_POS_0);
	data[2] |= PLACE_BITS(state->PDM.channels[14], BIT_LEN_2, BIT_POS_2);
	data[2] |= PLACE_BITS(state->PDM.channels[13], BIT_LEN_2, BIT_POS_4);
	data[2] |= PLACE_BITS(state->PDM.channels[12], BIT_LEN_2, BIT_POS_6);

	data[3] |= PLACE_BITS(state->PDM.channels[11], BIT_LEN_2, BIT_POS_0);
	data[3] |= PLACE_BITS(state->PDM.channels[10], BIT_LEN_2, BIT_POS_2);
	data[3] |= PLACE_BITS(state->PDM.channels[9], BIT_LEN_2, BIT_POS_4);
	data[3] |= PLACE_BITS(state->PDM.channels[8], BIT_LEN_2, BIT_POS_6);

	data[4] |= PLACE_BITS(state->PDM.channels[7], BIT_LEN_2, BIT_POS_0);
	data[4] |= PLACE_BITS(state->PDM.channels[6], BIT_LEN_2, BIT_POS_2);
	data[4] |= PLACE_BITS(state->PDM.channels[5], BIT_LEN_2, BIT_POS_4);
	data[4] |= PLACE_BITS(state->PDM.channels[4], BIT_LEN_2, BIT_POS_6);

	data[5] |= PLACE_BITS(state->PDM.channels[3], BIT_LEN_2, BIT_POS_0);
	data[5] |= PLACE_BITS(state->PDM.channels[2], BIT_LEN_2, BIT_POS_2);
	data[5] |= PLACE_BITS(state->PDM.channels[1], BIT_LEN_2, BIT_POS_4);
	data[5] |= PLACE_BITS(state->PDM.channels[0], BIT_LEN_2, BIT_POS_6);

	CANBUS_BASE_SEND_MSG(hcan, txHeader, data);
}

void CANBUS_RECEIVE_PDM_STATE(Car_State_Handle *state, uint8_t dlc, uint8_t *data)
{
	state->PDM.State = GET_BITS(data[1], BIT_LEN_3, BIT_POS_0);
	state->PDM.Board_ID = GET_BITS(data[1], BIT_LEN_3, BIT_POS_3);
	state->PDM.Battery_ID = GET_BITS(data[1], BIT_LEN_2, BIT_POS_5);

	state->PDM.channels[15] = GET_BITS(data[2], BIT_LEN_2, BIT_POS_0);
	state->PDM.channels[14] = GET_BITS(data[2], BIT_LEN_2, BIT_POS_2);
	state->PDM.channels[13] = GET_BITS(data[2], BIT_LEN_2, BIT_POS_4);
	state->PDM.channels[12] = GET_BITS(data[2], BIT_LEN_2, BIT_POS_6);

	state->PDM.channels[11] = GET_BITS(data[3], BIT_LEN_2, BIT_POS_0);
	state->PDM.channels[10] = GET_BITS(data[3], BIT_LEN_2, BIT_POS_2);
	state->PDM.channels[9] = GET_BITS(data[3], BIT_LEN_2, BIT_POS_4);
	state->PDM.channels[8] = GET_BITS(data[3], BIT_LEN_2, BIT_POS_6);

	state->PDM.channels[7] = GET_BITS(data[4], BIT_LEN_2, BIT_POS_0);
	state->PDM.channels[6] = GET_BITS(data[4], BIT_LEN_2, BIT_POS_2);
	state->PDM.channels[5] = GET_BITS(data[4], BIT_LEN_2, BIT_POS_4);
	state->PDM.channels[4] = GET_BITS(data[4], BIT_LEN_2, BIT_POS_6);

	state->PDM.channels[3] = GET_BITS(data[5], BIT_LEN_2, BIT_POS_0);
	state->PDM.channels[2] = GET_BITS(data[5], BIT_LEN_2, BIT_POS_2);
	state->PDM.channels[1] = GET_BITS(data[5], BIT_LEN_2, BIT_POS_4);
	state->PDM.channels[0] = GET_BITS(data[5], BIT_LEN_2, BIT_POS_6);
}

/*************************************************************************/
/* 								    UIM 								 */
/*************************************************************************/
void CANBUS_SEND_UIM_STATE(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state)
{
	txHeader->StdId = MSGID_UIM_STATE;
	txHeader->DLC = 4;

	uint8_t data[4] = {0};
	data[1] |= PLACE_BITS(state->UIM.State, BIT_LEN_3, BIT_POS_0);
	data[1] |= PLACE_BITS(state->UIM.Board_ID, BIT_LEN_2, BIT_POS_3);
	data[1] |= PLACE_BITS(state->UIM.TS_EN_Btn_Pressed, BIT_LEN_1, BIT_POS_5);
	data[1] |= PLACE_BITS(state->UIM.Drive_Btn_Pressed, BIT_LEN_1, BIT_POS_6);
	data[1] |= PLACE_BITS(state->UIM.DRS_Btn_Pressed, BIT_LEN_1, BIT_POS_7);

	data[3] |= PLACE_BITS(state->UIM.SDC.CS_SENSE, BIT_LEN_1, BIT_POS_0);
	data[3] |= PLACE_BITS(state->UIM.SDC.CS_AUX, BIT_LEN_1, BIT_POS_1);
	data[3] |= PLACE_BITS(state->UIM.SDC.RSDB_SENSE, BIT_LEN_1, BIT_POS_2);
	data[3] |= PLACE_BITS(state->UIM.SDC.RSDB_AUX, BIT_LEN_1, BIT_POS_3);
	data[3] |= PLACE_BITS(state->UIM.SDC.CSDB_SENSE, BIT_LEN_1, BIT_POS_4);
	data[3] |= PLACE_BITS(state->UIM.SDC.CSDB_AUX, BIT_LEN_1, BIT_POS_5);
	data[3] |= PLACE_BITS(state->UIM.SDC.BOT_SENSE, BIT_LEN_1, BIT_POS_6);
	data[3] |= PLACE_BITS(state->UIM.SDC.BOT_AUX, BIT_LEN_1, BIT_POS_7);

	data[2] |= PLACE_BITS(state->UIM.SDC.LSDB_SENSE, BIT_LEN_1, BIT_POS_0);
	data[2] |= PLACE_BITS(state->UIM.SDC.LSDB_AUX, BIT_LEN_1, BIT_POS_1);

	CANBUS_BASE_SEND_MSG(hcan, txHeader, data);
}

void CANBUS_RECEIVE_UIM_STATE(Car_State_Handle *state, uint8_t dlc, uint8_t *data)
{
	state->UIM.State = GET_BITS(data[1], BIT_LEN_3, BIT_POS_0);
	state->UIM.Board_ID = GET_BITS(data[1], BIT_LEN_2, BIT_POS_3);
	state->UIM.TS_EN_Btn_Pressed = GET_BITS(data[1], BIT_LEN_1, BIT_POS_5);
	state->UIM.Drive_Btn_Pressed = GET_BITS(data[1], BIT_LEN_1, BIT_POS_6);
	state->UIM.DRS_Btn_Pressed = GET_BITS(data[1], BIT_LEN_1, BIT_POS_7);

	state->UIM.SDC.CS_SENSE = GET_BITS(data[3], BIT_LEN_1, BIT_POS_0);
	state->UIM.SDC.CS_AUX = GET_BITS(data[3], BIT_LEN_1, BIT_POS_1);
	state->UIM.SDC.RSDB_SENSE = GET_BITS(data[3], BIT_LEN_1, BIT_POS_2);
	state->UIM.SDC.RSDB_AUX = GET_BITS(data[3], BIT_LEN_1, BIT_POS_3);
	state->UIM.SDC.CSDB_SENSE = GET_BITS(data[3], BIT_LEN_1, BIT_POS_4);
	state->UIM.SDC.CSDB_AUX = GET_BITS(data[3], BIT_LEN_1, BIT_POS_5);
	state->UIM.SDC.BOT_SENSE = GET_BITS(data[3], BIT_LEN_1, BIT_POS_6);
	state->UIM.SDC.BOT_AUX = GET_BITS(data[3], BIT_LEN_1, BIT_POS_7);

	state->UIM.SDC.LSDB_SENSE = GET_BITS(data[2], BIT_LEN_1, BIT_POS_0);
	state->UIM.SDC.LSDB_AUX = GET_BITS(data[2], BIT_LEN_1, BIT_POS_1);
}

/*************************************************************************/
/* 								    SWB 								 */
/*************************************************************************/
void CANBUS_SEND_SWB_STATE(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, Car_State_Handle *state)
{
	txHeader->StdId = MSGID_SWB_STATE;
	txHeader->DLC = 2;

	uint8_t data[2] = {0};
	data[0] |= PLACE_BITS(state->SWB.DRS_Btn_Pressed, BIT_LEN_1, BIT_POS_0);
	data[0] |= PLACE_BITS(state->SWB.Launch_Btn_Pressed, BIT_LEN_1, BIT_POS_1);
	data[0] |= PLACE_BITS(state->SWB.Mark_Btn_Pressed, BIT_LEN_1, BIT_POS_2);
	data[0] |= PLACE_BITS(state->SWB.Cycle_Btn_Pressed, BIT_LEN_1, BIT_POS_3);

	data[1] = state->SWB.Rot_Pos;

	CANBUS_BASE_SEND_MSG(hcan, txHeader, data);
}

void CANBUS_RECEIVE_SWB_STATE(Car_State_Handle *state, uint8_t dlc, uint8_t *data)
{
	state->SWB.DRS_Btn_Pressed = GET_BITS(data[0], BIT_LEN_1, BIT_POS_0);
	state->SWB.Launch_Btn_Pressed = GET_BITS(data[0], BIT_LEN_1, BIT_POS_1);
	state->SWB.Mark_Btn_Pressed = GET_BITS(data[0], BIT_LEN_1, BIT_POS_2);
	state->SWB.Cycle_Btn_Pressed = GET_BITS(data[0], BIT_LEN_1, BIT_POS_3);

	state->SWB.Rot_Pos = data[1];
}
