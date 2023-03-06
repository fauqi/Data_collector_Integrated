#include "stdio.h"
#include <stdbool.h>
#include "stdlib.h"
#include "string.h"
#include "main.h"
#include "ws2812.h"

extern slot slot1,slot2;

float Vset=40.0;
float Aset=2.0;

extern FDCAN_TxHeaderTypeDef TxHeader;
extern FDCAN_RxHeaderTypeDef RxHeader;
extern uint32_t TxMailBox;
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_FilterTypeDef sFilterConfig;


float Vset2=40.0;
float Aset2=2.0;


extern FDCAN_TxHeaderTypeDef TxHeader2;
extern FDCAN_RxHeaderTypeDef RxHeader2;
extern uint32_t TxMailBox2;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_FilterTypeDef sFilterConfig2;


extern float Vcharger;
extern float Acharger;

extern float Vcharger2;
extern float Acharger2;

union Float_byte {
	float    m_float;
	uint8_t  m_bytes[sizeof(float)];};

union uint16_byte {
	uint16_t m_uint16_t;
	uint8_t  m_bytes[sizeof(float)];};

union uint16_byte 	set_voltage,set_arus;

void CAN_reset()
{
	slot1.TxData[0] = 0;
	slot1.TxData[1] = 0;
	slot1.TxData[2] = 0;
	slot1.TxData[3] = 0;
	slot1.TxData[4] = 0;
	slot1.TxData[5] = 0;
	slot1.TxData[6] = 0;
	slot1.TxData[7] = 0;

	slot2.TxData[0] = 0;
	slot2.TxData[1] = 0;
	slot2.TxData[2] = 0;
	slot2.TxData[3] = 0;
	slot2.TxData[4] = 0;
	slot2.TxData[5] = 0;
	slot2.TxData[6] = 0;
	slot2.TxData[7] = 0;
}

void CAN_TX()
{
	CAN_reset();
	TxHeader.Identifier=0x000001B2;
	TxHeader.IdType = FDCAN_EXTENDED_ID;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	slot1.TxData[0]=slot1.batt_state;

	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, slot1.TxData)!= HAL_OK) {
//	Error_Handler();
	}

	TxHeader.Identifier=0X1806E5F4;
	TxHeader.IdType = FDCAN_EXTENDED_ID;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	set_voltage.m_uint16_t=(Vset*10);
	set_arus.m_uint16_t=(Aset*10);
	slot1.TxData[0] = set_voltage.m_bytes[1];
	slot1.TxData[1] = set_voltage.m_bytes[0];
	slot1.TxData[2] = set_arus.m_bytes[1];
	slot1.TxData[3] = set_arus.m_bytes[0];
	slot1.TxData[4] = slot1.charger_start_DIS;
	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, slot1.TxData)!= HAL_OK) {
//	Error_Handler();
	}

	CAN_reset();
	TxHeader2.Identifier=0x000005B2;
	TxHeader2.IdType = FDCAN_EXTENDED_ID;
	TxHeader2.DataLength = FDCAN_DLC_BYTES_8;
	slot2.TxData[0]=slot2.batt_state;

	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader2, slot2.TxData)!= HAL_OK) {
//	Error_Handler();
	}

	TxHeader2.Identifier=0X1806E5F4;
	TxHeader2.IdType = FDCAN_EXTENDED_ID;
	TxHeader2.DataLength = FDCAN_DLC_BYTES_8;

	set_voltage.m_uint16_t=(Vset*10);
	set_arus.m_uint16_t=(Aset*10);
	slot2.TxData[0] = set_voltage.m_bytes[1];
	slot2.TxData[1] = set_voltage.m_bytes[0];
	slot2.TxData[2] = set_arus.m_bytes[1];
	slot2.TxData[3] = set_arus.m_bytes[0];
	slot2.TxData[4] = slot2.charger_start_DIS;
	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader2, slot2.TxData)!= HAL_OK) {
//	Error_Handler();
	}

//	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBox);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		if(hfdcan==&hfdcan1)
		  {
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, slot1.RxData) == HAL_OK)
			{
				if((RxHeader.Identifier>>20)==0X0B0)//batre
				{
					slot1.batt_protocol1=1;
					slot1.Vbatt=(((slot1.RxData[1]<<8)|slot1.RxData[0])*0.01);
					slot1.Abatt=(((slot1.RxData[3]<<8)|slot1.RxData[2])*0.1);
					slot1.SOC_batt=(((slot1.RxData[5]<<8)|slot1.RxData[4]));
					slot1.temp_batt=(slot1.RxData[6]-40);
					slot1.flag_recieve_batt1=1;
					slot1.can_batt_trouble_state=0;
					slot1.batt_id = (RxHeader.Identifier&0x000FFFFF);
				}
				else if((RxHeader.Identifier>>20)==0X0B1)//batre
				{
					slot1.batt_protocol2=1;

					slot1.SOH_batt=(((slot1.RxData[3]<<8)|slot1.RxData[2])*0.01);
					slot1.cycle_batt=(((slot1.RxData[5]<<8)|slot1.RxData[4]));
					slot1.batt_discharge_overcurrent=(slot1.RxData[6])&(0x01<<0);
					slot1.batt_charge_overCurrent = (slot1.RxData[6])&(0x01<<1);
					slot1.batt_short_circuit=(slot1.RxData[6])&(0x01<<2);
					slot1.batt_discharge_overtempreature=(slot1.RxData[6])&(0x01<<3);
					slot1.batt_discharge_undertempreature=(slot1.RxData[6])&(0x01<<4);
					slot1.batt_charge_overtempreature=(slot1.RxData[6])&(0x01<<5);
					slot1.batt_charge_undertempreature=(slot1.RxData[6])&(0x01<<6);
					slot1.batt_under_voltage=(slot1.RxData[6])&(0x01<<7);

					slot1.batt_over_voltage=(slot1.RxData[7])&(0x01<<0);
					slot1.batt_over_discharge_capacity=(slot1.RxData[7])&(0x01<<1);
					slot1.batt_unbalance=(slot1.RxData[7])&(0x01<<2);
					slot1.batt_system_failure=(slot1.RxData[7])&(0x01<<3);
					slot1.batt_charge_state=(slot1.RxData[7])&(0x01<<4);
					slot1.batt_discharge_state=(slot1.RxData[7])&(0x01<<5);
					slot1.batt_sleep_state=(slot1.RxData[7])&(0x01<<6);

					slot1.flag_recieve_batt2=1;
					slot1.can_batt2_trouble_state=0;
					slot1.batt_id = (RxHeader.Identifier&0x000FFFFF);
				}
				if(RxHeader.Identifier==0x18FF50E5)//charger
				{
					slot1.isCharger=1;
					slot1.Vcharger=(((slot1.RxData[0]<<8)|slot1.RxData[1])*0.1);
					slot1.Acharger=(((slot1.RxData[2]<<8)|slot1.RxData[3])*0.1);
					slot1.charger_hardware_error=(slot1.RxData[4])&(0x01<<0);
					slot1.charger_temp=(slot1.RxData[4])&(0x01<<1);
					slot1.charger_input_voltage=(slot1.RxData[4])&(0x01<<2);
					slot1.charger_work_condition=(slot1.RxData[4])&(0x01<<3);
					slot1.charger_communication_error=(slot1.RxData[4])&(0x01<<4);
					slot1.can_charger_trouble_state=0;
					slot1.flag_recieve_charger=1;
				}
			}
			else if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader2, slot2.RxData) != HAL_OK)Error_Handler();


		  }
		else if (hfdcan==&hfdcan2)
		{
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader2, slot2.RxData) == HAL_OK)
			{
				if((RxHeader2.Identifier>>20)==0X0B0)//batre
				{
				slot2.batt_protocol1=1;

				slot2.Vbatt=(((slot2.RxData[1]<<8)|slot2.RxData[0])*0.01);
				slot2.Abatt=(((slot2.RxData[3]<<8)|slot2.RxData[2])*0.1);
				slot2.SOC_batt=(((slot2.RxData[5]<<8)|slot2.RxData[4]));
				slot2.temp_batt=(slot2.RxData[6]-40);

				slot2.flag_recieve_batt1=1;
				slot2.can_batt_trouble_state=0;
				slot2.batt_id = (RxHeader2.Identifier&0x000FFFFF);
				}
				else if((RxHeader2.Identifier>>20)==0X0B1)//batre
				{
				slot2.batt_protocol2=1;
				slot2.SOH_batt=(((slot2.RxData[3]<<8)|slot2.RxData[2])*0.01);
				slot2.cycle_batt=(((slot2.RxData[5]<<8)|slot2.RxData[4]));
				slot2.batt_discharge_overcurrent=(slot2.RxData[6])&(0x01<<0);
				slot2.batt_charge_overCurrent = (slot2.RxData[6])&(0x01<<1);
				slot2.batt_short_circuit=(slot2.RxData[6])&(0x01<<2);
				slot2.batt_discharge_overtempreature=(slot2.RxData[6])&(0x01<<3);
				slot2.batt_discharge_undertempreature=(slot2.RxData[6])&(0x01<<4);
				slot2.batt_charge_overtempreature=(slot2.RxData[6])&(0x01<<5);
				slot2.batt_charge_undertempreature=(slot2.RxData[6])&(0x01<<6);
				slot2.batt_under_voltage=(slot2.RxData[6])&(0x01<<7);

				slot2.batt_over_voltage=(slot2.RxData[7])&(0x01<<0);
				slot2.batt_over_discharge_capacity=(slot2.RxData[7])&(0x01<<1);
				slot2.batt_unbalance=(slot2.RxData[7])&(0x01<<2);
				slot2.batt_system_failure=(slot2.RxData[7])&(0x01<<3);
				slot2.batt_charge_state=(slot2.RxData[7])&(0x01<<4);
				slot2.batt_discharge_state=(slot2.RxData[7])&(0x01<<5);
				slot2.batt_sleep_state=(slot2.RxData[7])&(0x01<<6);


				slot2.flag_recieve_batt2=1;
				slot2.can_batt2_trouble_state=0;
				slot2.batt_id = (RxHeader.Identifier&0x000FFFFF);
				}
				if(RxHeader2.Identifier==0x18FF50E5)//charger
				{
				slot2.isCharger=1;
				slot2.Vcharger=(((slot2.RxData[0]<<8)|slot2.RxData[1])*0.1);
				slot2.Acharger=(((slot2.RxData[2]<<8)|slot2.RxData[3])*0.1);
				slot2.charger_hardware_error=(slot2.RxData[4])&(0x01<<0);
				slot2.charger_temp=(slot2.RxData[4])&(0x01<<1);
				slot2.charger_input_voltage=(slot2.RxData[4])&(0x01<<2);
				slot2.charger_work_condition=(slot2.RxData[4])&(0x01<<3);
				slot2.charger_communication_error=(slot2.RxData[4])&(0x01<<4);
				slot2.can_charger_trouble_state=0;
				slot2.flag_recieve_charger=1;
				}
			}
			else if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader2, slot2.RxData) != HAL_OK)Error_Handler();

		}

	}
}


void CAN_config()
{
	  sFilterConfig.IdType = FDCAN_EXTENDED_ID;
	  sFilterConfig.FilterIndex = 0;
	  sFilterConfig.FilterType = FDCAN_FILTER_DISABLE;
	  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	  sFilterConfig.FilterID1 = 0;
	  sFilterConfig.FilterID2 = 0;


	  sFilterConfig2.IdType = FDCAN_EXTENDED_ID;
	  sFilterConfig2.FilterIndex = 0;
	  sFilterConfig2.FilterType = FDCAN_FILTER_DISABLE;
	  sFilterConfig2.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	  sFilterConfig2.FilterID1 = 0;
	  sFilterConfig2.FilterID2 = 0;

	  TxHeader.Identifier = 0x01;
	  TxHeader.IdType = FDCAN_EXTENDED_ID;
	  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	  TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
	  TxHeader.MessageMarker = 0x00;

	  TxHeader2.Identifier = 0x02;
	  TxHeader2.IdType = FDCAN_EXTENDED_ID;
	  TxHeader2.DataLength = FDCAN_DLC_BYTES_8;
	  TxHeader2.TxFrameType = FDCAN_DATA_FRAME;
	  TxHeader2.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	  TxHeader2.BitRateSwitch = FDCAN_BRS_OFF;
	  TxHeader2.FDFormat = FDCAN_CLASSIC_CAN;
	  TxHeader2.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
	  TxHeader2.MessageMarker = 0x00;

	  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
	  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	  {
	    Error_Handler();
	  }


	  HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig2);
	  if(HAL_FDCAN_Start(&hfdcan2)!= HAL_OK)
	  {
	   Error_Handler();
	  }
	  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	  {
	    /* Notification Error */
	    Error_Handler();
	  }
    
}

void rutin_cek_can()
{

	  if (slot1.flag_recieve_batt1==0)slot1.can_batt_trouble_state=1;
	  else if(slot1.flag_recieve_batt1==1) slot1.flag_recieve_batt1=0;
	  if (slot1.flag_recieve_batt2==0)slot1.can_batt2_trouble_state=1;
	  else if(slot1.flag_recieve_batt2==1)slot1.flag_recieve_batt2=0;

	  if (slot1.flag_recieve_charger==0)slot1.can_charger_trouble_state=1;
	  else if(slot1.flag_recieve_charger==1)slot1.flag_recieve_charger=0;
	  if(slot1.can_batt_trouble_state==1)slot1.batt_protocol1=0;
	  if(slot1.can_batt2_trouble_state==1)slot1.batt_protocol2=0;

	  if (slot2.flag_recieve_batt1==0)slot2.can_batt_trouble_state=1;
	  else if(slot2.flag_recieve_batt1==1) slot2.flag_recieve_batt1=0;
	  if (slot2.flag_recieve_batt2==0)slot2.can_batt2_trouble_state=1;
	  else if(slot2.flag_recieve_batt2==1)slot2.flag_recieve_batt2=0;

	  if (slot2.flag_recieve_charger==0)slot2.can_charger_trouble_state=1;
	  else if(slot2.flag_recieve_charger==1)slot2.flag_recieve_charger=0;
	  if(slot2.can_batt_trouble_state==1)slot2.batt_protocol1=0;
	  if(slot2.can_batt2_trouble_state==1)slot2.batt_protocol2=0;

}
