/*
 * ws2812.h
 *
 *  Created on: Jan 24, 2023
 *      Author: fauqi
 */

#ifndef INC_WS2812_H_
#define INC_WS2812_H_

#define MAX_LED 10
#define USE_BRIGHTNESS 1


#include"main.h"
#include <stdbool.h>


enum step{standby=1,charging=2,remote=3,theft=4,fault=5,swap=6,maintenance=7};
enum led{led_standby=1,led_swap=2,led_charging=3,led_fault=4,led_ready_pick=5};
typedef struct {
	  bool stop_charge_command;
	  bool wake_trig;
	  bool isBattery;
	  bool isCharger;
	  bool charge_command;
	  bool charge_state;
	  bool lock_state;
	  bool lock_command;
	  bool unlock_command;
	  bool hardware_failure;
	  bool CAN_error_state;

	  bool current_lock_state;
	  bool last_lock_state;
	  bool trig1_state;
	  bool trig2_state;

	  bool batt_protocol1;
	  bool batt_protocol2;
	  bool batt_discharge_overcurrent;
	  bool batt_charge_overCurrent;
	  bool batt_short_circuit;
	  bool batt_discharge_overtempreature;
	  bool batt_discharge_undertempreature;
	  bool batt_charge_overtempreature;
	  bool batt_charge_undertempreature;
	  bool batt_under_voltage;
	  bool batt_over_voltage;
	  bool batt_over_discharge_capacity;
	  bool batt_unbalance;
	  bool batt_system_failure;
	  bool batt_charge_state;
	  bool batt_discharge_state;
	  bool batt_sleep_state;
	  bool flag_recieve_batt1;
	  bool flag_recieve_batt2;
	  bool can_batt_trouble_state;
	  bool can_batt2_trouble_state;

	  bool charger_start_DIS;
	  bool report_finish;
	  bool recieve_serial_error;
	  bool state_recieve;
	  bool wait_recieve;
	  bool time_out_state;

	  bool charger_hardware_error;
	  bool charger_temp;
	  bool charger_input_voltage;
	  bool charger_work_condition;
	  bool charger_communication_error;
	  bool flag_recieve_charger;
	  bool can_charger_trouble_state;


	  unsigned int current_millis_trig;
	  unsigned int can_tx_tick;
	  float Vbatt;
	  float Abatt;
	  float SOC_batt;
	  float temp_batt;
	  float SOH_batt;
	  int cycle_batt;
	  float Vcharger;
	  float Acharger;
	  unsigned int batt_id;
	  int batt_state;

	  uint8_t RxData[8];
	  uint8_t TxData[8];

	  enum step state;
	  enum step lastState;
//	  DATA LED
	  int datasentflag;
	  uint8_t LED_Data[MAX_LED][4];
	  uint8_t LED_Mod[MAX_LED][4];  // for brightness
	  uint16_t pwmData[(24*MAX_LED)+50];
	  enum led notif;
	  int j;
	  unsigned int tick_led_swap;
	  bool a;
	  unsigned int tick_led_charging;
	  bool flag_charge_led;

	  uint32_t backup[8];

 }slot;

void Set_LED (slot *Slot,int LEDnum, int Red, int Green, int Blue);
void Set_Brightness (slot *Slot,int brightness);
void charging_led(slot *Slot);
void fault_led(slot *Slot);
void swap_led(slot *Slot);
void standby_led(slot *Slot);
void ready_pick_led(slot *Slot);

void WS2812_Send (void);
void WS2812_Send2 (void);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void send_led(void);


#endif /* INC_WS2812_H_ */

