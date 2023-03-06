#include "stdio.h"
#include <stdbool.h>
#include "stdlib.h"
#include "string.h"

#include "CANbus.h"
#include "ws2812.h"


extern enum led notif;
extern slot slot1,slot2;
extern enum step state;
extern enum step lastState;

extern enum step state2;
extern enum step lastState2;
void standby_mode(slot *Slot)
{
	if(Slot->can_charger_trouble_state)
	{
		Slot->lastState=Slot->state;
		Slot->state=fault;
	}
	if(Slot->unlock_command==1)
	{
		Slot->lock_state=0;
		Slot->current_lock_state=0;
	}

	if(Slot->isBattery==1&&Slot->CAN_error_state==0)
	{
		Slot->lastState=Slot->state;
		Slot->state=swap;
		Slot->wake_trig=0;
	}
	else
	{
		Slot->wake_trig=1;

	}
	if(Slot->hardware_failure)Slot->state=fault;
}

void swap_mode(slot *Slot)
{
	if(Slot->can_batt_trouble_state==1||Slot->can_batt2_trouble_state==1)
	Slot->state=standby;
	if(Slot->lock_command)
	{
		Slot->lock_state=1;
		Slot->current_lock_state=1;
		Slot->lock_command=0;
	}
	if(Slot->unlock_command)
	{
		Slot->lock_state=0;
		Slot->current_lock_state=0;
		Slot->unlock_command=0;
	}
	if(Slot->lock_state==1&&Slot->charge_command==1)
	{
		Slot->lastState=Slot->state;
		Slot->state=charging;
	}
	if(Slot->hardware_failure)
	{
		Slot->lastState=Slot->state;
		Slot->state=fault;
	}
	if(Slot->can_charger_trouble_state==1)
	{
		Slot->lastState=Slot->state;
		Slot->state=fault;
	}

}

void charging_mode(slot *Slot)
{
	if (Slot->isBattery)
	{
		if(Slot->SOC_batt>=100||Slot->stop_charge_command==1)
		{
			Slot->stop_charge_command=0;
			Slot->batt_state=0x01;
			Slot->charger_start_DIS=1;
			Slot->charge_state=0;
			Slot->notif=led_standby;
		}
		if(Slot->unlock_command)
		{
			if(Slot->charge_state==0)
			{
				Slot->notif=led_ready_pick;
				if(Slot->can_batt_trouble_state==1||Slot->can_batt2_trouble_state==1)
				{
					Slot->unlock_command=0;
					Slot->lock_state=0;
					Slot->current_lock_state=0;
					Slot->lastState=Slot->state;
					Slot->state=standby;

				}

			}
		}
		if(Slot->charge_command)
		{
			//start batre and charge to charge
			Slot->batt_state=0x05;
			Slot->charger_start_DIS=0;
			Slot->charge_state=1;
			Slot->charge_command=0;
		}

	}
	else
	{
		Slot->unlock_command=0;
		Slot->lock_state=0;
		Slot->current_lock_state=0;
		Slot->lastState=Slot->state;
		Slot->state=standby;
	}
	if(Slot->hardware_failure)
	{
		Slot->lastState=Slot->state;
		Slot->state=fault;
	}
	if(Slot->unlock_command==0 && (Slot->can_batt_trouble_state==1||Slot->can_batt2_trouble_state==1||Slot->can_charger_trouble_state==1) )
	{
		Slot->lastState=Slot->state;
		Slot->state=fault;
	}
}

void remote_mode()
{

}

void theft_mode()
{

}

void fault_mode(slot *Slot)
{
	if(Slot->hardware_failure)
	{
		Slot->batt_state=0x01;
		Slot->charger_start_DIS=1;
	}
	else if(Slot->can_batt_trouble_state==1||Slot->can_batt2_trouble_state==1||Slot->can_charger_trouble_state==1)
	{
		if(Slot->lastState==standby)
		{
			if(Slot->can_charger_trouble_state==1)
				Slot->CAN_error_state=1;
			else Slot->state=standby;
		}
		// can error handler

		else if(Slot->lastState==swap)
		{
			if(Slot->can_charger_trouble_state==1)
				Slot->CAN_error_state=1;
		}
		else if(Slot->lastState==charging)
		{
			if(Slot->can_batt_trouble_state==1||Slot->can_batt2_trouble_state==1)
			Slot->state=theft;
			else if(Slot->can_batt_trouble_state==1||Slot->can_batt2_trouble_state==1||Slot->can_charger_trouble_state==1)
			{
				Slot->CAN_error_state=1;
			}
		}

	}
	else Slot->state=Slot->lastState;
}

void maintenance_mode()
{

}

