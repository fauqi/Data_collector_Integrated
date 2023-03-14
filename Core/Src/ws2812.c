/*
 * ws2812.c
 *
 *  Created on: Jan 24, 2023
 *      Author: fauqi
 */
#include"main.h"
#include"math.h"
#include "ws2812.h"
#include <stdbool.h>
extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_tim1_ch3;
extern DMA_HandleTypeDef hdma_tim1_ch2;
extern slot slot1,slot2;

void Set_LED (slot *Slot,int LEDnum, int Red, int Green, int Blue)
{
	Slot->LED_Data[LEDnum][0] = LEDnum;
	Slot->LED_Data[LEDnum][1] = Green;
	Slot->LED_Data[LEDnum][2] = Red;
	Slot->LED_Data[LEDnum][3] = Blue;
}
#define PI 3.14159265
void Set_Brightness (slot *Slot,int brightness)  // 0-45
{
#if USE_BRIGHTNESS

	if (brightness > 45) brightness = 45;
	for (int i=0; i<MAX_LED; i++)
	{
		Slot->LED_Mod[i][0] = Slot->LED_Data[i][0];
		for (int j=1; j<4; j++)
		{
			float angle = 90-brightness;  // in degrees
			angle = angle*PI / 180;  // in rad
			Slot->LED_Mod[i][j] = (Slot->LED_Data[i][j])/(tan(angle));
		}
	}

#endif

}
void WS2812_Send (void)
{
	uint32_t indx=0;
	uint32_t color;


	for (int i= 0; i<MAX_LED; i++)
	{
	  if (USE_BRIGHTNESS)
	  {
			color = ((slot1.LED_Mod[i][1]<<16) | (slot1.LED_Mod[i][2]<<8) | (slot1.LED_Mod[i][3]));
	  }
	  else
	  {
			color = ((slot1.LED_Data[i][1]<<16) | (slot1.LED_Data[i][2]<<8) | (slot1.LED_Data[i][3]));
	  }


		for (int i=23; i>=0; i--)
		{
			if (color&(1<<i))
			{
				slot1.pwmData[indx] = 51;  // 2/3 of 79
			}

			else slot1.pwmData[indx] = 29;  // 1/3 of 79

			indx++;
		}

	}

	for (int i=0; i<50; i++)
	{
		slot1.pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t *)slot1.pwmData, indx);
	while (!slot1.datasentflag){};
	slot2.datasentflag = 0;
}
void send_led()
{
	if(slot1.datasentflag==0&&slot2.datasentflag==1){WS2812_Send();}
	else if((slot2.datasentflag==0&&slot1.datasentflag==1)){WS2812_Send2();}

}
void WS2812_Send2 (void)
{
	uint32_t indx=0;
	uint32_t color;


	for (int i= 0; i<MAX_LED; i++)
	{
	  if (USE_BRIGHTNESS)
	  {
			color = ((slot2.LED_Mod[i][1]<<16) | (slot2.LED_Mod[i][2]<<8) | (slot2.LED_Mod[i][3]));
	  }
	  else
	  {
			color = ((slot2.LED_Data[i][1]<<16) | (slot2.LED_Data[i][2]<<8) | (slot2.LED_Data[i][3]));
	  }


		for (int i=23; i>=0; i--)
		{
			if (color&(1<<i))
			{
				slot2.pwmData[indx] = 51;  // 2/3 of 79
			}

			else slot2.pwmData[indx] = 29;  // 1/3 of 79

			indx++;
		}

	}

	for (int i=0; i<50; i++)
	{
		slot2.pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t *)slot2.pwmData, indx);

	while (!slot2.datasentflag){};
	slot1.datasentflag = 0;
}
void charging_led(slot *Slot)
{
	int i=Slot->SOC_batt/10;
	for (int j=0;j<=MAX_LED;j++)
	{

		if(j<i)Set_LED(Slot,j,255,0,0);
		else if(j==i)asm ("nop");
		else Set_LED(Slot,j,0,0,0);
	}
	if(HAL_GetTick()-Slot->tick_led_charging>=1000)
	{
		if(Slot->flag_charge_led==1)
		{
			Set_LED(Slot,i,255,0,0);
			Slot->flag_charge_led=0;
		}
		else
		{
			Set_LED(Slot,i,0,0,0);
			Slot->flag_charge_led=1;
		}
		Slot->tick_led_charging=HAL_GetTick();
	}
	Set_Brightness(Slot,46);
}
void ready_pick_led(slot *Slot)
{
	  for(int i=0;i<MAX_LED;i++)
	  {
		  Set_LED(Slot,i, 255, 255, 0);
	  }
	  if(HAL_GetTick()-Slot->tick_led_swap>=1000)
	  {
		  Slot->j++;
		if(Slot->j%2==0)Set_Brightness(Slot,35);
		else Set_Brightness(Slot,0);
		Slot->tick_led_swap=HAL_GetTick();
	  }

}
void swap_led(slot *Slot)
{
	  for(int i=0;i<=MAX_LED;i++)
	  {
		  Set_LED(Slot,i, 255, 255, 255);
	  }

	  if(HAL_GetTick()-Slot->tick_led_swap>=50)
	  {
		if(Slot->j>35)Slot->a=1;
		if(Slot->j<3)Slot->a=0;
		if(Slot->a==1)Slot->j--;
		else Slot->j++;
		Slot->tick_led_swap=HAL_GetTick();
	  }
		  Set_Brightness(Slot,Slot->j);
		  WS2812_Send();



}
void fault_led(slot *Slot)
{
	  for(int i=0;i<=MAX_LED;i++)
	  {

		  Set_LED(Slot,i, 255, 0, 0);

	  }

		Set_Brightness(Slot,46);
}
void standby_led(slot *Slot)
{
	  for(int i=0;i<=MAX_LED;i++)
	  {
		  Set_LED(Slot,i, 0, 0, 0);
	  }

		Set_Brightness(Slot,46);
}
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_2);
		slot1.datasentflag=1;
	}
	else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	{
		HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_3);
		slot2.datasentflag=1;
	}


}

