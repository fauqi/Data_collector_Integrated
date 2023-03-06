#include "stdio.h"
#include <stdbool.h>
#include "stdlib.h"
#include "string.h"
//#include <rs485-bus.h>
//#include <dma_streamer.h>
#include "CANbus.h"
#include "ws2812.h"

extern slot slot1,slot2;
void standby_mode(slot *Slot);
void charging_mode(slot *Slot);
void remote_mode(void);
void theft_mode(void);
void fault_mode(slot *Slot);
void swap_mode(slot *Slot);
void maintenance_mode(void);

