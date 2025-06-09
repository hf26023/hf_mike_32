#include "traceModule.h"
#include "main.h"

int la = 0, lb = 0 ,ra = 0 ,rb = 0;

void GettraceModule(void)
{
	if(HAL_GPIO_ReadPin( GPIO_PORT_LA,GPIO_PIN_LA)==GPIO_PIN_SET) la = 1;
	else la = 0;
	if(HAL_GPIO_ReadPin( GPIO_PORT_LB,GPIO_PIN_LB)==GPIO_PIN_SET) lb = 1;
	else lb = 0;
	if(HAL_GPIO_ReadPin( GPIO_PORT_RA,GPIO_PIN_RA)==GPIO_PIN_SET) ra = 1;
	else ra = 0;
	if(HAL_GPIO_ReadPin( GPIO_PORT_RB,GPIO_PIN_RB)==GPIO_PIN_SET) rb = 1;
	else rb = 0;

}


