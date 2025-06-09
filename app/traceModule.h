#ifndef _TRACEMODULE_H
#define _TRACEMODULE_H


#define GPIO_PORT_LA GPIOA
#define GPIO_PORT_LB GPIOB
#define GPIO_PORT_RA GPIOB
#define GPIO_PORT_RB GPIOB
#define GPIO_PIN_LA GPIO_PIN_15
#define GPIO_PIN_LB GPIO_PIN_12
#define GPIO_PIN_RA GPIO_PIN_13
#define GPIO_PIN_RB GPIO_PIN_14


void GettraceModule(void);

extern int la, lb ,ra ,rb;
#endif
