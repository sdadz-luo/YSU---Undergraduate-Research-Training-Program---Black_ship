#ifndef GPT_H_
#define GPT_H_

#include <stdint.h>

void gpt0_init(void);
void gpt1_init(void);
extern volatile uint8_t gpt1_send;  /* 0=¿ÕÏÐ, 1=·¢À×´ï, 2=·¢GPS */
void send_n10(void);
void send_gps(void);

#endif