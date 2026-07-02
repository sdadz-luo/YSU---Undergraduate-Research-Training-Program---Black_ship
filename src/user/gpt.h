#ifndef GPT_H_
#define GPT_H_

#include <stdint.h>

void gpt0_init(void);
void gpt1_init(void);
extern volatile uint8_t gpt1_flag;
void gpt1_send_4g(void);

#endif