#ifndef __TIMER_CONFIG_H
#define __TIMER_CONFIG_H
   
extern unsigned char HZ_50_FLAG, HZ_25_FLAG, HZ_10_FLAG;
extern unsigned int interuptCount;

void Initial_Timer3(void);
void system_init(void);
unsigned int micros(void);
#endif



