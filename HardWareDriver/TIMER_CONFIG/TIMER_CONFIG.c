#include "TIMER_CONFIG.h"
#include "delay.h"

void system_init(void)
{
   RCC_DeInit();
   /* 启动内部hsi时钟源 */
   RCC->CR |= (uint32_t)0x00000001;
   /* 选择hsi作为pll的源 */
   RCC->CFGR |= (uint32_t)RCC_CFGR_PLLSRC_HSI_Div2;
   /* pll倍频输出 8/2*9 = 36M */
   RCC->CFGR |= (uint32_t)RCC_CFGR_PLLMULL9;
   /* hclk = symclk / 4 */
   RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
   /* 使能PLL */
   RCC->CR |= RCC_CR_PLLON;
   while ((RCC->CR & RCC_CR_PLLRDY) == 0)
   {
   }
   /* 选择PLL输出作为系统时钟 */
   RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
   RCC->CFGR |= (uint32_t)(RCC_CFGR_SW_PLL);
   while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
   {
   };
}

/**************************实现函数********************************************
*函数原型:		static void Tim3_NVIC_Config(void)
*功　　能:	    配置定时器3 的中断优先级别 	 
*******************************************************************************/
static void Tim3_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
 	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;   //选择TIM3中断
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;   //
 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能
 	NVIC_Init(&NVIC_InitStructure); 
}

/**************************实现函数********************************************
*函数原型:		void Initial_Timer3(void)
*功　　能:	  初始化Tim2  Tim3 将两个定时器级联，以产生一个32位的定时器来提供系统us 级的计时	
输入参数：无
输出参数：没有	
*******************************************************************************/
void Initial_Timer3(void)
{
   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
   Tim3_NVIC_Config();
   /* Time Base configuration 基本配置 配置定时器的时基单元*/
   TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  		 
  	TIM_TimeBaseStructure.TIM_Period = 20000;
  	TIM_TimeBaseStructure.TIM_Prescaler = 36-1;	 //1M 的时钟  
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	//应用配置到TIM3 
  	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	// 使能TIM3重载寄存器ARR
  	TIM_ARRPreloadConfig(TIM3, ENABLE);	 

	//TIM3->DIER |= 0x0001;  //使能中断	 使能定时器3的 溢出中断
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
  	//启动定时器
	TIM_Cmd(TIM3, ENABLE);                 
}

/**************************实现函数********************************************
*函数原型:		void TIM3_IRQHandler(void)
*功　　能:	    定时器中断程序 
*******************************************************************************/
unsigned char HZ_50_FLAG = 0, HZ_25_FLAG = 0, HZ_10_FLAG = 0;
unsigned int interuptCount = 0, timer_counter = 0;
void TIM3_IRQHandler(void)//定时器中断函数
{
   if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) 
   {
      interuptCount++;
      timer_counter++;
      HZ_50_FLAG = 1;
      if(interuptCount % 2 == 0)
      {
         HZ_25_FLAG = 1;
      }
      if(interuptCount % 5 == 0)
      {
         HZ_10_FLAG = 1;
         interuptCount = 0;
      }
      TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
   }
}

/**************************实现函数********************************************
*函数原型:		uint32_t micros(void)
*功　　能:	  读取系统运行的时间 ，返回单位为us 的时间数。	
输入参数：无
输出参数：处理器当前时间，从上电开始计时  单位 us
*******************************************************************************/
uint32_t micros(void)
{
 	uint32_t temp=0;
 	temp = timer_counter * 20000; //读高16位时间
 	return temp;
}



