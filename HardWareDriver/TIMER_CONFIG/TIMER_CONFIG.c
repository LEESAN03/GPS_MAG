#include "TIMER_CONFIG.h"
#include "delay.h"

void system_init(void)
{
   RCC_DeInit();
   /* �����ڲ�hsiʱ��Դ */
   RCC->CR |= (uint32_t)0x00000001;
   /* ѡ��hsi��Ϊpll��Դ */
   RCC->CFGR |= (uint32_t)RCC_CFGR_PLLSRC_HSI_Div2;
   /* pll��Ƶ��� 8/2*9 = 36M */
   RCC->CFGR |= (uint32_t)RCC_CFGR_PLLMULL9;
   /* hclk = symclk / 4 */
   RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
   /* ʹ��PLL */
   RCC->CR |= RCC_CR_PLLON;
   while ((RCC->CR & RCC_CR_PLLRDY) == 0)
   {
   }
   /* ѡ��PLL�����Ϊϵͳʱ�� */
   RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
   RCC->CFGR |= (uint32_t)(RCC_CFGR_SW_PLL);
   while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
   {
   };
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		static void Tim3_NVIC_Config(void)
*��������:	    ���ö�ʱ��3 ���ж����ȼ��� 	 
*******************************************************************************/
static void Tim3_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
 	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;   //ѡ��TIM3�ж�
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;   //
 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ��
 	NVIC_Init(&NVIC_InitStructure); 
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void Initial_Timer3(void)
*��������:	  ��ʼ��Tim2  Tim3 ��������ʱ���������Բ���һ��32λ�Ķ�ʱ�����ṩϵͳus ���ļ�ʱ	
�����������
���������û��	
*******************************************************************************/
void Initial_Timer3(void)
{
   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
   Tim3_NVIC_Config();
   /* Time Base configuration �������� ���ö�ʱ����ʱ����Ԫ*/
   TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  		 
  	TIM_TimeBaseStructure.TIM_Period = 20000;
  	TIM_TimeBaseStructure.TIM_Prescaler = 36-1;	 //1M ��ʱ��  
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	//Ӧ�����õ�TIM3 
  	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	// ʹ��TIM3���ؼĴ���ARR
  	TIM_ARRPreloadConfig(TIM3, ENABLE);	 

	//TIM3->DIER |= 0x0001;  //ʹ���ж�	 ʹ�ܶ�ʱ��3�� ����ж�
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
  	//������ʱ��
	TIM_Cmd(TIM3, ENABLE);                 
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void TIM3_IRQHandler(void)
*��������:	    ��ʱ���жϳ��� 
*******************************************************************************/
unsigned char HZ_50_FLAG = 0, HZ_25_FLAG = 0, HZ_10_FLAG = 0;
unsigned int interuptCount = 0, timer_counter = 0;
void TIM3_IRQHandler(void)//��ʱ���жϺ���
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint32_t micros(void)
*��������:	  ��ȡϵͳ���е�ʱ�� �����ص�λΪus ��ʱ������	
�����������
�����������������ǰʱ�䣬���ϵ翪ʼ��ʱ  ��λ us
*******************************************************************************/
uint32_t micros(void)
{
 	uint32_t temp=0;
 	temp = timer_counter * 20000; //����16λʱ��
 	return temp;
}



