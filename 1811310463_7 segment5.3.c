/*Base register adddress header file*/
#include "stm32l1xx.h"
/*Library related header files*/
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_tim.h"

// PA1:Rising , PA2:Trigger , PA3:Falling 
/* Channel 1 TIM4 */
/*for 10ms update event*/
#define TIMx_PSC			32000 
#define TIMx_ARR			10

void SystemClock_Config(void);
uint32_t checkSeg(uint32_t);
void GPIO_Config(void)
{
		LL_GPIO_InitTypeDef timic_gpio;
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
		//GPIO_Config PA1 as alternate
		timic_gpio.Mode = LL_GPIO_MODE_ALTERNATE;
		timic_gpio.Pull = LL_GPIO_PULL_NO;
		timic_gpio.Pin = LL_GPIO_PIN_1;
		timic_gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		timic_gpio.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
		timic_gpio.Alternate = LL_GPIO_AF_1;
		LL_GPIO_Init(GPIOA, &timic_gpio);
		//GPIO_Config PA3 as alternate
		timic_gpio.Pin = LL_GPIO_PIN_3;
		LL_GPIO_Init(GPIOA, &timic_gpio);
		//GPIO_Config PA2 as output
		timic_gpio.Mode = LL_GPIO_MODE_OUTPUT;
		timic_gpio.Pull = LL_GPIO_PULL_NO;
		timic_gpio.Pin = LL_GPIO_PIN_2;
		timic_gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		timic_gpio.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
		LL_GPIO_Init(GPIOA, &timic_gpio);
		//GPIO_Config 7-segment
		LL_GPIO_InitTypeDef ltc4727;
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	
		ltc4727.Mode = LL_GPIO_MODE_OUTPUT;
		ltc4727.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		ltc4727.Pull = LL_GPIO_PULL_NO;
		ltc4727.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
		ltc4727.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | 
								LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | 
								LL_GPIO_PIN_13 | LL_GPIO_PIN_14 |
								LL_GPIO_PIN_15 ;
		LL_GPIO_Init(GPIOB, &ltc4727);
	
		ltc4727.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 |
								LL_GPIO_PIN_2 | LL_GPIO_PIN_3 ;
		LL_GPIO_Init(GPIOC, &ltc4727);
}
	
void TIMx_IC_Config(void)
{
		LL_TIM_IC_InitTypeDef timic;
	
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
		
		//TIM_IC Configure CH2
		timic.ICActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI;
		timic.ICFilter = LL_TIM_IC_FILTER_FDIV1_N2;
		timic.ICPolarity = LL_TIM_IC_POLARITY_RISING;
		timic.ICPrescaler = LL_TIM_ICPSC_DIV1;
		LL_TIM_IC_Init(TIM2, LL_TIM_CHANNEL_CH2, &timic);
		
		NVIC_SetPriority(TIM2_IRQn, 0);
		
		NVIC_EnableIRQ(TIM2_IRQn);
		LL_TIM_EnableIT_CC2(TIM2);
		LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH2);
		
		//TIM_IC Configure CH4
		timic.ICActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI;
		timic.ICFilter = LL_TIM_IC_FILTER_FDIV1_N2;
		timic.ICPolarity = LL_TIM_IC_POLARITY_FALLING;
		timic.ICPrescaler = LL_TIM_ICPSC_DIV1;
		LL_TIM_IC_Init(TIM2, LL_TIM_CHANNEL_CH4, &timic);
		
		NVIC_SetPriority(TIM2_IRQn, 0);
		
		NVIC_EnableIRQ(TIM2_IRQn);
		LL_TIM_EnableIT_CC4(TIM2);
		LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH4);
		
		
		LL_TIM_EnableCounter(TIM2);
}
void TIM_BASE_Config(void)
{
	LL_TIM_InitTypeDef timbase_initstructure;
	
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
	
	timbase_initstructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	timbase_initstructure.CounterMode = LL_TIM_COUNTERMODE_UP;
	timbase_initstructure.Autoreload = TIMx_ARR - 1;
	timbase_initstructure.Prescaler =  TIMx_PSC- 1;
	
	LL_TIM_Init(TIM4, &timbase_initstructure);

}

void TIM_OC_GPIO_Config(void)
{
	LL_GPIO_InitTypeDef gpio_initstructure;
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	
	gpio_initstructure.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio_initstructure.Alternate = LL_GPIO_AF_2;
	gpio_initstructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio_initstructure.Pin = LL_GPIO_PIN_6;
	gpio_initstructure.Pull = LL_GPIO_PULL_NO;
	gpio_initstructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOB, &gpio_initstructure);
}

void TIM_OC_Config(void)
{
	LL_TIM_OC_InitTypeDef tim_oc_initstructure;
	
	TIM_BASE_Config();
	TIM_OC_GPIO_Config();
	
	tim_oc_initstructure.OCState = LL_TIM_OCSTATE_DISABLE;
	tim_oc_initstructure.OCMode = LL_TIM_OCMODE_PWM1;
	tim_oc_initstructure.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
//	tim_oc_initstructure.CompareValue = LL_TIM_GetAutoReload(TIM4); //100% duty
//	tim_oc_initstructure.CompareValue = LL_TIM_GetAutoReload(TIM4) / 2; //50% duty
	tim_oc_initstructure.CompareValue = LL_TIM_GetAutoReload(TIM4) / 4; //25% duty
	LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &tim_oc_initstructure);
	/*Interrupt Configure*/
	NVIC_SetPriority(TIM4_IRQn, 1);
	NVIC_EnableIRQ(TIM4_IRQn);
	LL_TIM_EnableIT_CC1(TIM4);
	
	/*Start Output Compare in PWM Mode*/
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableCounter(TIM4);
}
uint16_t Val1 = 0;
uint16_t Val2 = 0;
uint16_t Echo_pulse = 0;
float period = 0;
float distance = 0;
uint8_t distance_cm = 0;
uint8_t state = 0;

uint32_t TIM2CLK;
uint32_t PSC;
uint32_t IC4PSC;

uint32_t seg[4]={0,0,10,11};
uint32_t digit[4]={LL_GPIO_PIN_0,LL_GPIO_PIN_1,LL_GPIO_PIN_2,LL_GPIO_PIN_3};

int main()
{
	
		SystemClock_Config();
		TIMx_IC_Config();
		GPIO_Config();
		uint8_t i;
	
	//send trigger
		LL_GPIO_SetOutputPin(GPIOA,LL_GPIO_PIN_2);
		LL_mDelay(1);
		LL_GPIO_SetOutputPin(GPIOA,LL_GPIO_PIN_2);
		while(1)
		{
			if(state == 2)
			{
				PSC = LL_TIM_GetPrescaler(TIM2) + 1;
				TIM2CLK = SystemCoreClock / PSC;
				IC4PSC = __LL_TIM_GET_ICPSC_RATIO(LL_TIM_IC_GetPrescaler(TIM2, LL_TIM_CHANNEL_CH4));
				period = (Echo_pulse*(PSC) * 1.0) / (TIM2CLK *IC4PSC * 1.0); //calculate uptime period (s) as PA3
				distance = period*170.0;//calculate distance (m)
				distance_cm = distance*100;
				seg[0] = distance_cm/10;
				seg[1] = distance_cm%10;
				state = 0;
			}
			//Display 7-segment
			
			for(i=0;i<4;i++)
			{
				LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_3);
				LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_2|LL_GPIO_PIN_10|LL_GPIO_PIN_11|LL_GPIO_PIN_12|
													 LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_15);	
				LL_GPIO_SetOutputPin(GPIOB, checkSeg(seg[i]));
				LL_GPIO_SetOutputPin(GPIOC, digit[i]);
				
				//LL_mDelay(1000);
			}
		}
}
uint32_t checkSeg(uint32_t num)
{
	switch(num)
	{
		case 0: 
			return LL_GPIO_PIN_2|LL_GPIO_PIN_10|LL_GPIO_PIN_11|
						 LL_GPIO_PIN_12|LL_GPIO_PIN_13|LL_GPIO_PIN_14;
			break;
		case 1:
			return LL_GPIO_PIN_10|LL_GPIO_PIN_11;
			break;
		case 2:
			return LL_GPIO_PIN_2|LL_GPIO_PIN_10|LL_GPIO_PIN_12|
						 LL_GPIO_PIN_13|LL_GPIO_PIN_15;
			break;
		case 3:
			return LL_GPIO_PIN_2|LL_GPIO_PIN_10|LL_GPIO_PIN_11|
						 LL_GPIO_PIN_12|LL_GPIO_PIN_15;
			break;
		case 4: 
			return LL_GPIO_PIN_10|LL_GPIO_PIN_11|LL_GPIO_PIN_14|
						 LL_GPIO_PIN_15;
			break;
		case 5: 
			return LL_GPIO_PIN_2|LL_GPIO_PIN_11|LL_GPIO_PIN_12|
						 LL_GPIO_PIN_14|LL_GPIO_PIN_15;
			break;
		case 6:
			return LL_GPIO_PIN_2|LL_GPIO_PIN_11|LL_GPIO_PIN_12|
				     LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_15;
			break;
		case 7:
			return LL_GPIO_PIN_2|LL_GPIO_PIN_10|LL_GPIO_PIN_11;
			break;
		case 8:
			return LL_GPIO_PIN_2|LL_GPIO_PIN_10|LL_GPIO_PIN_11|
						 LL_GPIO_PIN_12|LL_GPIO_PIN_13|LL_GPIO_PIN_14|
						 LL_GPIO_PIN_15;
			break;
		case 9:
			return LL_GPIO_PIN_2|LL_GPIO_PIN_10|LL_GPIO_PIN_11|
						 LL_GPIO_PIN_12|LL_GPIO_PIN_14|LL_GPIO_PIN_15;
			break;
		case 10:
			return LL_GPIO_PIN_12|LL_GPIO_PIN_13|LL_GPIO_PIN_15;
			break;
		case 11:
			return LL_GPIO_PIN_2|LL_GPIO_PIN_10|LL_GPIO_PIN_11|
						 LL_GPIO_PIN_13|LL_GPIO_PIN_14;
			break;
		default:
			return 0;
	}
}
void TIM2_IRQHandler(void)
{	  
	if(state == 0)
	{
		if(LL_TIM_IsActiveFlag_CC2(TIM2) == SET)
		{
			LL_TIM_ClearFlag_CC2(TIM2);
			Val1 = LL_TIM_IC_GetCaptureCH2(TIM2);
				state = 1;
		}
	}		
	else if(state == 1)
	{
		if(LL_TIM_IsActiveFlag_CC4(TIM2) == SET)
		{
			LL_TIM_ClearFlag_CC4(TIM2);
			Val2 = LL_TIM_IC_GetCaptureCH4(TIM2);
			if(Val2 > Val1)
						Echo_pulse = Val2 - Val1;
			else if(Val2 < Val1)
						Echo_pulse = ((LL_TIM_GetAutoReload(TIM2) - Val1) + Val2) + 1;
			state = 2;
		}
	}	
	/*if(LL_TIM_IsActiveFlag_CC1(TIM2) == SET)
		{
			LL_TIM_ClearFlag_CC1(TIM2);
				//Detect 1st rising edge
			if(uhICIndex == 0)
			{
				uwIC1 = LL_TIM_IC_GetCaptureCH1(TIM2);
				uhICIndex = 1;
			}
			//Detect 2nd rising edge
			else if(uhICIndex == 1)
			{
					uwIC2 = LL_TIM_IC_GetCaptureCH1(TIM2);
					
					if(uwIC2 > uwIC1)
						uwDiff = uwIC2 - uwIC1;
					else if(uwIC2 < uwIC1)
						uwDiff = ((LL_TIM_GetAutoReload(TIM2) - uwIC1) + uwIC2) + 1;
					uhICIndex = 2;

			}*/

}
void TIM4_IRQHandler(void)
{
	if(LL_TIM_IsActiveFlag_CC1(TIM4) == SET)
	{
		LL_TIM_ClearFlag_CC1(TIM4);
	}
}
void SystemClock_Config(void)
{
  /* Enable ACC64 access and set FLASH latency */ 
  LL_FLASH_Enable64bitAccess();; 
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  /* Set Voltage scale1 as MCU will run at 32MHz */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (LL_PWR_IsActiveFlag_VOSF() != 0)
  {
  };
  
  /* Enable HSI if not already activated*/
  if (LL_RCC_HSI_IsReady() == 0)
  {
    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady() != 1)
    {
    };
  }
  
	
  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3);

  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };
  
  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };
  
  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms in using frequency set to 32MHz                             */
  /* This frequency can be calculated through LL RCC macro                          */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ (HSI_VALUE, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3); */
  LL_Init1msTick(32000000);
  
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(32000000);
}