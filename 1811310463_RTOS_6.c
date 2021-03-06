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
#include "RTL.h"

void SystemClock_Config(void);
void LED(void);

OS_TID id1,id2,id3;

__task void task1(void);
__task void task2(void);
__task void task3(void);

int main()
{
	
	SystemClock_Config();
	LED();
	os_sys_init(task1);
	while(1);
	
}

__task void task1(void)
{
	id1 = os_tsk_self();
	id2 = os_tsk_create(task2, 0);
	id3 = os_tsk_create(task3, 0);
	
	for(;;){
		LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_6);
		LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_7);
		//os_evt_wait_or(0x0001,0xFFFF); //0xFFFF No itme out
		os_dly_wait(20);
		os_evt_set(0x0004,id3);
		
		}
}
__task void task2(void)
{
	for(;;){
		os_evt_wait_or(0x0002,0xFFFF);
		LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_6);
		LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_7);
		os_dly_wait(60);
	}
}

__task void task3(void)
{
	for(;;){
		os_evt_wait_or(0x0004,0xFFFF);
		LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_7);
		LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_6);
		os_dly_wait(40);
		os_evt_set(0x0002,id2);		
	}
}
void LED(void)
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB); 
	LL_GPIO_InitTypeDef gpio_led; 
	
	gpio_led.Mode = LL_GPIO_MODE_OUTPUT; 
	gpio_led.Pin = LL_GPIO_PIN_7 | LL_GPIO_PIN_6; 
	gpio_led.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH; 
	gpio_led.OutputType = LL_GPIO_OUTPUT_PUSHPULL; 
	gpio_led.Pull = LL_GPIO_PULL_NO; 
	
	LL_GPIO_Init(GPIOB, &gpio_led);
	
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
