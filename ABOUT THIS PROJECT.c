ABOUT THIS PROJECT

1. This project was created in STM32CubeMX for STM32F334R8
   Requirement: Two PWMs with 'htimx.Init.Period' and 'sConfigOC.Pulse' need to be adjusted by 4 variable resistors

2. TIM2-Chan2 & TIM16-Chan1 are currenly started by HAL driver and have fixed Frequency & DutyCycle.
	
	static void MX_TIM2_Init(void)
	{
	...
	htim2.Init.Period = 71;
	sConfigOC.Pulse = 35;
	
	...

	htim16.Init.Period = 71;
	sConfigOC.Pulse = 35;
	...
	} 	

3. The SETUP BARE TIMER in int main(void) is not working. Please fix this problem.
	{
	...	
	TIM2->ARR = 71 ;
	TIM2->CCMR2 = 35 ;
	...
	TIM16->ARR = 71 ;
	TIM16->CCMR1 = 35 ;
	...
	}

4. There are 4 variable resistors - RV1, RV2, RV3, RV4 - which create voltage divider networks.
   The outputs from 4 variable resistors will go into ADC pin PA0, PA1, PA4, PB4 (See schematic) 
   DMA must be used to save CPU power.

5. The data from ADCs will be mapped into a range (1-250)
	
	{
	...
	Data_ADC_PA0 = HAL_ADC_GetValue(&hadc1);
	mapped_ADC_PA0 = map(data_ADC_PA0, 0, 4095, 1, 250);
	
	...

	Data_ADC_PA1 = HAL_ADC_GetValue(&hadc1);
	mapped_ADC_PA1 = map(data_ADC_PA1, 0, 4095, 1, 250);
	...
	}

    //MAP function - fully working
	long map(long x, long in_min, long in_max, long out_min, long out_max)
  	  {
	  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  	  }

6.  Feed the 'mapped_ADC_xxx' into 'TIM2/TIM16->ARR' and 'TIM2/TIM16->CCMR' to change PWM Frequency and dutyCycle (WRONG!)
	mapped_ADC_PA0 = TIM16->ARR
	mapped_ADC_PA1 = TIM16->CCMR1 
	mapped_ADC_PA4 = TIM2->ARR
	mapped_ADC_PB0 = TIM2->CCMR2

7. Limit 'CCMR' to always 1-unit smaller than 'ARR'