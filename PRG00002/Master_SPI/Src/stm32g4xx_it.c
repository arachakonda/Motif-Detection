/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define EMG_O_TH 3000
#define EMG_C_TH 3000
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc3;
extern DMA_HandleTypeDef hdma_adc4;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;
extern COMP_HandleTypeDef hcomp4;
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_i2c2_tx;
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

extern uint32_t msTicks;
extern uint8_t thumb_sensor_detected;
extern uint16_t R,G,B;
extern TCS34725IntegrationTime_t IntegrationTime_t;


extern uint16_t TCS34725_ReadWord(uint8_t add);

extern uint8_t debounceFlag;

extern KalArm_MODE_TYPE KalArm_MODE;
extern KalArm_TIME_TYPE KalArm_MFSW;
extern KalArm_BLE_TYPE KalArm_BLE;
extern KalArm_GRIP_TYPE KalArm_GRIP;
extern KalArm_BATTERY_TYPE KalArm_BATTERY;

extern KalArm_EMG_TYPE KalArm_EMG_OPEN, KalArm_EMG_CLOSE;

extern KalArm_SPI_RECEIVE_TYPE KalArm_SPI_RECEIVE_FLAGS;

extern KalArm_CHG_DET_TYPE KalArm_CHG_DET;

extern uint16_t integration_state_number;
extern uint8_t update_event_open;
extern uint8_t update_event_close;

extern uint8_t uart_send;
extern uint8_t txData_UART[19];

extern uint16_t msTicks_T_7;

extern uint32_t ADC_raw[2];

extern KalArm_GEN_GRIP_STAT_RES_TYPE KalArm_GEN_GRIP_STAT_RES;
extern KalArm_CUST_GRIP_STAT_RES_TYPE KalArm_CUST_GRIP_STAT_RES;
extern KalArm_PATTERN_RECOGNITION_SUITE_PARAMS_TYPE KalArm_PATTERN_RECOGNITION_SUITE_PARAMS;

extern uint8_t cust_grip_execute_id;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c2_rx);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c2_tx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc4);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupt.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles I2C2 event interrupt / I2C2 wake-up interrupt through EXTI line 24.
  */
void I2C2_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_EV_IRQn 0 */

  /* USER CODE END I2C2_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c2);
  /* USER CODE BEGIN I2C2_EV_IRQn 1 */

  /* USER CODE END I2C2_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C2 error interrupt.
  */
void I2C2_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_ER_IRQn 0 */

  /* USER CODE END I2C2_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c2);
  /* USER CODE BEGIN I2C2_ER_IRQn 1 */

  /* USER CODE END I2C2_ER_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt / USART3 wake-up interrupt through EXTI line 28.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles ADC3 global interrupt.
  */
void ADC3_IRQHandler(void)
{
  /* USER CODE BEGIN ADC3_IRQn 0 */

  /* USER CODE END ADC3_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc3);
  /* USER CODE BEGIN ADC3_IRQn 1 */

  /* USER CODE END ADC3_IRQn 1 */
}

/**
  * @brief This function handles SPI3 global interrupt.
  */
void SPI3_IRQHandler(void)
{
  /* USER CODE BEGIN SPI3_IRQn 0 */

  /* USER CODE END SPI3_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi3);
  /* USER CODE BEGIN SPI3_IRQn 1 */

  /* USER CODE END SPI3_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC3 channel underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

	KalArm_MFSW.ms++;
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
	if(uart_send == 1){

			if((huart3.gState & 1) == 0){ // xxxxxxx0 & 1 == 0 means the system is free, == 1 means the system is busy, 
					HAL_UART_Transmit_IT(&huart3, txData_UART, 19);
					uart_send = 0;
			}	
	}
	if(integration_state_number >= 1 && KalArm_CHG_DET.charger_state == 0){
	HAL_ADC_Start_IT(&hadc1);
	/******************************************************MFSW CALIBRATION LOGIC**********************************************************/	
		if((KalArm_MODE.mode_state == 0) && ((KalArm_SPI_RECEIVE_FLAGS.calibration_result == CALIBRATION_IN_PROGRESS) || (KalArm_SPI_RECEIVE_FLAGS.calibration_result == CALIBRATION_FAIL))){
		
		if(KalArm_SPI_RECEIVE_FLAGS.calibration_result == CALIBRATION_IN_PROGRESS){
			if(KalArm_MFSW.ms%100 == 0 ) // 100
				cal_in_progress_LED();
			else if(KalArm_MFSW.ms%5 == 0) // 5
				reset_rgb_LED();
		}
		else if(KalArm_SPI_RECEIVE_FLAGS.calibration_result == CALIBRATION_FAIL){
			
			if(KalArm_MFSW.ms%1000 == 0)
				cal_fail_LED();
			else if(KalArm_MFSW.ms%500 == 0)
				reset_rgb_LED();
		}
	}
	/***************************************************MFSW HAPTIC FEEDBACK LOGIC******************************************************/
	if(KalArm_SPI_RECEIVE_FLAGS.calibration_result == CALIBRATION_PASS){

		if(HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin) == GPIO_PIN_RESET){
			
			KalArm_MFSW.haptic_window++;

			if(KalArm_MFSW.haptic_window > 800 && KalArm_MFSW.haptic_window <= 2000){
					//button to indicate battery status
				KalArm_MODE.mode_state = 0;
				KalArm_BLE.ble_on_off_flag = 0;
				
				if(KalArm_MFSW.view_battery_flag == 0){
					KalArm_BATTERY.battery_status_flag = 1;
					KalArm_BATTERY.battery_status_flag_start_time = KalArm_MFSW.ms;
					
					KalArm_MFSW.view_battery_flag = 1;
					reset_rgb_LED();
				}
			}
			else if(KalArm_MFSW.haptic_window> 3000 && KalArm_MFSW.haptic_window <=10000){
					//set BLE flag
				KalArm_BATTERY.battery_status_flag = 0;
				if(KalArm_MFSW.ble_on_off_flag == 0){
					if(!KalArm_BLE.ble_on_off_flag){
						KalArm_BLE.ble_on_off_flag = 1;
//						KalArm_BLE.ble_advertising_flag=0;
//						KalArm_BLE.ble_connected_flag=0;
						KalArm_MODE.mode_state = 0;
						reset_rgb_LED();
					}
					else {
						KalArm_BLE.ble_on_off_flag = 0;
//						KalArm_BLE.ble_advertising_flag = 0;
//						KalArm_BLE.ble_connected_flag = 0;
					}
					KalArm_MFSW.ble_on_off_flag =1;
				}				
			}
		
		}	
	}
	
	/*****************************************************************MFSW LED STATUS CONTROL********************************************/
	if(KalArm_SPI_RECEIVE_FLAGS.calibration_result == CALIBRATION_PASS){
		if((KalArm_MODE.mode_state ==1) && (KalArm_BLE.ble_on_off_flag ==0) && (KalArm_BATTERY.battery_status_flag == 0)){
			
			if(KalArm_MODE.mode_number == 0){
				
				if(KalArm_GRIP.ingrip){
					
					if(KalArm_MFSW.ms % 500  == 0)
						mode_1_LED_toggle();
				
				}
				else{
					
					mode_1_LED();
				}
				
			}
			else if(KalArm_MODE.mode_number == 1){
				
				if(KalArm_GRIP.ingrip){
					
					
					if(KalArm_MFSW.ms % 500  == 0)
						mode_2_LED_toggle();
					
				}
				else{
					
					mode_2_LED();
				}
				
			}
			else if(KalArm_MODE.mode_number == 2){
				
				if(KalArm_GRIP.ingrip){
					
					if(KalArm_MFSW.ms % 500  == 0)
						mode_3_LED_toggle();
					
				}
				else{
					
					switch(KalArm_SPI_RECEIVE_FLAGS.grip_id){	
						case 0: 
							if(KalArm_MFSW.ms%1800 == 0) // 100
									mode_3_LED();
							else if(KalArm_MFSW.ms%50 == 0) // 5
									reset_rgb_LED();
							break;
						case 1:
							if(KalArm_MFSW.ms%2100 == 0 || (KalArm_MFSW.ms%2100 == 300)) // 100
									mode_3_LED();
							else if(KalArm_MFSW.ms%50 == 0) // 5
									reset_rgb_LED();
							break;
						case 2:
							if(KalArm_MFSW.ms%2400 == 0 || (KalArm_MFSW.ms%2400 == 300) || (KalArm_MFSW.ms%2400 == 600)) // 100
									mode_3_LED();
							else if(KalArm_MFSW.ms%50 == 0) // 5
									reset_rgb_LED();
							break;
						case 3:
							if(KalArm_MFSW.ms%2700 == 0 || (KalArm_MFSW.ms%2700 == 300) || (KalArm_MFSW.ms%2700 == 600) || (KalArm_MFSW.ms%2700 == 900)) // 100
									mode_3_LED();
							else if(KalArm_MFSW.ms%50 == 0) // 5
									reset_rgb_LED();
							break;
						case 4:
							if(KalArm_MFSW.ms%3000 == 0 || (KalArm_MFSW.ms%3000 == 300) || (KalArm_MFSW.ms%3000 == 600) || (KalArm_MFSW.ms%3000 == 900) || (KalArm_MFSW.ms%3000 == 1200)) // 100
									mode_3_LED();
							else if(KalArm_MFSW.ms%50 == 0) // 5
									reset_rgb_LED();
						break;
							
					}
//					mode_3_LED();
					
					
				}
				
			}
			else if(KalArm_MODE.mode_number == 3){
				
				if(KalArm_GRIP.ingrip){
					

					if(KalArm_MFSW.ms % 500  == 0)
						mode_4_LED_toggle();
					
				}
				else{

					mode_4_LED();
				}
				
			}
			
		}
		
	} // END of KalArm Calibration PASS
	
	if((KalArm_BLE.ble_on_off_flag==1)  && (KalArm_MODE.mode_state==0) && (KalArm_BATTERY.battery_status_flag==0)){
		
			if(KalArm_BLE.ble_advertising_flag){
				
				if(KalArm_MFSW.ms % 500 == 0)
					toggle_bleMode_LED();
			
			}
			else if(KalArm_BLE.ble_connection_request_flag){
				
				if(KalArm_MFSW.ms % 100 == 0)
					toggle_bleMode_LED();
				
			}
			else if(KalArm_BLE.ble_connected_flag){
				
				set_ble_LED();
			}
		
		} // End of BLE On OFF
		
		else if((KalArm_BLE.ble_on_off_flag==0)  && (KalArm_MODE.mode_state==0) && (KalArm_BATTERY.battery_status_flag==1)){
			
			if(KalArm_MFSW.ms - KalArm_BATTERY.battery_status_flag_start_time < 1200){
				
				if(KalArm_BATTERY.battery_percentage < 35){
					
					if(KalArm_MFSW.ms%200 == 0){
						toggle_R_LED();
					}
				}
				else if(KalArm_BATTERY.battery_percentage >=35 && KalArm_BATTERY.battery_percentage < 80){
					if(KalArm_MFSW.ms%200 ==0){	
						bat_charge_75_LED();
					}
					
				}
				else if(KalArm_BATTERY.battery_percentage >= 80){
					if(KalArm_MFSW.ms%200 ==0){			
						bat_charge_100_LED();
					}
				}
				
			}
			else{
				/* reinitialize variables */
				KalArm_BATTERY.battery_status_flag=0;
				reset_rgb_LED();
				KalArm_MODE.mode_state = 1;
			}
		
		}
	
	}
	else if(KalArm_CHG_DET.charger_state == 1){
		
		if(KalArm_BATTERY.battery_percentage < 35){
			if(KalArm_MFSW.ms%500 == 0){
				toggle_R_LED();
			}
		}
		else if(KalArm_BATTERY.battery_percentage >= 35 && KalArm_BATTERY.battery_percentage <80){
			if(KalArm_MFSW.ms%500 ==0){	
				bat_charge_75_LED();
			}
			
		}
		else if(KalArm_BATTERY.battery_percentage >= 80){
			if(KalArm_MFSW.ms%500 ==0){			
				bat_charge_100_LED();
			}
		}
		

	}

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt, DAC2 and DAC4 channel underrun error interrupts.
  */
void TIM7_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_DAC_IRQn 0 */
	
	
	msTicks_T_7++;
  /* USER CODE END TIM7_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_DAC_IRQn 1 */

		//---------------------------------------------------------------------------------------------------------------------PATTERNS AND GRIPS CODE STARTS HERE---------------------------------------------------------------------------------------
	if((integration_state_number >= 1) && (KalArm_CHG_DET.charger_state ==0)){
		HAL_ADC_Start_IT(&hadc3);
		HAL_ADC_Start_IT(&hadc4);
		HAL_ADC_Start_IT(&hadc2);
//		HAL_ADC_Start_DMA(&hadc2, ADC_raw, 2);
	//-----------------------------------------------------------------------------------------------------------------------EVALUATING PATTERNS FOR CLOSE SENSOR------------------------------------------------------------------------------------
		
//	if(HAL_GPIO_ReadPin(LOD2_P_GPIO_Port, LOD2_P_Pin) == GPIO_PIN_RESET || HAL_GPIO_ReadPin(LOD2_N_GPIO_Port, LOD2_N_Pin) == GPIO_PIN_RESET){
	//here we keep track of previous state of emg close signal
	KalArm_EMG_CLOSE.pre_emg_state_flag=KalArm_EMG_CLOSE.emg_state_flag;
	//here we keep track of previous pattern
	KalArm_EMG_CLOSE.pre_pattern = KalArm_EMG_CLOSE.pattern;
	if( (KalArm_EMG_CLOSE.sensor_val)> KalArm_PATTERN_RECOGNITION_SUITE_PARAMS.close_threshold){//we increment the glich period and update emg close state to high
		KalArm_EMG_CLOSE.glitch++;
		KalArm_EMG_CLOSE.emg_state_flag =1;
	}
	else{ //we leave update emg state to low and donot increment glitch period
		KalArm_EMG_CLOSE.emg_state_flag =0;
		KalArm_EMG_CLOSE.emg_prop_flag=0;
	}

	if(KalArm_EMG_CLOSE.glitch > 50 || KalArm_EMG_CLOSE.peak_count>0){ // condition that glitch is greater than 50 or the number of peaks counted thus far is greater than 1
		
			if(KalArm_EMG_CLOSE.analyseTick< KalArm_PATTERN_RECOGNITION_SUITE_PARAMS.analysis_period){// beginning of analysis period's if	
						KalArm_EMG_CLOSE.analyseTick++;
						if((KalArm_EMG_CLOSE.sensor_val) > KalArm_PATTERN_RECOGNITION_SUITE_PARAMS.close_threshold && KalArm_EMG_CLOSE.emg_state_flag && !KalArm_EMG_CLOSE.pre_emg_state_flag){
							KalArm_EMG_CLOSE.pre_peak_stamp=KalArm_EMG_CLOSE.analyseTick;
						}
						
						else if((KalArm_EMG_CLOSE.sensor_val)  < KalArm_PATTERN_RECOGNITION_SUITE_PARAMS.close_threshold && !KalArm_EMG_CLOSE.emg_state_flag && KalArm_EMG_CLOSE.pre_emg_state_flag){
							KalArm_EMG_CLOSE.peak_stamp=KalArm_EMG_CLOSE.analyseTick;
							KalArm_EMG_CLOSE.pulse_period = KalArm_EMG_CLOSE.peak_stamp-KalArm_EMG_CLOSE.pre_peak_stamp;
							if(KalArm_EMG_CLOSE.pulse_period > 0 && KalArm_EMG_CLOSE.pulse_period <= KalArm_PATTERN_RECOGNITION_SUITE_PARAMS.pulse_period){
								KalArm_EMG_CLOSE.peak_count++;
								KalArm_EMG_CLOSE.total_pulse_period+= KalArm_EMG_CLOSE.pulse_period;
							}
							
						}

			}// end of analysis period's if
			
			else{// beginning of analysis period's else
						if(KalArm_EMG_CLOSE.peak_count >= 1){
									KalArm_EMG_CLOSE.avg_pulse_period = KalArm_EMG_CLOSE.total_pulse_period/KalArm_EMG_CLOSE.peak_count;

									if(KalArm_EMG_CLOSE.avg_pulse_period >= 70 && KalArm_EMG_CLOSE.avg_pulse_period <= KalArm_PATTERN_RECOGNITION_SUITE_PARAMS.pulse_period){
										
										switch(KalArm_EMG_CLOSE.peak_count){
											case 1:
												KalArm_EMG_CLOSE.pattern= SINGLE_IMPULSE;
												update_event_close = 1;
												update_event_open = 0;
												if(KalArm_MODE.mode_number == 3)
													cust_grip_execute_id--;
												KalArm_EMG_CLOSE.pattern_update_event ^=1;
											break;
											case 2:
												KalArm_EMG_CLOSE.pattern= DOUBLE_IMPULSE;
												update_event_close = 1;
												update_event_open = 0;
												KalArm_CUST_GRIP_STAT_RES.grip_execute_order = 1;
												KalArm_EMG_CLOSE.pattern_update_event ^=1;
											break;
											case 3:
												KalArm_EMG_CLOSE.pattern=TRIPLE_IMPULSE;
												update_event_close = 1;
												update_event_open = 0;
												if(KalArm_MODE.mode_number == 3)
													cust_grip_execute_id++;
												KalArm_EMG_CLOSE.pattern_update_event ^=1;
											break;
										} // end of close double and triple switch case
									} // end of close double and triple if
									
						} // end of peak count if 
						
						KalArm_EMG_CLOSE.peak_count=0;
						KalArm_EMG_CLOSE.total_pulse_period=0;
						KalArm_EMG_CLOSE.avg_pulse_period=0;
						KalArm_EMG_CLOSE.pre_peak_stamp=0;
						KalArm_EMG_CLOSE.peak_stamp=0;
						KalArm_EMG_CLOSE.analyseTick=0;
						KalArm_EMG_CLOSE.glitch =0;
						KalArm_EMG_CLOSE.pulse_period=0;
						
						
					}// end of analysis period else
							
	}// end of pattern glitch period check					
	//-----End of Pattern Evaluation-----
	if(KalArm_EMG_CLOSE.glitch > 300 && KalArm_EMG_CLOSE.peak_count == 0){ //consider it as proportional only if the 150 glitch ticks(dead-time for proportional signal)
		
				if((KalArm_EMG_CLOSE.sensor_val) > KalArm_PATTERN_RECOGNITION_SUITE_PARAMS.close_threshold && (KalArm_EMG_OPEN.sensor_val)< KalArm_PATTERN_RECOGNITION_SUITE_PARAMS.open_threshold){
					KalArm_EMG_CLOSE.emg_prop_flag = 1;
					KalArm_EMG_OPEN.emg_prop_flag = 0;
				}


	}
	
//	}
	//-----------------------------------------------------------------------------------------------------------------------EVALUATING PATTERNS FOR OPEN SENSOR------------------------------------------------------------------------------------
	
//	if(HAL_GPIO_ReadPin(LOD1_P_GPIO_Port, LOD1_P_Pin) == GPIO_PIN_RESET || HAL_GPIO_ReadPin(LOD1_N_GPIO_Port, LOD1_N_Pin) == GPIO_PIN_RESET){
	//here we keep track of previous state of emg close signal
	KalArm_EMG_OPEN.pre_emg_state_flag=KalArm_EMG_OPEN.emg_state_flag;
	//here we keep track of previous pattern
	KalArm_EMG_OPEN.pre_pattern = KalArm_EMG_OPEN.pattern;
	if( (KalArm_EMG_OPEN.sensor_val)> KalArm_PATTERN_RECOGNITION_SUITE_PARAMS.open_threshold){//we increment the glich period and update emg close state to high
		KalArm_EMG_OPEN.glitch++;
		KalArm_EMG_OPEN.emg_state_flag =1;
	}
	else{ //we leave update emg state to low and donot increment glitch period
		KalArm_EMG_OPEN.emg_state_flag =0;
		KalArm_EMG_OPEN.emg_prop_flag=0;
	}

	if(KalArm_EMG_OPEN.glitch > 50 || KalArm_EMG_OPEN.peak_count>0){ // condition that glitch is greater than 50 or the number of peaks counted thus far is greater than 1
		
			if(KalArm_EMG_OPEN.analyseTick< KalArm_PATTERN_RECOGNITION_SUITE_PARAMS.analysis_period){// beginning of analysis period's if	
						KalArm_EMG_OPEN.analyseTick++;
						if((KalArm_EMG_OPEN.sensor_val) > KalArm_PATTERN_RECOGNITION_SUITE_PARAMS.open_threshold && KalArm_EMG_OPEN.emg_state_flag && !KalArm_EMG_OPEN.pre_emg_state_flag){
							KalArm_EMG_OPEN.pre_peak_stamp=KalArm_EMG_OPEN.analyseTick;
						}
						
						else if((KalArm_EMG_OPEN.sensor_val)  < KalArm_PATTERN_RECOGNITION_SUITE_PARAMS.open_threshold && !KalArm_EMG_OPEN.emg_state_flag && KalArm_EMG_OPEN.pre_emg_state_flag){
							KalArm_EMG_OPEN.peak_stamp=KalArm_EMG_OPEN.analyseTick;
							KalArm_EMG_OPEN.pulse_period = KalArm_EMG_OPEN.peak_stamp-KalArm_EMG_OPEN.pre_peak_stamp;
							if(KalArm_EMG_OPEN.pulse_period > 0 && KalArm_EMG_OPEN.pulse_period <= KalArm_PATTERN_RECOGNITION_SUITE_PARAMS.pulse_period){
								KalArm_EMG_OPEN.peak_count++;
								KalArm_EMG_OPEN.total_pulse_period+= KalArm_EMG_OPEN.pulse_period;
							}
							
						}

			}// end of analysis period's if
			
			else{// beginning of analysis period's else
						if(KalArm_EMG_OPEN.peak_count >= 1){
									KalArm_EMG_OPEN.avg_pulse_period = KalArm_EMG_OPEN.total_pulse_period/KalArm_EMG_OPEN.peak_count;

									if(KalArm_EMG_OPEN.avg_pulse_period >= 70 && KalArm_EMG_OPEN.avg_pulse_period <= KalArm_PATTERN_RECOGNITION_SUITE_PARAMS.pulse_period){
										
										switch(KalArm_EMG_OPEN.peak_count){
											case 1:
												KalArm_EMG_OPEN.pattern= SINGLE_IMPULSE;
												update_event_open = 1;
												update_event_close = 0;
												KalArm_EMG_OPEN.pattern_update_event ^= 1;
											break;
											case 2:
												KalArm_EMG_OPEN.pattern= DOUBLE_IMPULSE;
												update_event_open = 1;
												update_event_close = 0;
												
												KalArm_EMG_OPEN.pattern_update_event ^=1;
											break;
											case 3:
												KalArm_EMG_OPEN.pattern=TRIPLE_IMPULSE;
												update_event_open = 1;
												update_event_close =0;
												KalArm_EMG_OPEN.pattern_update_event ^=1;
											break;
										} // end of close double and triple switch case
									} // end of close double and triple if
									
						} // end of peak count if 
						
						KalArm_EMG_OPEN.peak_count=0;
						KalArm_EMG_OPEN.total_pulse_period=0;
						KalArm_EMG_OPEN.avg_pulse_period=0;
						KalArm_EMG_OPEN.pre_peak_stamp=0;
						KalArm_EMG_OPEN.peak_stamp=0;
						KalArm_EMG_OPEN.analyseTick=0;
						KalArm_EMG_OPEN.glitch =0;
						KalArm_EMG_OPEN.pulse_period=0;
						
						
					}// end of analysis period else
							
	}// end of pattern glitch period check					
	//-----End of Pattern Evaluation-----
	if(KalArm_EMG_OPEN.glitch > 300 && KalArm_EMG_OPEN.peak_count == 0){ //consider it as proportional only if the 150 glitch ticks(dead-time for proportional signal)
		
				if((KalArm_EMG_OPEN.sensor_val) > KalArm_PATTERN_RECOGNITION_SUITE_PARAMS.open_threshold && (KalArm_EMG_CLOSE.sensor_val)< KalArm_PATTERN_RECOGNITION_SUITE_PARAMS.close_threshold){
					KalArm_EMG_CLOSE.emg_prop_flag = 0;
					KalArm_EMG_OPEN.emg_prop_flag = 1;
				}


	}
//}
	
//	if(HAL_GPIO_ReadPin(LOD2_P_GPIO_Port, LOD2_P_Pin) == GPIO_PIN_SET || HAL_GPIO_ReadPin(LOD2_N_GPIO_Port, LOD2_N_Pin) == GPIO_PIN_SET){
//	KalArm_EMG_CLOSE.emg_prop_flag = 0;
//	
//	}
//	
//	if(HAL_GPIO_ReadPin(LOD1_P_GPIO_Port, LOD1_P_Pin) == GPIO_PIN_SET || HAL_GPIO_ReadPin(LOD1_N_GPIO_Port, LOD1_N_Pin) == GPIO_PIN_SET){
//	KalArm_EMG_OPEN.emg_prop_flag = 0;
//	
//	}
	
}

	
  /* USER CODE END TIM7_DAC_IRQn 1 */
}

/**
  * @brief This function handles ADC4 global interrupt.
  */
void ADC4_IRQHandler(void)
{
  /* USER CODE BEGIN ADC4_IRQn 0 */

  /* USER CODE END ADC4_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc4);
  /* USER CODE BEGIN ADC4_IRQn 1 */

  /* USER CODE END ADC4_IRQn 1 */
}

/**
  * @brief This function handles COMP4, COMP5 and COMP6 interrupts through EXTI lines 30, 31 and 32.
  */
void COMP4_5_6_IRQHandler(void)
{
  /* USER CODE BEGIN COMP4_5_6_IRQn 0 */

  /* USER CODE END COMP4_5_6_IRQn 0 */
  HAL_COMP_IRQHandler(&hcomp4);
  /* USER CODE BEGIN COMP4_5_6_IRQn 1 */

  /* USER CODE END COMP4_5_6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
