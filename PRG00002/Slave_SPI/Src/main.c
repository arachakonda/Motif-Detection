/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
ADC_HandleTypeDef hadc5;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;
DMA_HandleTypeDef hdma_adc5;

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;
OPAMP_HandleTypeDef hopamp3;
OPAMP_HandleTypeDef hopamp4;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */


uint8_t rxData_SPI [4];
uint8_t txData_SPI [4]= {255,0,0,255};
	


KalArmMotorHandle M1, M2, M3, M4;

uint32_t msTicks_T_7;


uint32_t i=0;
uint32_t j=0;


uint8_t KalArm_Calibration_Flag;
uint8_t KalArm_EMG_Flag;

uint8_t KalArm_Calibration_times;

KalArm_MODE_TYPE KalArm_MODE;

KalArm_EMG_TYPE KalArm_EMG_OPEN, KalArm_EMG_CLOSE;

KalArm_GRIP_PATTERN_DETAILS KalArm_GRIPS[24];

uint8_t thumb_position;

uint8_t integration_state_number;



KalArm_CUST_GRIP_STAT_RES_TYPE KalArm_CUST_GRIP_STAT_RES;
volatile KalArm_GEN_GRIP_STAT_RES_TYPE KalArm_GEN_GRIP_STAT_RES;

volatile uint8_t grip_test_1_execute;
volatile uint8_t grip_test_2_execute;
volatile uint8_t grip_test_3_execute;
volatile uint8_t grip_test_4_execute;
volatile uint8_t grip_test_5_execute;
volatile uint8_t grip_test_6_execute;
volatile uint8_t grip_test_7_execute;
volatile uint8_t grip_test_8_execute;
volatile uint8_t cust_grip_test_execute;

uint16_t grip_test_clench_prev, grip_test_clench;

uint8_t KalArm_release;

float M1_ACCURACY, M2_ACCURACY, M3_ACCURACY, M4_ACCURACY;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI3_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC5_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_OPAMP3_Init(void);
static void MX_OPAMP4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	
	if(HAL_GPIO_ReadPin(REQ_CLK_GPIO_Port, REQ_CLK_Pin) == GPIO_PIN_RESET){
		//this is the case in which the received data at the motor controller end is dummy data or old data
		HAL_GPIO_WritePin(REQ_CLK_GPIO_Port, REQ_CLK_Pin, GPIO_PIN_SET);
	}
	else if(HAL_GPIO_ReadPin(REQ_CLK_GPIO_Port, REQ_CLK_Pin) == GPIO_PIN_SET){
		//this is the case where the received data at the motor controller end is actual data sent by master
		spi_receive_handler();
	}
	HAL_SPI_TransmitReceive_IT(&hspi3,txData_SPI, rxData_SPI,4);
  //HAL_SPI_Transmit_IT(&hspi3,(uint8_t *)TxBuf,2);
	//HAL_GPIO_WritePin(REQ_CLK_GPIO_Port, REQ_CLK_Pin, GPIO_PIN_RESET);
	
	// Motor controller knows what data it is sending and when it sends it is using the request clock pin
	// motor controller does not know when it is receiving new data from master
}



void HAL_TIMEx_EncoderIndexCallback(TIM_HandleTypeDef *htim){
	
	if(htim == &htim2){
		if(M4.motor_direction == ACLOCKWISE){
			(M4.current_index_counts++);
		if(M4.current_index_counts == M4.target_encoder_index_counts)
			motor_4_reset();
		}
		else if(M4.motor_direction == CLOCKWISE){
			(M4.current_index_counts--);
		if(M4.current_index_counts == M4.target_encoder_index_counts)
			motor_4_reset();

		}
		else if(M4.motor_direction == ACLOCKWISE_LIMIT || M4.motor_direction == ACLOCKWISE_LIMIT_ADAPTIVE){
			(M4.current_index_counts++);
		}
		else if(M4.motor_direction == CLOCKWISE_LIMIT || M4.motor_direction == CLOCKWISE_LIMIT_ADAPTIVE){
			(M4.current_index_counts--);
		}
		TIM2->SR &= ~(TIM_SR_IDXF); 

			
	}
	if(htim == &htim1){
		if(M3.motor_direction == ACLOCKWISE){
			(M3.current_index_counts++);
		if(M3.current_index_counts == M3.target_encoder_index_counts )
			motor_3_reset();
		}
		else if(M3.motor_direction == CLOCKWISE){
			(M3.current_index_counts--);
		if(M3.current_index_counts == M3.target_encoder_index_counts )
			motor_3_reset();
		}
		else if(M3.motor_direction == ACLOCKWISE_LIMIT || M3.motor_direction == ACLOCKWISE_LIMIT_ADAPTIVE){
			(M3.current_index_counts++);
		}
		else if(M3.motor_direction == CLOCKWISE_LIMIT || M3.motor_direction == CLOCKWISE_LIMIT_ADAPTIVE){
			(M3.current_index_counts--);
		}
		TIM1->SR &= ~(TIM_SR_IDXF); 

			
	}
	if(htim == &htim3){
		if(M2.motor_direction == ACLOCKWISE){
			(M2.current_index_counts++);
		if(M2.current_index_counts == M2.target_encoder_index_counts)
			motor_2_reset();
		}
		else if(M2.motor_direction == CLOCKWISE){
			(M2.current_index_counts--);
		if(M2.current_index_counts == M2.target_encoder_index_counts)
			motor_2_reset();

		}
		else if(M2.motor_direction == ACLOCKWISE_LIMIT || M2.motor_direction == ACLOCKWISE_LIMIT_ADAPTIVE){
			(M2.current_index_counts++);
		}
		else if(M2.motor_direction == CLOCKWISE_LIMIT || M2.motor_direction == CLOCKWISE_LIMIT_ADAPTIVE){
			(M2.current_index_counts--);
		}
		TIM3->SR &= ~(TIM_SR_IDXF);

	}
	if(htim == &htim8){
	if(M1.motor_direction == ACLOCKWISE){
			(M1.current_index_counts++);
		if(M1.current_index_counts == M1.target_encoder_index_counts)
			motor_1_reset();
		}
		else if(M1.motor_direction == CLOCKWISE){
			(M1.current_index_counts--);
		if(M1.current_index_counts == M1.target_encoder_index_counts)
			motor_1_reset();

		}
		else if(M1.motor_direction == ACLOCKWISE_LIMIT || M1.motor_direction == ACLOCKWISE_LIMIT_ADAPTIVE){
			(M1.current_index_counts++);
		}
		else if(M1.motor_direction == CLOCKWISE_LIMIT || M1.motor_direction == CLOCKWISE_LIMIT_ADAPTIVE){
			(M1.current_index_counts--);
		}
		TIM8->SR &= ~(TIM_SR_IDXF);

	}
	
	if(integration_state_number == 3 || integration_state_number == 5 || integration_state_number == 6){ // If no signal / emg prop open / emg pat open DI
		if((M1.current_index_counts <= 65535 && M1.current_index_counts > 65500) || (M1.current_index_counts <= ENCODER_LIMIT_ZERO/5 && M1.current_index_counts > 0)){
			motor_1_reset();
			M1.current_index_counts = 0;
		}
		if((M2.current_index_counts <= 65535 && M2.current_index_counts > 65500) || (M2.current_index_counts <= ENCODER_LIMIT_ZERO/5 && M2.current_index_counts > 0)){
			motor_2_reset();
			M2.current_index_counts = 0;
		}
		if((M3.current_index_counts <= 65535 && M3.current_index_counts > 65500) || (M3.current_index_counts <= ENCODER_LIMIT_ZERO/5 && M3.current_index_counts > 0)){
			motor_3_reset();
			M3.current_index_counts = 0;
		}
		if((M4.current_index_counts <= 65535 && M4.current_index_counts > 65500) || (M4.current_index_counts <= ENCODER_LIMIT_ZERO/5 && M4.current_index_counts > 0)){
			motor_4_reset();
			M4.current_index_counts = 0;
		}
	}

}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_ADC5_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_OPAMP4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	
	
	current_sense_init(1); // Initiate current sense time base
	motor_1_encoder_config_init(1); // set motor 1 encoder registers and initiate
	motor_2_encoder_config_init(1); // set motor 2 encoder registers and initiate
	motor_3_encoder_config_init(1); // set motor 3 encoder registers and initiate 
	motor_4_encoder_config_init(1); // set motor 4 encoder registers and initiate

	motor_1_current_config_init(1);
	motor_2_current_config_init(1);
	motor_3_current_config_init(1);
	motor_4_current_config_init(1);

	motors_init(1);
	
	HAL_GPIO_WritePin(STAT_MOT_GPIO_Port, STAT_MOT_Pin, GPIO_PIN_RESET);
	
	
	HAL_SPI_TransmitReceive_IT(&hspi3, txData_SPI, rxData_SPI, 4);
	
	


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(KalArm_Calibration_Flag == 1){
			calibration_routine();
			prep_arm();
			KalArm_Calibration_Flag = 0;
		}
		
		if(KalArm_Calibration_Flag == 0 && KalArm_EMG_Flag == 1){
			
			if(KalArm_release){
				
				while(M1.current_index_counts>ENCODER_LIMIT_ZERO*2 || M2.current_index_counts>ENCODER_LIMIT_ZERO*2 || M3.current_index_counts>ENCODER_LIMIT_ZERO*2 || M4.current_index_counts>ENCODER_LIMIT_ZERO*2){

					motor_3_set(CLOCKWISE, ENCODER_LIMIT_ZERO);
					if(M3.current_index_counts <= THUMB_OPEN_OPPOSITION_SAFE){

								motor_1_set(CLOCKWISE, ENCODER_LIMIT_ZERO);

								motor_2_set(CLOCKWISE, ENCODER_LIMIT_ZERO);

								motor_4_set(CLOCKWISE, ENCODER_LIMIT_ZERO);
					}

					if(grip_test_7_execute)
						break;
					HAL_Delay(200);
					if(M1.current_diff == 0 && M2.current_diff == 0 && M3.current_diff == 0 && M4.current_diff == 0){
						break;
					}
				} // end of while to OPEN
			
			
			KalArm_release = 0;
			
			}
		
			if(KalArm_EMG_OPEN.emg_prop_flag == 1 && KalArm_EMG_CLOSE.emg_prop_flag == 0){
				integration_state_number = 3;
				switch(KalArm_MODE.mode_number){
					case 0: // MODE 1
						if(thumb_position == THUMB_OPPOSITION){
							if(STEP_DOWN_M3_VALID)
								motor_3_set(CLOCKWISE, M3.current_index_counts- ENCODER_LIMIT_ZERO/10);
							if(M3.current_index_counts <=THUMB_OPEN_OPPOSITION_SAFE){
								if(STEP_DOWN_M1_VALID)
									motor_1_set(CLOCKWISE, M1.current_index_counts- ENCODER_LIMIT_ZERO/10);
								if(STEP_DOWN_M2_VALID)
									motor_2_set(CLOCKWISE, M2.current_index_counts- ENCODER_LIMIT_ZERO/10);
								if(STEP_DOWN_M4_VALID)
									motor_4_set(CLOCKWISE, M4.current_index_counts- ENCODER_LIMIT_ZERO/10);
							}
						}
						else if(thumb_position == THUMB_NON_OPPOSITION){
							if(STEP_DOWN_M3_VALID)
								motor_3_set(CLOCKWISE, M3.current_index_counts- ENCODER_LIMIT_ZERO/10);
							if(M3.current_index_counts <= THUMB_OPEN_NON_OPPOSITION_SAFE){
								if(STEP_DOWN_M1_VALID)
									motor_1_set(CLOCKWISE, M1.current_index_counts- ENCODER_LIMIT_ZERO/10);
								if(STEP_DOWN_M2_VALID)
									motor_2_set(CLOCKWISE, M2.current_index_counts- ENCODER_LIMIT_ZERO/10);
								if(STEP_DOWN_M4_VALID)
									motor_4_set(CLOCKWISE, M4.current_index_counts- ENCODER_LIMIT_ZERO/10);

							}
						}

					break;
					case 1: // MODE 2
						if(thumb_position == THUMB_OPPOSITION){
							if(KalArm_EMG_CLOSE.trigger == TRIGGER_NOT_SET){
								if(STEP_DOWN_M1_VALID)
									motor_1_set(CLOCKWISE, M1.current_index_counts- ENCODER_LIMIT_ZERO/10);
								if(STEP_DOWN_M2_VALID)
									motor_2_set(CLOCKWISE, M2.current_index_counts- ENCODER_LIMIT_ZERO/10);
								if(STEP_DOWN_M4_VALID)
									motor_4_set(CLOCKWISE, M4.current_index_counts- ENCODER_LIMIT_ZERO/10);
								if(M1.current_index_counts <= MODE_2_INDEX_OPEN_SAFE){
									if(STEP_DOWN_M3_VALID)
										motor_3_set(CLOCKWISE, M3.current_index_counts - ENCODER_LIMIT_ZERO/10);
								
								}
										
								
			
							}
							else{
									if(STEP_DOWN_M1_VALID)
										motor_1_set(CLOCKWISE, M1.current_index_counts- ENCODER_LIMIT_ZERO/10);
									if(STEP_DOWN_M2_VALID)
										motor_2_set(CLOCKWISE, M2.current_index_counts- ENCODER_LIMIT_ZERO/10);
							}
						}
						else if(thumb_position == THUMB_NON_OPPOSITION){
							if(STEP_DOWN_M3_VALID)
								motor_3_set(CLOCKWISE, M3.current_index_counts- ENCODER_LIMIT_ZERO/10);
							if(M3.current_index_counts <= THUMB_OPEN_NON_OPPOSITION_SAFE){
								if(STEP_DOWN_M1_VALID)
									motor_1_set(CLOCKWISE, M1.current_index_counts- ENCODER_LIMIT_ZERO/10);
								if(STEP_DOWN_M2_VALID)
									motor_2_set(CLOCKWISE, M2.current_index_counts- ENCODER_LIMIT_ZERO/10);
								if(STEP_DOWN_M4_VALID)
									motor_4_set(CLOCKWISE, M4.current_index_counts- ENCODER_LIMIT_ZERO/10);

							}
						}
					break;
					case 2: // MODE 3
						if(thumb_position == THUMB_OPPOSITION){
							if(KalArm_MODE.mode_3_grip_id == 2){
								
								if(STEP_DOWN_M1_VALID)
									motor_1_set(CLOCKWISE, M1.current_index_counts- ENCODER_LIMIT_ZERO/10);
								if(STEP_DOWN_M2_VALID)
									motor_2_set(CLOCKWISE, M2.current_index_counts- ENCODER_LIMIT_ZERO/10);
								if(STEP_DOWN_M4_VALID)
									motor_4_set(CLOCKWISE, M4.current_index_counts- ENCODER_LIMIT_ZERO/10);
								if(M1.current_index_counts <= MODE_3_INDEX_SPHERICAL_OPEN_SAFE){
									if(STEP_DOWN_M3_VALID)
										motor_3_set(CLOCKWISE, M3.current_index_counts- ENCODER_LIMIT_ZERO/10);
								}
							
							
							}
							else{
								if(STEP_DOWN_M3_VALID)
										motor_3_set(CLOCKWISE, M3.current_index_counts- ENCODER_LIMIT_ZERO/10);
									if(M3.current_index_counts <= THUMB_OPEN_OPPOSITION_SAFE){
										if(STEP_DOWN_M1_VALID)
											motor_1_set(CLOCKWISE, M1.current_index_counts- ENCODER_LIMIT_ZERO/10);
										if(STEP_DOWN_M2_VALID)
											motor_2_set(CLOCKWISE, M2.current_index_counts- ENCODER_LIMIT_ZERO/10);
										if(STEP_DOWN_M4_VALID)
											motor_4_set(CLOCKWISE, M4.current_index_counts- ENCODER_LIMIT_ZERO/10);
									}
								}
						}
						else if(thumb_position == THUMB_NON_OPPOSITION){
							if(STEP_DOWN_M3_VALID)
								motor_3_set(CLOCKWISE, M3.current_index_counts- ENCODER_LIMIT_ZERO/10);
							if(M3.current_index_counts <= THUMB_OPEN_NON_OPPOSITION_SAFE){
								if(STEP_DOWN_M1_VALID)
									motor_1_set(CLOCKWISE, M1.current_index_counts- ENCODER_LIMIT_ZERO/10);
								if(STEP_DOWN_M2_VALID)
									motor_2_set(CLOCKWISE, M2.current_index_counts- ENCODER_LIMIT_ZERO/10);
								if(STEP_DOWN_M4_VALID)
									motor_4_set(CLOCKWISE, M4.current_index_counts- ENCODER_LIMIT_ZERO/10);

							}
						}
					break;
					case 3: // MODE 4
						if(thumb_position == THUMB_OPPOSITION){
							if(STEP_DOWN_M3_VALID)
								motor_3_set(CLOCKWISE, M3.current_index_counts- ENCODER_LIMIT_ZERO/10);
							if(M3.current_index_counts <= THUMB_OPEN_OPPOSITION_SAFE){
								if(STEP_DOWN_M1_VALID)
									motor_1_set(CLOCKWISE, M1.current_index_counts- ENCODER_LIMIT_ZERO/10);
								if(STEP_DOWN_M2_VALID)
									motor_2_set(CLOCKWISE, M2.current_index_counts- ENCODER_LIMIT_ZERO/10);
								if(STEP_DOWN_M4_VALID)
									motor_4_set(CLOCKWISE, M4.current_index_counts- ENCODER_LIMIT_ZERO/10);
							}
						}
						else if(thumb_position == THUMB_NON_OPPOSITION){
							if(STEP_DOWN_M3_VALID)
								motor_3_set(CLOCKWISE, M3.current_index_counts- ENCODER_LIMIT_ZERO/10);
							if(M3.current_index_counts <= THUMB_OPEN_NON_OPPOSITION_SAFE){
								if(STEP_DOWN_M1_VALID)
									motor_1_set(CLOCKWISE, M1.current_index_counts- ENCODER_LIMIT_ZERO/10);
								if(STEP_DOWN_M2_VALID)
									motor_2_set(CLOCKWISE, M2.current_index_counts- ENCODER_LIMIT_ZERO/10);
								if(STEP_DOWN_M4_VALID)
									motor_4_set(CLOCKWISE, M4.current_index_counts- ENCODER_LIMIT_ZERO/10);

							}
						}
					
					break;
				}
				
			}
			else if(KalArm_EMG_CLOSE.emg_prop_flag == 1 && KalArm_EMG_OPEN.emg_prop_flag == 0){
				integration_state_number = 4;
				switch(KalArm_MODE.mode_number){
										case 0: // MODE 1
											if(thumb_position == THUMB_OPPOSITION){ /*.................................THUMB OPPOSITION.............................*/ 
												if(STEP_UP_M1_VALID)
													motor_1_set(ACLOCKWISE, M1.current_index_counts+ENCODER_LIMIT_ZERO/10);
												if(STEP_UP_M2_VALID)
													motor_2_set(ACLOCKWISE, M2.current_index_counts+ENCODER_LIMIT_ZERO/10);
												if(STEP_UP_M4_VALID)
													motor_4_set(ACLOCKWISE, M4.current_index_counts+ENCODER_LIMIT_ZERO/10);
												if(M1.current_diff == 0 || M2. current_diff == 0){
													if(M1.current_index_counts < MODE_1_INDEX_HANDSHAKE_LIMIT && M2.current_index_counts < MODE_1_MIDDLE_HANDSHAKE_LIMIT
														&& M4.current_index_counts< MODE_1_RING_AND_LITTLE_HANDSHAKE_LIMIT){
														if(STEP_UP_M3_VALID)
															motor_3_set(ACLOCKWISE, M3.current_index_counts+ENCODER_LIMIT_ZERO/10);
													}
													else{
														if(M1.current_index_counts > MODE_1_INDEX_ADDUCTION_THRESHOLD && M2.current_index_counts > MODE_1_MIDDLE_ADDUCTION_THRESHOLD){
															if(M3.current_index_counts < MODE_1_THUMB_ADDUCTION_LIMIT){
																if(STEP_UP_M3_VALID)
																	motor_3_set(ACLOCKWISE, M3.current_index_counts+ENCODER_LIMIT_ZERO/10);
															}		
														}
													}
												}
												else{
													if(M3.current_index_counts < THUMB_CLOSE_OPPOSITION_SAFE){
														if(STEP_UP_M3_VALID)
															motor_3_set(ACLOCKWISE, M3.current_index_counts+ENCODER_LIMIT_ZERO/10);
													}
													else{
														if(M1.current_index_counts > MODE_1_INDEX_ADDUCTION_THRESHOLD && M2.current_index_counts > MODE_1_MIDDLE_ADDUCTION_THRESHOLD){
															if(M3.current_index_counts < MODE_1_THUMB_ADDUCTION_LIMIT){
																if(STEP_UP_M3_VALID)
																	motor_3_set(ACLOCKWISE, M3.current_index_counts+ENCODER_LIMIT_ZERO/10);
															}
																	
														}
													}
											}

											}
											else if(thumb_position == THUMB_NON_OPPOSITION){ /*.................................THUMB NON OPPOSITION.....*/
												if(KalArm_EMG_CLOSE.trigger == TRIGGER_NOT_SET){
													if(STEP_UP_M1_VALID)
														motor_1_set(ACLOCKWISE, M1.current_index_counts+ENCODER_LIMIT_ZERO/10);
													if(STEP_UP_M2_VALID)
														motor_2_set(ACLOCKWISE, M2.current_index_counts+ENCODER_LIMIT_ZERO/10);
													if(STEP_UP_M4_VALID)
														motor_4_set(ACLOCKWISE, M4.current_index_counts+ENCODER_LIMIT_ZERO/10);
													if(M3.current_index_counts < THUMB_CLOSE_NON_OPPOSITION_SAFE){
														if(STEP_UP_M3_VALID)
															motor_3_set(ACLOCKWISE, M3.current_index_counts+ENCODER_LIMIT_ZERO/10);
													}
												}
												else if(KalArm_EMG_CLOSE.trigger == TRIGGER_SET){
													if(STEP_UP_M1_VALID)
														motor_1_set(ACLOCKWISE, M1.current_index_counts+ENCODER_LIMIT_ZERO/10);
													if(STEP_UP_M2_VALID)
														motor_2_set(ACLOCKWISE, M2.current_index_counts+ENCODER_LIMIT_ZERO/10);
													if(STEP_UP_M4_VALID)
														motor_4_set(ACLOCKWISE, M4.current_index_counts+ENCODER_LIMIT_ZERO/10);
													
												}
											}
										break;
										case 1: // MODE 2
											if(thumb_position == THUMB_OPPOSITION){ /*.................................THUMB OPPOSITION.............................*/
												if(KalArm_EMG_CLOSE.trigger != TRIGGER_SET){ //------------------------------------------------------------IF TRIGGER NOT SET----
													KalArm_MODE_2_OPPOSED_CLOSE_ROUTINE();
												}
												else if (KalArm_EMG_CLOSE.trigger == TRIGGER_SET){ //----------------------------------------------------------IF TRIGGER SET----
													if(M1.current_index_counts < MODE_2_INDEX_TRIGGER_LIMIT){
														if(STEP_UP_M1_VALID)
															motor_1_set(ACLOCKWISE, M1.current_index_counts+ENCODER_LIMIT_ZERO/10); //-----------------------------------CURL INDEX-----------------------------
													}
													if(M2.current_index_counts < MODE_2_MIDDLE_TRIGGER_LIMIT){
													if(STEP_UP_M2_VALID)
														motor_2_set(ACLOCKWISE, M2.current_index_counts+ENCODER_LIMIT_ZERO/10);//-----------------------------------CURL MIDDLE--------------------------
													}
												}
													
											}
											else if (thumb_position == THUMB_NON_OPPOSITION){/*.............................THUMB NON OPPOSITION.........*/
												if(STEP_UP_M1_VALID)
													motor_1_set(ACLOCKWISE, M1.current_index_counts+ENCODER_LIMIT_ZERO/10);
												if(STEP_UP_M2_VALID)
													motor_2_set(ACLOCKWISE, M2.current_index_counts+ENCODER_LIMIT_ZERO/10);
												if(STEP_UP_M3_VALID)
													motor_3_set(ACLOCKWISE, M3.current_index_counts+ENCODER_LIMIT_ZERO/10);
												if(STEP_UP_M4_VALID)
													motor_4_set(ACLOCKWISE, M4.current_index_counts+ENCODER_LIMIT_ZERO/10);
											}

										break;
										case 2: // MODE 3
											if(thumb_position == THUMB_OPPOSITION){ /*.................................THUMB OPPOSITION.............................*/
												if(KalArm_MODE.mode_3_grip_id == 0){ // mode 3 grip id in pattern routine TRIPOD OPEN
													

													if(M3.current_index_counts < MODE_2_TWO_FINGER_TRIG_THUMB_LIMIT){
														if(STEP_UP_M3_VALID)
															motor_3_set(ACLOCKWISE, M3.current_index_counts+ENCODER_LIMIT_ZERO/10); //--------------------------------CURL THUMB-----------------------------------------------------------------
													}
													else{
														motor_3_reset(); //--------------------------------STOP THUMB-----------------------------------------------------------------
													}
													if(M3.current_index_counts >= MODE_2_TWO_FINGER_TRIG_THUMB_LIMIT
														&& M2.current_index_counts <= MODE_2_MIDDLE_LIMIT){
															if(STEP_UP_M2_VALID)
																motor_2_set(ACLOCKWISE, M2.current_index_counts+ENCODER_LIMIT_ZERO/10); //----------------------------CURL  MIDDLE-------------------------------------------------------------------
													}
													else{
														motor_2_reset(); //-------------------------------STOP MIDDLE--------------------------------------------------------------------
													}
													if(M3.current_index_counts >= MODE_2_TWO_FINGER_TRIG_THUMB_LIMIT
														&& M1.current_index_counts <= MODE_2_INDEX_LIMIT){  
														if(M1.current_index_counts <= MODE_2_INDEX_LIMIT){
															if(STEP_UP_M1_VALID)
																motor_1_set(ACLOCKWISE, M1.current_index_counts+ENCODER_LIMIT_ZERO/10); //----------------------------CURL  INDEX-------------------------------------------------------------------
														}
														else{
																motor_1_reset(); //-------------------------------STOP INDEX--------------------------------------------------------------------
														}
													}
													else{
														motor_1_reset();
													}
												
												}												
												else if(KalArm_MODE.mode_3_grip_id == 2){ //mode 3 grip id incremented in pattern routine SPHERICAL
													if(STEP_UP_M3_VALID)
														motor_3_set(ACLOCKWISE, M3.current_index_counts + ENCODER_LIMIT_ZERO/10);
													if(M3.current_index_counts >= MODE_3_SPHERICAL_THUMB_LIMIT){
														if(M3.current_index_counts >= MODE_3_SPHERICAL_THUMB_SAFE){
															if(M1.current_index_counts < MODE_3_SPHERICAL_INDEX_LIMIT){
																if(STEP_UP_M1_VALID)
																		motor_1_set(ACLOCKWISE, M1.current_index_counts + ENCODER_LIMIT_ZERO/10); //-----------------------------CURL INDEX
																}
																if(M2.current_index_counts < MODE_3_SPHERICAL_MIDDLE_LIMIT){
																	if(M3.current_index_counts >= MODE_3_SPHERICAL_RING_AND_LITTLE_LIMIT){
																		if(STEP_UP_M2_VALID)
																			motor_2_set(ACLOCKWISE, M2.current_index_counts + ENCODER_LIMIT_ZERO/10); //----------------------------CURL MIDDLE
																	}
															}
														}
														if(STEP_UP_M4_VALID)
															motor_4_set(ACLOCKWISE, M4.current_index_counts + ENCODER_LIMIT_ZERO/10);  //------------------------------CURL RING AND LITTLE FINGERS
													}
												
												}
												
												else if(KalArm_MODE.mode_3_grip_id == 3){ //mode 3 grip id incremented in pattern routine DON/DOFF
													if(STEP_UP_M3_VALID)
														motor_3_set(ACLOCKWISE, M3.current_index_counts + ENCODER_LIMIT_ZERO/10); //-----------------------------------CURL THUMB DON/DOFF
												}
												
												else if(KalArm_MODE.mode_3_grip_id == 4){ //mode 3 grip id incremented in pattern routine PINCH / FINGER POINT
													if(M4.current_index_counts < MODE_3_PINCH_RING_AND_LITTLE_LIMIT){
														if(STEP_UP_M4_VALID)
															motor_4_set(ACLOCKWISE, M4.current_index_counts+ENCODER_LIMIT_ZERO/10); //-----------------------------CURL RING AND LITTLE INWARDS---------------------------------
													}
													else{
														motor_4_reset();
													}
													if(M2.current_index_counts < MODE_3_PINCH_MIDDLE_LIMIT){
															if(STEP_UP_M2_VALID)
																motor_2_set(ACLOCKWISE, M2.current_index_counts+ENCODER_LIMIT_ZERO/10); //----------------------------CURL  MIDDLE-------------------------------------------------------------------
													}
													else{
														motor_2_reset(); //-------------------------------STOP MIDDLE--------------------------------------------------------------------
													}
													if(M2.current_index_counts >= MODE_3_PINCH_MIDDLE_LIMIT && M3.current_index_counts < MODE_3_PINCH_THUMB_LIMIT){
														if(STEP_UP_M3_VALID)
															motor_3_set(ACLOCKWISE, M3.current_index_counts+ENCODER_LIMIT_ZERO/10); //--------------------------------CURL THUMB-----------------------------------------------------------------
													}
													else{
														motor_3_reset(); //--------------------------------STOP THUMB-----------------------------------------------------------------
													}
													if(M3.current_index_counts >= MODE_3_PINCH_THUMB_LIMIT
														&& M1.current_index_counts <= MODE_3_PINCH_INDEX_LIMIT){  
														if(M1.current_index_counts <= MODE_3_PINCH_INDEX_LIMIT){
															if(STEP_UP_M1_VALID)
																motor_1_set(ACLOCKWISE, M1.current_index_counts+ENCODER_LIMIT_ZERO/10); //----------------------------CURL  INDEX-------------------------------------------------------------------
														}
														else{
																motor_1_reset(); //-------------------------------STOP INDEX--------------------------------------------------------------------
														}
													
													
												
												}
											}
											else if(thumb_position == THUMB_NON_OPPOSITION){/*.............................THUMB NON OPPOSITION.........*/
												if(KalArm_MODE.mode_3_grip_id == 3){ //mode 3 grip id incremented in pattern routine MOUSE
													if(M4.current_index_counts <= MODE_3_MOUSE_THUMB_LIMIT){
														if(STEP_UP_M3_VALID)
															motor_3_set(ACLOCKWISE, M3.current_index_counts + ENCODER_LIMIT_ZERO/10);
													}
													else{
														motor_3_reset();
													}
												}
											}
										}
										break;
										case 3: // MODE 4
											if(thumb_position == THUMB_OPPOSITION){ /*.................................THUMB OPPOSITION.............................*/
																if(KalArm_MODE.mode_4_grip_id == 0){
																	if(M1.current_index_counts<=KalArm_GRIPS[18].motor1_position.target_encoder_index_counts)
																		motor_1_set(ACLOCKWISE, M1.current_index_counts+ENCODER_LIMIT_ZERO/10);
																	if(M2.current_index_counts<=KalArm_GRIPS[18].motor2_position.target_encoder_index_counts)
																		motor_2_set(ACLOCKWISE, M2.current_index_counts+ENCODER_LIMIT_ZERO/10);
																	if(M3.current_index_counts<=KalArm_GRIPS[18].motor3_position.target_encoder_index_counts)
																		motor_3_set(ACLOCKWISE, M3.current_index_counts+ENCODER_LIMIT_ZERO/10);
																	if(M4.current_index_counts<=KalArm_GRIPS[18].motor4_position.target_encoder_index_counts)
																		motor_4_set(ACLOCKWISE, M4.current_index_counts+ENCODER_LIMIT_ZERO/10);
																	
																}
																else if(KalArm_MODE.mode_4_grip_id == 1){
																	if(M1.current_index_counts<=KalArm_GRIPS[19].motor1_position.target_encoder_index_counts)
																		motor_1_set(ACLOCKWISE, M1.current_index_counts+ENCODER_LIMIT_ZERO/10);
																	if(M2.current_index_counts<=KalArm_GRIPS[19].motor2_position.target_encoder_index_counts)
																		motor_2_set(ACLOCKWISE, M2.current_index_counts+ENCODER_LIMIT_ZERO/10);
																	if(M3.current_index_counts<=KalArm_GRIPS[19].motor3_position.target_encoder_index_counts)
																		motor_3_set(ACLOCKWISE, M3.current_index_counts+ENCODER_LIMIT_ZERO/10);
																	if(M4.current_index_counts<=KalArm_GRIPS[19].motor4_position.target_encoder_index_counts)
																		motor_4_set(ACLOCKWISE, M4.current_index_counts+ENCODER_LIMIT_ZERO/10);
																	
																}
																else if(KalArm_MODE.mode_4_grip_id == 2){
																	if(M1.current_index_counts<=KalArm_GRIPS[20].motor1_position.target_encoder_index_counts)
																		motor_1_set(ACLOCKWISE, M1.current_index_counts+ENCODER_LIMIT_ZERO/10);
																	if(M2.current_index_counts<=KalArm_GRIPS[20].motor2_position.target_encoder_index_counts)
																		motor_2_set(ACLOCKWISE, M2.current_index_counts+ENCODER_LIMIT_ZERO/10);
																	if(M3.current_index_counts<=KalArm_GRIPS[20].motor3_position.target_encoder_index_counts)
																		motor_3_set(ACLOCKWISE, M3.current_index_counts+ENCODER_LIMIT_ZERO/10);
																	if(M4.current_index_counts<=KalArm_GRIPS[20].motor4_position.target_encoder_index_counts)
																		motor_4_set(ACLOCKWISE, M4.current_index_counts+ENCODER_LIMIT_ZERO/10);
																}						
											}
											else if(thumb_position == THUMB_NON_OPPOSITION){/*.............................THUMB NON OPPOSITION.........*/
																	if(KalArm_MODE.mode_4_grip_id == 3){
																	if(M1.current_index_counts<=KalArm_GRIPS[21].motor1_position.target_encoder_index_counts)
																		motor_1_set(ACLOCKWISE, M1.current_index_counts+ENCODER_LIMIT_ZERO/10);
																	if(M2.current_index_counts<=KalArm_GRIPS[21].motor2_position.target_encoder_index_counts)
																		motor_2_set(ACLOCKWISE, M2.current_index_counts+ENCODER_LIMIT_ZERO/10);
																	if(M3.current_index_counts<=KalArm_GRIPS[21].motor3_position.target_encoder_index_counts)
																		motor_3_set(ACLOCKWISE, M3.current_index_counts+ENCODER_LIMIT_ZERO/10);
																	if(M4.current_index_counts<=KalArm_GRIPS[21].motor4_position.target_encoder_index_counts)
																		motor_4_set(ACLOCKWISE, M4.current_index_counts+ENCODER_LIMIT_ZERO/10);
																	}
																	else if(KalArm_MODE.mode_4_grip_id == 4){
																		if(M1.current_index_counts<=KalArm_GRIPS[22].motor1_position.target_encoder_index_counts)
																			motor_1_set(ACLOCKWISE, M1.current_index_counts+ENCODER_LIMIT_ZERO/10);
																		if(M2.current_index_counts<=KalArm_GRIPS[22].motor2_position.target_encoder_index_counts)
																			motor_2_set(ACLOCKWISE, M2.current_index_counts+ENCODER_LIMIT_ZERO/10);
																		if(M3.current_index_counts<=KalArm_GRIPS[22].motor3_position.target_encoder_index_counts)
																			motor_3_set(ACLOCKWISE, M3.current_index_counts+ENCODER_LIMIT_ZERO/10);
																		if(M4.current_index_counts<=KalArm_GRIPS[22].motor4_position.target_encoder_index_counts)
																			motor_4_set(ACLOCKWISE, M4.current_index_counts+ENCODER_LIMIT_ZERO/10);
																	}
																	else if(KalArm_MODE.mode_4_grip_id == 5){
																		if(M1.current_index_counts<=KalArm_GRIPS[23].motor1_position.target_encoder_index_counts)
																			motor_1_set(ACLOCKWISE, M1.current_index_counts+ENCODER_LIMIT_ZERO/10);
																		if(M2.current_index_counts<=KalArm_GRIPS[23].motor2_position.target_encoder_index_counts)
																			motor_2_set(ACLOCKWISE, M2.current_index_counts+ENCODER_LIMIT_ZERO/10);
																		if(M3.current_index_counts<=KalArm_GRIPS[23].motor3_position.target_encoder_index_counts)
																			motor_3_set(ACLOCKWISE, M3.current_index_counts+ENCODER_LIMIT_ZERO/10);
																		if(M4.current_index_counts<=KalArm_GRIPS[23].motor4_position.target_encoder_index_counts)
																			motor_4_set(ACLOCKWISE, M4.current_index_counts+ENCODER_LIMIT_ZERO/10);
																	}
											}

			}
			
			
			}
			else if(KalArm_EMG_OPEN.emg_prop_flag == 0 && KalArm_EMG_CLOSE.emg_prop_flag == 0){
				integration_state_number = 5;
				if(KalArm_MODE.mode_number == 0){
					
					if(thumb_position == THUMB_NON_OPPOSITION){
						if(KalArm_EMG_CLOSE.trigger == TRIGGER_SET){ //WHEN TRIGGER IS SET
							while(M1.current_index_counts > ENCODER_LIMIT_ZERO || M2.current_index_counts > ENCODER_LIMIT_ZERO || M4.current_index_counts > ENCODER_LIMIT_ZERO){ // system will not accept any other input during this elastic state and only focus on going back to full extension
								if(STEP_DOWN_M1_VALID)
									motor_1_set(CLOCKWISE, M1.current_index_counts- ENCODER_LIMIT_ZERO/10);
								if(STEP_DOWN_M2_VALID)
									motor_2_set(CLOCKWISE, M2.current_index_counts- ENCODER_LIMIT_ZERO/10);
								if(STEP_DOWN_M4_VALID)
									motor_4_set(CLOCKWISE, M4.current_index_counts- ENCODER_LIMIT_ZERO/10);
								
								if((KalArm_EMG_CLOSE.pattern == DOUBLE_IMPULSE) || (KalArm_EMG_CLOSE.emg_prop_flag)){
									motor_1_reset();
									motor_2_reset();
									motor_4_reset();
									break;
								}
							}

						}
					
					}
				
				}
				else if(KalArm_MODE.mode_number == 1){ // MODE 2 TWO FINGER TRIGGER
					if(thumb_position == THUMB_OPPOSITION){
						if(KalArm_EMG_CLOSE.trigger == TRIGGER_SET){ //WHEN TRIGGER IS SET
							while(M1.current_index_counts > ENCODER_LIMIT_ZERO || M2.current_index_counts > ENCODER_LIMIT_ZERO){ // system will not accept any other input during this elastic state and only focus on going back to full extension
								if(STEP_DOWN_M1_VALID)
									motor_1_set(CLOCKWISE, M1.current_index_counts- ENCODER_LIMIT_ZERO/10);
								if(STEP_DOWN_M2_VALID)
									motor_2_set(CLOCKWISE, M2.current_index_counts- ENCODER_LIMIT_ZERO/10);
								
								if((KalArm_EMG_CLOSE.pattern == DOUBLE_IMPULSE) || (KalArm_EMG_CLOSE.emg_prop_flag)){
									motor_1_reset();
									motor_2_reset();
									break;
								}
							}

						}
					}
				}
				else if(KalArm_MODE.mode_number == 2){ // MODE 3 MOUSE
					if(thumb_position == THUMB_NON_OPPOSITION){
						if(KalArm_EMG_CLOSE.trigger == TRIGGER_SET){ //WHEN TRIGGER IS SET
							while(M1.current_index_counts > ENCODER_LIMIT_ZERO){ // system will not accept any other input during this elastic state and only focus on going back to full extension
								if(STEP_DOWN_M1_VALID)
									motor_1_set(CLOCKWISE, STEP_DOWN_M1);
								if((KalArm_EMG_CLOSE.pattern == DOUBLE_IMPULSE) || (KalArm_EMG_CLOSE.emg_prop_flag)){
									motor_1_reset();
									break;
								}
								
								
								
								}

							}
						}
					}
				
				else{//NORMAL OPERATION
					motor_1_reset();
					motor_2_reset();
					motor_3_reset();
					motor_4_reset();
				}
			
			}
			if(KalArm_EMG_OPEN.pattern != UNDEFINED && KalArm_EMG_CLOSE.pattern == UNDEFINED){
				integration_state_number = 6;
				switch(KalArm_MODE.mode_number){
					case 0: // MODE 1
						if(KalArm_EMG_OPEN.pattern == DOUBLE_IMPULSE){ // MODE 1 DOUBLE IMPULSE
											while(M1.current_index_counts>ENCODER_LIMIT_ZERO || M2.current_index_counts>ENCODER_LIMIT_ZERO || M3.current_index_counts>ENCODER_LIMIT_ZERO || M4.current_index_counts>ENCODER_LIMIT_ZERO){
													if(thumb_position == THUMB_OPPOSITION){
														if(STEP_DOWN_M3_VALID)
															motor_3_set(CLOCKWISE, M3.current_index_counts- ENCODER_LIMIT_ZERO/10);
														if(M3.current_index_counts <= THUMB_OPEN_OPPOSITION_SAFE){
																if(STEP_DOWN_M1_VALID)
																	motor_1_set(CLOCKWISE, M1.current_index_counts- ENCODER_LIMIT_ZERO/10);
																if(STEP_DOWN_M2_VALID)
																	motor_2_set(CLOCKWISE, M2.current_index_counts- ENCODER_LIMIT_ZERO/10);
																if(STEP_DOWN_M4_VALID)
																	motor_4_set(CLOCKWISE, M4.current_index_counts- ENCODER_LIMIT_ZERO/10);
														}
													}
													else if(thumb_position == THUMB_NON_OPPOSITION){
														if(STEP_DOWN_M3_VALID)
															motor_3_set(CLOCKWISE, M3.current_index_counts- ENCODER_LIMIT_ZERO/10);
														if(M3.current_index_counts <= THUMB_OPEN_NON_OPPOSITION_SAFE){
																if(STEP_DOWN_M1_VALID)
																	motor_1_set(CLOCKWISE, M1.current_index_counts- ENCODER_LIMIT_ZERO/10);
																if(STEP_DOWN_M2_VALID)
																	motor_2_set(CLOCKWISE, M2.current_index_counts- ENCODER_LIMIT_ZERO/10);
																if(STEP_DOWN_M4_VALID)
																	motor_4_set(CLOCKWISE, M4.current_index_counts- ENCODER_LIMIT_ZERO/10);
														}
													}
													if(KalArm_EMG_OPEN.emg_prop_flag || KalArm_EMG_CLOSE.emg_prop_flag || KalArm_EMG_CLOSE.pattern != UNDEFINED){
														motor_1_reset();
														motor_2_reset();
														motor_3_reset();
														motor_4_reset();
														break;
													}

												
											}
											KalArm_EMG_OPEN.pattern= UNDEFINED;
											KalArm_EMG_CLOSE.pattern =  UNDEFINED;
						}	
					break;
					case 1: // MODE 2
						if(KalArm_EMG_OPEN.pattern == DOUBLE_IMPULSE){  // MODE 2 DOUBLE IMPULSE
											while(M1.current_index_counts>ENCODER_LIMIT_ZERO || M2.current_index_counts>ENCODER_LIMIT_ZERO || M3.current_index_counts>ENCODER_LIMIT_ZERO || M4.current_index_counts>ENCODER_LIMIT_ZERO){
													if(thumb_position == THUMB_OPPOSITION){
														if(STEP_DOWN_M3_VALID)
															motor_3_set(CLOCKWISE, M3.current_index_counts- ENCODER_LIMIT_ZERO/10);
														if(M3.current_index_counts <= THUMB_OPEN_OPPOSITION_SAFE){
																if(STEP_DOWN_M1_VALID)
																	motor_1_set(CLOCKWISE, M1.current_index_counts- ENCODER_LIMIT_ZERO/10);
																if(STEP_DOWN_M2_VALID)
																	motor_2_set(CLOCKWISE, M2.current_index_counts- ENCODER_LIMIT_ZERO/10);
																if(STEP_DOWN_M4_VALID)
																	motor_4_set(CLOCKWISE, M4.current_index_counts- ENCODER_LIMIT_ZERO/10);
														}
													}
													else if(thumb_position == THUMB_NON_OPPOSITION){
														if(STEP_DOWN_M3_VALID)
															motor_3_set(CLOCKWISE, M3.current_index_counts- ENCODER_LIMIT_ZERO/10);
														if(M3.current_index_counts <= THUMB_OPEN_NON_OPPOSITION_SAFE){
																if(STEP_DOWN_M1_VALID)
																	motor_1_set(CLOCKWISE, M1.current_index_counts- ENCODER_LIMIT_ZERO/10);
																if(STEP_DOWN_M2_VALID)
																	motor_2_set(CLOCKWISE, M2.current_index_counts- ENCODER_LIMIT_ZERO/10);
																if(STEP_DOWN_M4_VALID)
																	motor_4_set(CLOCKWISE, M4.current_index_counts- ENCODER_LIMIT_ZERO/10);
														}
													}
													if(KalArm_EMG_OPEN.emg_prop_flag || KalArm_EMG_CLOSE.emg_prop_flag || KalArm_EMG_CLOSE.pattern != UNDEFINED){
														motor_1_reset();
														motor_2_reset();
														motor_3_reset();
														motor_4_reset();
														break;
													}

												
											}
											KalArm_EMG_OPEN.pattern = UNDEFINED;
											KalArm_EMG_CLOSE.pattern = UNDEFINED;
						}
					break;
					case 2: // MODE 3
						if(KalArm_EMG_OPEN.pattern== DOUBLE_IMPULSE){ // MODE 3 DOUBLE IMPULSE
											while(M1.current_index_counts>ENCODER_LIMIT_ZERO  || M2.current_index_counts>ENCODER_LIMIT_ZERO || M3.current_index_counts>ENCODER_LIMIT_ZERO || M4.current_index_counts>=ENCODER_LIMIT_ZERO){
													if(thumb_position == THUMB_OPPOSITION){
														if(KalArm_MODE.mode_3_grip_id == 2){
															
															if(STEP_DOWN_M1_VALID)
																motor_1_set(CLOCKWISE, M1.current_index_counts- ENCODER_LIMIT_ZERO/10);
															if(STEP_DOWN_M2_VALID)
																motor_2_set(CLOCKWISE, M2.current_index_counts- ENCODER_LIMIT_ZERO/10);
															if(STEP_DOWN_M4_VALID)
																motor_4_set(CLOCKWISE, M4.current_index_counts- ENCODER_LIMIT_ZERO/10);
															if(M1.current_index_counts <= MODE_3_INDEX_SPHERICAL_OPEN_SAFE){
																if(STEP_DOWN_M3_VALID)
																	motor_3_set(CLOCKWISE, M3.current_index_counts- ENCODER_LIMIT_ZERO/10);
															}
														
														
														}
														else{
															if(STEP_DOWN_M3_VALID)
																	motor_3_set(CLOCKWISE, M3.current_index_counts- ENCODER_LIMIT_ZERO/10);
																if(M3.current_index_counts <= THUMB_OPEN_OPPOSITION_SAFE){
																	if(STEP_DOWN_M1_VALID)
																		motor_1_set(CLOCKWISE, M1.current_index_counts- ENCODER_LIMIT_ZERO/10);
																	if(STEP_DOWN_M2_VALID)
																		motor_2_set(CLOCKWISE, M2.current_index_counts- ENCODER_LIMIT_ZERO/10);
																	if(STEP_DOWN_M4_VALID)
																		motor_4_set(CLOCKWISE, M4.current_index_counts- ENCODER_LIMIT_ZERO/10);
															}
														}
													}
													else if(thumb_position == THUMB_NON_OPPOSITION){
														if(STEP_DOWN_M3_VALID)
															motor_3_set(CLOCKWISE, M3.current_index_counts- ENCODER_LIMIT_ZERO/10);
														if(M3.current_index_counts <= THUMB_OPEN_NON_OPPOSITION_SAFE){
																if(STEP_DOWN_M1_VALID)
																	motor_1_set(CLOCKWISE, M1.current_index_counts- ENCODER_LIMIT_ZERO/10);
																if(STEP_DOWN_M2_VALID)
																	motor_2_set(CLOCKWISE, M2.current_index_counts- ENCODER_LIMIT_ZERO/10);
																if(STEP_DOWN_M4_VALID)
																	motor_4_set(CLOCKWISE, M4.current_index_counts- ENCODER_LIMIT_ZERO/10);
														}
													}
													if(KalArm_EMG_OPEN.emg_prop_flag || KalArm_EMG_CLOSE.emg_prop_flag || KalArm_EMG_CLOSE.pattern != UNDEFINED){
														motor_1_reset();
														motor_2_reset();
														motor_3_reset();
														motor_4_reset();
														break;
													}

												
											}
											KalArm_EMG_OPEN.pattern = UNDEFINED;
											KalArm_EMG_CLOSE.pattern = UNDEFINED;
						}
					break;
					case 3: // MODE 4
						if(KalArm_EMG_OPEN.pattern == DOUBLE_IMPULSE){ // MODE 4 DOUBLE IMPULSE
											while(M1.current_index_counts>ENCODER_LIMIT_ZERO || M2.current_index_counts>ENCODER_LIMIT_ZERO || M3.current_index_counts>ENCODER_LIMIT_ZERO || M4.current_index_counts>ENCODER_LIMIT_ZERO){
													if(thumb_position == THUMB_OPPOSITION){
														if(STEP_DOWN_M3_VALID)
															motor_3_set(CLOCKWISE, STEP_DOWN_M3);
														if(M3.current_index_counts < THUMB_OPEN_OPPOSITION_SAFE){
																if(STEP_DOWN_M1_VALID)
																	motor_1_set(CLOCKWISE, STEP_DOWN_M1);
																if(STEP_DOWN_M2_VALID)
																	motor_2_set(CLOCKWISE, STEP_DOWN_M2);
																if(STEP_DOWN_M4_VALID)
																	motor_4_set(CLOCKWISE, STEP_DOWN_M4);
														}
													}
													else if(thumb_position == THUMB_NON_OPPOSITION){
														if(STEP_DOWN_M3_VALID)
															motor_3_set(CLOCKWISE, STEP_DOWN_M3);
														if(M3.current_index_counts < THUMB_OPEN_NON_OPPOSITION_SAFE){
																if(STEP_DOWN_M1_VALID)
																	motor_1_set(CLOCKWISE, STEP_DOWN_M1);
																if(STEP_DOWN_M2_VALID)
																	motor_2_set(CLOCKWISE, STEP_DOWN_M2);
																if(STEP_DOWN_M4_VALID)
																	motor_4_set(CLOCKWISE, STEP_DOWN_M4);
														}
													}
													if(KalArm_EMG_OPEN.emg_prop_flag || KalArm_EMG_CLOSE.emg_prop_flag || KalArm_EMG_CLOSE.pattern != UNDEFINED ){
														motor_1_reset();
														motor_2_reset();
														motor_3_reset();
														motor_4_reset();
														break;
													}

												
											}
											KalArm_EMG_OPEN.pattern = UNDEFINED;
											KalArm_EMG_CLOSE.pattern = UNDEFINED;
						}
					break;
				}

				
			
			}
			
			
			else if(KalArm_EMG_CLOSE.pattern != UNDEFINED && KalArm_EMG_OPEN.pattern == UNDEFINED){				
				integration_state_number = 7;
				switch(KalArm_MODE.mode_number){
					
					case 0: // MODE 1
						if(thumb_position == THUMB_NON_OPPOSITION){
							if(M3.current_index_counts < THUMB_CLOSE_NON_OPPOSITION_SAFE){
								if(KalArm_EMG_CLOSE.pattern== DOUBLE_IMPULSE & KalArm_EMG_CLOSE.trigger == TRIGGER_NOT_SET){ // MODE 1 DOUBLE IMPULSE ON CLOSE TRIGGER SET
									KalArm_EMG_CLOSE.trigger = TRIGGER_SET;
									KalArm_EMG_CLOSE.pattern= UNDEFINED;
								}
								else if(KalArm_EMG_CLOSE.pattern== DOUBLE_IMPULSE & KalArm_EMG_CLOSE.trigger == TRIGGER_SET){ // MODE 1 DOUBLE IMPULSE ON CLOSE TRIGGER RESET
									KalArm_EMG_CLOSE.trigger = TRIGGER_NOT_SET;
									KalArm_EMG_CLOSE.pattern = UNDEFINED;
								}
							}
						}
						else if(thumb_position == THUMB_OPPOSITION){
							KalArm_EMG_CLOSE.trigger = TRIGGER_NOT_SET;
							KalArm_EMG_CLOSE.pattern = UNDEFINED;
						
						}
						
						break;
					case 1:  // MODE 2
						if(thumb_position == THUMB_OPPOSITION){
							if(KalArm_EMG_CLOSE.pattern== DOUBLE_IMPULSE & KalArm_EMG_CLOSE.trigger == TRIGGER_NOT_SET){ // MODE 2 DOUBLE IMPULSE ON CLOSE TRIGGER SET
								KalArm_EMG_CLOSE.trigger = TRIGGER_SET;

								KalArm_EMG_CLOSE.pattern= UNDEFINED;
							}
							else if(KalArm_EMG_CLOSE.pattern== DOUBLE_IMPULSE & KalArm_EMG_CLOSE.trigger == TRIGGER_SET){ // MODE 2 DOUBLE IMPULSE ON CLOSE TRIGGER RESET
								KalArm_EMG_CLOSE.trigger = TRIGGER_NOT_SET;

								KalArm_EMG_CLOSE.pattern = UNDEFINED;
							}
						}
						else if(thumb_position == THUMB_NON_OPPOSITION){
							KalArm_EMG_CLOSE.trigger = TRIGGER_NOT_SET;
							KalArm_EMG_CLOSE.pattern = UNDEFINED;
						}
						break;
					case 2: // MODE 3
						if(KalArm_EMG_CLOSE.pattern == SINGLE_IMPULSE){ // MODE 3 SINGLE IMPULSE to decrement grip id
								if(M1.current_index_counts <= ENCODER_LIMIT_ZERO*4 && M2.current_index_counts <= ENCODER_LIMIT_ZERO*4 && M3.current_index_counts <= ENCODER_LIMIT_ZERO*4 && M4.current_index_counts <= ENCODER_LIMIT_ZERO*4){
									if(KalArm_MODE.mode_3_grip_id == 0)
										KalArm_MODE.mode_3_grip_id = 4;
									else
										KalArm_MODE.mode_3_grip_id --;
									
							}
								KalArm_EMG_CLOSE.pattern=UNDEFINED;
						}
						if(KalArm_EMG_CLOSE.pattern == TRIPLE_IMPULSE){ // MODE 3 TRIPLE IMPULSE to increment  grip id
							if(M1.current_index_counts <= ENCODER_LIMIT_ZERO*4 && M2.current_index_counts <= ENCODER_LIMIT_ZERO*4 && M3.current_index_counts <= ENCODER_LIMIT_ZERO*4 && M4.current_index_counts <= ENCODER_LIMIT_ZERO*4){
								if(KalArm_MODE.mode_3_grip_id == 4)
									KalArm_MODE.mode_3_grip_id = 0;
								else
									KalArm_MODE.mode_3_grip_id ++;
								
							}
							KalArm_EMG_CLOSE.pattern =UNDEFINED;
						}
						if(thumb_position == THUMB_OPPOSITION){
							KalArm_EMG_CLOSE.trigger = TRIGGER_NOT_SET;
							if(KalArm_EMG_CLOSE.pattern == DOUBLE_IMPULSE && KalArm_MODE.mode_3_grip_id == 1){
								//Implement Adaptive Grip here
								do{
								adaptive_routine();
								}while(M1.motor_direction != HALT &&M2.motor_direction != HALT && M3.motor_direction != HALT && M4.motor_direction != HALT);
								KalArm_EMG_CLOSE.pattern = UNDEFINED;
								
							}
						}
						else if(thumb_position == THUMB_NON_OPPOSITION){
							if(KalArm_EMG_CLOSE.pattern== DOUBLE_IMPULSE && KalArm_EMG_CLOSE.trigger == TRIGGER_NOT_SET){
								KalArm_EMG_CLOSE.trigger = TRIGGER_SET;
								KalArm_EMG_CLOSE.pattern= UNDEFINED;
							}
							else if(KalArm_EMG_CLOSE.pattern== DOUBLE_IMPULSE && KalArm_EMG_CLOSE.trigger == TRIGGER_SET){
								KalArm_EMG_CLOSE.trigger = TRIGGER_NOT_SET;
								KalArm_EMG_CLOSE.pattern = UNDEFINED;
							}
						}
						break;
					case 3: // MODE 4
						KalArm_EMG_CLOSE.trigger = TRIGGER_NOT_SET;
						if(KalArm_EMG_CLOSE.pattern == SINGLE_IMPULSE){ // MODE 4 SINGLE IMPULSE to decrement grip id
							if(KalArm_MODE.mode_4_grip_id == 0)
								KalArm_MODE.mode_4_grip_id = 3;
							else
								KalArm_MODE.mode_4_grip_id --;
							KalArm_EMG_CLOSE.pattern= UNDEFINED;
						}
						if(KalArm_EMG_CLOSE.pattern == TRIPLE_IMPULSE){ // MODE 4 TRIPLE IMPULSE to increment grip id
							if(KalArm_MODE.mode_4_grip_id == 3)
								KalArm_MODE.mode_4_grip_id = 0;
							else
								KalArm_MODE.mode_4_grip_id ++;
							KalArm_EMG_CLOSE.pattern = UNDEFINED;
						}
						break;
				
				
				
				
				
				}
			
			}
			
			if(KalArm_MODE.mode_change == 1){
					KalArm_EMG_OPEN.pattern = UNDEFINED;
					KalArm_EMG_CLOSE.pattern = UNDEFINED;
					KalArm_EMG_CLOSE.trigger = TRIGGER_NOT_SET;
					KalArm_MODE.mode_change = 0;
					while(M1.current_index_counts>ENCODER_LIMIT_ZERO || M2.current_index_counts>ENCODER_LIMIT_ZERO || M3.current_index_counts>ENCODER_LIMIT_ZERO || M4.current_index_counts>ENCODER_LIMIT_ZERO){
						if(thumb_position == THUMB_OPPOSITION){
							if(STEP_DOWN_M3_VALID)
								motor_3_set(CLOCKWISE, M3.current_index_counts- ENCODER_LIMIT_ZERO/10);
							if(M3.current_index_counts <= THUMB_OPEN_OPPOSITION_SAFE){
									if(STEP_DOWN_M1_VALID)
										motor_1_set(CLOCKWISE, M1.current_index_counts- ENCODER_LIMIT_ZERO/10);
									if(STEP_DOWN_M2_VALID)
										motor_2_set(CLOCKWISE, M2.current_index_counts- ENCODER_LIMIT_ZERO/10);
									if(STEP_DOWN_M4_VALID)
										motor_4_set(CLOCKWISE, M4.current_index_counts- ENCODER_LIMIT_ZERO/10);
							}
						}
						else if(thumb_position == THUMB_NON_OPPOSITION){
							if(STEP_DOWN_M3_VALID)
								motor_3_set(CLOCKWISE, M3.current_index_counts- ENCODER_LIMIT_ZERO/10);
							if(M3.current_index_counts <= THUMB_OPEN_NON_OPPOSITION_SAFE){
									if(STEP_DOWN_M1_VALID)
										motor_1_set(CLOCKWISE, M1.current_index_counts- ENCODER_LIMIT_ZERO/10);
									if(STEP_DOWN_M2_VALID)
										motor_2_set(CLOCKWISE, M2.current_index_counts- ENCODER_LIMIT_ZERO/10);
									if(STEP_DOWN_M4_VALID)
										motor_4_set(CLOCKWISE, M4.current_index_counts- ENCODER_LIMIT_ZERO/10);
							}
						}
						if((KalArm_EMG_OPEN.emg_prop_flag || KalArm_EMG_CLOSE.emg_prop_flag)){
							motor_1_reset();
							motor_2_reset();
							motor_3_reset();
							motor_4_reset();
							break;
						}

					
				}


				
			
			}
		
		} // End of EMG flag Check
		
		else{ // execute only if EMG Flag is not set
				if(KalArm_GEN_GRIP_STAT_RES.grip_execute_order == 1){			// General Grip Test
					switch(KalArm_GEN_GRIP_STAT_RES.grip_id){
							case 1: grip_test_1_(); // power
							break;
							case 2: grip_test_2_(); // tripod close
							break;
							case 3: grip_test_3_(); // tripod open
							break;
							case 4: grip_test_4_(); // adaptive
							break;
							case 5: grip_test_5_(); // spherical
							break;
							case 6: grip_test_6_(); // don/doff
							break;
							case 7: grip_test_7_(); //  pinch
							break;
							case 8: grip_test_8_(); // Key
							break;
						}
			}
			if(KalArm_CUST_GRIP_STAT_RES.grip_execute_order == 1){  	// 	Custom Grip Test
					execute_custom_grip();
			}
	}

		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_ADC345;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T7_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode 
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_VOPAMP1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T7_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_VOPAMP2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Common config 
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.GainCompensation = 0;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T7_TRGO;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode 
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_VOPAMP3_ADC3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief ADC5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC5_Init(void)
{

  /* USER CODE BEGIN ADC5_Init 0 */

  /* USER CODE END ADC5_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC5_Init 1 */

  /* USER CODE END ADC5_Init 1 */
  /** Common config 
  */
  hadc5.Instance = ADC5;
  hadc5.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc5.Init.Resolution = ADC_RESOLUTION_12B;
  hadc5.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc5.Init.GainCompensation = 0;
  hadc5.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc5.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc5.Init.LowPowerAutoWait = DISABLE;
  hadc5.Init.ContinuousConvMode = DISABLE;
  hadc5.Init.NbrOfConversion = 1;
  hadc5.Init.DiscontinuousConvMode = DISABLE;
  hadc5.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T7_TRGO;
  hadc5.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc5.Init.DMAContinuousRequests = ENABLE;
  hadc5.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc5.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc5) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_VOPAMP4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC5_Init 2 */

  /* USER CODE END ADC5_Init 2 */

}

/**
  * @brief OPAMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP1_Init(void)
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */
	

  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
  hopamp1.Init.Mode = OPAMP_PGA_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO2;
  hopamp1.Init.InternalOutput = ENABLE;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0;
  hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP1_Init 2 */
	if(HAL_OPAMP_DeInit(&hopamp1) != HAL_OK)
	{
	Error_Handler();
	}
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
  hopamp1.Init.Mode = OPAMP_PGA_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO2;
  hopamp1.Init.InternalOutput = ENABLE;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
	hopamp1.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
  hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END OPAMP1_Init 2 */

}

/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void)
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
  hopamp2.Init.Mode = OPAMP_PGA_MODE;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO2;
  hopamp2.Init.InternalOutput = ENABLE;
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp2.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0;
  hopamp2.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP2_Init 2 */
	if(HAL_OPAMP_DeInit(&hopamp2) != HAL_OK)
	{
	Error_Handler();
	}
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
  hopamp2.Init.Mode = OPAMP_PGA_MODE;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO2;
  hopamp2.Init.InternalOutput = ENABLE;
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
	hopamp2.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
  hopamp2.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END OPAMP2_Init 2 */

}

/**
  * @brief OPAMP3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP3_Init(void)
{

  /* USER CODE BEGIN OPAMP3_Init 0 */

  /* USER CODE END OPAMP3_Init 0 */

  /* USER CODE BEGIN OPAMP3_Init 1 */

  /* USER CODE END OPAMP3_Init 1 */
  hopamp3.Instance = OPAMP3;
  hopamp3.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
  hopamp3.Init.Mode = OPAMP_PGA_MODE;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO1;
  hopamp3.Init.InternalOutput = ENABLE;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp3.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0;
  hopamp3.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP3_Init 2 */
	if(HAL_OPAMP_DeInit(&hopamp3) != HAL_OK)
	{
	Error_Handler();
	}
  hopamp3.Instance = OPAMP3;
  hopamp3.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
  hopamp3.Init.Mode = OPAMP_PGA_MODE;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO1;
  hopamp3.Init.InternalOutput = ENABLE;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
	hopamp3.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
  hopamp3.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp3) != HAL_OK)
  {
    Error_Handler();
  }


  /* USER CODE END OPAMP3_Init 2 */

}

/**
  * @brief OPAMP4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP4_Init(void)
{

  /* USER CODE BEGIN OPAMP4_Init 0 */

  /* USER CODE END OPAMP4_Init 0 */

  /* USER CODE BEGIN OPAMP4_Init 1 */

  /* USER CODE END OPAMP4_Init 1 */
  hopamp4.Instance = OPAMP4;
  hopamp4.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
  hopamp4.Init.Mode = OPAMP_PGA_MODE;
  hopamp4.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO2;
  hopamp4.Init.InternalOutput = ENABLE;
  hopamp4.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp4.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0;
  hopamp4.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp4.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP4_Init 2 */
	if(HAL_OPAMP_DeInit(&hopamp4) != HAL_OK)
	{
	Error_Handler();
	}
  hopamp4.Instance = OPAMP4;
  hopamp4.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
  hopamp4.Init.Mode = OPAMP_PGA_MODE;
  hopamp4.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO2;
  hopamp4.Init.InternalOutput = ENABLE;
  hopamp4.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
	hopamp4.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
  hopamp4.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp4.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp4) != HAL_OK)
  {
    Error_Handler();
  }


  /* USER CODE END OPAMP4_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_EncoderIndexConfigTypeDef sEncoderIndexConfig = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 32;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 1;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 1;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sEncoderIndexConfig.Polarity = TIM_ENCODERINDEX_POLARITY_NONINVERTED;
  sEncoderIndexConfig.Prescaler = TIM_ENCODERINDEX_PRESCALER_DIV4;
  sEncoderIndexConfig.Filter = 0;
  sEncoderIndexConfig.FirstIndexEnable = DISABLE;
  sEncoderIndexConfig.Position = TIM_ENCODERINDEX_POSITION_00;
  sEncoderIndexConfig.Direction = TIM_ENCODERINDEX_DIRECTION_UP_DOWN;
  if (HAL_TIMEx_ConfigEncoderIndex(&htim1, &sEncoderIndexConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_EncoderIndexConfigTypeDef sEncoderIndexConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 32;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 1;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 1;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sEncoderIndexConfig.Polarity = TIM_ENCODERINDEX_POLARITY_NONINVERTED;
  sEncoderIndexConfig.Prescaler = TIM_ENCODERINDEX_PRESCALER_DIV4;
  sEncoderIndexConfig.Filter = 0;
  sEncoderIndexConfig.FirstIndexEnable = DISABLE;
  sEncoderIndexConfig.Position = TIM_ENCODERINDEX_POSITION_00;
  sEncoderIndexConfig.Direction = TIM_ENCODERINDEX_DIRECTION_UP_DOWN;
  if (HAL_TIMEx_ConfigEncoderIndex(&htim2, &sEncoderIndexConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_EncoderIndexConfigTypeDef sEncoderIndexConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 32;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 1;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 1;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sEncoderIndexConfig.Polarity = TIM_ENCODERINDEX_POLARITY_NONINVERTED;
  sEncoderIndexConfig.Prescaler = TIM_ENCODERINDEX_PRESCALER_DIV4;
  sEncoderIndexConfig.Filter = 0;
  sEncoderIndexConfig.FirstIndexEnable = DISABLE;
  sEncoderIndexConfig.Position = TIM_ENCODERINDEX_POSITION_00;
  sEncoderIndexConfig.Direction = TIM_ENCODERINDEX_DIRECTION_UP_DOWN;
  if (HAL_TIMEx_ConfigEncoderIndex(&htim3, &sEncoderIndexConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 31;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */
	
	TIM6->CR1 |= TIM_CR1_CEN;
	TIM6->DIER |= TIM_DIER_UIE;

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 31;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_EncoderIndexConfigTypeDef sEncoderIndexConfig = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 32;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 1;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 1;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sEncoderIndexConfig.Polarity = TIM_ENCODERINDEX_POLARITY_NONINVERTED;
  sEncoderIndexConfig.Prescaler = TIM_ENCODERINDEX_PRESCALER_DIV4;
  sEncoderIndexConfig.Filter = 0;
  sEncoderIndexConfig.FirstIndexEnable = DISABLE;
  sEncoderIndexConfig.Position = TIM_ENCODERINDEX_POSITION_00;
  sEncoderIndexConfig.Direction = TIM_ENCODERINDEX_DIRECTION_UP_DOWN;
  if (HAL_TIMEx_ConfigEncoderIndex(&htim8, &sEncoderIndexConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 15;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 15;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 1000;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STAT_MOT_GPIO_Port, STAT_MOT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, M4_PP_Pin|M4_PN_Pin|M3_PP_Pin|M1_PP_Pin 
                          |M1_PN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MS1_Pin|MS2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M2_PP_Pin|M2_PN_Pin|REQ_CLK_Pin|M3_PN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : STAT_MOT_Pin */
  GPIO_InitStruct.Pin = STAT_MOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STAT_MOT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : M4_PP_Pin M4_PN_Pin M3_PP_Pin M1_PP_Pin 
                           M1_PN_Pin */
  GPIO_InitStruct.Pin = M4_PP_Pin|M4_PN_Pin|M3_PP_Pin|M1_PP_Pin 
                          |M1_PN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DEF1_Pin DEF2_Pin */
  GPIO_InitStruct.Pin = DEF1_Pin|DEF2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MS1_Pin MS2_Pin REQ_CLK_Pin */
  GPIO_InitStruct.Pin = MS1_Pin|MS2_Pin|REQ_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : M2_PP_Pin M2_PN_Pin M3_PN_Pin */
  GPIO_InitStruct.Pin = M2_PP_Pin|M2_PN_Pin|M3_PN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PWR_G_Pin */
  GPIO_InitStruct.Pin = PWR_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PWR_G_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void spi_receive_handler(void){
	
	if((rxData_SPI[0] & 0xF0) == HEADER && (rxData_SPI[3] & 0x0F) == FOOTER){
		// Incoming data is actual data from main controller
		switch(COMMAND){
		
			case 0: // Calibration Request
				KalArm_Calibration_Flag = 1;
			break;
			case 1: // EMG Threshold
				
			break;
			case 2: // Pulse Period
				
			break;
			case 3: // Test Custom Grip
				KalArm_EMG_Flag=0;
				switch(CUST_GRIP_MOT_ID){
				
					case 0:
						KalArm_CUST_GRIP_STAT_RES.motor_1_index = (uint16_t)((M1.limit_index_counts*rxData_SPI[1])/88);
						break;
					case 1:
						KalArm_CUST_GRIP_STAT_RES.motor_2_index =  (uint16_t)((M2.limit_index_counts*rxData_SPI[1])/88);
						break;
					case 2:
						KalArm_CUST_GRIP_STAT_RES.motor_3_index =  (uint16_t)((M3.limit_index_counts*rxData_SPI[1])/88);
						break;
					case 3:
						KalArm_CUST_GRIP_STAT_RES.motor_4_index = (uint16_t)((M4.limit_index_counts*rxData_SPI[1])/88);
						break;
					case 4:
						KalArm_CUST_GRIP_STAT_RES.thumb_position = rxData_SPI[1];
						break;
					case 5:
						KalArm_CUST_GRIP_STAT_RES.current_thumb_position = rxData_SPI[1];
						break;
					case 6:
						KalArm_CUST_GRIP_STAT_RES.grip_execute_order = rxData_SPI[1];
						cust_grip_test_execute = KalArm_CUST_GRIP_STAT_RES.grip_execute_order;
						break;
					
				}
				
			break;
			case 4: // Test General Grip
				KalArm_EMG_Flag = 0;
				KalArm_GEN_GRIP_STAT_RES.grip_id = GEN_GRIP_ID;
				KalArm_GEN_GRIP_STAT_RES.grip_execute_order = GEN_GRIP_EXEC_ORDER;
				switch(KalArm_GEN_GRIP_STAT_RES.grip_id){
					case 1:
						grip_test_1_execute = KalArm_GEN_GRIP_STAT_RES.grip_execute_order;
						grip_test_2_execute = 0;
						grip_test_3_execute = 0;
						grip_test_4_execute = 0;
						grip_test_5_execute = 0;
						grip_test_6_execute = 0;
						grip_test_7_execute = 0;
						grip_test_8_execute = 0;
					break;
					case 2:
						grip_test_2_execute = KalArm_GEN_GRIP_STAT_RES.grip_execute_order;
						grip_test_1_execute = 0;
						grip_test_3_execute = 0;
						grip_test_4_execute = 0;
						grip_test_5_execute = 0;
						grip_test_6_execute = 0;
						grip_test_7_execute = 0;
						grip_test_8_execute = 0;
					break;
					case 3:
						grip_test_3_execute = KalArm_GEN_GRIP_STAT_RES.grip_execute_order;
						grip_test_2_execute = 0;
						grip_test_1_execute = 0;
						grip_test_4_execute = 0;
						grip_test_5_execute = 0;
						grip_test_6_execute = 0;
						grip_test_7_execute = 0;
						grip_test_8_execute = 0;
					break;
					case 4:
						grip_test_4_execute = KalArm_GEN_GRIP_STAT_RES.grip_execute_order;
						grip_test_2_execute = 0;
						grip_test_3_execute = 0;
						grip_test_1_execute = 0;
						grip_test_5_execute = 0;
						grip_test_6_execute = 0;
						grip_test_7_execute = 0;
						grip_test_8_execute = 0;
					break;
					case 5:
						grip_test_5_execute = KalArm_GEN_GRIP_STAT_RES.grip_execute_order;
						grip_test_2_execute = 0;
						grip_test_3_execute = 0;
						grip_test_4_execute = 0;
						grip_test_1_execute = 0;
						grip_test_6_execute = 0;
						grip_test_7_execute = 0;
						grip_test_8_execute = 0;
					break;
					case 6:
						grip_test_6_execute = KalArm_GEN_GRIP_STAT_RES.grip_execute_order;
						grip_test_2_execute = 0;
						grip_test_3_execute = 0;
						grip_test_4_execute = 0;
						grip_test_5_execute = 0;
						grip_test_1_execute = 0;
						grip_test_7_execute = 0;
						grip_test_8_execute = 0;
					break;
					case 7:
						grip_test_7_execute = KalArm_GEN_GRIP_STAT_RES.grip_execute_order;
						grip_test_2_execute = 0;
						grip_test_3_execute = 0;
						grip_test_4_execute = 0;
						grip_test_5_execute = 0;
						grip_test_6_execute = 0;
						grip_test_1_execute = 0;
						grip_test_8_execute = 0;
					break;
					case 8:
						grip_test_8_execute = KalArm_GEN_GRIP_STAT_RES.grip_execute_order;
						grip_test_2_execute = 0;
						grip_test_3_execute = 0;
						grip_test_4_execute = 0;
						grip_test_5_execute = 0;
						grip_test_6_execute = 0;
						grip_test_7_execute = 0;
						grip_test_1_execute = 0;
					break;
				}
			break;
			case 5: // Add Custom Grip
				
				
				
			break;
			case 6: // EMG Control
				if(KalArm_EMG_Flag == 0){
					KalArm_release = 1;
				}
				KalArm_EMG_Flag = 1;
				if(KalArm_MODE.mode_number != MODE){
					KalArm_MODE.mode_change = 1;
					KalArm_MODE.mode_number = MODE;
				}
				thumb_position = THUMB_POSITION;
				switch(PAT_PROP){
					case 4:
						KalArm_EMG_OPEN.emg_prop_flag = 1;
						KalArm_EMG_CLOSE.emg_prop_flag = 0;

						break;
					case 8:
						KalArm_EMG_CLOSE.emg_prop_flag = 1;
						KalArm_EMG_OPEN.emg_prop_flag = 0;

						break;
					case 3:
						KalArm_EMG_OPEN.emg_prop_flag = 0;
						KalArm_EMG_CLOSE.emg_prop_flag = 0;
						if(KalArm_EMG_OPEN.pattern_update_event != PAT_UD_O){ // if pattern is updated on open
							KalArm_EMG_OPEN.pattern = PATTERN_OPEN;
							KalArm_EMG_CLOSE.pattern = UNDEFINED;
							KalArm_EMG_OPEN.pattern_update_event = PAT_UD_O;
						}
						if(KalArm_EMG_CLOSE.pattern_update_event != PAT_UD_C){ // if pattern is updated on close
							KalArm_EMG_CLOSE.pattern = PATTERN_CLOSE;
							KalArm_EMG_OPEN.pattern = UNDEFINED;
							KalArm_EMG_CLOSE.pattern_update_event = PAT_UD_C; 
						}

						break;
				
				}
				

			break;		
		}
		
	
	
	}
	
	else if(rxData_SPI[0] == HEADER_DUMMY && rxData_SPI[3] == FOOTER_DUMMY){
		
		// Incoming data is dummy data
	
	}
	



}




void motor_1_set(KalMotorDir direction, uint16_t counts){
	
	if(direction == CLOCKWISE){
		HAL_GPIO_WritePin(M1_PP_GPIO_Port, M1_PP_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M1_PN_GPIO_Port, M1_PN_Pin, GPIO_PIN_RESET);
		M1.motor_direction = CLOCKWISE;
	}
	else if(direction == ACLOCKWISE){
		HAL_GPIO_WritePin(M1_PP_GPIO_Port, M1_PP_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M1_PN_GPIO_Port, M1_PN_Pin, GPIO_PIN_SET);
		M1.motor_direction = ACLOCKWISE;
	}
	else if(direction == CLOCKWISE_LIMIT){
		HAL_GPIO_WritePin(M1_PP_GPIO_Port, M1_PP_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M1_PN_GPIO_Port, M1_PN_Pin, GPIO_PIN_RESET);
		M1.motor_direction = CLOCKWISE_LIMIT;
		
	}
	else if(direction == ACLOCKWISE_LIMIT){
		HAL_GPIO_WritePin(M1_PP_GPIO_Port, M1_PP_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M1_PN_GPIO_Port, M1_PN_Pin, GPIO_PIN_SET);
		M1.motor_direction = ACLOCKWISE_LIMIT;
	}
	else if(direction == ACLOCKWISE_LIMIT_ADAPTIVE){
		HAL_GPIO_WritePin(M1_PP_GPIO_Port, M1_PP_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M1_PN_GPIO_Port, M1_PN_Pin, GPIO_PIN_SET);
		M1.motor_direction = ACLOCKWISE_LIMIT_ADAPTIVE;
	}
	M1.target_encoder_index_counts = counts;
}
void motor_2_set(KalMotorDir direction, uint16_t counts){
	
	if(direction == CLOCKWISE){
		HAL_GPIO_WritePin(M2_PP_GPIO_Port, M2_PP_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M2_PN_GPIO_Port, M2_PN_Pin, GPIO_PIN_RESET);
		M2.motor_direction = CLOCKWISE;
	}
	else if(direction == ACLOCKWISE){
		HAL_GPIO_WritePin(M2_PP_GPIO_Port, M2_PP_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M2_PN_GPIO_Port, M2_PN_Pin, GPIO_PIN_SET);
		M2.motor_direction = ACLOCKWISE;
	}
	else if(direction == CLOCKWISE_LIMIT){
		HAL_GPIO_WritePin(M2_PP_GPIO_Port, M2_PP_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M2_PN_GPIO_Port, M2_PN_Pin, GPIO_PIN_RESET);
		M2.motor_direction =  CLOCKWISE_LIMIT;
	}
	else if(direction == ACLOCKWISE_LIMIT){
		HAL_GPIO_WritePin(M2_PP_GPIO_Port, M2_PP_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M2_PN_GPIO_Port, M2_PN_Pin, GPIO_PIN_SET);
		M2.motor_direction = ACLOCKWISE_LIMIT;
	}
	else if(direction == ACLOCKWISE_LIMIT_ADAPTIVE){
		HAL_GPIO_WritePin(M2_PP_GPIO_Port, M2_PP_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M2_PN_GPIO_Port, M2_PN_Pin, GPIO_PIN_SET);
		M2.motor_direction = ACLOCKWISE_LIMIT_ADAPTIVE;
	}
	
	M2.target_encoder_index_counts = counts;
}
void motor_3_set(KalMotorDir direction, uint16_t counts){
	if(direction ==  ACLOCKWISE){
		HAL_GPIO_WritePin(M3_PP_GPIO_Port, M3_PP_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M3_PN_GPIO_Port, M3_PN_Pin, GPIO_PIN_RESET);
		M3.motor_direction = ACLOCKWISE;
	}
	else if(direction ==  CLOCKWISE){
		HAL_GPIO_WritePin(M3_PP_GPIO_Port, M3_PP_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M3_PN_GPIO_Port, M3_PN_Pin, GPIO_PIN_SET);
		M3.motor_direction = CLOCKWISE;
	}
	else if(direction ==  ACLOCKWISE_LIMIT){
		HAL_GPIO_WritePin(M3_PP_GPIO_Port, M3_PP_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M3_PN_GPIO_Port, M3_PN_Pin, GPIO_PIN_RESET);
		M3.motor_direction = ACLOCKWISE_LIMIT;
	}
	else if(direction ==  CLOCKWISE_LIMIT){
		HAL_GPIO_WritePin(M3_PP_GPIO_Port, M3_PP_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M3_PN_GPIO_Port, M3_PN_Pin, GPIO_PIN_SET);
		M3.motor_direction = CLOCKWISE_LIMIT;
	}
	else if(direction ==  ACLOCKWISE_LIMIT){
		HAL_GPIO_WritePin(M3_PP_GPIO_Port, M3_PP_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M3_PN_GPIO_Port, M3_PN_Pin, GPIO_PIN_SET);
		M3.motor_direction = CLOCKWISE_LIMIT;
	}
	else if(direction ==  ACLOCKWISE_LIMIT_ADAPTIVE){
		HAL_GPIO_WritePin(M3_PP_GPIO_Port, M3_PP_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M3_PN_GPIO_Port, M3_PN_Pin, GPIO_PIN_RESET);
		M3.motor_direction = ACLOCKWISE_LIMIT_ADAPTIVE;
	}
	
	M3.target_encoder_index_counts = counts;
}
void motor_4_set(KalMotorDir direction, uint16_t counts){
	if(direction == CLOCKWISE){
		HAL_GPIO_WritePin(M4_PP_GPIO_Port, M4_PP_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M4_PN_GPIO_Port, M4_PN_Pin, GPIO_PIN_RESET);
		M4.motor_direction = CLOCKWISE;
	}
	else if(direction == ACLOCKWISE){
		HAL_GPIO_WritePin(M4_PP_GPIO_Port, M4_PP_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M4_PN_GPIO_Port, M4_PN_Pin, GPIO_PIN_SET);
		M4.motor_direction = ACLOCKWISE;
	}
	else if(direction == CLOCKWISE_LIMIT){
		HAL_GPIO_WritePin(M4_PP_GPIO_Port, M4_PP_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M4_PN_GPIO_Port, M4_PN_Pin, GPIO_PIN_RESET);
		M4.motor_direction = CLOCKWISE_LIMIT;
	}
	else if(direction == ACLOCKWISE_LIMIT){
		HAL_GPIO_WritePin(M4_PP_GPIO_Port, M4_PP_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M4_PN_GPIO_Port, M4_PN_Pin, GPIO_PIN_SET);
		M4.motor_direction = ACLOCKWISE_LIMIT;
	}
	else if(direction == ACLOCKWISE_LIMIT_ADAPTIVE){
		HAL_GPIO_WritePin(M4_PP_GPIO_Port, M4_PP_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M4_PN_GPIO_Port, M4_PN_Pin, GPIO_PIN_SET);
		M4.motor_direction = ACLOCKWISE_LIMIT_ADAPTIVE;
	}
	
	M4.target_encoder_index_counts = counts;
}


void motor_1_reset(){
	
	HAL_GPIO_WritePin(M1_PP_GPIO_Port, M1_PP_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(M1_PN_GPIO_Port, M1_PN_Pin, GPIO_PIN_SET);
	if(M1.motor_direction == CLOCKWISE_LIMIT){		
			M1.current_index_counts = 0;
			M1.encoder_handle->Instance->CNT = 0;
	}
	else if(M1.motor_direction == ACLOCKWISE_LIMIT){
		M1.limit_index_counts = M1.current_index_counts - 100;
	}
	
	M1.motor_direction = HALT;


}
void motor_2_reset(){

		HAL_GPIO_WritePin(M2_PP_GPIO_Port, M2_PP_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M2_PN_GPIO_Port, M2_PN_Pin, GPIO_PIN_SET);
		if(M2.motor_direction == CLOCKWISE_LIMIT){		
				M2.current_index_counts = 0;
				M2.encoder_handle->Instance->CNT = 0;
		}	
		else if(M2.motor_direction == ACLOCKWISE_LIMIT){
			M2.limit_index_counts = M2.current_index_counts - 100;
		}
		M2.motor_direction = HALT;

}
void motor_3_reset(){
	
	HAL_GPIO_WritePin(M3_PP_GPIO_Port, M3_PP_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(M3_PN_GPIO_Port, M3_PN_Pin, GPIO_PIN_SET);
	if(M3.motor_direction == CLOCKWISE_LIMIT){		
			M3.current_index_counts = 0;
			M3.encoder_handle->Instance->CNT = 0;
	}
	else if(M3.motor_direction == ACLOCKWISE_LIMIT){
			M3.limit_index_counts = M3.current_index_counts - 100;
		}
	
	M3.motor_direction = HALT;

}
void motor_4_reset(){

	HAL_GPIO_WritePin(M4_PP_GPIO_Port, M4_PP_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(M4_PN_GPIO_Port, M4_PN_Pin, GPIO_PIN_SET);
	if(M4.motor_direction == CLOCKWISE_LIMIT){		
			M4.current_index_counts = 0;
			M4.encoder_handle->Instance->CNT = 0;
	}
	else if(M4.motor_direction == ACLOCKWISE_LIMIT){
			M4.limit_index_counts = M4.current_index_counts - 100;
			//	M4.limit_index_counts = ENCODER_LIMIT_RING_LITTLE;
		}
	M4.motor_direction = HALT;

}

void current_sense_init(uint8_t init_deinit){
	
	if(init_deinit){
		HAL_TIM_Base_Start_IT(&htim7);
	}
	else{
	
		HAL_TIM_Base_Stop_IT(&htim7);
	}
	

}

void motor_4_encoder_config_init(uint8_t init_deinit){
	
	TIM2->DIER |= TIM_DIER_IDXIE;
	TIM2->DIER |= TIM_DIER_DIRIE;
	TIM2->DIER |= TIM_DIER_UIE;
	
	if(init_deinit){
		HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
	}
	else{
		HAL_TIM_Encoder_Stop_IT(&htim2, TIM_CHANNEL_ALL);
	}

}
void motor_3_encoder_config_init(uint8_t init_deinit){
	
	TIM1->DIER |= TIM_DIER_IDXIE;
	TIM1->DIER |= TIM_DIER_DIRIE;
	TIM1->DIER |= TIM_DIER_UIE;
	
	if(init_deinit){
		HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);
	}
	else{
		HAL_TIM_Encoder_Stop_IT(&htim1, TIM_CHANNEL_ALL);
	}
	

}
void motor_2_encoder_config_init(uint8_t init_deinit){
		
	TIM3->DIER |= TIM_DIER_IDXIE;
	TIM3->DIER |= TIM_DIER_DIRIE;
	TIM3->DIER |= TIM_DIER_UIE;
	
	if(init_deinit){
		HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
	}
	else{
		HAL_TIM_Encoder_Stop_IT(&htim3, TIM_CHANNEL_ALL);
	}

}
void motor_1_encoder_config_init(uint8_t init_deinit){
	
	TIM8->DIER |= TIM_DIER_IDXIE;
	TIM8->DIER |= TIM_DIER_DIRIE;
	TIM8->DIER |= TIM_DIER_UIE;
	
	if(init_deinit){
		HAL_TIM_Encoder_Start_IT(&htim8, TIM_CHANNEL_ALL);
	}
	else{
		HAL_TIM_Encoder_Stop_IT(&htim8, TIM_CHANNEL_ALL);
	}

}

void motors_init(uint8_t init_deinit){
	
	if(init_deinit){
		
			HAL_GPIO_WritePin(MS2_GPIO_Port, MS2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MS1_GPIO_Port, MS1_Pin, GPIO_PIN_SET);

	
	}
	else{
			HAL_GPIO_WritePin(MS2_GPIO_Port, MS2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MS1_GPIO_Port, MS1_Pin, GPIO_PIN_RESET);

	
	
	}


}


void motor_4_current_config_init(uint8_t init_deinit){
	
	  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

		ADC_Enable(&hadc1);
    HAL_OPAMP_SelfCalibrate(&hopamp1);

		
    HAL_OPAMP_Start(&hopamp1);

	
		if(init_deinit){
			HAL_ADC_Start_DMA(&hadc1, &M4.current_current_sense, 1);

		}
		else{
			HAL_ADC_Stop_DMA(&hadc1);

		}
		HAL_Delay(10);
}
void motor_3_current_config_init(uint8_t init_deinit){
	
		HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	
		ADC_Enable(&hadc2);
    HAL_OPAMP_SelfCalibrate(&hopamp2);
    HAL_OPAMP_Start(&hopamp2);
	
		if(init_deinit){
			HAL_ADC_Start_DMA(&hadc2, &M3.current_current_sense, 1);
		}
		else{
			HAL_ADC_Stop_DMA(&hadc2);
		}	
		HAL_Delay(10);

}
void motor_2_current_config_init(uint8_t init_deinit){
	
		HAL_ADCEx_Calibration_Start(&hadc5, ADC_SINGLE_ENDED);
	
		ADC_Enable(&hadc5);
    HAL_OPAMP_SelfCalibrate(&hopamp4);
    HAL_OPAMP_Start(&hopamp4);
		if(init_deinit)
			HAL_ADC_Start_DMA(&hadc5, &M2.current_current_sense, 1);
		else
			HAL_ADC_Stop_DMA(&hadc5);
			HAL_Delay(10);
	


}
void motor_1_current_config_init(uint8_t init_deinit){
			
		HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
	
		ADC_Enable(&hadc3);
    HAL_OPAMP_SelfCalibrate(&hopamp3);
    HAL_OPAMP_Start(&hopamp3);
		if(init_deinit)
			HAL_ADC_Start_DMA(&hadc3, &M1.current_current_sense, 1);
		else
			HAL_ADC_Stop_DMA(&hadc3);
		HAL_Delay(10);

}

void calibration_routine(void){
			
	if(KalArm_Calibration_times <= 2){
			/*This piece of code is to indicate to the master that calibration is in progress*/
//			txData_SPI[0]=0x90;
//			txData_SPI[1]=0x00;
//			txData_SPI[1] |= 1 << 4; // Insert "Calibration in Progress" bit at the 20th index position of the 32 bit packet
//			txData_SPI[2]=0x00;
//			txData_SPI[3]=0x0B;

		
				txData_SPI[0] =0x90;
				txData_SPI[1] = 0x00;
				txData_SPI[1] |= KalArm_Calibration_times <<5;
				txData_SPI[2]=0x00; // Make 0 before inserting motor and motor current sense fail bits
				txData_SPI[3]=0x0B; // Make 0 before inserting Calibration fail, Calibration Command and footer
				if(KalArm_Calibration_times <=2){
					txData_SPI[1] |= 1 << 4; // Insert "Calibration in Progress" bit at the 20th index position of the 32 bit packet
				}


			
			if(HAL_GPIO_ReadPin(REQ_CLK_GPIO_Port, REQ_CLK_Pin)== GPIO_PIN_SET){
					HAL_GPIO_WritePin(REQ_CLK_GPIO_Port, REQ_CLK_Pin, GPIO_PIN_RESET); // send data only when not receiving data from master
			}
			
			
	
			do{ // Motor 3 Close Thumb
				motor_3_set(ACLOCKWISE_LIMIT, 0);
			}while(M3.motor_direction != HALT);
			
			HAL_Delay(100);
			do{ // Motor 3 Open Thumb
				
				motor_3_set(CLOCKWISE_LIMIT, 0);
			}while(M3.motor_direction != HALT);
			HAL_Delay(100);
			do{ // Motor 1 Close Index
		
				motor_1_set(ACLOCKWISE_LIMIT, 0);

			}while(M1.motor_direction != HALT);
			HAL_Delay(100);
			do{ // Motor 1 Open Index
				
				motor_1_set(CLOCKWISE_LIMIT, 0);
			}while(M1.motor_direction != HALT);
			HAL_Delay(100);
			do{ // Motor 2 Close Middle
		
				motor_2_set(ACLOCKWISE_LIMIT, 0);
			}while(M2.motor_direction != HALT);
			HAL_Delay(100);
			do{ // Motor 2 Open Middle
				
				motor_2_set(CLOCKWISE_LIMIT, 0);
			}while(M2.motor_direction != HALT);
			
			HAL_Delay(100);
			do{ // Motor 4 Close Ring and Little
		
				motor_4_set(ACLOCKWISE_LIMIT, 0);
			}while(M4.motor_direction != HALT);
			HAL_Delay(100);
			do{ // Motor 4 Open Ring and Little
	
				motor_4_set(CLOCKWISE_LIMIT, 0);
			}while(M4.motor_direction != HALT);
			HAL_Delay(100);
			
			if(CAL_P_F){
				txData_SPI[1]=0x00; // Make Calibration in progress bit 0 and encoders pass
				txData_SPI[2]=0x00; // Motors and Motor Current Sense Pass
				txData_SPI[3]=0x0B; // Calibration Command, Calibration Pass, and footer
			}
			else{
					if(KalArm_Calibration_times == 2){
						txData_SPI[3] |= 1<<7; // Insert Calibration failed
						txData_SPI[1] &= ~(1 << 4); // calibration not in progress
					}
					else{
						txData_SPI[1] |= (1<<4);
					}
				if(CAL_M1_F){
					txData_SPI[2] |= 1<<0;
					if(CAL_M1_CS_F)
						txData_SPI[2] |= 1<<4;
					if(CAL_M1_EN_F)
						txData_SPI[1] |= 1<<0;
				}
				if(CAL_M2_F){
					txData_SPI[2] |= 1<<1;
					if(CAL_M2_CS_F)
						txData_SPI[2] |= 1<<5;
					if(CAL_M2_EN_F)
						txData_SPI[1] |= 1<<1;
				}
				if(CAL_M3_F){
					txData_SPI[2] |= 1<<2;
					if(CAL_M3_CS_F)
						txData_SPI[2] |= 1<<6;
					if(CAL_M3_EN_F)
						txData_SPI[1] |= 1<<2;
				}
				if(CAL_M4_F){
					txData_SPI[2] |= 1<<3;
					if(CAL_M4_CS_F)
						txData_SPI[2] |= 1<<7;
					if(CAL_M4_EN_F)
						txData_SPI[1] |= 1<<3;
				}
			
			}
			

		KalArm_Calibration_times++;	

		}
			HAL_Delay(200);

			
}

void adaptive_filter(uint8_t adaptive_factor){

	if(M1.motor_direction == CLOCKWISE_LIMIT_ADAPTIVE || M1.motor_direction == ACLOCKWISE_LIMIT_ADAPTIVE){
		
		if(M1.current_current_sense > CS_LIMIT ){
			if(M1.current_cutoff_pre_stamp == 0){
				M1.current_cutoff_pre_stamp = msTicks_T_7;
			}
			
			M1.current_check_filter_stamp +=msTicks_T_7;
			M1.current_cutoff_window++;
		}
		
		i=(M1.current_check_filter_stamp/M1.current_cutoff_window);
		j=(M1.current_cutoff_pre_stamp);

		
		if(M1.current_cutoff_window == adaptive_factor ){ // thousand instances of high current sense
			motor_1_reset();
			M1.current_cutoff_window = 0;
			M1.current_cutoff_pre_stamp = 0;
			M1.current_check_filter_stamp = 0;
		}
		
	}
	
	if(M2.motor_direction == CLOCKWISE_LIMIT_ADAPTIVE || M2.motor_direction == ACLOCKWISE_LIMIT_ADAPTIVE){
		
		if(M2.current_current_sense > CS_LIMIT ){
			if(M2.current_cutoff_pre_stamp == 0){
				M2.current_cutoff_pre_stamp = msTicks_T_7;
			}
			
			M2.current_check_filter_stamp +=msTicks_T_7;
			M2.current_cutoff_window++;
		}
		
		i=(M2.current_check_filter_stamp/M2.current_cutoff_window);
		j=(M2.current_cutoff_pre_stamp);

		
		if(M2.current_cutoff_window == adaptive_factor){ // thousand instances of high current sense
			motor_2_reset();
			M2.current_cutoff_window = 0;
			M2.current_cutoff_pre_stamp = 0;
			M2.current_check_filter_stamp = 0;
		}
		
	}
	
	if(M3.motor_direction == CLOCKWISE_LIMIT_ADAPTIVE || M3.motor_direction == ACLOCKWISE_LIMIT_ADAPTIVE){
		
		if(M3.current_current_sense > 2*CS_LIMIT ){
			if(M3.current_cutoff_pre_stamp == 0){
				M3.current_cutoff_pre_stamp = msTicks_T_7;
			}
			
			M3.current_check_filter_stamp +=msTicks_T_7;
			M3.current_cutoff_window++;
		}
		
		i=(M3.current_check_filter_stamp/M3.current_cutoff_window);
		j=(M3.current_cutoff_pre_stamp);

		
		if(M3.current_cutoff_window == 4*adaptive_factor ){ // thousand instances of high current sense
			motor_3_reset();
			M3.current_cutoff_window = 0;
			M3.current_cutoff_pre_stamp = 0;
			M3.current_check_filter_stamp = 0;
		}
		
	}
	if(M4.motor_direction == CLOCKWISE_LIMIT_ADAPTIVE || M4.motor_direction == ACLOCKWISE_LIMIT_ADAPTIVE){
		
		if(M4.current_current_sense > CS_LIMIT ){
			if(M4.current_cutoff_pre_stamp == 0){
				M4.current_cutoff_pre_stamp = msTicks_T_7;
			}
			
			M4.current_check_filter_stamp +=msTicks_T_7;
			M4.current_cutoff_window++;
		}
		
		i=(M4.current_check_filter_stamp/M4.current_cutoff_window);
		j=(M4.current_cutoff_pre_stamp);

		
		if(M4.current_cutoff_window == adaptive_factor){ // thousand instances of high current sense
			motor_4_reset();
			M4.current_cutoff_window = 0;
			M4.current_cutoff_pre_stamp = 0;
			M4.current_check_filter_stamp = 0;
		}
		
	}







}

void calibration_filter(uint8_t calibration_factor){

	if(M1.motor_direction == CLOCKWISE_LIMIT || M1.motor_direction == ACLOCKWISE_LIMIT){
		
		if(M1.current_current_sense > CS_LIMIT ){
			if(M1.current_cutoff_pre_stamp == 0){
				M1.current_cutoff_pre_stamp = msTicks_T_7;
			}
			
			M1.current_check_filter_stamp +=msTicks_T_7;
			M1.current_cutoff_window++;
		}
		
		i=(M1.current_check_filter_stamp/M1.current_cutoff_window);
		j=(M1.current_cutoff_pre_stamp);

		
		if(M1.current_cutoff_window == 2*calibration_factor){ // thousand instances of high current sense
			motor_1_reset();
			M1.current_cutoff_window = 0;
			M1.current_cutoff_pre_stamp = 0;
			M1.current_check_filter_stamp = 0;
		}
		
	}
	
	if(M2.motor_direction == CLOCKWISE_LIMIT || M2.motor_direction == ACLOCKWISE_LIMIT){
		
		if(M2.current_current_sense > CS_LIMIT ){
			if(M2.current_cutoff_pre_stamp == 0){
				M2.current_cutoff_pre_stamp = msTicks_T_7;
			}
			
			M2.current_check_filter_stamp +=msTicks_T_7;
			M2.current_cutoff_window++;
		}
		
		i=(M2.current_check_filter_stamp/M2.current_cutoff_window);
		j=(M2.current_cutoff_pre_stamp);

		
		if(M2.current_cutoff_window == 2*calibration_factor){ // thousand instances of high current sense
			motor_2_reset();
			M2.current_cutoff_window = 0;
			M2.current_cutoff_pre_stamp = 0;
			M2.current_check_filter_stamp = 0;
		}
		
	}
	
	if(M3.motor_direction == CLOCKWISE_LIMIT || M3.motor_direction == ACLOCKWISE_LIMIT){
		
		if(M3.current_current_sense > CS_LIMIT ){
			if(M3.current_cutoff_pre_stamp == 0){
				M3.current_cutoff_pre_stamp = msTicks_T_7;
			}
			
			M3.current_check_filter_stamp +=msTicks_T_7;
			M3.current_cutoff_window++;
		}
		
		i=(M3.current_check_filter_stamp/M3.current_cutoff_window);
		j=(M3.current_cutoff_pre_stamp);

		
		if(M3.current_cutoff_window == 3*calibration_factor){ // thousand instances of high current sense
			motor_3_reset();
			M3.current_cutoff_window = 0;
			M3.current_cutoff_pre_stamp = 0;
			M3.current_check_filter_stamp = 0;
		}
		
	}
	if(M4.motor_direction == CLOCKWISE_LIMIT || M4.motor_direction == ACLOCKWISE_LIMIT){
		
		if(M4.current_current_sense > CS_LIMIT ){
			if(M4.current_cutoff_pre_stamp == 0){
				M4.current_cutoff_pre_stamp = msTicks_T_7;
			}
			
			M4.current_check_filter_stamp +=msTicks_T_7;
			M4.current_cutoff_window++;
		}
		
		i=(M4.current_check_filter_stamp/M4.current_cutoff_window);
		j=(M4.current_cutoff_pre_stamp);

		
		if(M4.current_cutoff_window == 4*calibration_factor){ // thousand instances of high current sense
			motor_4_reset();
			M4.current_cutoff_window = 0;
			M4.current_cutoff_pre_stamp = 0;
			M4.current_check_filter_stamp = 0;
		}
		
	}



}

void finger_current_cutoff(uint8_t cutoff_factor){

		if(M1.motor_direction == CLOCKWISE || M1.motor_direction == ACLOCKWISE){
		
		if(M1.current_current_sense > CS_LIMIT && ((M1.current_index_counts <= 5*ENCODER_LIMIT_ZERO) || (M1.current_index_counts >= ENCODER_LIMIT_INDEX_CO))){

			if(M1.current_diff == 0)
				M1.current_cutoff_window++;
		}
				
		if(M1.current_cutoff_window == cutoff_factor){ // thousand instances of high current sense
			if(M1.current_index_counts <= 5* ENCODER_LIMIT_ZERO)
				M1.current_index_counts = ENCODER_LIMIT_ZERO;
			else if(M1.current_index_counts >= ENCODER_LIMIT_INDEX_CO)
				M1.current_index_counts = M1.limit_index_counts;
			M1.target_encoder_index_counts = M1.current_index_counts;
			motor_1_reset();
			M1.current_cutoff_window = 0;
			M1.current_cutoff_pre_stamp = 0;
			M1.current_check_filter_stamp = 0;
		}
		
	}
	
	if(M2.motor_direction == CLOCKWISE || M2.motor_direction == ACLOCKWISE){
		
		if(M2.current_current_sense > CS_LIMIT && ((M2.current_index_counts <= 5*ENCODER_LIMIT_ZERO) || (M2.current_index_counts >= ENCODER_LIMIT_MIDDLE_CO))){
			
			
			if(M2.current_diff == 0)
					M2.current_cutoff_window++;
		}
		
			if(M2.current_cutoff_window == cutoff_factor){ // thousand instances of high current sense
				if(M2.current_index_counts <= 5* ENCODER_LIMIT_ZERO)
					M2.current_index_counts = ENCODER_LIMIT_ZERO;
				else if(M2.current_index_counts >= ENCODER_LIMIT_INDEX_CO)
					M2.current_index_counts = M2.limit_index_counts;
				M2.target_encoder_index_counts = M2.current_index_counts;
				motor_2_reset();
				M2.current_cutoff_window = 0;
				M2.current_cutoff_pre_stamp = 0;
				M2.current_check_filter_stamp = 0;
			}
		
	}
	
	if(M3.motor_direction == CLOCKWISE || M3.motor_direction == ACLOCKWISE){
		
		if(M3.current_current_sense > CS_LIMIT  && ((M3.current_index_counts <= 5*ENCODER_LIMIT_ZERO) || (M3.current_index_counts >= ENCODER_LIMIT_THUMB_CO))){
			
			if(M3.current_diff == 0)
				M3.current_cutoff_window++;
		}
		
		if(M3.current_cutoff_window == cutoff_factor ){ // thousand instances of high current sense
			if(M3.current_index_counts <= 5* ENCODER_LIMIT_ZERO)
				M3.current_index_counts = ENCODER_LIMIT_ZERO;
			else if(M3.current_index_counts >= ENCODER_LIMIT_INDEX_CO)
				M3.current_index_counts = M3.limit_index_counts;
			M3.target_encoder_index_counts = M3.current_index_counts;
			motor_3_reset();
			M3.current_cutoff_window = 0;
			M3.current_cutoff_pre_stamp = 0;
			M3.current_check_filter_stamp = 0;
		}
		
	}
	if(M4.motor_direction == CLOCKWISE || M4.motor_direction == ACLOCKWISE){
		
		if(M4.current_current_sense > CS_LIMIT && ((M4.current_index_counts <= 5*ENCODER_LIMIT_ZERO) || (M4.current_index_counts >= ENCODER_LIMIT_RING_LITTLE_CO))){
			
			if(M4.current_diff == 0)
				M4.current_cutoff_window++;
		}
		

		
		if(M4.current_cutoff_window == 2*cutoff_factor ){ // thousand instances of high current sense
			if(M4.current_index_counts <= 5* ENCODER_LIMIT_ZERO)
				M4.current_index_counts = ENCODER_LIMIT_ZERO;
			else if(M4.current_index_counts >= ENCODER_LIMIT_INDEX_CO)
				M4.current_index_counts = M4.limit_index_counts;
			M4.target_encoder_index_counts = M4.current_index_counts;
			motor_4_reset();
			M4.current_cutoff_window = 0;
			M4.current_cutoff_pre_stamp = 0;
			M4.current_check_filter_stamp = 0;
		}
		
	}






}



void adaptive_routine(void){
	
	motor_1_set(ACLOCKWISE_LIMIT_ADAPTIVE, 0);
	motor_2_set(ACLOCKWISE_LIMIT_ADAPTIVE, 0);
	motor_4_set(ACLOCKWISE_LIMIT_ADAPTIVE, 0);
	HAL_Delay(200);
	motor_3_set(ACLOCKWISE_LIMIT_ADAPTIVE, 0);
}

void grip_test_1_(void){ // power grip
	
		
			while(M1.current_index_counts < M1.limit_index_counts-25 || M2.current_index_counts < M2.limit_index_counts-25 || M3.current_index_counts < 2600 || M4.current_index_counts < M4.limit_index_counts-25){
				integration_state_number = 9;

				motor_1_set(ACLOCKWISE, M1.limit_index_counts-25);
				motor_2_set(ACLOCKWISE, M2.limit_index_counts-25);
				motor_4_set(ACLOCKWISE, M4.limit_index_counts-25);

				if(M1.current_index_counts > MODE_1_INDEX_ADDUCTION_THRESHOLD && M2.current_index_counts > MODE_1_MIDDLE_ADDUCTION_THRESHOLD){
						if(M3.current_index_counts < MODE_1_THUMB_ADDUCTION_LIMIT){
								motor_3_set(ACLOCKWISE, 2700);
						}
				}

				
				txData_SPI[0]=0x90;
				txData_SPI[1]=0x00;
				txData_SPI[2]=0x00;
				txData_SPI[3]=0x0B;
				txData_SPI[3]|=(1<<4) | (1<<5); // gen grip test command
				txData_SPI[1] |=1<<5; // gen grip test in progress
				HAL_Delay(200);
				if(M1.current_diff == 0 && M2.current_diff == 0 && M4.current_diff == 0 && M3.current_diff == 0){
					if(M3.current_index_counts < 50)
						motor_3_set(ACLOCKWISE, 2700);
					else
						break;
					}
				if(!grip_test_1_execute)
					break;

				
			} // end of while to CLOSE
			while(grip_test_1_execute){
				txData_SPI[0]=0x90;
				txData_SPI[1]=0x00;
				txData_SPI[2]=0x00;
				txData_SPI[3]=0x0B;
				txData_SPI[3]|=(1<<4) | (1<<5); // gen grip test command
				txData_SPI[1] |=1<<4; // gen grip test passed
				M1_ACCURACY= (uint8_t)(((float)M1.current_index_counts/M1.limit_index_counts)*100);
				M2_ACCURACY=(uint8_t)(((float)M2.current_index_counts/M2.limit_index_counts)*100);
				M3_ACCURACY=(uint8_t)(((float)M3.current_index_counts/M3.limit_index_counts)*100);
				M4_ACCURACY=(uint8_t)(((float)M4.current_index_counts/M4.limit_index_counts)*100);
				KalArm_GEN_GRIP_STAT_RES.grip_accuracy = GRIP_TEST_ACCURACY;
				txData_SPI[3] |= (1 & GRIP_TEST_ACCURACY) <<7;
				txData_SPI[2] |=  (0x3F & (GRIP_TEST_ACCURACY>>1));
				if(KalArm_release){ // include routine to execute on different grip test
					grip_test_1_execute = 0;
					break;
				}
			}			
			
			while(M1.current_index_counts>ENCODER_LIMIT_ZERO*2 || M2.current_index_counts>ENCODER_LIMIT_ZERO*2 || M3.current_index_counts>ENCODER_LIMIT_ZERO*2 || M4.current_index_counts>ENCODER_LIMIT_ZERO*2){
					if(STEP_DOWN_M3_VALID)
						motor_3_set(CLOCKWISE, STEP_DOWN_M3);
					if(M3.current_index_counts <= THUMB_OPEN_OPPOSITION_SAFE){
							if(STEP_DOWN_M1_VALID)
								motor_1_set(CLOCKWISE, STEP_DOWN_M1);
							if(STEP_DOWN_M2_VALID)
								motor_2_set(CLOCKWISE, STEP_DOWN_M2);
							if(STEP_DOWN_M4_VALID)
								motor_4_set(CLOCKWISE, STEP_DOWN_M4);
						}
					txData_SPI[0]=0x90;
					txData_SPI[1]=0x00;
					txData_SPI[2]=0x00;
					txData_SPI[3]=0x0B;
					txData_SPI[3]|=(1<<4) | (1<<5); // gen grip test command
					if(grip_test_1_execute || grip_test_2_execute || grip_test_3_execute || grip_test_4_execute || grip_test_5_execute || grip_test_6_execute || grip_test_7_execute || grip_test_8_execute)
						break;
			} // end of while to OPEN
			
}
void grip_test_2_(void){ // tripod close
	
	while(M1.current_index_counts < MODE_2_INDEX_LIMIT-ENCODER_LIMIT_ZERO || M2.current_index_counts <  MODE_2_MIDDLE_LIMIT-ENCODER_LIMIT_ZERO || M3.current_index_counts < MODE_2_TWO_FINGER_TRIG_THUMB_LIMIT-ENCODER_LIMIT_ZERO || M4.current_index_counts < MODE_2_RING_AND_LITTLE_LIMIT-ENCODER_LIMIT_ZERO){
			if(M4.current_index_counts < MODE_2_RING_AND_LITTLE_LIMIT){
				if(STEP_UP_M4_VALID)
					motor_4_set(ACLOCKWISE, M4.current_index_counts+ENCODER_LIMIT_ZERO/10); //-----------------------------CURL RING AND LITTLE INWARDS---------------------------------
			}
			if(M3.current_index_counts < MODE_2_TWO_FINGER_TRIG_THUMB_LIMIT){
				if(STEP_UP_M3_VALID)
					motor_3_set(ACLOCKWISE, M3.current_index_counts+ENCODER_LIMIT_ZERO/10); //--------------------------------CURL THUMB-----------------------------------------------------------------
			}

			if(M4.current_index_counts >= MODE_2_RING_AND_LITTLE_LIMIT && M3.current_index_counts >= MODE_2_TWO_FINGER_TRIG_THUMB_LIMIT
				&& M2.current_index_counts <= MODE_2_MIDDLE_LIMIT){
					if(STEP_UP_M2_VALID)
						motor_2_set(ACLOCKWISE, M2.current_index_counts+ENCODER_LIMIT_ZERO/10); //----------------------------CURL  MIDDLE-------------------------------------------------------------------
			}

			if(M4.current_index_counts >= MODE_2_RING_AND_LITTLE_LIMIT && M3.current_index_counts >= MODE_2_TWO_FINGER_TRIG_THUMB_LIMIT
				&& M1.current_index_counts <= MODE_2_INDEX_LIMIT){
				if(M1.current_index_counts <= MODE_2_INDEX_LIMIT){
					if(STEP_UP_M1_VALID)
						motor_1_set(ACLOCKWISE, M1.current_index_counts+ENCODER_LIMIT_ZERO/10); //----------------------------CURL  INDEX-------------------------------------------------------------------
				}
			}
			txData_SPI[0]=0x90;
			txData_SPI[1]=0x00;
			txData_SPI[2]=0x00;
			txData_SPI[3]=0x0B;
			txData_SPI[3]|=(1<<4) | (1<<5); // gen grip test result
			txData_SPI[1] |=1<<5; // gen grip test in progress
			
				if(M1.current_index_counts > ENCODER_LIMIT_ZERO*50 && M2.current_index_counts > ENCODER_LIMIT_ZERO*50 && M3.current_index_counts > ENCODER_LIMIT_ZERO*50 && M4.current_index_counts > ENCODER_LIMIT_ZERO*50){
					if(M1.current_diff == 0 && M2.current_diff == 0 && M4.current_diff == 0 && M3.current_diff == 0){
						break;
					}
				}
				if(!grip_test_2_execute)
					break;
			
			
			
	}
	while(grip_test_2_execute){
		txData_SPI[0]=0x90;
		txData_SPI[1]=0x00;
		txData_SPI[2]=0x00;
		txData_SPI[3]=0x0B;
		txData_SPI[3]|=(1<<4) | (1<<5); // gen grip test command
		txData_SPI[1] |=1<<4; // gen grip test passed
		M1_ACCURACY= (uint8_t)(((float)M1.current_index_counts/MODE_2_INDEX_LIMIT)*100);
		M2_ACCURACY=(uint8_t)(((float)M2.current_index_counts/MODE_2_MIDDLE_LIMIT)*100);
		M3_ACCURACY=(uint8_t)(((float)M3.current_index_counts/MODE_2_TWO_FINGER_TRIG_THUMB_LIMIT)*100);
		M4_ACCURACY=(uint8_t)(((float)M4.current_index_counts/(MODE_2_RING_AND_LITTLE_LIMIT+50))*100);
		KalArm_GEN_GRIP_STAT_RES.grip_accuracy = GRIP_TEST_ACCURACY;
		txData_SPI[3] |= (1 & GRIP_TEST_ACCURACY) <<7;
		txData_SPI[2] |=  (0x3F & (GRIP_TEST_ACCURACY>>1));
		if(KalArm_release){
			grip_test_2_execute = 0;
			break;
		}
	}
	while(M1.current_index_counts>ENCODER_LIMIT_ZERO*2 || M2.current_index_counts>ENCODER_LIMIT_ZERO*2 || M3.current_index_counts>ENCODER_LIMIT_ZERO*2 || M4.current_index_counts>ENCODER_LIMIT_ZERO*2){
		if(STEP_DOWN_M3_VALID)
			motor_3_set(CLOCKWISE, STEP_DOWN_M3);
		if(M3.current_index_counts <= THUMB_OPEN_OPPOSITION_SAFE){
				if(STEP_DOWN_M1_VALID)
					motor_1_set(CLOCKWISE, STEP_DOWN_M1);
				if(STEP_DOWN_M2_VALID)
					motor_2_set(CLOCKWISE, STEP_DOWN_M2);
				if(STEP_DOWN_M4_VALID)
					motor_4_set(CLOCKWISE, STEP_DOWN_M4);
			}
		txData_SPI[0]=0x90;
		txData_SPI[1]=0x00;
		txData_SPI[2]=0x00;
		txData_SPI[3]=0x0B;
		txData_SPI[3]|=(1<<4) | (1<<5); // gen grip test command
		if(grip_test_1_execute || grip_test_2_execute || grip_test_3_execute || grip_test_4_execute || grip_test_5_execute || grip_test_6_execute || grip_test_7_execute || grip_test_8_execute)
			break;
//			if(M1.current_index_counts< ENCODER_LIMIT_INDEX_CO && M2.current_index_counts < ENCODER_LIMIT_MIDDLE_CO && M3.current_index_counts< ENCODER_LIMIT_THUMB_CO && M4.current_index_counts < ENCODER_LIMIT_RING_LITTLE_CO){
//					if(M1.current_diff == 0 && M2.current_diff == 0 && M4.current_diff == 0 && M3.current_diff == 0){
//						motor_1_reset();
//						motor_2_reset();
//						motor_3_reset();
//						motor_4_reset();
//						break;
//					}
//				}
	} // end of while to OPEN
			



}
void grip_test_3_(void){ // tripod open
	
	while(M1.current_index_counts < MODE_2_INDEX_LIMIT-ENCODER_LIMIT_ZERO || M2.current_index_counts <  MODE_2_MIDDLE_LIMIT-ENCODER_LIMIT_ZERO || M3.current_index_counts < MODE_2_TWO_FINGER_TRIG_THUMB_LIMIT-ENCODER_LIMIT_ZERO){

		if(M3.current_index_counts < MODE_2_TWO_FINGER_TRIG_THUMB_LIMIT){
			if(STEP_UP_M3_VALID)
				motor_3_set(ACLOCKWISE, M3.current_index_counts+ENCODER_LIMIT_ZERO/10); //--------------------------------CURL THUMB-----------------------------------------------------------------
		}
		if(M3.current_index_counts >= MODE_2_TWO_FINGER_TRIG_THUMB_LIMIT
			&& M2.current_index_counts <= MODE_2_MIDDLE_LIMIT){
				if(STEP_UP_M2_VALID)
					motor_2_set(ACLOCKWISE, M2.current_index_counts+ENCODER_LIMIT_ZERO/10); //----------------------------CURL  MIDDLE-------------------------------------------------------------------
		}
		if(M3.current_index_counts >= MODE_2_TWO_FINGER_TRIG_THUMB_LIMIT
			&& M1.current_index_counts <= MODE_2_INDEX_LIMIT){  
			if(M1.current_index_counts <= MODE_2_INDEX_LIMIT){
				if(STEP_UP_M1_VALID)
					motor_1_set(ACLOCKWISE, M1.current_index_counts+ENCODER_LIMIT_ZERO/10); //----------------------------CURL  INDEX-------------------------------------------------------------------
			}
		}
		txData_SPI[0]=0x90;
		txData_SPI[1]=0x00;
		txData_SPI[2]=0x00;
		txData_SPI[3]=0x0B;
		txData_SPI[3]|=(1<<4) | (1<<5); // gen grip test result
		txData_SPI[1] |=1<<5; // gen grip test in progress
		
		if(!grip_test_3_execute)
			break;
	}
	while(grip_test_3_execute){
		txData_SPI[0]=0x90;
		txData_SPI[1]=0x00;
		txData_SPI[2]=0x00;
		txData_SPI[3]=0x0B;
		txData_SPI[3]|=(1<<4) | (1<<5); // gen grip test command
		txData_SPI[1] |=1<<4; // gen grip test passed
		M1_ACCURACY= (uint8_t)(((float)M1.current_index_counts/MODE_2_INDEX_LIMIT)*100);
		M2_ACCURACY=(uint8_t)(((float)M2.current_index_counts/MODE_2_MIDDLE_LIMIT)*100);
		M3_ACCURACY=(uint8_t)(((float)M3.current_index_counts/MODE_2_TWO_FINGER_TRIG_THUMB_LIMIT)*100);
		M4_ACCURACY=(uint8_t)(((float)M4.current_index_counts/(M4.current_index_counts))*100);
		KalArm_GEN_GRIP_STAT_RES.grip_accuracy = GRIP_TEST_ACCURACY;
		txData_SPI[3] |= (1 & GRIP_TEST_ACCURACY) <<7;
		txData_SPI[2] |=  (0x3F & (GRIP_TEST_ACCURACY>>1));
		if(KalArm_release){
			grip_test_3_execute = 0;
			break;
		}
	}
	while(M1.current_index_counts>ENCODER_LIMIT_ZERO*2 || M2.current_index_counts>ENCODER_LIMIT_ZERO*2 || M3.current_index_counts>ENCODER_LIMIT_ZERO*2){
		if(STEP_DOWN_M3_VALID)
			motor_3_set(CLOCKWISE, STEP_DOWN_M3);
		if(M3.current_index_counts <= THUMB_OPEN_OPPOSITION_SAFE){
				if(STEP_DOWN_M1_VALID)
					motor_1_set(CLOCKWISE, STEP_DOWN_M1);
				if(STEP_DOWN_M2_VALID)
					motor_2_set(CLOCKWISE, STEP_DOWN_M2);
			}
		txData_SPI[0]=0x90;
		txData_SPI[1]=0x00;
		txData_SPI[2]=0x00;
		txData_SPI[3]=0x0B;
		txData_SPI[3]|=(1<<4) | (1<<5); // gen grip test command
		if(grip_test_1_execute || grip_test_2_execute || grip_test_3_execute || grip_test_4_execute || grip_test_5_execute || grip_test_6_execute || grip_test_7_execute || grip_test_8_execute)
			break;

	} // end of while to OPEN
			

	
	

}
void grip_test_4_(void){ // Adaptive

do{
	adaptive_routine();
	txData_SPI[0]=0x90;
	txData_SPI[1]=0x00;
	txData_SPI[2]=0x00;
	txData_SPI[3]=0x0B;
	txData_SPI[3]|=(1<<4) | (1<<5); // gen grip test result
	txData_SPI[1] |=1<<5; // gen grip test in progress
	if(M1.current_index_counts > ENCODER_LIMIT_ZERO*50 && M2.current_index_counts > ENCODER_LIMIT_ZERO*50 && M3.current_index_counts > ENCODER_LIMIT_ZERO*50 && M4.current_index_counts > ENCODER_LIMIT_ZERO*50){
		if(M1.current_diff == 0 && M2.current_diff == 0 && M4.current_diff == 0 && M3.current_diff == 0){
			break;
		}
	}
	if(!grip_test_4_execute)
		break;
}while(M1.motor_direction != HALT &&M2.motor_direction != HALT && M3.motor_direction != HALT && M4.motor_direction != HALT);


while(grip_test_4_execute){
	txData_SPI[0]=0x90;
	txData_SPI[1]=0x00;
	txData_SPI[2]=0x00;
	txData_SPI[3]=0x0B;
	txData_SPI[3]|=(1<<4) | (1<<5); // gen grip test command
	txData_SPI[1] |=1<<4; // gen grip test passed
	M1_ACCURACY= (uint8_t)(((float)M1.current_index_counts/M1.limit_index_counts)*100);
	M2_ACCURACY=(uint8_t)(((float)M2.current_index_counts/M2.limit_index_counts)*100);
	M3_ACCURACY=(uint8_t)(((float)M3.current_index_counts/M3.limit_index_counts)*100);
	M4_ACCURACY=(uint8_t)(((float)M4.current_index_counts/M4.limit_index_counts)*100);
	KalArm_GEN_GRIP_STAT_RES.grip_accuracy = GRIP_TEST_ACCURACY;
	txData_SPI[3] |= (1 & GRIP_TEST_ACCURACY) <<7;
	txData_SPI[2] |=  (0x3F & (GRIP_TEST_ACCURACY>>1));
	if(KalArm_release){
		grip_test_4_execute = 0;
		break;
	}

}
		
	while(M1.current_index_counts>ENCODER_LIMIT_ZERO*2 || M2.current_index_counts>ENCODER_LIMIT_ZERO*2 || M3.current_index_counts>ENCODER_LIMIT_ZERO*2 || M4.current_index_counts>ENCODER_LIMIT_ZERO*2){
		if(STEP_DOWN_M3_VALID)
			motor_3_set(CLOCKWISE, STEP_DOWN_M3);
		if(M3.current_index_counts <= THUMB_OPEN_OPPOSITION_SAFE){
				if(STEP_DOWN_M1_VALID)
					motor_1_set(CLOCKWISE, STEP_DOWN_M1);
				if(STEP_DOWN_M2_VALID)
					motor_2_set(CLOCKWISE, STEP_DOWN_M2);
				if(STEP_DOWN_M4_VALID)
					motor_4_set(CLOCKWISE, STEP_DOWN_M4);
			}
		txData_SPI[0]=0x90;
		txData_SPI[1]=0x00;
		txData_SPI[2]=0x00;
		txData_SPI[3]=0x0B;
		txData_SPI[3]|=(1<<4) | (1<<5); // gen grip test command
		if(grip_test_1_execute || grip_test_2_execute || grip_test_3_execute || grip_test_4_execute || grip_test_5_execute || grip_test_6_execute || grip_test_7_execute || grip_test_8_execute)
			break;
			if(M1.current_index_counts< ENCODER_LIMIT_INDEX_CO && M2.current_index_counts < ENCODER_LIMIT_MIDDLE_CO && M3.current_index_counts< ENCODER_LIMIT_THUMB_CO && M4.current_index_counts < ENCODER_LIMIT_RING_LITTLE_CO){
					if(M1.current_diff == 0 && M2.current_diff == 0 && M4.current_diff == 0 && M3.current_diff == 0){
						motor_1_reset();
						motor_2_reset();
						motor_3_reset();
						motor_4_reset();
						break;
					}
				}
	} // end of while to OPEN
			


}
void grip_test_5_(void){ // Spherical 
	while(M3.current_index_counts < M3.limit_index_counts - ENCODER_LIMIT_ZERO || M2.current_index_counts < MODE_3_SPHERICAL_MIDDLE_LIMIT - ENCODER_LIMIT_ZERO || M1.current_index_counts < MODE_3_SPHERICAL_INDEX_LIMIT - ENCODER_LIMIT_ZERO || M4.current_index_counts < M4.limit_index_counts -  ENCODER_LIMIT_ZERO){
			if(STEP_UP_M3_VALID)
				motor_3_set(ACLOCKWISE, M3.current_index_counts + ENCODER_LIMIT_ZERO/10);
			if(M3.current_index_counts >= MODE_3_SPHERICAL_THUMB_LIMIT){
				if(M3.current_index_counts >= MODE_3_SPHERICAL_THUMB_SAFE){
					if(M1.current_index_counts < MODE_3_SPHERICAL_INDEX_LIMIT){
						if(STEP_UP_M1_VALID)
								motor_1_set(ACLOCKWISE, M1.current_index_counts + ENCODER_LIMIT_ZERO/10); //-----------------------------CURL INDEX
						}
						if(M2.current_index_counts < MODE_3_SPHERICAL_MIDDLE_LIMIT){
							if(M3.current_index_counts >= MODE_3_SPHERICAL_RING_AND_LITTLE_LIMIT){
								if(STEP_UP_M2_VALID)
									motor_2_set(ACLOCKWISE, M2.current_index_counts + ENCODER_LIMIT_ZERO/10); //----------------------------CURL MIDDLE
							}
					}
				}
				if(STEP_UP_M4_VALID)
					motor_4_set(ACLOCKWISE, M4.current_index_counts + ENCODER_LIMIT_ZERO/10);  //------------------------------CURL RING AND LITTLE FINGERS
			}
			txData_SPI[0]=0x90;
			txData_SPI[1]=0x00;
			txData_SPI[2]=0x00;
			txData_SPI[3]=0x0B;
			txData_SPI[3]|=(1<<4) | (1<<5); // gen grip test result
			txData_SPI[1] |=1<<5; // gen grip test in progress
			if(!grip_test_5_execute)
				break;
	} // close routine
	
	while(grip_test_5_execute){
		txData_SPI[0]=0x90;
		txData_SPI[1]=0x00;
		txData_SPI[2]=0x00;
		txData_SPI[3]=0x0B;
		txData_SPI[3]|=(1<<4) | (1<<5); // gen grip test command
		txData_SPI[1] |=1<<4; // gen grip test passed
		M1_ACCURACY= (uint8_t)(((float)M1.current_index_counts/MODE_3_SPHERICAL_INDEX_LIMIT)*100);
		M2_ACCURACY=(uint8_t)(((float)M2.current_index_counts/MODE_3_SPHERICAL_MIDDLE_LIMIT)*100);
		M3_ACCURACY=(uint8_t)(((float)M3.current_index_counts/(M3.limit_index_counts -  ENCODER_LIMIT_ZERO))*100);
		M4_ACCURACY=(uint8_t)(((float)M4.current_index_counts/(M4.limit_index_counts -  ENCODER_LIMIT_ZERO))*100);
		KalArm_GEN_GRIP_STAT_RES.grip_accuracy = GRIP_TEST_ACCURACY;
		txData_SPI[3] |= (1 & GRIP_TEST_ACCURACY) <<7;
		txData_SPI[2] |=  (0x3F & (GRIP_TEST_ACCURACY>>1));
		if(KalArm_release){
			grip_test_5_execute = 0;
			break;
		}
	}
	while(M1.current_index_counts>ENCODER_LIMIT_ZERO*2 || M2.current_index_counts>ENCODER_LIMIT_ZERO*2 || M3.current_index_counts>ENCODER_LIMIT_ZERO*2 || M4.current_index_counts>ENCODER_LIMIT_ZERO*2){
		if(STEP_DOWN_M1_VALID)
			motor_1_set(CLOCKWISE, M1.current_index_counts- ENCODER_LIMIT_ZERO/10);
		if(STEP_DOWN_M2_VALID)
			motor_2_set(CLOCKWISE, M2.current_index_counts- ENCODER_LIMIT_ZERO/10);
		if(STEP_DOWN_M4_VALID)
			motor_4_set(CLOCKWISE, M4.current_index_counts- ENCODER_LIMIT_ZERO/10);
		if(M1.current_index_counts <= MODE_3_INDEX_SPHERICAL_OPEN_SAFE){
			if(STEP_DOWN_M3_VALID)
				motor_3_set(CLOCKWISE, M3.current_index_counts- ENCODER_LIMIT_ZERO/10);
		}
		txData_SPI[0]=0x90;
		txData_SPI[1]=0x00;
		txData_SPI[2]=0x00;
		txData_SPI[3]=0x0B;
		txData_SPI[3]|=(1<<4) | (1<<5); // gen grip test command
		
		if(grip_test_1_execute || grip_test_2_execute || grip_test_3_execute || grip_test_4_execute || grip_test_5_execute || grip_test_6_execute || grip_test_7_execute || grip_test_8_execute)
			break;
	}

	

}
void grip_test_6_(void){ // don / doff
	
	while(M3.current_index_counts < M3.limit_index_counts - ENCODER_LIMIT_ZERO){
		
		motor_3_set(ACLOCKWISE, M3.limit_index_counts -  ENCODER_LIMIT_ZERO);
		txData_SPI[0]=0x90;
		txData_SPI[1]=0x00;
		txData_SPI[2]=0x00;
		txData_SPI[3]=0x0B;
		txData_SPI[3]|=(1<<4) | (1<<5); // gen grip test result
		txData_SPI[1] |=1<<5; // gen grip test in progress
		HAL_Delay(200);
		if(M1.current_diff == 0 && M2.current_diff == 0 && M4.current_diff == 0 && M3.current_diff == 0){
				break;
			}
		if(!grip_test_6_execute)
			break;

	}
		while(grip_test_6_execute){
				txData_SPI[0]=0x90;
				txData_SPI[1]=0x00;
				txData_SPI[2]=0x00;
				txData_SPI[3]=0x0B;
				txData_SPI[3]|=(1<<4) | (1<<5); // gen grip test command
				txData_SPI[1] |=1<<4; // gen grip test passed
				M1_ACCURACY= (uint8_t)(((float)M1.current_index_counts/M1.current_index_counts)*100);
				M2_ACCURACY=(uint8_t)(((float)M2.current_index_counts/M2.current_index_counts)*100);
				M3_ACCURACY=(uint8_t)(((float)M3.current_index_counts/(M3.limit_index_counts -  ENCODER_LIMIT_ZERO))*100);
				M4_ACCURACY=(uint8_t)(((float)M4.current_index_counts/M4.current_index_counts)*100);
				KalArm_GEN_GRIP_STAT_RES.grip_accuracy = GRIP_TEST_ACCURACY;
				txData_SPI[3] |= (1 & GRIP_TEST_ACCURACY) <<7;
				txData_SPI[2] |=  (0x3F & (GRIP_TEST_ACCURACY>>1));
				if(KalArm_release){
					grip_test_6_execute = 0;
					break;
				}
	}
	while(M3.current_index_counts>ENCODER_LIMIT_ZERO*2){
		motor_3_set(CLOCKWISE, ENCODER_LIMIT_ZERO);
		txData_SPI[0]=0x90;
		txData_SPI[1]=0x00;
		txData_SPI[2]=0x00;
		txData_SPI[3]=0x0B;
		txData_SPI[3]|=(1<<4) | (1<<5); // gen grip test command
		if(grip_test_1_execute || grip_test_2_execute || grip_test_3_execute || grip_test_4_execute || grip_test_5_execute || grip_test_6_execute || grip_test_7_execute || grip_test_8_execute)
			break;
		HAL_Delay(200);
		if(M1.current_diff == 0 && M2.current_diff == 0 && M3.current_diff == 0 && M4.current_diff == 0){
			break;
		}
	}


}
void grip_test_7_(void){ // Pinch
	
	while(M1.current_index_counts < MODE_3_PINCH_INDEX_LIMIT - ENCODER_LIMIT_ZERO*4 || M2.current_index_counts < MODE_3_PINCH_MIDDLE_LIMIT - ENCODER_LIMIT_ZERO*4 || M3.current_index_counts < MODE_3_PINCH_THUMB_LIMIT - ENCODER_LIMIT_ZERO*4 || M4.current_index_counts < MODE_3_PINCH_RING_AND_LITTLE_LIMIT - ENCODER_LIMIT_ZERO*4){
					if(M4.current_index_counts < MODE_3_PINCH_RING_AND_LITTLE_LIMIT){
						if(STEP_UP_M4_VALID)
							motor_4_set(ACLOCKWISE, MODE_3_PINCH_RING_AND_LITTLE_LIMIT); //-----------------------------CURL RING AND LITTLE INWARDS---------------------------------
					}
					if(M2.current_index_counts < MODE_3_PINCH_MIDDLE_LIMIT){
							if(STEP_UP_M2_VALID)
								motor_2_set(ACLOCKWISE, M2.current_index_counts+ENCODER_LIMIT_ZERO/10); //----------------------------CURL  MIDDLE-------------------------------------------------------------------
					}
					if(M2.current_index_counts >= MODE_3_PINCH_MIDDLE_LIMIT && M3.current_index_counts < MODE_3_PINCH_THUMB_LIMIT){
						if(STEP_UP_M3_VALID)
							motor_3_set(ACLOCKWISE, M3.current_index_counts+ENCODER_LIMIT_ZERO/10); //--------------------------------CURL THUMB-----------------------------------------------------------------
					}
					if(M3.current_index_counts >= MODE_3_PINCH_THUMB_LIMIT
						&& M1.current_index_counts <= MODE_3_PINCH_INDEX_LIMIT){  
						if(M1.current_index_counts <= MODE_3_PINCH_INDEX_LIMIT){
							if(STEP_UP_M1_VALID)
								motor_1_set(ACLOCKWISE, M1.current_index_counts+ENCODER_LIMIT_ZERO/10); //----------------------------CURL  INDEX-------------------------------------------------------------------
						}
					}
					txData_SPI[0]=0x90;
					txData_SPI[1]=0x00;
					txData_SPI[2]=0x00;
					txData_SPI[3]=0x0B;
					txData_SPI[3]|=(1<<4) | (1<<5); // gen grip test result
					txData_SPI[1] |=1<<5; // gen grip test in progress
					if(M1.current_index_counts > ENCODER_LIMIT_ZERO*50 && M2.current_index_counts > ENCODER_LIMIT_ZERO*50 && M3.current_index_counts > ENCODER_LIMIT_ZERO*50 && M4.current_index_counts > ENCODER_LIMIT_ZERO*50){
						if(M1.current_diff == 0 && M2.current_diff == 0 && M4.current_diff == 0 && M3.current_diff == 0){
							break;
						}
					}
					if(!grip_test_7_execute)
						break;
	} // end of while to CLOSE
	while(grip_test_7_execute){
		txData_SPI[0]=0x90;
		txData_SPI[1]=0x00;
		txData_SPI[2]=0x00;
		txData_SPI[3]=0x0B;
		txData_SPI[3]|=(1<<4) | (1<<5); // gen grip test command
		txData_SPI[1] |=1<<4; // gen grip test passed
		M1_ACCURACY= (uint8_t)(((float)(M1.current_index_counts-10)/MODE_3_PINCH_INDEX_LIMIT)*100);
		M2_ACCURACY=(uint8_t)(((float)(M2.current_index_counts-10)/MODE_3_PINCH_MIDDLE_LIMIT)*100);
		M3_ACCURACY=(uint8_t)(((float)(M3.current_index_counts-10)/MODE_3_PINCH_THUMB_LIMIT)*100);
		M4_ACCURACY=(uint8_t)(((float)(M4.current_index_counts-10)/MODE_3_PINCH_RING_AND_LITTLE_LIMIT)*100);
		KalArm_GEN_GRIP_STAT_RES.grip_accuracy = GRIP_TEST_ACCURACY;
		txData_SPI[3] |= (1 & GRIP_TEST_ACCURACY) <<7;
		txData_SPI[2] |=  (0x3F & (GRIP_TEST_ACCURACY>>1));
		if(KalArm_release){
			grip_test_7_execute = 0;
			break;
		}
	}
	while(M1.current_index_counts>ENCODER_LIMIT_ZERO*2 || M2.current_index_counts>ENCODER_LIMIT_ZERO*2 || M3.current_index_counts>ENCODER_LIMIT_ZERO*2 || M4.current_index_counts>ENCODER_LIMIT_ZERO*2){
					if(STEP_DOWN_M3_VALID)
						motor_3_set(CLOCKWISE, STEP_DOWN_M3);
					if(M3.current_index_counts <= THUMB_OPEN_OPPOSITION_SAFE){
							if(STEP_DOWN_M1_VALID)
								motor_1_set(CLOCKWISE, STEP_DOWN_M1);
							if(STEP_DOWN_M2_VALID)
								motor_2_set(CLOCKWISE, STEP_DOWN_M2);
							if(STEP_DOWN_M4_VALID)
								motor_4_set(CLOCKWISE, STEP_DOWN_M4);
						}
					txData_SPI[0]=0x90;
					txData_SPI[1]=0x00;
					txData_SPI[2]=0x00;
					txData_SPI[3]=0x0B;
					txData_SPI[3]|=(1<<4) | (1<<5); // gen grip test command
					if(grip_test_1_execute || grip_test_2_execute || grip_test_3_execute || grip_test_4_execute || grip_test_5_execute || grip_test_6_execute || grip_test_7_execute || grip_test_8_execute)
						break;
	} // end of while to OPEN
	


	
	
	

}
void grip_test_8_(void){ // Key
	
		while(M1.current_index_counts < M1.limit_index_counts-25 || M2.current_index_counts < M2.limit_index_counts-25 || M3.current_index_counts < 2200 || M4.current_index_counts < M4.limit_index_counts-25){
				integration_state_number = 9;

				motor_1_set(ACLOCKWISE, M1.limit_index_counts-25);
				motor_2_set(ACLOCKWISE, M2.limit_index_counts-25);
				motor_4_set(ACLOCKWISE, M4.limit_index_counts-25);

				if(M1.current_index_counts > MODE_1_INDEX_ADDUCTION_THRESHOLD && M2.current_index_counts > MODE_1_MIDDLE_ADDUCTION_THRESHOLD){
						if(M3.current_index_counts < MODE_1_THUMB_ADDUCTION_LIMIT){
								motor_3_set(ACLOCKWISE, 2200);
						}
				}

				
				txData_SPI[0]=0x90;
				txData_SPI[1]=0x00;
				txData_SPI[2]=0x00;
				txData_SPI[3]=0x0B;
				txData_SPI[3]|=(1<<4) | (1<<5); // gen grip test command
				txData_SPI[1] |=1<<5; // gen grip test in progress
				HAL_Delay(200);
				
				if(!grip_test_8_execute)
					break;

				
			} // end of while to CLOSE
			while(grip_test_8_execute){
				txData_SPI[0]=0x90;
				txData_SPI[1]=0x00;
				txData_SPI[2]=0x00;
				txData_SPI[3]=0x0B;
				txData_SPI[3]|=(1<<4) | (1<<5); // gen grip test command
				txData_SPI[1] |=1<<4; // gen grip test passed
				M1_ACCURACY= (uint8_t)(((float)M1.current_index_counts/M1.limit_index_counts)*100);
				M2_ACCURACY=(uint8_t)(((float)M2.current_index_counts/M2.limit_index_counts)*100);
				M3_ACCURACY=(uint8_t)(((float)M3.current_index_counts/2200)*100);
				M4_ACCURACY=(uint8_t)(((float)M4.current_index_counts/M4.limit_index_counts)*100);
				KalArm_GEN_GRIP_STAT_RES.grip_accuracy = GRIP_TEST_ACCURACY;
				txData_SPI[3] |= (1 & GRIP_TEST_ACCURACY) <<7;
				txData_SPI[2] |=  (0x3F & (GRIP_TEST_ACCURACY>>1));
				if(KalArm_release){
					grip_test_8_execute = 0;
					break;
				}
			}			
			
			while(M1.current_index_counts>ENCODER_LIMIT_ZERO*2 || M2.current_index_counts>ENCODER_LIMIT_ZERO*2 || M3.current_index_counts>ENCODER_LIMIT_ZERO*2 || M4.current_index_counts>ENCODER_LIMIT_ZERO*2){
					if(STEP_DOWN_M3_VALID)
						motor_3_set(CLOCKWISE, STEP_DOWN_M3);
					if(M3.current_index_counts <= THUMB_OPEN_OPPOSITION_SAFE){
							if(STEP_DOWN_M1_VALID)
								motor_1_set(CLOCKWISE, STEP_DOWN_M1);
							if(STEP_DOWN_M2_VALID)
								motor_2_set(CLOCKWISE, STEP_DOWN_M2);
							if(STEP_DOWN_M4_VALID)
								motor_4_set(CLOCKWISE, STEP_DOWN_M4);
						}
					txData_SPI[0]=0x90;
					txData_SPI[1]=0x00;
					txData_SPI[2]=0x00;
					txData_SPI[3]=0x0B;
					txData_SPI[3]|=(1<<4) | (1<<5); // gen grip test command
					if(grip_test_1_execute || grip_test_2_execute || grip_test_3_execute || grip_test_4_execute || grip_test_5_execute || grip_test_6_execute || grip_test_7_execute || grip_test_8_execute)
						break;

			} // end of while to OPEN

}
void execute_custom_grip(void){
	

	while((M1.current_index_counts < KalArm_CUST_GRIP_STAT_RES.motor_1_index)  || (M2.current_index_counts < KalArm_CUST_GRIP_STAT_RES.motor_2_index)
		 || (M4.current_index_counts < KalArm_CUST_GRIP_STAT_RES.motor_4_index)){
		
			if(KalArm_CUST_GRIP_STAT_RES.thumb_position == THUMB_OPPOSITION && KalArm_CUST_GRIP_STAT_RES.current_thumb_position == THUMB_OPPOSITION){ /*.................................THUMB OPPOSITION.............................*/ 
				if(M1.current_index_counts < KalArm_CUST_GRIP_STAT_RES.motor_1_index){
					if(STEP_UP_M1_VALID)
						motor_1_set(ACLOCKWISE, M1.current_index_counts+ENCODER_LIMIT_ZERO/10);
				}
				if(M2.current_index_counts < KalArm_CUST_GRIP_STAT_RES.motor_2_index){
					if(STEP_UP_M2_VALID)
						motor_2_set(ACLOCKWISE, M2.current_index_counts+ENCODER_LIMIT_ZERO/10);
				}
				if(M4.current_index_counts < KalArm_CUST_GRIP_STAT_RES.motor_4_index){
					if(STEP_UP_M4_VALID)
						motor_4_set(ACLOCKWISE, M4.current_index_counts+ENCODER_LIMIT_ZERO/10);
				}
				if(M3.current_index_counts < THUMB_CLOSE_OPPOSITION_SAFE){
					if(M3.current_index_counts < KalArm_CUST_GRIP_STAT_RES.motor_3_index){
						if(STEP_UP_M3_VALID)
							motor_3_set(ACLOCKWISE, M3.current_index_counts+ENCODER_LIMIT_ZERO/10);
					}
				}
				else{
					if(M1.current_index_counts > MODE_1_INDEX_ADDUCTION_THRESHOLD && M2.current_index_counts > MODE_1_MIDDLE_ADDUCTION_THRESHOLD){
						if(M3.current_index_counts < MODE_1_THUMB_ADDUCTION_LIMIT){
							if(M3.current_index_counts < KalArm_CUST_GRIP_STAT_RES.motor_3_index){
								if(STEP_UP_M3_VALID)
									motor_3_set(ACLOCKWISE, M3.current_index_counts+ENCODER_LIMIT_ZERO/10);
							}
						}
							
					}
				}
			}
			else if(KalArm_CUST_GRIP_STAT_RES.thumb_position == THUMB_NON_OPPOSITION && KalArm_CUST_GRIP_STAT_RES.current_thumb_position == THUMB_NON_OPPOSITION){
				
				if(M1.current_index_counts < KalArm_CUST_GRIP_STAT_RES.motor_1_index){
					if(STEP_UP_M1_VALID)
						motor_1_set(ACLOCKWISE, M1.current_index_counts+ENCODER_LIMIT_ZERO/10);
				}
				if(M2.current_index_counts < KalArm_CUST_GRIP_STAT_RES.motor_2_index){
					if(STEP_UP_M2_VALID)
						motor_2_set(ACLOCKWISE, M2.current_index_counts+ENCODER_LIMIT_ZERO/10);
				}
				if(M4.current_index_counts < KalArm_CUST_GRIP_STAT_RES.motor_4_index){
					if(STEP_UP_M4_VALID)
						motor_4_set(ACLOCKWISE, M4.current_index_counts+ENCODER_LIMIT_ZERO/10);
				}
				if(M3.current_index_counts < THUMB_CLOSE_NON_OPPOSITION_SAFE){
					if(M3.current_index_counts < KalArm_CUST_GRIP_STAT_RES.motor_3_index){
						if(STEP_UP_M3_VALID)
							motor_3_set(ACLOCKWISE, M3.current_index_counts+ENCODER_LIMIT_ZERO/10);
					}
				}
				
			}
			txData_SPI[0]=0x90;
			txData_SPI[1]=0x00;
			txData_SPI[2]=0x00;
			txData_SPI[3]=0x0B;
			txData_SPI[3]|=(1<<6); // gen grip test result
			txData_SPI[1] |=1<<5; // gen grip test in progress
		} // END of while CLOSE
		while(cust_grip_test_execute){
			txData_SPI[0]=0x90;
			txData_SPI[1]=0x00;
			txData_SPI[1] |= 1<<4 ; // Insert "GRIP TEST SUCCESSFUL" bit at the 20th index position of the 32 bit packet
			txData_SPI[2]=0x00;
			txData_SPI[3]=0x0B;
			txData_SPI[3] |= (1<<6); // COMMAND 4
		}
		while(M1.current_index_counts>ENCODER_LIMIT_ZERO || M2.current_index_counts>ENCODER_LIMIT_ZERO || M3.current_index_counts>ENCODER_LIMIT_ZERO || M4.current_index_counts>ENCODER_LIMIT_ZERO){
						if(thumb_position == THUMB_OPPOSITION){
							if(STEP_DOWN_M3_VALID)
								motor_3_set(CLOCKWISE, STEP_DOWN_M3);
							if(M3.current_index_counts <= THUMB_OPEN_OPPOSITION_SAFE){
									if(STEP_DOWN_M1_VALID)
										motor_1_set(CLOCKWISE, STEP_DOWN_M1);
									if(STEP_DOWN_M2_VALID)
										motor_2_set(CLOCKWISE, STEP_DOWN_M2);
									if(STEP_DOWN_M4_VALID)
										motor_4_set(CLOCKWISE, STEP_DOWN_M4);
							}
						}
						else if(thumb_position == THUMB_NON_OPPOSITION){
							if(STEP_DOWN_M3_VALID)
								motor_3_set(CLOCKWISE, STEP_DOWN_M3);
							if(M3.current_index_counts <= THUMB_OPEN_NON_OPPOSITION_SAFE){
									if(STEP_DOWN_M1_VALID)
										motor_1_set(CLOCKWISE, STEP_DOWN_M1);
									if(STEP_DOWN_M2_VALID)
										motor_2_set(CLOCKWISE, STEP_DOWN_M2);
									if(STEP_DOWN_M4_VALID)
										motor_4_set(CLOCKWISE, STEP_DOWN_M4);
								}
							}
							txData_SPI[0]=0x90;
							txData_SPI[1]=0x00;
							txData_SPI[2]=0x00;
							txData_SPI[3]=0x0B;
							txData_SPI[3] |= (1<<6); // COMMAND 4
						if(cust_grip_test_execute)
							break;

					
			}
		
				

		

}

void KalArm_MODE_2_OPPOSED_CLOSE_ROUTINE(void){

			if(M4.current_index_counts < MODE_2_RING_AND_LITTLE_LIMIT){
				if(STEP_UP_M4_VALID)
					motor_4_set(ACLOCKWISE, M4.current_index_counts+ENCODER_LIMIT_ZERO/10); //-----------------------------CURL RING AND LITTLE INWARDS---------------------------------
			}
//			else{
//				motor_4_reset(); //--------------------------------STOP RING AND LITTLE-------------------------------------------------
//			}
			if(M3.current_index_counts < MODE_2_TWO_FINGER_TRIG_THUMB_LIMIT){
				if(STEP_UP_M3_VALID)
					motor_3_set(ACLOCKWISE, M3.current_index_counts+ENCODER_LIMIT_ZERO/10); //--------------------------------CURL THUMB-----------------------------------------------------------------
			}
//			else{
//				motor_3_reset(); //--------------------------------STOP THUMB-----------------------------------------------------------------
//			}
			if(M4.current_index_counts >= MODE_2_RING_AND_LITTLE_LIMIT && M3.current_index_counts >= MODE_2_TWO_FINGER_TRIG_THUMB_LIMIT
				&& M2.current_index_counts <= MODE_2_MIDDLE_LIMIT){
					if(STEP_UP_M2_VALID)
						motor_2_set(ACLOCKWISE, M2.current_index_counts+ENCODER_LIMIT_ZERO/10); //----------------------------CURL  MIDDLE-------------------------------------------------------------------
			}
//			else{
//				motor_2_reset(); //-------------------------------STOP MIDDLE--------------------------------------------------------------------
//			}
			if(M4.current_index_counts >= MODE_2_RING_AND_LITTLE_LIMIT && M3.current_index_counts >= MODE_2_TWO_FINGER_TRIG_THUMB_LIMIT
				&& M1.current_index_counts <= MODE_2_INDEX_LIMIT){
				if(M1.current_index_counts <= MODE_2_INDEX_LIMIT){
					if(STEP_UP_M1_VALID)
						motor_1_set(ACLOCKWISE, M1.current_index_counts+ENCODER_LIMIT_ZERO/10); //----------------------------CURL  INDEX-------------------------------------------------------------------
				}
//				else{
//						motor_1_reset(); //-------------------------------STOP INDEX--------------------------------------------------------------------
//				}
			}
//			else{
//				motor_1_reset();
//			}

}

void prep_arm(void){
			
	motor_1_set(ACLOCKWISE, M1.current_index_counts+ENCODER_LIMIT_ZERO);
	motor_2_set(ACLOCKWISE, M2.current_index_counts+ENCODER_LIMIT_ZERO);
	motor_3_set(ACLOCKWISE, M3.current_index_counts+ENCODER_LIMIT_ZERO);
	motor_4_set(ACLOCKWISE, M4.current_index_counts+ENCODER_LIMIT_ZERO);

}

void mode_1_emg_prop_open(void){
}

void mode_2_emg_prop_open(void){

}

void mode_3_emg_prop_open(void){

}

void mode_4_emg_prop_open(void){

}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
