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
ADC_HandleTypeDef hadc4;
DMA_HandleTypeDef hdma_adc3;
DMA_HandleTypeDef hdma_adc4;

COMP_HandleTypeDef hcomp1;
COMP_HandleTypeDef hcomp2;
COMP_HandleTypeDef hcomp4;
COMP_HandleTypeDef hcomp6;
COMP_HandleTypeDef hcomp7;

I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

uint8_t txData_SPI[4];
uint8_t rxData_SPI[4];

KalArm_SPI_RECEIVE_TYPE KalArm_SPI_RECEIVE_FLAGS;

KalArm_SPI_COMM_STATUS_FLAGS spi_status_flags;
	
uint8_t txData_UART[19];
uint8_t rxData_UART[19];
	

	
uint8_t debounceFlag;


/********************************KalArm CHARGER VARIABLES****************************************/

KalArm_CHG_DET_TYPE KalArm_CHG_DET;
/*******************************/

KalArm_TIME_TYPE KalArm_MFSW;
KalArm_BLE_TYPE KalArm_BLE;

/************************KALARM GRIPS INFORMATION VARIABLES*********************************/

KalArm_GRIP_TYPE KalArm_GRIP;
KalArm_CUST_GRIP_STAT_RES_TYPE KalArm_CUST_GRIP_STAT_RES;
KalArm_GEN_GRIP_STAT_RES_TYPE KalArm_GEN_GRIP_STAT_RES;



/**************************************KALARM BATTERY VARIABLES*********************************/

KalArm_BATTERY_TYPE KalArm_BATTERY;

/**************************************KALARM BOARD VARIABLES************************************/

KalArm_BOARD_TEMP_TYPE KalArm_BOARD_TEMP;

/**************************************KALARM EMG VARIABLES***************************************/
KalArm_EMG_TYPE KalArm_EMG_OPEN, KalArm_EMG_CLOSE;
uint16_t msTicks_T_7;

/**********************************KALARM SOFTWARE VARIABLES**********************************/

uint16_t integration_state_number;
KalArm_MODE_TYPE KalArm_MODE;
KalArm_CALIBRATION_INFO_TYPE KalArm_CALIBRATION_INFO;

/**************************** KALARM THUMB SENSOR & EEPROM VARIABLES*******************/
uint8_t R_buffer[2];
uint8_t buffer[2];
uint16_t D[4];
uint8_t R, G, B, C;
uint8_t thumb_sensor_detected;
TCS34725IntegrationTime_t IntegrationTime_t = TCS34725_INTEGRATIONTIME_2_4MS;
TCS34725Gain_t  Gain_t = TCS34725_GAIN_4X;
int TCS34725_R_Offset;
int TCS34725_G_Offset;
int TCS34725_B_Offset;

TS_Color color_detected;
KalArm_TS_TYPE KalArm_TS;
/***************************************************************************************************************/

uint8_t uart_send;

uint8_t update_event_open;
uint8_t update_event_close;

/****************BATT VOL BRD TEMP BAT TEMP ADC VALS ADC VALUES************************/

uint32_t ADC_raw[2];
uint8_t adc_value_id;


/**Board overheated variable***/

uint8_t board_temp_flag;


KalArm_LAST_COMMAND_STATE_TYPE KalArm_LAST_BLE_COMMAND_STATE, KalArm_LAST_NON_BLE_COMMAND_STATE;


KalArm_CLINICIAN_DATA_TYPE KalArm_CLINICIAN_DATA;

KalArm_CUST_GRIP_DATA_TYPE KalArm_CUST_GRIP_DATA;

KalArm_PATTERN_RECOGNITION_SUITE_PARAMS_TYPE KalArm_PATTERN_RECOGNITION_SUITE_PARAMS;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC4_Init(void);
static void MX_COMP4_Init(void);
static void MX_COMP6_Init(void);
static void MX_TIM7_Init(void);
static void MX_ADC2_Init(void);
static void MX_COMP7_Init(void);
static void MX_COMP1_Init(void);
static void MX_COMP2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t i=0;
uint16_t j=0;

uint8_t short_press_event;
uint8_t long_press_event;

uint8_t cust_grip_execute_id;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){


	if(GPIO_Pin == STAT_MOT_Pin){
		if(HAL_GPIO_ReadPin(STAT_MOT_GPIO_Port,STAT_MOT_Pin) == GPIO_PIN_RESET){
			HAL_SPI_Init(&hspi3);
		}
		else if(HAL_GPIO_ReadPin(STAT_MOT_GPIO_Port,STAT_MOT_Pin) == GPIO_PIN_SET){
			HAL_SPI_DeInit(&hspi3);
		}
	}
		
	 if(GPIO_Pin == REQ_CLK_Pin){
		if(HAL_GPIO_ReadPin(REQ_CLK_GPIO_Port,REQ_CLK_Pin) == GPIO_PIN_RESET){ // receiving data from slave to master
			HAL_SPI_DeInit(&hspi3);
			HAL_SPI_Init(&hspi3);

			
			spi_status_flags.receive_spi_data=1;
			spi_status_flags.send_spi_data = 0;
			
		}
		if(HAL_GPIO_ReadPin(REQ_CLK_GPIO_Port, REQ_CLK_Pin) == GPIO_PIN_SET){ // sending data from master to slave
			
			HAL_SPI_DeInit(&hspi3);
			HAL_SPI_Init(&hspi3);
			
			spi_status_flags.receive_spi_data=0;
			spi_status_flags.send_spi_data=1;
		}
	}
	
	
	if(GPIO_Pin == SW_Pin){
		/* Interrupt Logic for calculating  switch actions */
		if(KalArm_SPI_RECEIVE_FLAGS.calibration_result == CALIBRATION_PASS){
			if (HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin) == GPIO_PIN_SET){
				// Button is in release state
				KalArm_MFSW.ms2 = KalArm_MFSW.ms; // capture the release time stamp
				KalArm_MFSW.haptic_window = 0;
				KalArm_MFSW.view_battery_flag = 0;
				KalArm_MFSW.mode_change_flag = 0;
				KalArm_MFSW.ble_on_off_flag = 0;
			}
			else if(HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin) == GPIO_PIN_RESET){
				// Button is in pressed state
				KalArm_MFSW.ms1 = KalArm_MFSW.ms; // capture the pressed time stamp
			}
			KalArm_MFSW.window = KalArm_MFSW.ms2 - KalArm_MFSW.ms1;
			
			if(KalArm_BLE.ble_on_off_flag != 1){
				if(KalArm_MFSW.window > 100 && KalArm_MFSW.window <= 800){
					//set mode change flag
					KalArm_BATTERY.battery_status_flag = 0;
					if(KalArm_MODE.mode_number< 3){
						KalArm_MODE.mode_number++;
						KalArm_MODE.mode_state =1;
					}
					else{
						KalArm_MODE.mode_number =0;
					}
					KalArm_EMG_OPEN.pattern = UNDEFINED; // reset pattern 
					KalArm_EMG_CLOSE.pattern = UNDEFINED;
					reset_rgb_LED();
				}
			}
			else{ // secure connect or disconnect using shortpress
				if(KalArm_MFSW.window > 150 && KalArm_MFSW.window <= 800){
					if(HAL_GPIO_ReadPin(HS_BLE_GPIO_Port, HS_BLE_Pin) == GPIO_PIN_SET){  
						HAL_GPIO_WritePin(HS_BLE_GPIO_Port, HS_BLE_Pin, GPIO_PIN_RESET);
					}
					else{
						HAL_GPIO_WritePin(HS_BLE_GPIO_Port, HS_BLE_Pin, GPIO_PIN_SET);
					}
				}
				else if(KalArm_MFSW.window > 3000 && KalArm_MFSW.window <= 10000){
//					if(HAL_GPIO_ReadPin(RST_BLE_GPIO_Port, RST_BLE_Pin) == GPIO_PIN_RESET){ // if BLE switched off switch on ble
//						HAL_GPIO_WritePin(RST_BLE_GPIO_Port, RST_BLE_Pin, GPIO_PIN_SET);
//						HAL_UART_Receive_IT(&huart3, rxData_UART, 19);
//						HAL_UART_Transmit_IT(&huart3, txData_UART, 19);
//					}
//					else{ 																																																											// if BLE switched on switch off ble
//						HAL_GPIO_WritePin(RST_BLE_GPIO_Port, RST_BLE_Pin, GPIO_PIN_RESET);

//					}

					if(HAL_GPIO_ReadPin(HS_BLE_GPIO_Port, HS_BLE_Pin) == GPIO_PIN_SET){  
						HAL_GPIO_WritePin(HS_BLE_GPIO_Port, HS_BLE_Pin, GPIO_PIN_RESET);
					}
					else{
						HAL_GPIO_WritePin(HS_BLE_GPIO_Port, HS_BLE_Pin, GPIO_PIN_SET);
					}
				}
			}
		}
	}
	
	if(GPIO_Pin == CHG_DET_Pin){
		/*Interrupt logic for calculating charge detect state*/
		reset_rgb_LED();
		if(HAL_GPIO_ReadPin(CHG_DET_GPIO_Port, CHG_DET_Pin) == GPIO_PIN_SET){
			

			KalArm_CHG_DET.charger_state = 1;
			
	
		}
		else if(HAL_GPIO_ReadPin(CHG_DET_GPIO_Port, CHG_DET_Pin) == GPIO_PIN_RESET){
			

			KalArm_CHG_DET.charger_state = 0;
		
		}
	
	
	}
	

	
	
}



void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	
	
	j++;
	
	if((spi_status_flags.receive_spi_data == 1) && (spi_status_flags.send_spi_data == 0)){
		spi_receive_handler();
	}
		
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	
		if(rxData_UART[0] == 0xFF && rxData_UART[18] == 0xFB){
			switch(rxData_UART[1]){
				case 0:
					KalArm_BLEC_Tx_CLINICIAN_ACK();
					break;
				case 15:
					KalArm_BLEC_Tx_BATT_Board();
					break;
				case 16:
					KalArm_BLEC_Tx_EMG_Viz();
					break;
				case 14:
					KalArm_BLEC_Tx_CAL_INFO();
					break;
				case 9:
					KalArm_BLEC_Tx_GEN_GRIP_TEST_STAT_RES();
					break;
				case 8:
					KalArm_BLEC_Tx_CUST_GRIP_TEST_STAT_RES();
					break;
			
			
			}
	}
	

	uart_send = 1;
	
	debounceFlag++;

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	
	if(rxData_UART[0] == 0xFF && rxData_UART[18] == 0xFB){
		switch(rxData_UART[1]){
			case 0:
				KalArm_BLEC_Tx_CLINICIAN_ACK();
				KalArm_CLINICIAN_DATA.data_fresh = 1;
				KalArm_CLINICIAN_DATA.open_threshold = CLINICIAN_OPEN_THRESHOLD;
				KalArm_CLINICIAN_DATA.close_threshold = CLINICIAN_CLOSE_THRESHOLD;
				KalArm_CLINICIAN_DATA.pulse_period = CLINICIAN_PULSE_PERIOD;
				KalArm_CLINICIAN_DATA.double_impulse_period = CLINICIAN_DOUBLE_IMPULSE_PERIOD;
				KalArm_CLINICIAN_DATA.triple_impulse_period = CLINICIAN_TRIPLE_IMPULSE_PERIOD;
				for(int k = 0; k<=9; k++){
					KalArm_CLINICIAN_DATA.data_array[k] = rxData_UART[k+2];
				}
				break;
			case 2:
				KalArm_CUST_GRIP_DATA.grip_id = CUST_GRIP_DATA_GRIP_ID;
				KalArm_CUST_GRIP_DATA.motor_1_index = CUST_GRIP_DATA_M1_INDEX;
				KalArm_CUST_GRIP_DATA.motor_2_index = CUST_GRIP_DATA_M2_INDEX;
				KalArm_CUST_GRIP_DATA.motor_3_index = CUST_GRIP_DATA_M3_INDEX;
				KalArm_CUST_GRIP_DATA.motor_4_index = CUST_GRIP_DATA_M4_INDEX;
				KalArm_CUST_GRIP_DATA.data_array[0] = KalArm_CUST_GRIP_DATA.grip_id;
				KalArm_CUST_GRIP_DATA.data_array[1] = KalArm_CUST_GRIP_DATA.motor_1_index;
				KalArm_CUST_GRIP_DATA.data_array[2] = KalArm_CUST_GRIP_DATA.motor_2_index;
				KalArm_CUST_GRIP_DATA.data_array[3] = KalArm_CUST_GRIP_DATA.motor_3_index;
				KalArm_CUST_GRIP_DATA.data_array[4] = KalArm_CUST_GRIP_DATA.motor_4_index;
				break;
			case 3:
				KalArm_CUST_GRIP_DATA.grip_id = CUST_GRIP_DATA_GRIP_ID;
				KalArm_CUST_GRIP_DATA.motor_1_index = CUST_GRIP_DATA_M1_INDEX;
				KalArm_CUST_GRIP_DATA.motor_2_index = CUST_GRIP_DATA_M2_INDEX;
				KalArm_CUST_GRIP_DATA.motor_3_index = CUST_GRIP_DATA_M3_INDEX;
				KalArm_CUST_GRIP_DATA.motor_4_index = CUST_GRIP_DATA_M4_INDEX;
				KalArm_CUST_GRIP_DATA.data_array[0] = KalArm_CUST_GRIP_DATA.grip_id;
				KalArm_CUST_GRIP_DATA.data_array[1] = KalArm_CUST_GRIP_DATA.motor_1_index;
				KalArm_CUST_GRIP_DATA.data_array[2] = KalArm_CUST_GRIP_DATA.motor_2_index;
				KalArm_CUST_GRIP_DATA.data_array[3] = KalArm_CUST_GRIP_DATA.motor_3_index;
				KalArm_CUST_GRIP_DATA.data_array[4] = KalArm_CUST_GRIP_DATA.motor_4_index;
				break;
			case 4:
				KalArm_CUST_GRIP_DATA.grip_id = CUST_GRIP_DATA_GRIP_ID;
				KalArm_CUST_GRIP_DATA.motor_1_index = CUST_GRIP_DATA_M1_INDEX;
				KalArm_CUST_GRIP_DATA.motor_2_index = CUST_GRIP_DATA_M2_INDEX;
				KalArm_CUST_GRIP_DATA.motor_3_index = CUST_GRIP_DATA_M3_INDEX;
				KalArm_CUST_GRIP_DATA.motor_4_index = CUST_GRIP_DATA_M4_INDEX;
				KalArm_CUST_GRIP_DATA.data_array[0] = KalArm_CUST_GRIP_DATA.grip_id;
				KalArm_CUST_GRIP_DATA.data_array[1] = KalArm_CUST_GRIP_DATA.motor_1_index;
				KalArm_CUST_GRIP_DATA.data_array[2] = KalArm_CUST_GRIP_DATA.motor_2_index;
				KalArm_CUST_GRIP_DATA.data_array[3] = KalArm_CUST_GRIP_DATA.motor_3_index;
				KalArm_CUST_GRIP_DATA.data_array[4] = KalArm_CUST_GRIP_DATA.motor_4_index;
			break;
			case 5:
				KalArm_CUST_GRIP_DATA.grip_id = CUST_GRIP_DATA_GRIP_ID;
				KalArm_CUST_GRIP_DATA.motor_1_index = CUST_GRIP_DATA_M1_INDEX;
				KalArm_CUST_GRIP_DATA.motor_2_index = CUST_GRIP_DATA_M2_INDEX;
				KalArm_CUST_GRIP_DATA.motor_3_index = CUST_GRIP_DATA_M3_INDEX;
				KalArm_CUST_GRIP_DATA.motor_4_index = CUST_GRIP_DATA_M4_INDEX;
				KalArm_CUST_GRIP_DATA.data_array[0] = KalArm_CUST_GRIP_DATA.grip_id;
				KalArm_CUST_GRIP_DATA.data_array[1] = KalArm_CUST_GRIP_DATA.motor_1_index;
				KalArm_CUST_GRIP_DATA.data_array[2] = KalArm_CUST_GRIP_DATA.motor_2_index;
				KalArm_CUST_GRIP_DATA.data_array[3] = KalArm_CUST_GRIP_DATA.motor_3_index;
				KalArm_CUST_GRIP_DATA.data_array[4] = KalArm_CUST_GRIP_DATA.motor_4_index;
				break;
			case 6:
				KalArm_CUST_GRIP_DATA.grip_id = CUST_GRIP_DATA_GRIP_ID;
				KalArm_CUST_GRIP_DATA.motor_1_index = CUST_GRIP_DATA_M1_INDEX;
				KalArm_CUST_GRIP_DATA.motor_2_index = CUST_GRIP_DATA_M2_INDEX;
				KalArm_CUST_GRIP_DATA.motor_3_index = CUST_GRIP_DATA_M3_INDEX;
				KalArm_CUST_GRIP_DATA.motor_4_index = CUST_GRIP_DATA_M4_INDEX;
				KalArm_CUST_GRIP_DATA.data_array[0] = KalArm_CUST_GRIP_DATA.grip_id;
				KalArm_CUST_GRIP_DATA.data_array[1] = KalArm_CUST_GRIP_DATA.motor_1_index;
				KalArm_CUST_GRIP_DATA.data_array[2] = KalArm_CUST_GRIP_DATA.motor_2_index;
				KalArm_CUST_GRIP_DATA.data_array[3] = KalArm_CUST_GRIP_DATA.motor_3_index;
				KalArm_CUST_GRIP_DATA.data_array[4] = KalArm_CUST_GRIP_DATA.motor_4_index;
				break;
			case 7:
				KalArm_CUST_GRIP_DATA.grip_id = CUST_GRIP_DATA_GRIP_ID;
				KalArm_CUST_GRIP_DATA.motor_1_index = CUST_GRIP_DATA_M1_INDEX;
				KalArm_CUST_GRIP_DATA.motor_2_index = CUST_GRIP_DATA_M2_INDEX;
				KalArm_CUST_GRIP_DATA.motor_3_index = CUST_GRIP_DATA_M3_INDEX;
				KalArm_CUST_GRIP_DATA.motor_4_index = CUST_GRIP_DATA_M4_INDEX;
				KalArm_CUST_GRIP_DATA.data_array[0] = KalArm_CUST_GRIP_DATA.grip_id;
				KalArm_CUST_GRIP_DATA.data_array[1] = KalArm_CUST_GRIP_DATA.motor_1_index;
				KalArm_CUST_GRIP_DATA.data_array[2] = KalArm_CUST_GRIP_DATA.motor_2_index;
				KalArm_CUST_GRIP_DATA.data_array[3] = KalArm_CUST_GRIP_DATA.motor_3_index;
				KalArm_CUST_GRIP_DATA.data_array[4] = KalArm_CUST_GRIP_DATA.motor_4_index;
				break;
			case 15:
				KalArm_BLEC_Tx_BATT_Board();
				break;
			case 16:
				KalArm_BLEC_Tx_EMG_Viz();
				break;
			case 14:
				KalArm_BLEC_Tx_CAL_INFO();
				break;
			case 9:
				KalArm_BLEC_Tx_GEN_GRIP_TEST_STAT_RES();
				KalArm_GEN_GRIP_STAT_RES.grip_id = rxData_UART[2];
				KalArm_SPI_RECEIVE_FLAGS.ggt_result = GGT_NOT_INIT;
				KalArm_GEN_GRIP_STAT_RES.grip_execute_order = rxData_UART[3];
				break;
			case 8:
				KalArm_BLEC_Tx_CUST_GRIP_TEST_STAT_RES();
				KalArm_SPI_RECEIVE_FLAGS.cgt_result = CGT_NOT_INIT;
				KalArm_CUST_GRIP_STAT_RES.grip_execute_order = rxData_UART[7];
				KalArm_CUST_GRIP_STAT_RES.motor_1_index = rxData_UART[2];
				KalArm_CUST_GRIP_STAT_RES.motor_2_index = rxData_UART[3];
				KalArm_CUST_GRIP_STAT_RES.motor_3_index = rxData_UART[4];
				KalArm_CUST_GRIP_STAT_RES.motor_4_index = rxData_UART[5];
				KalArm_CUST_GRIP_STAT_RES.thumb_position = rxData_UART[6];
				break;
		
		}
		KalArm_BLE.ble_connected_flag = 1;
		KalArm_BLE.ble_advertising_flag = 0;
		KalArm_BLE.ble_connection_request_flag = 0;
	}
	
	else if(rxData_UART[0] == 0xFF && rxData_UART[18] == 0xFA){
	
			KalArm_BLE.ble_advertising_flag = 1;
			KalArm_BLE.ble_connected_flag = 0;
			KalArm_BLE.ble_connection_request_flag = 0;
			HAL_GPIO_WritePin(HS_BLE_GPIO_Port, HS_BLE_Pin, GPIO_PIN_SET);

	}
	
	else if(rxData_UART[0] == 0xFF && rxData_UART[18] == 0xFE){
		
		KalArm_BLE.ble_connection_request_flag = 1;
		KalArm_BLE.ble_advertising_flag = 0;
		KalArm_BLE.ble_connected_flag=0;
		HAL_GPIO_WritePin(HS_BLE_GPIO_Port, HS_BLE_Pin, GPIO_PIN_SET);
	
	}
	
	else if(rxData_UART[0] == 0xFF && rxData_UART[18] == 0xFC){
			
			
//		if(KalArm_BLE.ble_on_off_flag == 1){ // BLE is switched on
			KalArm_BLE.ble_advertising_flag = 0;
			KalArm_BLE.ble_connected_flag = 1;
			KalArm_BLE.ble_connection_request_flag = 0;
//		}
//		else{
//			KalArm_BLE.ble_connected_flag=0;
//			KalArm_BLE.ble_advertising_flag=0;
//		}
			HAL_GPIO_WritePin(HS_BLE_GPIO_Port, HS_BLE_Pin, GPIO_PIN_SET);
	
	}
	
	
		
	
	
	
	HAL_UART_Receive_IT(&huart3, rxData_UART, 19);

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	
	if(hadc == &hadc3){
		KalArm_EMG_CLOSE.sensor_val = HAL_ADC_GetValue(&hadc3);
		KalArm_EMG_CLOSE.ms = msTicks_T_7;

	}
	else if(hadc == &hadc4){
		KalArm_EMG_OPEN.sensor_val = HAL_ADC_GetValue(&hadc4);
		KalArm_EMG_OPEN.ms = msTicks_T_7;
	}
	
	else if(hadc == &hadc2){

				

//		KalArm_BATTERY.battery_voltage_raw = ADC_raw[0];
//		KalArm_BOARD_TEMP.board_temperature_raw = ADC_raw[1];
		
//		if(__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC))
					KalArm_BATTERY.battery_voltage_raw = HAL_ADC_GetValue(&hadc2);
//		if(__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOS))
//					KalArm_BOARD_TEMP.board_temperature_raw = HAL_ADC_GetValue(&hadc2);
		KalArm_BATTERY.battery_voltage = KalArm_BATTERY.battery_voltage_raw *0.00229;
		if(KalArm_BATTERY.battery_voltage> 7.6 && KalArm_BATTERY.battery_voltage < 8.6){
			KalArm_BATTERY.battery_percentage = (uint8_t)((KalArm_BATTERY.battery_voltage - 7.6)*20+80);
		}
		else if(KalArm_BATTERY.battery_voltage > 7.2 && KalArm_BATTERY.battery_voltage <=7.6){
			KalArm_BATTERY.battery_percentage = (uint8_t)(((KalArm_BATTERY.battery_voltage-7.2))*100+50);
		}
		else if(KalArm_BATTERY.battery_voltage >7.0 && KalArm_BATTERY.battery_voltage <= 7.2){
			KalArm_BATTERY.battery_percentage = (uint8_t)(((KalArm_BATTERY.battery_voltage-7.0))*100+30);
		}
		else if(KalArm_BATTERY.battery_voltage >6.5 && KalArm_BATTERY.battery_voltage <= 7.0){
			KalArm_BATTERY.battery_percentage = (uint8_t)(((KalArm_BATTERY.battery_voltage-6.5))*40+10);
		}
		else{
			KalArm_BATTERY.battery_percentage = ((KalArm_BATTERY.battery_voltage-6))*20;
			KalArm_BATTERY.battery_critical = 1;
		}

			
		
	}
	else if(hadc == &hadc1){
		KalArm_BATTERY.battery_temperature_raw = HAL_ADC_GetValue(&hadc1);
		KalArm_BATTERY.battery_temperature = (0.04638)* KalArm_BATTERY.battery_temperature_raw-40;
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
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_TIM6_Init();
  MX_ADC3_Init();
  MX_ADC4_Init();
  MX_COMP4_Init();
  MX_COMP6_Init();
  MX_TIM7_Init();
  MX_ADC2_Init();
  MX_COMP7_Init();
  MX_COMP1_Init();
  MX_COMP2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	
	
	


	battery_voltage_config_init(1);
	battery_temperature_config_init(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); //Thumb sensor LED Turned off
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // Reset Motor Controller
	while(HAL_GPIO_ReadPin(STAT_MOT_GPIO_Port, STAT_MOT_Pin) != GPIO_PIN_RESET){
	}
	spi_status_flags.receive_spi_data=0;
	spi_status_flags.send_spi_data=1;
	
	
	HAL_UART_Receive_IT(&huart3, rxData_UART, 19);
	HAL_UART_Transmit_IT(&huart3, txData_UART, 19);
	

	thumb_sensor_detected=i2c_thumb_sensor_setup();

	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // turn on led for thumb sense
	
	emg_sensor_close_config_init(1);
	emg_sensor_open_config_init(1);
	
	integration_state_number =1;	
	
	
	while(KalArm_BATTERY.battery_percentage<20);
	
	do{
		TCS34725_Calibrate();
	}while(! IS_BLACK);
	
	if(calibration_request() == CALIBRATION_PASS){
		KalArm_MODE.mode_state = 1;
		KalArm_MODE.mode_number = 0;
	}
	else{
		while(1);
	}
	
	if(HAL_COMP_Start(&hcomp2) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
	
	// Read EEPROM for CLINICIAN PARAMETERS and feed values into the pattern recognition suite
	
	clinician_parameters_load();



//	HAL_GPIO_WritePin(HS_BLE_GPIO_Port, HS_BLE_Pin, GPIO_PIN_SET);
//	HAL_Delay(10);
//	HAL_GPIO_WritePin(HS_BLE_GPIO_Port, HS_BLE_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (KalArm_SPI_RECEIVE_FLAGS.calibration_result == CALIBRATION_PASS)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		  if (HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUT_LEVEL_HIGH)
				board_temp_flag = 1;
			else
				board_temp_flag = 0;

		
			if(KalArm_CHG_DET.charger_state == 0){
				integration_state_number = 2;
				
				if(KalArm_BLE.ble_on_off_flag == 0){
					KalArm_GEN_GRIP_STAT_RES.grip_execute_order = 0;
					KalArm_CUST_GRIP_STAT_RES.grip_execute_order = 0;
						if(HAL_GPIO_ReadPin(REQ_CLK_GPIO_Port, REQ_CLK_Pin) == GPIO_PIN_SET){ // send SPI transmit data while there are no receive requests from the motor controller
							// we do not do this inside the spi tx rx complete callback because there it will be oerormed only once when the chanfge of pin state controled by motor controller occurs
							// this transmission must be in total control of the main controller and not at the mercy of the slave controllers clock request hence it is implemented here
							/*Insert fresh Transmit data to send to motor controller*/

				//			HAL_SPI_TransmitReceive_IT(&hspi3,txData_SPI,rxData_SPI,4);
//							
//							if(KalArm_MODE.mode_number != 3){
								emg_control(KalArm_EMG_OPEN, KalArm_EMG_CLOSE);
//							}
//							else{
//									if(KalArm_CUST_GRIP_STAT_RES.grip_execute_order){
//										// execute custom grip test
//										KalArm_LAST_NON_BLE_COMMAND_STATE = CUST_GRIP_TEST;
//										custom_grip_test(KalArm_CUST_GRIP_STAT_RES);

//									}
//									if(KalArm_LAST_NON_BLE_COMMAND_STATE == CUST_GRIP_TEST){
//							
//										if(HAL_GPIO_ReadPin(REQ_CLK_GPIO_Port, REQ_CLK_Pin) == GPIO_PIN_SET){
//											txData_SPI[0] = 0x90;
//											txData_SPI[1] = KalArm_CUST_GRIP_STAT_RES.grip_execute_order; // Motor Index Counts
//											txData_SPI[2] = 0x03; // grip_execute_order
//											txData_SPI[2] |= 6<<2;
//											txData_SPI[3] = 0x0B;	
//											txData_SPI[3] |= (1<<4) | (1<<5); // COMMAND 3
//											// Send the Calibration Request Packet 1001 0000 0000 0000 0000 0000 0000 1011
//											HAL_SPI_TransmitReceive_IT(&hspi3,txData_SPI,rxData_SPI,4);
//										}
//										else if((spi_status_flags.receive_spi_data == 1) && (spi_status_flags.send_spi_data == 0)){
//												// Insert dummy data to send to the motor controller
//												txData_SPI[0] = 0xF0;
//												txData_SPI[1] = 0x00;
//												txData_SPI[2] = 0x00;
//												txData_SPI[3] = 0x0F;
//												// send the dummy data packet 1111 0000 0000 0000 0000 0000 0000 1111
//												HAL_SPI_TransmitReceive_IT(&hspi3,txData_SPI,rxData_SPI,4);
//										}
//									
//									}
//								
//							}
							
						}
						else if((spi_status_flags.receive_spi_data == 1) && (spi_status_flags.send_spi_data == 0)){
							HAL_SPI_TransmitReceive_IT(&hspi3,txData_SPI,rxData_SPI,4);
						}
				}
				if(KalArm_BLE.ble_on_off_flag == 1){
					if(KalArm_CUST_GRIP_STAT_RES.grip_execute_order){
						// execute custom grip test
						KalArm_LAST_BLE_COMMAND_STATE = CUST_GRIP_TEST;
						custom_grip_test(KalArm_CUST_GRIP_STAT_RES);

					}
					else if(KalArm_GEN_GRIP_STAT_RES.grip_execute_order){
						// execute general grip test
						KalArm_LAST_BLE_COMMAND_STATE = GEN_GRIP_TEST;
						general_grip_test(KalArm_GEN_GRIP_STAT_RES.grip_id, KalArm_TS);

					}
					else{
						
						if(KalArm_LAST_BLE_COMMAND_STATE == GEN_GRIP_TEST){
							if(HAL_GPIO_ReadPin(REQ_CLK_GPIO_Port, REQ_CLK_Pin) == GPIO_PIN_SET){
								txData_SPI[0] = 0x90;
								txData_SPI[1] = 0x00;
								txData_SPI[2] = 0x00;
								txData_SPI[3] = 0x0B;
								txData_SPI[3] |= 1<<6;
								txData_SPI[2] |= KalArm_GEN_GRIP_STAT_RES.grip_id << 2;
								txData_SPI[1] |= KalArm_GEN_GRIP_STAT_RES.grip_execute_order;
								// Send the Calibration Request Packet 1001 0000 0000 0000 0000 0000 0000 1011
								HAL_SPI_TransmitReceive_IT(&hspi3,txData_SPI,rxData_SPI,4);
							}
							else if((spi_status_flags.receive_spi_data == 1) && (spi_status_flags.send_spi_data == 0)){
									// Insert dummy data to send to the motor controller
									txData_SPI[0] = 0xF0;
									txData_SPI[1] = 0x00;
									txData_SPI[2] = 0x00;
									txData_SPI[3] = 0x0F;
									// send the dummy data packet 1111 0000 0000 0000 0000 0000 0000 1111
									HAL_SPI_TransmitReceive_IT(&hspi3,txData_SPI,rxData_SPI,4);
							}			
						}
						else if(KalArm_LAST_BLE_COMMAND_STATE == CUST_GRIP_TEST){
							
							if(HAL_GPIO_ReadPin(REQ_CLK_GPIO_Port, REQ_CLK_Pin) == GPIO_PIN_SET){
								txData_SPI[0] = 0x90;
								txData_SPI[1] = KalArm_CUST_GRIP_STAT_RES.grip_execute_order; // Motor Index Counts
								txData_SPI[2] = 0x03; // grip_execute_order
								txData_SPI[2] |= 6<<2;
								txData_SPI[3] = 0x0B;	
								txData_SPI[3] |= (1<<4) | (1<<5); // COMMAND 3
								// Send the Calibration Request Packet 1001 0000 0000 0000 0000 0000 0000 1011
								HAL_SPI_TransmitReceive_IT(&hspi3,txData_SPI,rxData_SPI,4);
							}
							else if((spi_status_flags.receive_spi_data == 1) && (spi_status_flags.send_spi_data == 0)){
									// Insert dummy data to send to the motor controller
									txData_SPI[0] = 0xF0;
									txData_SPI[1] = 0x00;
									txData_SPI[2] = 0x00;
									txData_SPI[3] = 0x0F;
									// send the dummy data packet 1111 0000 0000 0000 0000 0000 0000 1111
									HAL_SPI_TransmitReceive_IT(&hspi3,txData_SPI,rxData_SPI,4);
							}
						
						}
					}
					
					if(KalArm_CLINICIAN_DATA.data_fresh){
					
						 while(HAL_OK != I2C_WRITEBUF(18,KalArm_CLINICIAN_DATA.data_array,10));


						 while(HAL_OK != I2C_READBUF(18,KalArm_CLINICIAN_DATA.eeprom_data_array,10));

						KalArm_CLINICIAN_DATA.data_fresh = 0;
					}
					
					if(KalArm_CUST_GRIP_DATA.data_fresh){
						
						
						switch(KalArm_CUST_GRIP_DATA.grip_id){
							case 2: 
								while(HAL_OK != I2C_WRITEBUF(19,KalArm_CLINICIAN_DATA.data_array,4));
								while(HAL_OK != I2C_READBUF(19,KalArm_CLINICIAN_DATA.eeprom_data_array,4));
								break;
							case 3:
								while(HAL_OK != I2C_WRITEBUF(20,KalArm_CLINICIAN_DATA.data_array,4));
								while(HAL_OK != I2C_READBUF(20,KalArm_CLINICIAN_DATA.eeprom_data_array,4));
								break;
							case 4:
								while(HAL_OK != I2C_WRITEBUF(21,KalArm_CLINICIAN_DATA.data_array,4));
								while(HAL_OK != I2C_READBUF(21,KalArm_CLINICIAN_DATA.eeprom_data_array,4));
								break;
							case 5:
								while(HAL_OK != I2C_WRITEBUF(22,KalArm_CLINICIAN_DATA.data_array,4));
								while(HAL_OK != I2C_READBUF(22,KalArm_CLINICIAN_DATA.eeprom_data_array,4));
								break;
							case 6:
								while(HAL_OK != I2C_WRITEBUF(23,KalArm_CLINICIAN_DATA.data_array,4));
								while(HAL_OK != I2C_READBUF(23,KalArm_CLINICIAN_DATA.eeprom_data_array,4));
								break;
							case 7:
								while(HAL_OK != I2C_WRITEBUF(24,KalArm_CLINICIAN_DATA.data_array,4));
								while(HAL_OK != I2C_READBUF(24,KalArm_CLINICIAN_DATA.eeprom_data_array,4));
								break;
						}
						
						KalArm_CUST_GRIP_DATA.data_fresh = 0;
					
					}
					
					
				}

				
				

			i2c_thumb_sensor_routine();
				

				integration_state_number=3;
				if(RGBC_VALID){
					if(R_NOT_IN_CAL_RANGE || G_NOT_IN_CAL_RANGE || B_NOT_IN_CAL_RANGE)
						color_detected = CYAN;
					else
						color_detected = BLACK;
					
					KalArm_GEN_GRIP_STAT_RES.current_thumb_position = (color_detected == BLACK)? THUMB_OPPOSITION: THUMB_NON_OPPOSITION;
				}
				integration_state_number = 4;

				
				
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
  RCC_OscInitStruct.PLL.PLLN = 16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_ADC345;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_PLL;
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
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_1;
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
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_15;
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
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
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
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */
  /** Common config 
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.GainCompensation = 0;
  hadc4.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.ContinuousConvMode = DISABLE;
  hadc4.Init.NbrOfConversion = 1;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc4.Init.DMAContinuousRequests = DISABLE;
  hadc4.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc4.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */

  /* USER CODE END ADC4_Init 2 */

}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp1.Init.InputMinus = COMP_INPUT_MINUS_3_4VREFINT;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}

/**
  * @brief COMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP2_Init(void)
{

  /* USER CODE BEGIN COMP2_Init 0 */

  /* USER CODE END COMP2_Init 0 */

  /* USER CODE BEGIN COMP2_Init 1 */

  /* USER CODE END COMP2_Init 1 */
  hcomp2.Instance = COMP2;
  hcomp2.Init.InputPlus = COMP_INPUT_PLUS_IO2;
  hcomp2.Init.InputMinus = COMP_INPUT_MINUS_1_4VREFINT;
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp2.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP2_Init 2 */

  /* USER CODE END COMP2_Init 2 */

}

/**
  * @brief COMP4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP4_Init(void)
{

  /* USER CODE BEGIN COMP4_Init 0 */

  /* USER CODE END COMP4_Init 0 */

  /* USER CODE BEGIN COMP4_Init 1 */

  /* USER CODE END COMP4_Init 1 */
  hcomp4.Instance = COMP4;
  hcomp4.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp4.Init.InputMinus = COMP_INPUT_MINUS_1_4VREFINT;
  hcomp4.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp4.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp4.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp4.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
  if (HAL_COMP_Init(&hcomp4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP4_Init 2 */

  /* USER CODE END COMP4_Init 2 */

}

/**
  * @brief COMP6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP6_Init(void)
{

  /* USER CODE BEGIN COMP6_Init 0 */

  /* USER CODE END COMP6_Init 0 */

  /* USER CODE BEGIN COMP6_Init 1 */

  /* USER CODE END COMP6_Init 1 */
  hcomp6.Instance = COMP6;
  hcomp6.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp6.Init.InputMinus = COMP_INPUT_MINUS_1_4VREFINT;
  hcomp6.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp6.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp6.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp6.Init.TriggerMode = COMP_TRIGGERMODE_EVENT_RISING;
  if (HAL_COMP_Init(&hcomp6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP6_Init 2 */

  /* USER CODE END COMP6_Init 2 */

}

/**
  * @brief COMP7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP7_Init(void)
{

  /* USER CODE BEGIN COMP7_Init 0 */

  /* USER CODE END COMP7_Init 0 */

  /* USER CODE BEGIN COMP7_Init 1 */

  /* USER CODE END COMP7_Init 1 */
  hcomp7.Instance = COMP7;
  hcomp7.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp7.Init.InputMinus = COMP_INPUT_MINUS_1_4VREFINT;
  hcomp7.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp7.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp7.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp7.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP7_Init 2 */

  /* USER CODE END COMP7_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10707DBC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim6.Init.Prescaler = 63;
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
	
	htim6.Instance->CR1 |= TIM_CR1_CEN;
	htim6.Instance->DIER |= TIM_DIER_UIE;

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
  htim7.Init.Prescaler = 63;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */
	
		
	htim7.Instance->CR1 |= TIM_CR1_CEN;
	htim7.Instance->DIER |= TIM_DIER_UIE;

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, LED_R_Pin|LED_G_Pin|LED_B_Pin|HS_BLE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOT_RST_Pin|GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_BLE_GPIO_Port, RST_BLE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LOD1_N_Pin LOD2_P_Pin LOD2_N_Pin */
  GPIO_InitStruct.Pin = LOD1_N_Pin|LOD2_P_Pin|LOD2_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CHG_DET_Pin */
  GPIO_InitStruct.Pin = CHG_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CHG_DET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_R_Pin LED_G_Pin LED_B_Pin */
  GPIO_InitStruct.Pin = LED_R_Pin|LED_G_Pin|LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_Pin */
  GPIO_InitStruct.Pin = SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOT_RST_Pin PB8 */
  GPIO_InitStruct.Pin = MOT_RST_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EMG_1_EN_Pin EMG_2_EN_Pin TS_INT_Pin */
  GPIO_InitStruct.Pin = EMG_1_EN_Pin|EMG_2_EN_Pin|TS_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_BLE_Pin */
  GPIO_InitStruct.Pin = RST_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RST_BLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LOD1_P_Pin */
  GPIO_InitStruct.Pin = LOD1_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LOD1_P_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HS_BLE_Pin */
  GPIO_InitStruct.Pin = HS_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HS_BLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STAT_BLE_Pin */
  GPIO_InitStruct.Pin = STAT_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(STAT_BLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STAT_MOT_Pin */
  GPIO_InitStruct.Pin = STAT_MOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(STAT_MOT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : REQ_CLK_Pin */
  GPIO_InitStruct.Pin = REQ_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(REQ_CLK_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */



void clearSPITxData(void){
    for(int i=0; i<20; i++){
        txData_SPI[i] = 0;
    }
}

void clearSPIRxData(void){
    for(int i=0; i<20; i++){
        rxData_SPI[i] = 0;
    }
}

void spi_receive_handler(void){
	
	if((rxData_SPI[0] & 0xF0) == HEADER && (rxData_SPI[3] & 0x0F) == FOOTER){
		switch(COMMAND){
			
			case 0: // COMMAND 0 or Calibration Step
				if(CAL_IN_PGRESS){
					
					KalArm_SPI_RECEIVE_FLAGS.calibration_result = CALIBRATION_IN_PROGRESS;
					KalArm_CALIBRATION_INFO.calibration_times = CAL_TIMES;
				}
				else{
					if(CAL_P_F){ 
						KalArm_CALIBRATION_INFO.motor_1_status = CAL_M1;
						KalArm_CALIBRATION_INFO.motor_1_cs = CAL_M1_CS;
						KalArm_CALIBRATION_INFO.motor_1_en = CAL_M1_EN;
						KalArm_CALIBRATION_INFO.motor_2_status = CAL_M2;
						KalArm_CALIBRATION_INFO.motor_2_cs = CAL_M2_CS;
						KalArm_CALIBRATION_INFO.motor_2_en = CAL_M2_EN;
						KalArm_CALIBRATION_INFO.motor_3_status = CAL_M3;
						KalArm_CALIBRATION_INFO.motor_3_cs = CAL_M3_CS;
						KalArm_CALIBRATION_INFO.motor_3_en = CAL_M3_EN;
						KalArm_CALIBRATION_INFO.motor_4_status = CAL_M4;
						KalArm_CALIBRATION_INFO.motor_4_cs = CAL_M4_CS;
						KalArm_CALIBRATION_INFO.motor_4_en = CAL_M4_EN;		
						KalArm_SPI_RECEIVE_FLAGS.calibration_result = CALIBRATION_FAIL;
					}
					else{
						KalArm_SPI_RECEIVE_FLAGS.calibration_result = CALIBRATION_PASS;
					}
			}
			break;
			case 1: // COMMAND 1 or In Grip
					KalArm_GRIP.grip_id = GRIP_ID;
					KalArm_SPI_RECEIVE_FLAGS.in_grip = IN_GRIP;
					if(KalArm_MODE.mode_number == 2)
						KalArm_SPI_RECEIVE_FLAGS.grip_id = GRIP_ID;
					KalArm_GRIP.ingrip = IN_GRIP;
			break;
			case 2: // COMMAND 2 or Feedback Information
				KalArm_SPI_RECEIVE_FLAGS.driver_error_1 = DEF1;
				KalArm_SPI_RECEIVE_FLAGS.driver_error_2 = DEF2;
				KalArm_SPI_RECEIVE_FLAGS.power_good = PG;
				KalArm_SPI_RECEIVE_FLAGS.feedback_grip_id = GRIP_ID_FEEDBACK;
				KalArm_SPI_RECEIVE_FLAGS.add_grip_success = ADD_S ;
				KalArm_SPI_RECEIVE_FLAGS.delete_grip_success = DEL_S ;
				KalArm_SPI_RECEIVE_FLAGS.threshold_write_success = TH_S;
				KalArm_SPI_RECEIVE_FLAGS.pulse_period_write_success = PP_S;
				KalArm_SPI_RECEIVE_FLAGS.mode_change_success = MODE_S; // Mode Change Success
				KalArm_SPI_RECEIVE_FLAGS.emg_control_feedback = EMG_S; 
			break;
			case 3: // COMMAND 3 or General Grip Test Result
				KalArm_SPI_RECEIVE_FLAGS.ggt_result = GGT_STAT_RES;
				KalArm_GEN_GRIP_STAT_RES.grip_accuracy = GGT_ACCURACY;	
				KalArm_GEN_GRIP_STAT_RES.status = GGT_STAT_RES;
			break;	
				
			case 4: // COMMAND 4 or Custom Grip Test Result
				KalArm_SPI_RECEIVE_FLAGS.cgt_result = CGT_STAT_RES;
				KalArm_CUST_GRIP_STAT_RES.grip_accuracy = CGT_ACCURACY;
				KalArm_CUST_GRIP_STAT_RES.status = CGT_STAT_RES;
			break;
			
		} // end of switch case
	
	} 
	
	

}


uint8_t DEV_I2C_ReadByte(uint8_t add_){
	uint8_t Buf[1]={add_};
//	while(HAL_I2C_Mem_Read_IT(&hi2c2, 0x29<<1, add_, I2C_MEMADD_SIZE_8BIT, Buf, 1) != HAL_OK){
//	}
HAL_I2C_Mem_Read(&hi2c2, 0x29<<1, add_, I2C_MEMADD_SIZE_8BIT, Buf, 1, 0x10);
	return Buf[0];
}

void DEV_I2C_WriteByte(uint8_t add_, uint8_t data_){
	uint8_t Buf[1] = {0};
	Buf[0] = data_;
//	while(HAL_I2C_Mem_Write_IT(&hi2c2, 0x29<<1, add_, I2C_MEMADD_SIZE_8BIT, Buf, 1) != HAL_OK){
//	}
	HAL_I2C_Mem_Write(&hi2c2, 0x29<<1, add_, I2C_MEMADD_SIZE_8BIT, Buf, 1, 0x10);
}

uint8_t TCS34725_ReadByte(uint8_t add){
    add = add | TCS34725_CMD_BIT;
    return DEV_I2C_ReadByte(add);
}
void TCS34725_WriteByte(uint8_t add, uint8_t data){
    //Note: remember to add this when users write their own
    //Responsible for not finding the register, 
    //refer to the data sheet Command Register CMD(Bit 7)
    add = add | TCS34725_CMD_BIT;
    DEV_I2C_WriteByte(add, data);
}

void TCS34725_Set_Integration_Time(TCS34725IntegrationTime_t time){
    /* Update the timing register */
    TCS34725_WriteByte(TCS34725_ATIME, time);
    IntegrationTime_t = time;
}
void TCS34725_Set_Gain(TCS34725Gain_t gain){
	TCS34725_WriteByte(TCS34725_CONTROL, gain); 
    Gain_t = gain;
}
void TCS34725_Interrupt_Enable(){
    uint8_t data = 0;
    data = TCS34725_ReadByte(TCS34725_ENABLE);
    TCS34725_WriteByte(TCS34725_ENABLE, data | TCS34725_ENABLE_AIEN);
}
void TCS34725_Enable(void){
    TCS34725_WriteByte(TCS34725_ENABLE, TCS34725_ENABLE_PON);
    DEV_Delay_ms(3);
    TCS34725_WriteByte(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
    DEV_Delay_ms(3);  
}
void TCS34725_Set_Interrupt_Persistence_Reg(uint8_t TCS34725_PER){
    if(TCS34725_PER < 0x10)
        TCS34725_WriteByte(TCS34725_PERS, TCS34725_PER);
    else 
        TCS34725_WriteByte(TCS34725_PERS, TCS34725_PERS_60_CYCLE);
}
void TCS34725_Set_Interrupt_Threshold(uint16_t Threshold_H, uint16_t Threshold_L){
    TCS34725_WriteByte(TCS34725_AILTL, Threshold_L & 0xff);
    TCS34725_WriteByte(TCS34725_AILTH, Threshold_L >> 8);
    TCS34725_WriteByte(TCS34725_AIHTL, Threshold_H & 0xff);
    TCS34725_WriteByte(TCS34725_AIHTH, Threshold_H >> 8);
}

uint16_t DEV_I2C_ReadWord(uint8_t add_){
    uint8_t Buf[2]={0, 0};
		while(HAL_I2C_Mem_Read_DMA(&hi2c2, 0x29<<1, add_, I2C_MEMADD_SIZE_8BIT, Buf, 2) != HAL_OK){
		}
//		HAL_I2C_Mem_Read(&hi2c2, 0x29<<1, add_, I2C_MEMADD_SIZE_8BIT, Buf, 2, 0x10);
    return ((Buf[1] << 8) | (Buf[0] & 0xff));
}

void TCS34725_Calibrate(void){
	
	TCS34725_Get_RGBData();
	KalArm_TS.R_CAL = R;
	KalArm_TS.G_CAL = G;
	KalArm_TS.B_CAL = B;
	


}

uint16_t TCS34725_ReadWord(uint8_t add){

    add = add | TCS34725_CMD_BIT;
    return DEV_I2C_ReadWord(add);
}


void TCS34725_Get_RGBData(void){
		C = TCS34725_ReadWord(TCS34725_CDATAL | TCS34725_CMD_Read_Word);
		C = TCS34725_ReadWord(TCS34725_CDATAL | TCS34725_CMD_Read_Word);
		C = TCS34725_ReadWord(TCS34725_CDATAL | TCS34725_CMD_Read_Word);		
		switch (IntegrationTime_t){
		    case TCS34725_INTEGRATIONTIME_2_4MS:
              DEV_Delay_ms(3);
              break;
        case TCS34725_INTEGRATIONTIME_24MS:
              DEV_Delay_ms(24);
              break;
        case TCS34725_INTEGRATIONTIME_50MS:
              DEV_Delay_ms(50);
              break;
        case TCS34725_INTEGRATIONTIME_101MS:
              DEV_Delay_ms(101);
              break;
        case TCS34725_INTEGRATIONTIME_154MS:
              DEV_Delay_ms(154);
              break;
        case TCS34725_INTEGRATIONTIME_700MS:
              DEV_Delay_ms(700);
              break;
    }	
	
	
	
	
		G = TCS34725_ReadWord(TCS34725_GDATAL | TCS34725_CMD_Read_Word) + TCS34725_G_Offset;
		G = TCS34725_ReadWord(TCS34725_GDATAL | TCS34725_CMD_Read_Word) + TCS34725_G_Offset;
		G = TCS34725_ReadWord(TCS34725_GDATAL | TCS34725_CMD_Read_Word) + TCS34725_G_Offset;

  	switch (IntegrationTime_t){
        case TCS34725_INTEGRATIONTIME_2_4MS:
              DEV_Delay_ms(3);
              break;
        case TCS34725_INTEGRATIONTIME_24MS:
              DEV_Delay_ms(24);
              break;
        case TCS34725_INTEGRATIONTIME_50MS:
              DEV_Delay_ms(50);
              break;
        case TCS34725_INTEGRATIONTIME_101MS:
              DEV_Delay_ms(101);
              break;
        case TCS34725_INTEGRATIONTIME_154MS:
              DEV_Delay_ms(154);
              break;
        case TCS34725_INTEGRATIONTIME_700MS:
              DEV_Delay_ms(700);
              break;
    }
		R = TCS34725_ReadWord(TCS34725_RDATAL | TCS34725_CMD_Read_Word)-6 + TCS34725_R_Offset;
		R = TCS34725_ReadWord(TCS34725_RDATAL | TCS34725_CMD_Read_Word)-6 + TCS34725_R_Offset;
		R = TCS34725_ReadWord(TCS34725_RDATAL | TCS34725_CMD_Read_Word)-6 + TCS34725_R_Offset;
		 switch (IntegrationTime_t){
		    case TCS34725_INTEGRATIONTIME_2_4MS:
              DEV_Delay_ms(3);
              break;
        case TCS34725_INTEGRATIONTIME_24MS:
              DEV_Delay_ms(24);
              break;
        case TCS34725_INTEGRATIONTIME_50MS:
              DEV_Delay_ms(50);
              break;
        case TCS34725_INTEGRATIONTIME_101MS:
              DEV_Delay_ms(101);
              break;
        case TCS34725_INTEGRATIONTIME_154MS:
              DEV_Delay_ms(154);
              break;
        case TCS34725_INTEGRATIONTIME_700MS:
              DEV_Delay_ms(700);
              break;
    }
		B = TCS34725_ReadWord(TCS34725_BDATAL | TCS34725_CMD_Read_Word) + TCS34725_B_Offset;
		B = TCS34725_ReadWord(TCS34725_BDATAL | TCS34725_CMD_Read_Word) + TCS34725_B_Offset;
		B = TCS34725_ReadWord(TCS34725_BDATAL | TCS34725_CMD_Read_Word) + TCS34725_B_Offset;		
		switch (IntegrationTime_t){
		    case TCS34725_INTEGRATIONTIME_2_4MS:
              DEV_Delay_ms(3);
              break;
        case TCS34725_INTEGRATIONTIME_24MS:
              DEV_Delay_ms(24);
              break;
        case TCS34725_INTEGRATIONTIME_50MS:
              DEV_Delay_ms(50);
              break;
        case TCS34725_INTEGRATIONTIME_101MS:
              DEV_Delay_ms(101);
              break;
        case TCS34725_INTEGRATIONTIME_154MS:
              DEV_Delay_ms(154);
              break;
        case TCS34725_INTEGRATIONTIME_700MS:
              DEV_Delay_ms(700);
              break;
    }		

	i=123;
		
}


uint8_t i2c_thumb_sensor_setup(void){

	uint8_t ID = 0;
	ID = TCS34725_ReadByte(TCS34725_ID);
	if(ID != 0x44 && ID != 0x4D){
			return 0;
	}
	//Set the integration time and gain
	TCS34725_Set_Integration_Time(IntegrationTime_t);	
	TCS34725_Set_Gain(Gain_t);


	//Set Interrupt
	TCS34725_Set_Interrupt_Threshold(0xff00, 0x00ff);//Interrupt upper and lower threshold
	TCS34725_Set_Interrupt_Persistence_Reg(TCS34725_PERS_2_CYCLE);
	TCS34725_Enable();
	TCS34725_Interrupt_Enable();


	return 1;
}

void i2c_thumb_sensor_routine(void){
	TCS34725_Get_RGBData();
}



uint8_t debounce(){
	
	HAL_Delay(30);
	return 1;

}

void reset_rgb_LED(void){
	set_R_LED(0);
	set_G_LED(0);
	set_B_LED(0);
	
}
void set_rgb_LED(void){
	set_R_LED(100);
	set_G_LED(100);
	set_B_LED(100);
}
void set_rgb_LED_ingrip(void){
	set_G_LED(100);
	set_R_LED(0);
	set_B_LED(0);
	
}

void set_ble_LED(void){
	set_B_LED(100);
	set_G_LED(0);
	set_R_LED(0);
}
void toggle_bleMode_LED(void){
	
	set_R_LED(0);
	set_G_LED(0);
	toggle_B_LED();
	
}


void low_battery_LED(void){
	
	set_R_LED(100);
	set_G_LED(0);
	set_B_LED(0);
	
	
}
void cal_fail_LED(void){
	set_R_LED(100);
	set_G_LED(0);
	set_B_LED(0);
}
void cal_in_progress_LED(void){
	set_R_LED(100);
	set_G_LED(100);
	set_B_LED(100);

}
void over_temp_LED(void){
	set_R_LED(100);
	set_G_LED(0);
	set_B_LED(0);
}
void in_grip_LED(void){
	set_G_LED(100);
	set_R_LED(0);
	set_B_LED(0);
}

void bat_charge_25_LED(void){

	toggle_R_LED();
	toggle_B_LED();

}
void bat_charge_50_LED(void){

	toggle_G_LED();
	toggle_B_LED();

}
void bat_charge_75_LED(void){

	toggle_G_LED();
	toggle_R_LED();
}
void bat_charge_100_LED(void){

	toggle_G_LED();
	
}

void mode_1_LED(void){
	set_G_LED(100);
	set_R_LED(0);
	set_B_LED(0);
}

void mode_1_LED_toggle(void){
	toggle_G_LED();
	set_R_LED(0);
	set_B_LED(0);
}
void mode_2_LED(void){
	set_G_LED(100);
	set_B_LED(100);
	set_R_LED(0);
}
void mode_2_LED_toggle(void){	
	toggle_G_LED();
	toggle_B_LED();
	set_R_LED(0);
}
void mode_3_LED(void){
	set_G_LED(100);
	set_R_LED(100);
	set_B_LED(0);
}
void mode_3_LED_toggle(void){
	toggle_G_LED();
	toggle_R_LED();
	set_B_LED(0);

}
void mode_4_LED(void){
	set_R_LED(100);
	set_B_LED(100);
	set_G_LED(0);
}
void mode_4_LED_toggle(void){
	toggle_R_LED();
	toggle_B_LED();
	set_G_LED(0);

}

void set_R_LED(uint8_t duty_cycle){
	
//	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // R
//	htim2.Instance->CCR1 = (duty_cycle/100)*TIM_CONFIG_PULSE_VAL;
	if(duty_cycle> 0)
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

	
}
void set_G_LED(uint8_t duty_cycle){
	
//	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1); //G
//	htim16.Instance->CCR1 = (duty_cycle/100)*TIM_CONFIG_PULSE_VAL;
	if(duty_cycle> 0)
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	
}
void set_B_LED(uint8_t duty_cycle){
	
//	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1); //B
//	htim17.Instance->CCR1 = (duty_cycle/100)*TIM_CONFIG_PULSE_VAL;
	if(duty_cycle> 0)
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

}
void toggle_R_LED(void){
	
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
}
void toggle_G_LED(void){
	
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_6);

}
void toggle_B_LED(void){

		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_7);

}

void emg_sensor_open_config_init(uint8_t init_deinit){
	
	  HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);

		ADC_Enable(&hadc3);

		if(init_deinit){
			HAL_ADC_Start_IT(&hadc3);

		}
		else{
			HAL_ADC_Stop_IT(&hadc3);

		}
		HAL_Delay(10);
}

void emg_sensor_close_config_init(uint8_t init_deinit){
	
		HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED);
		ADC_Enable(&hadc4);
	
		if(init_deinit){
			HAL_ADC_Start_IT(&hadc4);
		}
		else{
			HAL_ADC_Stop_IT(&hadc4);
		}	
		HAL_Delay(10);

}

void battery_voltage_config_init(uint8_t init_deinit){

	  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
		ADC_Enable(&hadc2);

	
		if(init_deinit){
			HAL_ADC_Start_IT(&hadc2);
//			HAL_ADC_Start_DMA(&hadc2, ADC_raw, 2);
		}
		else{
			HAL_ADC_Stop_IT(&hadc2);
//			HAL_ADC_Stop_DMA(&hadc2);
		}
		HAL_Delay(10);

}

void battery_temperature_config_init(uint8_t init_deinit){

	  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
		ADC_Enable(&hadc1);

	
		if(init_deinit){
			HAL_ADC_Start_IT(&hadc1);
		}
		else{
			HAL_ADC_Stop_IT(&hadc1);
		}
		HAL_Delay(10);

}

KalArm_CALIBRATION_STATUS calibration_request(void){
	
		while(KalArm_SPI_RECEIVE_FLAGS.calibration_result != CALIBRATION_PASS && KalArm_SPI_RECEIVE_FLAGS.calibration_result != CALIBRATION_FAIL){
			
				i2c_thumb_sensor_routine();
				
				if(RGBC_VALID){
					if(R_NOT_IN_CAL_RANGE || G_NOT_IN_CAL_RANGE || B_NOT_IN_CAL_RANGE)
						color_detected = CYAN;
					else
						color_detected = BLACK;
				}
			if(HAL_GPIO_ReadPin(REQ_CLK_GPIO_Port, REQ_CLK_Pin) == GPIO_PIN_SET){ // send SPI transmit data while there are no receive requests from the motor controller
					// we do not do this inside the spi tx rx complete callback because there it will be oerormed only once when the chanfge of pin state controled by motor controller occurs
					// this transmission must be in total control of the main controller and not at the mercy of the slave controllers clock request hence it is implemented here
					/*Insert fresh Transmit data to send to motor controller*/
					txData_SPI[0] = 0x90;
					txData_SPI[1] = 0x00;
					txData_SPI[2] = 0x00;
					txData_SPI[3] = 0x0B;
					// Send the Calibration Request Packet 1001 0000 0000 0000 0000 0000 0000 1011
					HAL_SPI_TransmitReceive_IT(&hspi3,txData_SPI,rxData_SPI,4);
				}
				else if((spi_status_flags.receive_spi_data == 1) && (spi_status_flags.send_spi_data == 0)){
					// Insert dummy data to send to the motor controller
					txData_SPI[0] = 0xF0;
					txData_SPI[1] = 0x00;
					txData_SPI[2] = 0x00;
					txData_SPI[3] = 0x0F;
					// send the dummy data packet 1111 0000 0000 0000 0000 0000 0000 1111
					HAL_SPI_TransmitReceive_IT(&hspi3,txData_SPI,rxData_SPI,4);
				}
				
		} // end of calibration check while
		return KalArm_SPI_RECEIVE_FLAGS.calibration_result;

	
}

KalArm_EMG_CONTROL_STATUS emg_control(KalArm_EMG_TYPE KalArm_EMG_OPEN, KalArm_EMG_TYPE KalArm_EMG_CLOSE){
	
	
	txData_SPI[0]=0x90;
	txData_SPI[1]=0x00;
	txData_SPI[2]=0x00;
	txData_SPI[3]=0x0B;
	txData_SPI[3] |= (1<<6) | (1<<5); // Insert EMG Control Command
	
	if(KalArm_EMG_OPEN.emg_prop_flag == 1){
		txData_SPI[2] |=1<<1; // Insert YES Proportional on Open
		txData_SPI[2] &= ~(1<<2); // Insert NO Proportional on Close
	}
	else 	if(KalArm_EMG_CLOSE.emg_prop_flag == 1){
		txData_SPI[2]|=1<<2; // Insert YES Proportional on Close	
		txData_SPI[2] &= ~(1<<1); // Insert NO Proportional on Open
	}
	else if( KalArm_EMG_OPEN.emg_prop_flag == 0 && KalArm_EMG_CLOSE.emg_prop_flag == 0){
		
			// dig into what pattern has occured on the open sensor
			if(KalArm_EMG_OPEN.pattern == SINGLE_IMPULSE){ // 3,4,5
				txData_SPI[2] |= (1<<3); 
				txData_SPI[2] &= ~(1<<4);
				txData_SPI[2] &= ~(1<<5);
			}
			else if(KalArm_EMG_OPEN.pattern == DOUBLE_IMPULSE){
				txData_SPI[2] &= ~(1<<3);
				txData_SPI[2] |= (1<<4);
				txData_SPI[2] &= ~(1<<5);
			}
			else if(KalArm_EMG_OPEN.pattern == TRIPLE_IMPULSE){
				txData_SPI[2] |= (1<<3);
				txData_SPI[2] |= (1<<4);
				txData_SPI[2] &= ~(1<<5);
			}
			
			if(KalArm_EMG_CLOSE.pattern == SINGLE_IMPULSE){ // 4,5,6
				txData_SPI[1] |= (1<<4); 
				txData_SPI[1] &= ~(1<<5);
				txData_SPI[1] &= ~(1<<6);
			}
			else if(KalArm_EMG_CLOSE.pattern == DOUBLE_IMPULSE){
				txData_SPI[1] &= ~(1<<4);
				txData_SPI[1] |= (1<<5);
				txData_SPI[1] &= ~(1<<6);
			}
			else if(KalArm_EMG_CLOSE.pattern == TRIPLE_IMPULSE){
				txData_SPI[1] |= (1<<4);
				txData_SPI[1] |= (1<<5);
				txData_SPI[1] &= ~(1<<6);
			}

			txData_SPI[3] |= (1<<7); // no pattern on open sensor
			txData_SPI[2] |= (1<<0); // no pattern on close sensor
			
			if(KalArm_EMG_OPEN.pattern_update_event == 1){
				txData_SPI[1] |= 1<<2;
			}
			else{
				txData_SPI[1] &=~(1<<2);
			}
			if(KalArm_EMG_CLOSE.pattern_update_event == 1){
				txData_SPI[1] |= 1<<3;
			}
			else{
				txData_SPI[1] &= ~(1<<3);
			}
			

					
	}
	
	switch(KalArm_MODE.mode_number){
		
		case 0:
			txData_SPI[2] &= ~(1<<6); // Ma  0
			txData_SPI[2] &= ~(1<<7); // Mb  0
			break;
		case 1:
			txData_SPI[2] |= 1<<6 ; // Ma 1
			txData_SPI[2] &= ~(1<<7); // Mb 0
			break;
		case 2:
			txData_SPI[2] |= 1<<7; // Mb 1
			txData_SPI[2] &= ~(1<<6); // Ma 0
			break;
		case 3:
			txData_SPI[2] |= (1<<6) | (1<<7); // Ma | Mb (1|1)
			break;
	
	}
	
	if(color_detected == CYAN){
		txData_SPI[1] |= (1<<0);
		txData_SPI[1] &= ~(1<<1);
	}
	else if(color_detected == BLACK){
		txData_SPI[1] &= ~(1<<0);
		txData_SPI[1] |= (1<<1);
	}
	
	HAL_SPI_TransmitReceive_IT(&hspi3,txData_SPI,rxData_SPI,4);
	
	return EMG_CONTROL_PASS;

}



void KalArm_BLEC_Tx_BATT_Board(void){
	
	txData_UART[0] = 0xFF;
	txData_UART[1]= 25;
	txData_UART[2]= KalArm_BATTERY.battery_percentage;
	txData_UART[3]= KalArm_BATTERY.battery_temperature;
	txData_UART[4]= 28;
	txData_UART[5]= 0;
	txData_UART[6]=0;
	txData_UART[7]=0;
	txData_UART[8]=0;
	txData_UART[9]=0;
	txData_UART[10]=0;
	txData_UART[11]=0;
	txData_UART[12]=0;
	txData_UART[13]=0;
	txData_UART[14]=0;
	txData_UART[15]=0;
	txData_UART[16]=0;
	txData_UART[17]=0;
	txData_UART[18]=0xFB;

}
void KalArm_BLEC_Tx_EMG_Viz(void){
	txData_UART[0] = 0xFF;
	txData_UART[1]= 26;
	txData_UART[2]= KalArm_EMG_OPEN.sensor_val >> 8;
	txData_UART[3]= KalArm_EMG_OPEN.sensor_val & 0xFF;
	txData_UART[4]= KalArm_EMG_CLOSE.sensor_val >> 8;
	txData_UART[5]= KalArm_EMG_CLOSE.sensor_val & 0xFF;
	txData_UART[6]=KalArm_EMG_OPEN.ms >> 8;
	txData_UART[7]=KalArm_EMG_OPEN.ms & 0xFF;
	txData_UART[8]=KalArm_EMG_CLOSE.ms >>8;
	txData_UART[9]=KalArm_EMG_CLOSE.ms & 0xFF;
	txData_UART[10]=0;
	txData_UART[11]=0;
	txData_UART[12]=0;
	txData_UART[13]=0;
	txData_UART[14]=0;
	txData_UART[15]=0;
	txData_UART[16]=0;
	txData_UART[17]=0;
	txData_UART[18]=0xFB;

}
void KalArm_BLEC_Tx_CAL_INFO(void){
	
	txData_UART[0] = 0xFF;
	txData_UART[1]= 24;
	txData_UART[2]= KalArm_CALIBRATION_INFO.motor_1_en;
	txData_UART[3]= KalArm_CALIBRATION_INFO.motor_1_cs;
	txData_UART[4]= KalArm_CALIBRATION_INFO.motor_2_en;
	txData_UART[5]= KalArm_CALIBRATION_INFO.motor_2_cs;
	txData_UART[6]= KalArm_CALIBRATION_INFO.motor_3_en;
	txData_UART[7]= KalArm_CALIBRATION_INFO.motor_3_cs;
	txData_UART[8]= KalArm_CALIBRATION_INFO.motor_4_en;
	txData_UART[9]= KalArm_CALIBRATION_INFO.motor_4_cs;
	txData_UART[10]=0;
	txData_UART[11]=0;
	txData_UART[12]=0;
	txData_UART[13]=0;
	txData_UART[14]=0;
	txData_UART[15]=0;
	txData_UART[16]=0;
	txData_UART[17]=0;
	txData_UART[18]=0xFB;

}
void KalArm_BLEC_Tx_CUST_GRIP_TEST_STAT_RES(void){
		
	txData_UART[0] = 0xFF;
	txData_UART[1]= 19;
	txData_UART[2]= KalArm_CUST_GRIP_STAT_RES.status;
	txData_UART[3]= KalArm_CUST_GRIP_STAT_RES.grip_accuracy;
	txData_UART[4]= KalArm_CUST_GRIP_STAT_RES.thumb_position;
	txData_UART[5]= 0;
	txData_UART[6]= 0;
	txData_UART[7]= 0;
	txData_UART[8]= 0;
	txData_UART[9]= 0;
	txData_UART[10]=0;
	txData_UART[11]=0;
	txData_UART[12]=0;
	txData_UART[13]=0;
	txData_UART[14]=0;
	txData_UART[15]=0;
	txData_UART[16]=0;
	txData_UART[17]=0;
	txData_UART[18]=0xFB;

}

void KalArm_BLEC_Tx_GEN_GRIP_TEST_STAT_RES(void){
	
	txData_UART[0] = 0xFF;
	txData_UART[1]= 19;
	txData_UART[2]= KalArm_GEN_GRIP_STAT_RES.status;
	txData_UART[3]= KalArm_GEN_GRIP_STAT_RES.grip_accuracy;
	txData_UART[4]= KalArm_GEN_GRIP_STAT_RES.current_thumb_position;
	txData_UART[5]= 0;
	txData_UART[6]= 0;
	txData_UART[7]= 0;
	txData_UART[8]= 0;
	txData_UART[9]= 0;
	txData_UART[10]=0;
	txData_UART[11]=0;
	txData_UART[12]=0;
	txData_UART[13]=0;
	txData_UART[14]=0;
	txData_UART[15]=0;
	txData_UART[16]=0;
	txData_UART[17]=0;
	txData_UART[18]=0xFB;


}
void custom_grip_test(KalArm_CUST_GRIP_STAT_RES_TYPE KalArm_CUST_GRIP_STAT_RES){
	
	if(KalArm_MODE.mode_state == 1){
		if(KalArm_MODE.mode_number == 3){
			switch(cust_grip_execute_id){
				case 1:
					while(HAL_OK != I2C_READBUF(19,KalArm_CLINICIAN_DATA.eeprom_data_array,4));
					break;
				case 2:
					while(HAL_OK != I2C_READBUF(20,KalArm_CLINICIAN_DATA.eeprom_data_array,4));					
					break;
				case 3:
					while(HAL_OK != I2C_READBUF(21,KalArm_CLINICIAN_DATA.eeprom_data_array,4));					
					break;
				case 4:
					while(HAL_OK != I2C_READBUF(22,KalArm_CLINICIAN_DATA.eeprom_data_array,4));					
					break;
				case 5:
					while(HAL_OK != I2C_READBUF(23,KalArm_CLINICIAN_DATA.eeprom_data_array,4));					
					break;
				case 6:
					while(HAL_OK != I2C_READBUF(24,KalArm_CLINICIAN_DATA.eeprom_data_array,4));					
					break;
				
			}
			KalArm_CUST_GRIP_STAT_RES.motor_1_index = KalArm_CLINICIAN_DATA.eeprom_data_array[0];
			KalArm_CUST_GRIP_STAT_RES.motor_2_index = KalArm_CLINICIAN_DATA.eeprom_data_array[1];
			KalArm_CUST_GRIP_STAT_RES.motor_3_index = KalArm_CLINICIAN_DATA.eeprom_data_array[2];
			KalArm_CUST_GRIP_STAT_RES.motor_4_index = KalArm_CLINICIAN_DATA.eeprom_data_array[3];
		
		
		}
			
		
	}

	while(KalArm_SPI_RECEIVE_FLAGS.cgt_result != CGT_PASS && KalArm_SPI_RECEIVE_FLAGS.cgt_result != CGT_FAIL){
				
				i2c_thumb_sensor_routine();
				
				if(RGBC_VALID){
					if(R_NOT_IN_CAL_RANGE || G_NOT_IN_CAL_RANGE || B_NOT_IN_CAL_RANGE){
						color_detected = CYAN;
						KalArm_CUST_GRIP_STAT_RES.current_thumb_position = THUMB_NON_OPPOSITION;
						
					}
					else{
						color_detected = BLACK;
						KalArm_CUST_GRIP_STAT_RES.current_thumb_position = THUMB_OPPOSITION;
					}
			
					
				}
				
				
				if(HAL_GPIO_ReadPin(REQ_CLK_GPIO_Port, REQ_CLK_Pin) == GPIO_PIN_SET){ // send SPI transmit data while there are no receive requests from the motor controller
					// we do not do this inside the spi tx rx complete callback because there it will be oerormed only once when the chanfge of pin state controled by motor controller occurs
					// this transmission must be in total control of the main controller and not at the mercy of the slave controllers clock request hence it is implemented here
					/*Insert fresh Transmit data to send to motor controller*/
					
					if(KalArm_CUST_GRIP_STAT_RES.custom_grip_tx_times % 2 == 0){
						txData_SPI[0] = 0x90;
						txData_SPI[1] = KalArm_CUST_GRIP_STAT_RES.motor_1_index; // Motor Index Counts
						txData_SPI[2] = 0x03; // Motor ID 1
						txData_SPI[3] = 0x0B;
						txData_SPI[3] |= (1<<4) | (1<<5); // COMMAND 3
					}
					else if(KalArm_CUST_GRIP_STAT_RES.custom_grip_tx_times % 3 == 0){
						txData_SPI[0] = 0x90;
						txData_SPI[1] = KalArm_CUST_GRIP_STAT_RES.motor_2_index; // Motor Index Counts
						txData_SPI[2] = 0x03; // Motor ID 2
						txData_SPI[2] |= 1<<2;
						txData_SPI[3] = 0x0B;
						txData_SPI[3] |= (1<<4) | (1<<5); // COMMAND 3
					}
					else if(KalArm_CUST_GRIP_STAT_RES.custom_grip_tx_times % 5 == 0){
						txData_SPI[0] = 0x90;
						txData_SPI[1] = KalArm_CUST_GRIP_STAT_RES.motor_3_index; // Motor Index Counts
						txData_SPI[2] = 0x03; // Motor ID 3
						txData_SPI[2] |= 2<<2;
						txData_SPI[3] = 0x0B;
						txData_SPI[3] |= (1<<4) | (1<<5); // COMMAND 3
					}
					else if(KalArm_CUST_GRIP_STAT_RES.custom_grip_tx_times % 7 == 0){
						txData_SPI[0] = 0x90;
						txData_SPI[1] = KalArm_CUST_GRIP_STAT_RES.motor_4_index; // Motor Index Counts
						txData_SPI[2] = 0x03; // Motor ID 4
						txData_SPI[2] |= 3<<2;
						txData_SPI[3] = 0x0B;	
						txData_SPI[3] |= (1<<4) | (1<<5); // COMMAND 3
					}
					else if(KalArm_CUST_GRIP_STAT_RES.custom_grip_tx_times % 11 == 0){
						txData_SPI[0] = 0x90;
						txData_SPI[1] = KalArm_CUST_GRIP_STAT_RES.thumb_position; // Motor Index Counts
						txData_SPI[2] = 0x03; // expected thumb position
						txData_SPI[2] |= 4<<2;
						txData_SPI[3] = 0x0B;	
						txData_SPI[3] |= (1<<4) | (1<<5); // COMMAND 3
					}
					else if(KalArm_CUST_GRIP_STAT_RES.custom_grip_tx_times % 13 == 0){
						txData_SPI[0] = 0x90;
						txData_SPI[1] = KalArm_CUST_GRIP_STAT_RES.current_thumb_position; // Motor Index Counts
						txData_SPI[2] = 0x03; // current thumb position
						txData_SPI[2] |= 5<<2;
						txData_SPI[3] = 0x0B;	
						txData_SPI[3] |= (1<<4) | (1<<5); // COMMAND 3
					}
					else if(KalArm_CUST_GRIP_STAT_RES.custom_grip_tx_times % 17 == 0){
						txData_SPI[0] = 0x90;
						txData_SPI[1] = KalArm_CUST_GRIP_STAT_RES.grip_execute_order; // Motor Index Counts
						txData_SPI[2] = 0x03; // grip_execute_order
						txData_SPI[2] |= 6<<2;
						txData_SPI[3] = 0x0B;	
						txData_SPI[3] |= (1<<4) | (1<<5); // COMMAND 3
					}
					// Send the Calibration Request Packet 1001 0000 0000 0000 0000 0000 0000 1011
					HAL_SPI_TransmitReceive_IT(&hspi3,txData_SPI,rxData_SPI,4);
				}

				else if((spi_status_flags.receive_spi_data == 1) && (spi_status_flags.send_spi_data == 0)){
					// Insert dummy data to send to the motor controller
					txData_SPI[0] = 0xF0;
					txData_SPI[1] = 0x00;
					txData_SPI[2] = 0x00;
					txData_SPI[3] = 0x0F;
					// send the dummy data packet 1111 0000 0000 0000 0000 0000 0000 1111
					HAL_SPI_TransmitReceive_IT(&hspi3,txData_SPI,rxData_SPI,4);
				}
				
				KalArm_CUST_GRIP_STAT_RES.custom_grip_tx_times++;
				
				if(KalArm_BLE.ble_on_off_flag == 0 && KalArm_MODE.mode_state == 1){
					KalArm_CUST_GRIP_STAT_RES.grip_execute_order = 0;
					KalArm_CUST_GRIP_STAT_RES.custom_grip_tx_times=0;
					KalArm_CUST_GRIP_STAT_RES.grip_accuracy = 0;
					txData_SPI[0] = 0x90;
					txData_SPI[1] = KalArm_CUST_GRIP_STAT_RES.grip_execute_order; // Motor Index Counts
					txData_SPI[2] = 0x03; // grip_execute_order
					txData_SPI[2] |= 6<<2;
					txData_SPI[3] = 0x0B;	
					txData_SPI[3] |= (1<<4) | (1<<5); // COMMAND 3
					// Send the Calibration Request Packet 1001 0000 0000 0000 0000 0000 0000 1011
					HAL_SPI_TransmitReceive_IT(&hspi3,txData_SPI,rxData_SPI,4);
					break;
				}
				
	
	} // end of custom grip while
	

}

void general_grip_test(uint8_t grip_id, KalArm_TS_TYPE thumb_sensor){
	
		while(KalArm_SPI_RECEIVE_FLAGS.ggt_result != GGT_PASS && KalArm_SPI_RECEIVE_FLAGS.ggt_result != GGT_FAIL){
			
				i2c_thumb_sensor_routine();
				
				if(RGBC_VALID){
					if(R_NOT_IN_CAL_RANGE || G_NOT_IN_CAL_RANGE || B_NOT_IN_CAL_RANGE)
						color_detected = CYAN;
					else
						color_detected = BLACK;
					
					KalArm_GEN_GRIP_STAT_RES.current_thumb_position = (color_detected == BLACK)? THUMB_OPPOSITION: THUMB_NON_OPPOSITION;
				}
			if(HAL_GPIO_ReadPin(REQ_CLK_GPIO_Port, REQ_CLK_Pin) == GPIO_PIN_SET){ // send SPI transmit data while there are no receive requests from the motor controller
					// we do not do this inside the spi tx rx complete callback because there it will be oerormed only once when the chanfge of pin state controled by motor controller occurs
					// this transmission must be in total control of the main controller and not at the mercy of the slave controllers clock request hence it is implemented here
					/*Insert fresh Transmit data to send to motor controller*/
					txData_SPI[0] = 0x90;
					txData_SPI[1] = 0x00;
					txData_SPI[2] = 0x00;
					txData_SPI[3] = 0x0B;
					txData_SPI[3] |= 1<<6;
					txData_SPI[2] |=  KalArm_GEN_GRIP_STAT_RES.grip_id << 2;
					txData_SPI[1] |= KalArm_GEN_GRIP_STAT_RES.grip_execute_order;
					// Send the Calibration Request Packet 1001 0000 0000 0000 0000 0000 0000 1011
					HAL_SPI_TransmitReceive_IT(&hspi3,txData_SPI,rxData_SPI,4);
				}
				else if((spi_status_flags.receive_spi_data == 1) && (spi_status_flags.send_spi_data == 0)){
					// Insert dummy data to send to the motor controller
					txData_SPI[0] = 0xF0;
					txData_SPI[1] = 0x00;
					txData_SPI[2] = 0x00;
					txData_SPI[3] = 0x0F;
					// send the dummy data packet 1111 0000 0000 0000 0000 0000 0000 1111
					HAL_SPI_TransmitReceive_IT(&hspi3,txData_SPI,rxData_SPI,4);
				}
				
				if(KalArm_BLE.ble_on_off_flag == 0 && KalArm_MODE.mode_state == 1){
					KalArm_GEN_GRIP_STAT_RES.grip_execute_order = 0;
					KalArm_GEN_GRIP_STAT_RES.general_grip_tx_times=0;
					KalArm_GEN_GRIP_STAT_RES.grip_accuracy = 0;
					txData_SPI[0] = 0x90;
					txData_SPI[1] = 0x00;
					txData_SPI[2] = 0x00;
					txData_SPI[3] = 0x0B;
					txData_SPI[3] |= 1<<6;
					txData_SPI[2] |= KalArm_GEN_GRIP_STAT_RES.grip_id << 2;
					txData_SPI[1] |= KalArm_GEN_GRIP_STAT_RES.grip_execute_order;
					// Send the Calibration Request Packet 1001 0000 0000 0000 0000 0000 0000 1011
					HAL_SPI_TransmitReceive_IT(&hspi3,txData_SPI,rxData_SPI,4);
					break;
				}
				
		} // end of general grip while
	

	
}


 uint8_t I2C_READBUF(uint8_t *pAddr, uint8_t *pData, uint16_t len){
		uint16_t addr = *pAddr;
		if(HAL_I2C_Mem_Read_DMA(&hi2c2, EEPROM_ADD|0x01, addr, I2C_MEMADD_SIZE_8BIT, pData, len) != HAL_OK)
		return HAL_BUSY; 
		return HAL_OK;
	  
}
 
 
 uint8_t I2C_WRITEBUF(uint8_t *pAddr, uint8_t *pData, uint16_t len){
		uint16_t addr = *pAddr; 
	
		if(HAL_I2C_Mem_Write_DMA(&hi2c2, EEPROM_ADD|0x00, addr, I2C_MEMADD_SIZE_8BIT, pData, len) != HAL_OK) 
		return HAL_BUSY; 
		return HAL_OK;
	
		
}

void KalArm_BLEC_Tx_CLINICIAN_ACK(void){
	
	txData_UART[0] = 0xFF;
	txData_UART[1]= 0;
	for(int k= 2; k<=11; k++){
		txData_UART[k] = KalArm_CLINICIAN_DATA.eeprom_data_array[k-2];
	}
	txData_UART[18]=0xFB;

}

void clinician_parameters_load(void){



	while(HAL_OK != I2C_READBUF(18,KalArm_CLINICIAN_DATA.eeprom_data_array,10));
	while(HAL_OK != I2C_READBUF(18,KalArm_CLINICIAN_DATA.eeprom_data_array,10));
	
	KalArm_PATTERN_RECOGNITION_SUITE_PARAMS.open_threshold = CLINICIAN_DATA_OPEN_THRESHOLD;
	KalArm_PATTERN_RECOGNITION_SUITE_PARAMS.close_threshold = CLINICIAN_DATA_CLOSE_THRESHOLD;
	KalArm_PATTERN_RECOGNITION_SUITE_PARAMS.pulse_period = CLINICIAN_DATA_PULSE_PERIOD;
	KalArm_PATTERN_RECOGNITION_SUITE_PARAMS.double_impulse_period = CLINICIAN_DATA_DOUBLE_IMPULSE_PERIOD;
	KalArm_PATTERN_RECOGNITION_SUITE_PARAMS.triple_impulse_period = CLINICIAN_DATA_TRIPLE_IMPULSE_PERIOD;
	KalArm_PATTERN_RECOGNITION_SUITE_PARAMS.analysis_period = KalArm_PATTERN_RECOGNITION_SUITE_PARAMS.triple_impulse_period + 300;

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
