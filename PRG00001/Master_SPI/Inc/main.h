/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_pwr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef union {
    uint16_t position;
    uint8_t bytes[2];
} encoder_index_type;

typedef union {
    uint16_t adc_value;
    uint8_t bytes[2];
} motor_current_type;

typedef struct{

	uint8_t send_spi_data;
	uint8_t receive_spi_data;
	
}KalArm_SPI_COMM_STATUS_FLAGS;



typedef enum
{
  TCS34725_INTEGRATIONTIME_2_4MS  = 0xFF,   /**<  2.4ms - 1 cycle    - Max Count: 1024  */
  TCS34725_INTEGRATIONTIME_24MS   = 0xF6,   /**<  24ms  - 10 cycles  - Max Count: 10240 */
  TCS34725_INTEGRATIONTIME_50MS   = 0xEB,   /**<  50ms  - 20 cycles  - Max Count: 20480 */
  TCS34725_INTEGRATIONTIME_101MS  = 0xD5,   /**<  101ms - 42 cycles  - Max Count: 43008 */
  TCS34725_INTEGRATIONTIME_154MS  = 0xC0,   /**<  154ms - 64 cycles  - Max Count: 65535 */
  TCS34725_INTEGRATIONTIME_700MS  = 0x00    /**<  700ms - 256 cycles - Max Count: 65535 */
}
TCS34725IntegrationTime_t;

typedef enum
{
  TCS34725_GAIN_1X                = 0x00,   /**<  No gain  */
  TCS34725_GAIN_4X                = 0x01,   /**<  4x gain  */
  TCS34725_GAIN_16X               = 0x02,   /**<  16x gain */
  TCS34725_GAIN_60X               = 0x03    /**<  60x gain */
}
TCS34725Gain_t;

typedef enum{
	
	MAGENTA=0,
	CYAN = 1,
	BLACK = 2,
	YELLOW = 3

}TS_Color;


typedef struct{
	
	uint8_t R_CAL;
	uint8_t G_CAL;
	uint8_t B_CAL;

}KalArm_TS_TYPE;

typedef enum{

	THUMB_NON_OPPOSITION=1,
	THUMB_OPPOSITION = 2
	
}KalArm_THUMB_POSITION_TYPE;



typedef struct{
	uint8_t mode_state; // for seeing whether a particular mode is in operation
	uint8_t mode_number; // for keeping track of mode number
}KalArm_MODE_TYPE;

typedef struct{
	
	uint8_t ble_on_off_flag;
	uint8_t ble_advertising_flag;
	uint8_t ble_connection_request_flag;
	uint8_t ble_connected_flag;

}KalArm_BLE_TYPE;

typedef struct{
	uint32_t ms; // timer ticks of the MFSW timer
	uint32_t ms1; // for seeing whether a particular mode is in operation
	uint32_t ms2; // for switch push or pull state
	uint16_t window;
	uint16_t haptic_window;
	uint8_t view_battery_flag;
	uint8_t mode_change_flag;
	uint8_t ble_on_off_flag;
	
}KalArm_TIME_TYPE;

typedef struct{
	
	uint8_t ingrip;
	uint8_t grip_id;

}KalArm_GRIP_TYPE;


typedef struct{
	
	uint8_t battery_status_flag;
	uint32_t battery_status_flag_start_time;
	double battery_percentage;
	double battery_voltage;
	double battery_voltage_raw;
	double battery_temperature;
	double battery_temperature_raw;
	uint8_t battery_critical;

}KalArm_BATTERY_TYPE;

typedef struct{

	uint8_t board_status_flag;
	uint32_t board_status_flag_start_time;
	double board_percentage;
	double board_voltage;
	double board_voltage_raw;
	double board_temperature;
	double board_temperature_raw;
	uint8_t board_critical;

}KalArm_BOARD_TEMP_TYPE;


typedef enum{
	
	UNDEFINED = 0,
	SINGLE_IMPULSE = 1,
	DOUBLE_IMPULSE = 2,
	TRIPLE_IMPULSE = 3
	
}KalArm_PATTERN_TYPE;

typedef struct{
	
	uint16_t open_threshold;
	uint16_t close_threshold;
	uint16_t pulse_period;
	uint16_t double_impulse_period;
	uint16_t triple_impulse_period;
	uint16_t analysis_period;

}KalArm_PATTERN_RECOGNITION_SUITE_PARAMS_TYPE;

typedef struct{
	
	volatile uint8_t pattern_update_event;
	uint8_t emg_state_flag;
	uint8_t pre_emg_state_flag;
	uint16_t pulse_period;
	uint16_t total_pulse_period;
	uint16_t avg_pulse_period;
	KalArm_PATTERN_TYPE pre_pattern;
	KalArm_PATTERN_TYPE pattern;
	uint32_t pre_peak_stamp;
	uint32_t peak_stamp;
	uint16_t analyseTick;
	uint16_t sensor_val;
	uint8_t peak_count;
	uint16_t glitch;
	uint8_t emg_prop_flag;
	uint16_t ms;
	

}KalArm_EMG_TYPE;




typedef enum{

CALIBRATION_PASS = 1,
CALIBRATION_IN_PROGRESS = 2,
CALIBRATION_FAIL = 3
	
}KalArm_CALIBRATION_STATUS;

typedef enum{
	CGT_NOT_INIT = 0,
	CGT_PASS = 1,
	CGT_IN_PROGRESS = 2,
	CGT_FAIL = 3

}KalArm_CGT_STATUS;

typedef enum{
	
	GGT_NOT_INIT = 0,
	GGT_PASS = 1,
	GGT_IN_PROGRESS = 2,
	GGT_FAIL = 3

}KalArm_GGT_STATUS;

typedef enum{

EMG_CONTROL_PASS = 1,
EMG_CONTROL_FAIL = 0

}KalArm_EMG_CONTROL_STATUS;

typedef struct{
	
	uint8_t grip_accuracy;
	uint8_t grip_id_ga;

}KalArm_GRIP_TEST_TYPE;

typedef struct{
	
	uint8_t motor_1_status;
	uint8_t motor_2_status;
	uint8_t motor_3_status;
	uint8_t motor_4_status;
	uint8_t motor_1_cs;
	uint8_t motor_2_cs;
	uint8_t motor_3_cs;
	uint8_t motor_4_cs;
	uint8_t motor_1_en;
	uint8_t motor_2_en;
	uint8_t motor_3_en;
	uint8_t motor_4_en;
	
	uint8_t calibration_times;


}KalArm_CALIBRATION_INFO_TYPE;

typedef struct{
	
	uint8_t status;
	uint8_t grip_accuracy;
	uint8_t thumb_position;
	KalArm_THUMB_POSITION_TYPE current_thumb_position;
	uint8_t grip_execute_order;
	uint8_t grip_id;
	uint8_t custom_grip_tx_times;
	uint8_t motor_1_index;
	uint8_t motor_2_index;
	uint8_t motor_3_index;
	uint8_t motor_4_index;


}KalArm_CUST_GRIP_STAT_RES_TYPE;

typedef struct{
	
	uint8_t status;
	uint8_t grip_accuracy;
	uint8_t thumb_position;
	KalArm_THUMB_POSITION_TYPE current_thumb_position;
	uint8_t grip_execute_order;
	uint8_t grip_id;
	uint8_t general_grip_tx_times;


}KalArm_GEN_GRIP_STAT_RES_TYPE;

typedef struct{
	
	KalArm_CALIBRATION_STATUS calibration_result;
	KalArm_CALIBRATION_INFO_TYPE KalArm_CALIBRATION_INFO;
	uint8_t in_grip;
	uint8_t driver_error_1;
	uint8_t driver_error_2;
	uint8_t power_good;
	uint8_t add_grip_success;
	uint8_t delete_grip_success;
	uint8_t threshold_write_success;
	uint8_t pulse_period_write_success;
	uint8_t mode_change_success;
	uint8_t emg_control_feedback;
	uint8_t grip_id;
	uint8_t feedback_grip_id;
	KalArm_CUST_GRIP_STAT_RES_TYPE KalArm_CUST_GRIP_STAT_RES;
	KalArm_GEN_GRIP_STAT_RES_TYPE KalArm_GEN_GRIP_STAT_RES;
	KalArm_CGT_STATUS cgt_result;
	KalArm_GGT_STATUS ggt_result;
	

}KalArm_SPI_RECEIVE_TYPE;




typedef struct{
	
	uint8_t charger_state;

}KalArm_CHG_DET_TYPE;

typedef enum{
	
	DUMP_PARAMETERS,
	GEN_GRIP_TEST,
	CUST_GRIP_TEST,
	
}KalArm_LAST_COMMAND_STATE_TYPE;

typedef struct{
	uint8_t data_fresh;
	uint16_t open_max_threshold;
	uint16_t close_max_threshold;
	uint16_t open_threshold;
	uint16_t close_threshold;
	uint16_t pulse_period;
	uint16_t double_impulse_period;
	uint16_t triple_impulse_period;
	uint8_t data_array[10];
	uint8_t eeprom_data_array[10];

}KalArm_CLINICIAN_DATA_TYPE;

typedef struct{
	uint8_t data_fresh;
	uint8_t grip_id;
	uint8_t motor_1_index;
	uint8_t motor_2_index;
	uint8_t motor_3_index;
	uint8_t motor_4_index;
	uint8_t data_array[5];
	uint8_t eeprom_data_array[5];

}KalArm_CUST_GRIP_DATA_TYPE;


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void SystemClock_Config_16(void);
void SystemClock_Config_64(void);
void UART_Config_Init_115200(UART_HandleTypeDef huart);
void UART_Config_Init_9600(UART_HandleTypeDef huart);
void SPI_Config_Init_2MBPS(SPI_HandleTypeDef hspi);
void SPI_Config_Init_8MBPS(SPI_HandleTypeDef hspi);
void I2C_Config_Init_3KHZ(I2C_HandleTypeDef hi2c2); 
void I2C_Config_Init_100KHZ(I2C_HandleTypeDef hi2c2); 

void clearSPIRxData(void);
void clearSPITxData(void);
void spi_receive_handler(void);

uint8_t i2c_thumb_sensor_setup(void);
void i2c_thumb_sensor_routine(void);

void TCS34725_Calibrate(void);

uint16_t DEV_I2C_ReadWord(uint8_t add_);
void TCS34725_Get_RGBData(void);

void battery_voltage_config_init(uint8_t init_deinit);
void battery_temperature_config_init(uint8_t init_deinit);



uint8_t debounce(void);



void reset_rgb_LED(void);
void set_rgb_LED(void);
void set_rgb_LED_ingrip(void);

void reset_hand_LED(void);
void set_hand_LED(void);

void reset_ble_LED(void);
void set_ble_LED(void);
void toggle_bleMode_LED(void);


void low_battery_LED(void);
void cal_in_progress_LED(void);
void cal_fail_LED(void);
void over_temp_LED(void);
void in_grip_LED(void);

void bat_charge_25_LED(void);
void bat_charge_50_LED(void);
void bat_charge_75_LED(void);
void bat_charge_100_LED(void);

void mode_1_LED(void);
void mode_1_LED_toggle(void);
void mode_2_LED(void);
void mode_2_LED_toggle(void);
void mode_3_LED(void);
void mode_3_LED_toggle(void);
void mode_4_LED(void);
void mode_4_LED_toggle(void);

void set_R_LED(uint8_t duty_cycle);
void toggle_R_LED(void);
void set_G_LED(uint8_t duty_cycle);
void toggle_G_LED(void);
void set_B_LED(uint8_t duty_cycle);
void toggle_B_LED(void);


void emg_sensor_open_config_init(uint8_t config_init);
void emg_sensor_close_config_init(uint8_t config_init);

KalArm_CALIBRATION_STATUS calibration_request(void);
KalArm_EMG_CONTROL_STATUS emg_control(KalArm_EMG_TYPE KalArm_EMG_OPEN, KalArm_EMG_TYPE KalArm_EMG_CLOSE);


void KalArm_BLEC_Tx_BATT_Board(void);
void KalArm_BLEC_Tx_EMG_Viz(void);
void KalArm_BLEC_Tx_CAL_INFO(void);
void KalArm_BLEC_Tx_GEN_GRIP_TEST_STAT_RES(void);
void KalArm_BLEC_Tx_CUST_GRIP_TEST_STAT_RES(void);
void KalArm_BLEC_Tx_CLINICIAN_ACK(void);

void custom_grip_test(KalArm_CUST_GRIP_STAT_RES_TYPE KalArm_CUST_GRIP_STAT_RES);
void custom_grip_execute(uint8_t grip_id, KalArm_CUST_GRIP_STAT_RES_TYPE KalArm_CUST_GRIP_STAT_RES);
void general_grip_test(uint8_t grip_id, KalArm_TS_TYPE thumb_sensor);



uint8_t I2C_READBUF(uint8_t *pAddr, uint8_t *pData, uint16_t len);
uint8_t I2C_WRITEBUF(uint8_t *pAddr, uint8_t *pData, uint16_t len);

void clinician_parameters_load(void);



/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LOD1_N_Pin GPIO_PIN_13
#define LOD1_N_GPIO_Port GPIOC
#define LOD2_P_Pin GPIO_PIN_14
#define LOD2_P_GPIO_Port GPIOC
#define LOD2_N_Pin GPIO_PIN_15
#define LOD2_N_GPIO_Port GPIOC
#define BAT_TEMP_Pin GPIO_PIN_0
#define BAT_TEMP_GPIO_Port GPIOA
#define CHG_DET_Pin GPIO_PIN_2
#define CHG_DET_GPIO_Port GPIOA
#define CHG_DET_EXTI_IRQn EXTI2_IRQn
#define LED_R_Pin GPIO_PIN_5
#define LED_R_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_6
#define LED_G_GPIO_Port GPIOA
#define LED_B_Pin GPIO_PIN_7
#define LED_B_GPIO_Port GPIOA
#define SW_Pin GPIO_PIN_4
#define SW_GPIO_Port GPIOC
#define SW_EXTI_IRQn EXTI4_IRQn
#define EMG_1_Pin GPIO_PIN_1
#define EMG_1_GPIO_Port GPIOB
#define MOT_RST_Pin GPIO_PIN_2
#define MOT_RST_GPIO_Port GPIOB
#define EMG_1_EN_Pin GPIO_PIN_10
#define EMG_1_EN_GPIO_Port GPIOB
#define EMG_2_Pin GPIO_PIN_12
#define EMG_2_GPIO_Port GPIOB
#define EMG_2_EN_Pin GPIO_PIN_13
#define EMG_2_EN_GPIO_Port GPIOB
#define BAT_VOL_Pin GPIO_PIN_15
#define BAT_VOL_GPIO_Port GPIOB
#define RST_BLE_Pin GPIO_PIN_6
#define RST_BLE_GPIO_Port GPIOC
#define LOD1_P_Pin GPIO_PIN_10
#define LOD1_P_GPIO_Port GPIOA
#define HS_BLE_Pin GPIO_PIN_11
#define HS_BLE_GPIO_Port GPIOA
#define STAT_BLE_Pin GPIO_PIN_12
#define STAT_BLE_GPIO_Port GPIOA
#define STAT_MOT_Pin GPIO_PIN_6
#define STAT_MOT_GPIO_Port GPIOB
#define STAT_MOT_EXTI_IRQn EXTI9_5_IRQn
#define REQ_CLK_Pin GPIO_PIN_7
#define REQ_CLK_GPIO_Port GPIOB
#define REQ_CLK_EXTI_IRQn EXTI9_5_IRQn
#define TS_INT_Pin GPIO_PIN_9
#define TS_INT_GPIO_Port GPIOB
#define TS_INT_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

//#define V_REF_ADC (double)3.3
//#define VOLTAGE_DIVIDER_FACTOR (double)(14/4.7)

#define HEADER 0x90
#define FOOTER 0x0B

#define TCS34725_ADDRESS          (0x29<<1)
#define TCS34725_CMD_BIT          0x80

#define TCS34725_CDATAL           0x14    /* Clear channel data */
#define TCS34725_CDATAH           0x15
#define TCS34725_RDATAL           0x16    /* Red channel data */
#define TCS34725_RDATAH           0x17
#define TCS34725_GDATAL           0x18    /* Green channel data */
#define TCS34725_GDATAH           0x19
#define TCS34725_BDATAL           0x1A    /* Blue channel data */
#define TCS34725_BDATAH           0x1B

#define TCS34725_CMD_Read_Word    0x20

#define TCS34725_ID               0x12

#define TCS34725_ATIME            0x01

#define TCS34725_ENABLE           0x00     
#define TCS34725_ENABLE_AIEN      0x10    /* RGBC Interrupt Enable */
#define TCS34725_ENABLE_WEN       0x08     /* Wait enable - Writing 1 activates the wait timer */
#define TCS34725_ENABLE_AEN       0x02     /* RGBC Enable - Writing 1 actives the ADC, 0 disables it */
#define TCS34725_ENABLE_PON       0x01    /* Power on - Writing 1 activates the internal oscillator, 0 disables it */

#define TCS34725_AILTL            0x04    /* Clear channel lower interrupt threshold */
#define TCS34725_AILTH            0x05
#define TCS34725_AIHTL            0x06    /* Clear channel upper interrupt threshold */
#define TCS34725_AIHTH            0x07

#define TCS34725_CONTROL          0x0F    /* Set the gain level for the sensor */

#define TCS34725_PERS             0x0C    /* Persistence register - basic SW filtering mechanism for interrupts */
#define TCS34725_PERS_60_CYCLE    0x0f  /* 60 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_2_CYCLE     0x02  /* 2 clean channel values outside threshold range generates an interrupt  */

#define RGBC_VALID 1//((R>0 && R < 30)&& (B > 0 && B< 30) && (G> 0 && G< 30) && C>0)
#define IS_CYAN R< ((G+B)/2)
#define IS_MAGENTA G<((R+B)/2)
#define IS_YELLOW B<((R+G)/2)
#define IS_BLACK (B == G) && (R == G)


#define DEV_Delay_ms(__xms)	HAL_Delay(__xms)


#define KALARM_THRESHOLD_OPEN 1200
#define KALARM_THRESHOLD_CLOSE 1200
#define KALARM_ANALYSIS_PERIOD 1500
#define KALARM_PULSE_PERIOD 300
#define KALARM_DOUBLE_IMPULSE_PERIOD 700
#define KALARM_TRIPLE_IMPULSE_PERIOD 1200

#define COMMAND ((rxData_SPI[3] >> 4) & 0x07)

#define CAL_IN_PGRESS ((rxData_SPI[1]&0x10) >> 4)
#define CAL_TIMES (rxData_SPI[1] >> 5)
#define CAL_P_F (rxData_SPI[3] >> 0x07)

#define CAL_M1 (rxData_SPI[2] & 0x01)
#define CAL_M2 ((rxData_SPI[2] & 0x02) >> 1)
#define CAL_M3 ((rxData_SPI[2] & 0x04) >> 2)
#define CAL_M4 ((rxData_SPI[2] & 0x08) >> 3)
#define CAL_M1_CS ((rxData_SPI[2] & 0x10) >> 4)
#define CAL_M2_CS ((rxData_SPI[2] & 0x20) >> 5)
#define CAL_M3_CS ((rxData_SPI[2] & 0x40) >> 6)
#define CAL_M4_CS (rxData_SPI[2] >> 7)
#define CAL_M1_EN (rxData_SPI[1] & 0x01)
#define CAL_M2_EN ((rxData_SPI[1] & 0x02) >> 1)
#define CAL_M3_EN ((rxData_SPI[1] & 0x04) >> 2)
#define CAL_M4_EN ((rxData_SPI[1] & 0x08) >> 3)



#define IN_GRIP (rxData_SPI[3] >> 0x07)

#define GRIP_ID (rxData_SPI[2] & 0x1F)

#define DEF1 (rxData_SPI[3] >> 0x07)
#define DEF2 (rxData_SPI[2] & 0x01)
#define PG (rxData_SPI[2] & 0x02) >>1
#define GRIP_ID_FEEDBACK ((rxData_SPI[2] & 0x7C) >>2)
#define ADD_S (rxData_SPI[2] >> 7)
#define DEL_S (rxData_SPI[1] & 0x01)
#define TH_S (rxData_SPI[1] & 0x02) >> 1
#define PP_S (rxData_SPI[1] & 0x04) >> 2
#define MODE_S (rxData_SPI[1] & 0x08) >> 3 // mode change success
#define EMG_S (rxData_SPI[1] & 0x10) >> 4
#define GRIP_ACCURACY (rxData_SPI[3] >> 7)|(0x3F &rxData_SPI[2])
#define GRIP_ID_GA (rxData_SPI[2] >> 6) | (rxData_SPI[1] & 7)

#define CGT_STAT_RES (rxData_SPI[1] >> 4)
#define CGT_ACCURACY ((0x3F&rxData_SPI[2])<<1)|(rxData_SPI[3]>>7)
#define GGT_STAT_RES (rxData_SPI[1] >> 4)
#define GGT_ACCURACY ((0x3F&rxData_SPI[2])<<1)|(rxData_SPI[3]>>7)

#define R_NOT_IN_CAL_RANGE R<KalArm_TS.R_CAL-3 || R > KalArm_TS.R_CAL+3
#define G_NOT_IN_CAL_RANGE G<KalArm_TS.G_CAL-3 || G > KalArm_TS.G_CAL+3
#define B_NOT_IN_CAL_RANGE B<KalArm_TS.B_CAL-3 || B > KalArm_TS.B_CAL+3

#define CLINICIAN_OPEN_THRESHOLD (rxData_UART[2]<<8) | (rxData_UART[3])
#define CLINICIAN_CLOSE_THRESHOLD (rxData_UART[4]<<8) | (rxData_UART[5])
#define CLINICIAN_PULSE_PERIOD (rxData_UART[6]<<8) | (rxData_UART[7])
#define CLINICIAN_DOUBLE_IMPULSE_PERIOD (rxData_UART[8]<<8) | (rxData_UART[9])
#define CLINICIAN_TRIPLE_IMPULSE_PERIOD (rxData_UART[10]<<8) | (rxData_UART[11])


#define EEPROM_ADD 0xA0

#define CUST_GRIP_DATA_GRIP_ID rxData_UART[1]

#define CUST_GRIP_DATA_M1_INDEX rxData_UART[2]
#define CUST_GRIP_DATA_M2_INDEX rxData_UART[3]
#define CUST_GRIP_DATA_M3_INDEX rxData_UART[4]
#define CUST_GRIP_DATA_M4_INDEX rxData_UART[5]

#define CLINICIAN_DATA_OPEN_THRESHOLD ((KalArm_CLINICIAN_DATA.eeprom_data_array[0]<<8) | (KalArm_CLINICIAN_DATA.eeprom_data_array[1]))
#define CLINICIAN_DATA_CLOSE_THRESHOLD ((KalArm_CLINICIAN_DATA.eeprom_data_array[2]<<8) | (KalArm_CLINICIAN_DATA.eeprom_data_array[3]))
#define CLINICIAN_DATA_PULSE_PERIOD ((KalArm_CLINICIAN_DATA.eeprom_data_array[4]<<8) | (KalArm_CLINICIAN_DATA.eeprom_data_array[5]))
#define CLINICIAN_DATA_DOUBLE_IMPULSE_PERIOD ((KalArm_CLINICIAN_DATA.eeprom_data_array[6]<<8) | (KalArm_CLINICIAN_DATA.eeprom_data_array[7]))
#define CLINICIAN_DATA_TRIPLE_IMPULSE_PERIOD ((KalArm_CLINICIAN_DATA.eeprom_data_array[8]<<8) | (KalArm_CLINICIAN_DATA.eeprom_data_array[9]))

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
