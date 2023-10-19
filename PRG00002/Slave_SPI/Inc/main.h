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
#include <stdlib.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef enum{

	DLS_0, DLS_1, DLS_2, DLS_3, DLS_4, DLS_5, DLS_6, DLS_7, DLS_8

}KalArmActuatorDLS;

typedef enum{


	CLOCKWISE=1,
	ACLOCKWISE=2,
	CLOCKWISE_LIMIT=3,
	ACLOCKWISE_LIMIT=4,
	HALT=5,
	CLOCKWISE_LIMIT_ADAPTIVE=6,
	ACLOCKWISE_LIMIT_ADAPTIVE=7,

}KalMotorDir;


typedef struct{
	
	TIM_HandleTypeDef *encoder_handle;
	KalMotorDir motor_direction;
	uint16_t current_index_counts;
	uint16_t previous_index_counts;
	uint16_t limit_index_counts;
	uint8_t current_sub_counts;
	uint16_t target_encoder_index_counts;
	uint8_t target_encoder_sub_counts;
	uint32_t current_current_sense;
	uint32_t pre_current_sense;
	uint32_t current_cutoff_window;
	uint32_t current_cutoff_pre_stamp;
	uint32_t current_check_filter_stamp;
	
	long current_diff;
	long previous_diff;
	long curve;


}KalArmMotorHandle;


typedef struct{
	uint8_t mode_state; // for seeing whether a particular mode is in operation
	uint8_t mode_number; // for keeping track of mode number
	uint8_t mode_change;
	uint8_t mode_3_grip_id;
	uint8_t mode_4_grip_id;
}KalArm_MODE_TYPE;

typedef enum{
	
	UNDEFINED=0,
	SINGLE_IMPULSE = 1,
	DOUBLE_IMPULSE = 2,
	TRIPLE_IMPULSE = 3
	
}KalArm_PATTERN_TYPE;

typedef enum{

	THUMB_NON_OPPOSITION=1,
	THUMB_OPPOSITION = 2
	
}KalArm_THUMB_POSITION_TYPE;

typedef enum{

	TRIGGER_SET =1,
	TRIGGER_NOT_SET =0
	

}KalArm_TRIGGER_TYPE;

typedef struct{
	
	uint8_t pattern_update_event;
	uint8_t pattern_update_event_emg_control;
	KalArm_PATTERN_TYPE pre_pattern;
	KalArm_PATTERN_TYPE pattern;
	uint8_t emg_prop_flag;
	KalArm_TRIGGER_TYPE trigger;
	

}KalArm_EMG_TYPE;

/*KalArm Grip Classification based on trigger or non trigger grip*/
typedef enum{ //type enum KalArm_GRIP_CLASS
	GRIP_TRIGGER,
	GRIP_NON_TRIGGER
}KalArm_GRIP_CLASS;
/*KalArm Motor Position Tracking Structure*/
typedef struct{ //type KalArm_GRIP_MOT_POS

	uint16_t target_encoder_index_counts;

	
}KalArm_GRIP_MOT_POS;

/*KalArm GRIP address in the EEPROM for writing grip usage stats*/

typedef enum{ //type GRIP_ADDRESSES EEPROM

	GRIP_0=0,
	GRIP_1=1,
	GRIP_2=2,
	GRIP_3=3,
	GRIP_4=4,
	GRIP_5=5,
	GRIP_6=6,
	GRIP_7=7,
	GRIP_8=8,
	GRIP_9=9,
	GRIP_10=10,
	GRIP_11=11,
	GRIP_12=12,
	GRIP_13=13,
	GRIP_14=14,
	GRIP_15=15,
	GRIP_16=16,
	GRIP_17=17,
	GRIP_18=18,
	GRIP_19=19,
	GRIP_20=20,
	GRIP_21=21,
	GRIP_22=22,
	GRIP_23=23,
	

}GRIP_ADDRESSES;

/* KalArm Grip Address to Grip ID Map*/

typedef enum{ //type KalArm_GRIP_ID

	POWER = GRIP_0,
	HANDSHAKE = GRIP_1, 
	HOOK= GRIP_2,
	ADDUCTION = GRIP_3, 
	OPEN_PALM = GRIP_4, 
	HELLO = GRIP_5, 
	RELAXED = GRIP_6,
	KEY = GRIP_7,
	TWO_FINGER_TRIG = GRIP_8,
	FINGER_POINT = GRIP_9, 
	PRECISION_CLOSE = GRIP_10, 
	FLEXI = GRIP_11,
	ADAPTIVE = GRIP_12, 
	SPHERICAL = GRIP_13, 
	DON_DOFF = GRIP_14, 
	MOUSE = GRIP_15,
	TRIPOD_OPEN = GRIP_16,
	TRIPOD_CLOSE = GRIP_17,
	CG_1 = GRIP_18, 
	CG_2 = GRIP_19,
	CG_3 = GRIP_20, 
	CG_4 = GRIP_21, 
	CG_5 = GRIP_22,
	CG_6 = GRIP_23
	
}KalArm_GRIP_ID;




/*type definition to store details of both inbuilt and custom grips in one place*/

typedef struct{ //type GRIP_PATTERN_DETAILS
	
	KalArm_GRIP_ID grip_id; //the grip ID in disguise is the EEPROM address itself
	KalArm_THUMB_POSITION_TYPE grip_thumb_config;
	KalArm_GRIP_CLASS grip_class;
	KalArm_GRIP_MOT_POS motor1_position;
	KalArm_GRIP_MOT_POS motor2_position;
	KalArm_GRIP_MOT_POS motor3_position;
	KalArm_GRIP_MOT_POS motor4_position;

}KalArm_GRIP_PATTERN_DETAILS;



typedef struct{
	
	uint8_t status;
	uint8_t grip_accuracy;
	uint8_t thumb_position;
	KalArm_THUMB_POSITION_TYPE current_thumb_position;
	uint8_t grip_execute_order;
	uint8_t grip_id;
	uint8_t custom_grip_tx_times;
	uint16_t motor_1_index;
	uint16_t motor_2_index;
	uint16_t motor_3_index;
	uint16_t motor_4_index;


}KalArm_CUST_GRIP_STAT_RES_TYPE;

typedef struct{
	
	uint8_t status;
	uint8_t grip_accuracy;
	uint8_t thumb_position;
	uint8_t grip_execute_order;
	uint8_t grip_id;
	uint8_t general_grip_tx_times;


}KalArm_GEN_GRIP_STAT_RES_TYPE;



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

void spi_receive_handler(void);
void spi_transmit_handler(void);

void motor_1_set(KalMotorDir direction, uint16_t counts);
void motor_2_set(KalMotorDir direction, uint16_t counts);
void motor_3_set(KalMotorDir direction, uint16_t counts);
void motor_4_set(KalMotorDir direction, uint16_t counts);

void motor_1_reset(void);
void motor_2_reset(void);
void motor_3_reset(void);
void motor_4_reset(void);

void motor_1_current_config_init(uint8_t init_deinit);
void motor_2_current_config_init(uint8_t init_deinit);
void motor_3_current_config_init(uint8_t init_deinit);
void motor_4_current_config_init(uint8_t init_deinit);

void motor_1_encoder_config_init(uint8_t init_deinit);
void motor_2_encoder_config_init(uint8_t init_deinit);
void motor_3_encoder_config_init(uint8_t init_deinit);
void motor_4_encoder_config_init(uint8_t init_deinit);

void current_sense_init(uint8_t init_deinit);
void motors_init(uint8_t init_deinit);

void calibration_filter(uint8_t calibration_factor);
void calibration_routine(void);

void adaptive_filter(uint8_t adaptive_factor);
void adaptive_routine(void);

void finger_current_cutoff(uint8_t cutoff_factor);

void execute_custom_grip(void);

void grip_test_1_(void);
void grip_test_2_(void);
void grip_test_3_(void);
void grip_test_4_(void);
void grip_test_5_(void);
void grip_test_6_(void);
void grip_test_7_(void);
void grip_test_8_(void);

void KalArm_MODE_2_OPPOSED_CLOSE_ROUTINE(void);

void prep_arm(void);

void mode_1_emg_prop_open(void);
void mode_2_emg_prop_open(void);
void mode_3_emg_prop_open(void);
void mode_4_emg_prop_open(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STAT_MOT_Pin GPIO_PIN_13
#define STAT_MOT_GPIO_Port GPIOC
#define M4_EA_Pin GPIO_PIN_0
#define M4_EA_GPIO_Port GPIOA
#define M4_EB_Pin GPIO_PIN_1
#define M4_EB_GPIO_Port GPIOA
#define M4_PP_Pin GPIO_PIN_2
#define M4_PP_GPIO_Port GPIOA
#define M4_PN_Pin GPIO_PIN_3
#define M4_PN_GPIO_Port GPIOA
#define M2_EB_Pin GPIO_PIN_4
#define M2_EB_GPIO_Port GPIOA
#define M4_EI_Pin GPIO_PIN_5
#define M4_EI_GPIO_Port GPIOA
#define M3_PP_Pin GPIO_PIN_6
#define M3_PP_GPIO_Port GPIOA
#define M3_EI_Pin GPIO_PIN_4
#define M3_EI_GPIO_Port GPIOC
#define DEF1_Pin GPIO_PIN_1
#define DEF1_GPIO_Port GPIOB
#define DEF1_EXTI_IRQn EXTI1_IRQn
#define DEF2_Pin GPIO_PIN_2
#define DEF2_GPIO_Port GPIOB
#define DEF2_EXTI_IRQn EXTI2_IRQn
#define MS1_Pin GPIO_PIN_10
#define MS1_GPIO_Port GPIOB
#define MS2_Pin GPIO_PIN_12
#define MS2_GPIO_Port GPIOB
#define M2_PP_Pin GPIO_PIN_14
#define M2_PP_GPIO_Port GPIOB
#define M2_PN_Pin GPIO_PIN_15
#define M2_PN_GPIO_Port GPIOB
#define M1_EA_Pin GPIO_PIN_6
#define M1_EA_GPIO_Port GPIOC
#define M3_EA_Pin GPIO_PIN_8
#define M3_EA_GPIO_Port GPIOA
#define M3_EB_Pin GPIO_PIN_9
#define M3_EB_GPIO_Port GPIOA
#define PWR_G_Pin GPIO_PIN_10
#define PWR_G_GPIO_Port GPIOA
#define PWR_G_EXTI_IRQn EXTI15_10_IRQn
#define M1_PP_Pin GPIO_PIN_11
#define M1_PP_GPIO_Port GPIOA
#define M1_PN_Pin GPIO_PIN_12
#define M1_PN_GPIO_Port GPIOA
#define M2_EI_Pin GPIO_PIN_3
#define M2_EI_GPIO_Port GPIOB
#define M2_EA_Pin GPIO_PIN_4
#define M2_EA_GPIO_Port GPIOB
#define M1_EI_Pin GPIO_PIN_6
#define M1_EI_GPIO_Port GPIOB
#define REQ_CLK_Pin GPIO_PIN_7
#define REQ_CLK_GPIO_Port GPIOB
#define M1_EB_Pin GPIO_PIN_8
#define M1_EB_GPIO_Port GPIOB
#define M3_PN_Pin GPIO_PIN_9
#define M3_PN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define CS_LIMIT 2000

#define CS_LIMIT_INDEX 2000
#define CS_LIMIT_MIDDLE 2000
#define CS_LIMIT_THUMB 2000
#define CS_LIMIT_RING_LITTLE 2000

#define ENCODER_LIMIT_ZERO 25

#define ENCODER_LIMIT_INDEX  3400 // 3244 // 3000 // 3900
#define ENCODER_LIMIT_MIDDLE 3421 // 3100 // 3700
#define ENCODER_LIMIT_THUMB 3200 // 3600 //2900 // 3600 // 3900
#define ENCODER_LIMIT_RING_LITTLE 3100 // 3700 // 3700

#define ENCODER_LIMIT_INDEX_CO M1.limit_index_counts - 5*ENCODER_LIMIT_ZERO
#define ENCODER_LIMIT_MIDDLE_CO M2.limit_index_counts - 5*ENCODER_LIMIT_ZERO
#define ENCODER_LIMIT_THUMB_CO M3.limit_index_counts - 5*ENCODER_LIMIT_ZERO
#define ENCODER_LIMIT_RING_LITTLE_CO M4.limit_index_counts - 5*ENCODER_LIMIT_ZERO

#define THUMB_OPEN_OPPOSITION_SAFE 400
#define THUMB_CLOSE_OPPOSITION_SAFE 800
#define THUMB_CLOSE_NON_OPPOSITION_SAFE M3.limit_index_counts-800
#define THUMB_OPEN_NON_OPPOSITION_SAFE 1000

#define CAL_M1_CS_F (M1.current_current_sense > CS_LIMIT)
#define CAL_M1_EN_F ((M1.limit_index_counts > ENCODER_LIMIT_INDEX) || (M1.limit_index_counts < (ENCODER_LIMIT_INDEX-500)))
#define CAL_M1_F (CAL_M1_EN_F || CAL_M1_CS_F)
#define CAL_M2_CS_F (M2.current_current_sense > CS_LIMIT)
#define CAL_M2_EN_F ((M2.limit_index_counts > ENCODER_LIMIT_MIDDLE) || (M2.limit_index_counts < (ENCODER_LIMIT_MIDDLE-500)))
#define CAL_M2_F (CAL_M2_EN_F || CAL_M2_CS_F)
#define CAL_M3_CS_F (M3.current_current_sense > CS_LIMIT)
#define CAL_M3_EN_F ((M3.limit_index_counts > ENCODER_LIMIT_THUMB) || (M3.limit_index_counts < (ENCODER_LIMIT_THUMB-500)))
#define CAL_M3_F (CAL_M3_EN_F || CAL_M3_CS_F)
#define CAL_M4_CS_F (M4.current_current_sense > CS_LIMIT)
#define CAL_M4_EN_F ((M4.limit_index_counts > ENCODER_LIMIT_RING_LITTLE) || (M4.limit_index_counts < (ENCODER_LIMIT_RING_LITTLE-500)))
#define CAL_M4_F (CAL_M4_EN_F || CAL_M4_CS_F)

#define CAL_P_F (!CAL_M1_F && !CAL_M2_F && !CAL_M3_F && !CAL_M4_F)

#define HEADER 0x90
#define FOOTER 0x0B

#define HEADER_DUMMY 0xFF
#define FOOTER_DUMMY 0xFF

#define COMMAND ((rxData_SPI[3] >> 4) & 0x07)

#define MODE (rxData_SPI[2] >> 6)

#define THUMB_POSITION (rxData_SPI[1]&0x03)
#define PAT_PROP ((rxData_SPI[3]>>7) | ((0x07&rxData_SPI[2]))<<1)
#define PATTERN_OPEN ((rxData_SPI[2] & 0x38) >> 3)
#define PATTERN_CLOSE (rxData_SPI[1] >> 4)

#define MODE_1_INDEX_ADDUCTION_THRESHOLD 2100
#define MODE_1_MIDDLE_ADDUCTION_THRESHOLD 2100
#define MODE_1_THUMB_ADDUCTION_LIMIT 2200
#define MODE_1_INDEX_HANDSHAKE_LIMIT 1800
#define MODE_1_MIDDLE_HANDSHAKE_LIMIT 1800
#define MODE_1_RING_AND_LITTLE_HANDSHAKE_LIMIT 2000


#define MODE_2_INDEX_TRIGGER_LIMIT 1400
#define MODE_2_MIDDLE_TRIGGER_LIMIT 1400
#define MODE_2_RING_AND_LITTLE_LIMIT  M4.limit_index_counts - ENCODER_LIMIT_ZERO
#define MODE_2_TWO_FINGER_TRIG_THUMB_LIMIT 1500
#define MODE_2_MIDDLE_LIMIT 1400
#define MODE_2_INDEX_LIMIT 1400
#define MODE_2_INDEX_OPEN_SAFE 1000

#define MODE_3_SPHERICAL_THUMB_LIMIT 1000
#define MODE_3_SPHERICAL_RING_AND_LITTLE_LIMIT 800
#define MODE_3_SPHERICAL_INDEX_LIMIT 1700
#define MODE_3_SPHERICAL_MIDDLE_LIMIT 1700
#define MODE_3_SPHERICAL_THUMB_SAFE 1500
#define MODE_3_INDEX_SPHERICAL_OPEN_SAFE 700

#define MODE_3_PINCH_RING_AND_LITTLE_LIMIT M4.limit_index_counts - 100
#define MODE_3_PINCH_MIDDLE_LIMIT M2.limit_index_counts - 100
#define MODE_3_PINCH_THUMB_LIMIT 1500
#define MODE_3_PINCH_INDEX_LIMIT 1400


#define MODE_3_MOUSE_THUMB_LIMIT 3000

#define MIN(a,b) (((a)<(b))?(a):(b))

#define PAT_UD_O ((rxData_SPI[1] & 0x04) >>2)
#define PAT_UD_C ((rxData_SPI[1] & 0x08) >>3)

#define STEP_DOWN_M1 M1.current_index_counts- ENCODER_LIMIT_ZERO/5
#define STEP_UP_M1 M1.current_index_counts+ ENCODER_LIMIT_ZERO/10
#define STEP_DOWN_M2 M2.current_index_counts- ENCODER_LIMIT_ZERO/5
#define STEP_UP_M2 M2.current_index_counts+ ENCODER_LIMIT_ZERO/10
#define STEP_DOWN_M3 M3.current_index_counts- ENCODER_LIMIT_ZERO/5
#define STEP_UP_M3 M3.current_index_counts+ENCODER_LIMIT_ZERO/10
#define STEP_DOWN_M4 M4.current_index_counts- ENCODER_LIMIT_ZERO/5
#define STEP_UP_M4 M4.current_index_counts+ ENCODER_LIMIT_ZERO/10

#define STEP_DOWN_M1_VALID (STEP_DOWN_M1 >= 0) && !(STEP_DOWN_M1 >65500 && STEP_DOWN_M1 <=65535) && !(M1.current_index_counts < ENCODER_LIMIT_ZERO && M1.target_encoder_index_counts <= ENCODER_LIMIT_ZERO)
#define STEP_UP_M1_VALID STEP_UP_M1 <= M1.limit_index_counts
#define STEP_DOWN_M2_VALID (STEP_DOWN_M2 >= 0) && !(STEP_DOWN_M2 >65500 && STEP_DOWN_M2 <=65535) && !(M2.current_index_counts < ENCODER_LIMIT_ZERO && M2.target_encoder_index_counts <= ENCODER_LIMIT_ZERO)
#define STEP_UP_M2_VALID STEP_UP_M2 <= M2.limit_index_counts
#define STEP_DOWN_M3_VALID (STEP_DOWN_M3 >= 0) && !(STEP_DOWN_M3 >65500 && STEP_DOWN_M3 <=65535) && !(M3.current_index_counts < ENCODER_LIMIT_ZERO && M3.target_encoder_index_counts <= ENCODER_LIMIT_ZERO)
#define STEP_UP_M3_VALID STEP_UP_M3 <= M3.limit_index_counts
#define STEP_DOWN_M4_VALID (STEP_DOWN_M4 >= 0) && !(STEP_DOWN_M4 >65500 && STEP_DOWN_M4 <=65535) && !(M4.current_index_counts < ENCODER_LIMIT_ZERO && M4.target_encoder_index_counts <= ENCODER_LIMIT_ZERO)
#define STEP_UP_M4_VALID STEP_UP_M4 <= M4.limit_index_counts

#define AVG_CS (M1.current_current_sense+M2.current_current_sense+M3.current_current_sense+M4.current_current_sense)/4
#define ADAPTIVE_FACTOR 20
#define CALIBRATION_FACTOR 40
#define CUTOFF_FACTOR 19


#define CUST_GRIP_MOT_ID (rxData_SPI[2] >> 2)

#define GEN_GRIP_ID (rxData_SPI[2] >>2)
#define GEN_GRIP_EXEC_ORDER rxData_SPI[1]

#define DIFFERENTIAL_FACTOR 100

#define DIFFERENTIAL_LOW_PASS_SHIFT (msTicks_T_7%DIFFERENTIAL_FACTOR) == (DIFFERENTIAL_FACTOR/2)
#define DIFFERENTIAL_LOW_PASS_CAPTURE (msTicks_T_7%DIFFERENTIAL_FACTOR) == 0


#define GRIP_TEST_ACCURACY (uint8_t)((M1_ACCURACY+M2_ACCURACY+M3_ACCURACY+M4_ACCURACY)/4)


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
