/*
@file: motor.h
@author: Asaka(Yuzupi)
@time: 2023/11/15 21:08
@brief: The file uses STM32 HAL API to drive a Brush DC-Motor
*/

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"
#include "pid.h"
#include "stdlib.h"

/*user custom configuration*/
#define TB6612_DRIVER 				1
#define MOTOR_CLOSEDLOOP 			1
#define LOWPASS_FILTER 				1

#if (LOWPASS_FILTER == 1)
#define FILTER_NUM 						5
#endif

/*const parameters*/
#define _PI (3.14159)

/*enums*/
typedef enum
{
	CLOCKWISE = 1,
	ANTICLOCKWISE = -1
}rotateDir_t;

typedef enum
{
	MOTOR_FALSE = 0,
	MOTOR_TRUE = 1
}motor_bool;

typedef enum
{
	MOTOR_INITING = 0,
	MOTOR_INIT_OK,
}motor_initStatus_t;

typedef enum
{
	rad = 0,
	angle,
	line_mm,
	line_m
}motor_dataType;


#if (MOTOR_CLOSEDLOOP == 1)
/*motor closedloop struct*/
typedef enum
{
	MOTOR_CTRL_NON = 0,
	MOTOR_CTRL_VEL,
	MOTOR_CTRL_POS,
}motor_ctrl_t;

typedef struct motor_param_t
{
	uint16_t gearRatio;
	uint16_t pulsePerRound;
	float rotateRadius_mm;			/*Measured by mm*/
}motor_param_t;

typedef struct motor_tick_t
{
	uint32_t usTickPeriod;
	uint32_t nowTick;
	uint32_t lastTick;
}motor_tick_t;

typedef struct motor_data_t
{
	rotateDir_t rotateDir;
	motor_dataType dataType;
	float velocity;
	float position;
	float targetVelo;
	float targetPos;
}motor_data_t;

typedef struct motor_encoder_t
{
	TIM_HandleTypeDef * pEncHtim;
	int lastEncCnt;
	int nowEncCnt;
	int totalEncCnt;
	int encOverflowNum;
}motor_encoder_t;

#if (LOWPASS_FILTER == 1)
/*lowpass filter struct*/
typedef struct filter_t{
	float filter[FILTER_NUM];
} filter_t;
#endif

#endif
/*Motor driver struct*/
#if (TB6612_DRIVER == 1)
typedef struct motor_drv_t
{
	/*Driver PIN*/
	GPIO_TypeDef * pDrvGPIO;
	uint16_t drvGPIO_Pin;
	/*Direction Control PIN*/
	GPIO_TypeDef * pGPIO_IN1;
	uint16_t pGPIO_IN1_Pin;
	GPIO_TypeDef * pGPIO_IN2;
	uint16_t pGPIO_IN2_Pin;
}motor_drv_t;
#endif
/*motor struct*/
typedef struct motor_t
{
	/*PWM peripherals*/
	TIM_HandleTypeDef * phtim;
	uint32_t motorChannel;
	/*Motor driver*/
#if (TB6612_DRIVER == 1)
	motor_drv_t driver;
#endif
	
#if (MOTOR_CLOSEDLOOP == 1)
	motor_encoder_t encoder;
	/*Motor parameters*/
	motor_param_t param;
	/*Bind PID typedef*/
	pid_typedef * pVelPID;
	pid_typedef * pPosPID;
	/*closedloop mode data*/
	motor_data_t data;
	/*tick parameters*/
	motor_tick_t tick;
	/*motor status*/
	motor_initStatus_t initStatus;
	/*motor ctrl type*/
	motor_ctrl_t ctrlType;
#endif

#if (LOWPASS_FILTER == 1 && MOTOR_CLOSEDLOOP == 1)
	filter_t filter;
#endif
	/*test whether motor rotates correctly*/
	motor_bool ifDrvInv;
	motor_bool ifCalInv;
}motor_t;

/*declare extern variables*/
// extern motor_t rMotor;
// extern motor_t lMotor;
/*peripheral operation functions*/
void motor_channelSetCompare(motor_t * motor, uint32_t compareVal);
uint32_t motor_channelGetCompare(motor_t * motor);
uint32_t motor_channelGetPeriod(motor_t * motor);

/*Driver functions*/
#if (TB6612_DRIVER == 1)
void motor_driverInit_TB6612 (motor_t * motor, 
															GPIO_TypeDef * drvGPIO,  uint16_t drvGPIO_Pin,		/*STBY PIN*/
															GPIO_TypeDef * GPIO_IN1, uint16_t GPIO_IN1_PIN,		/*A/BIN1 PIN*/
															GPIO_TypeDef * GPIO_IN2, uint16_t GPIO_IN2_PIN,		/*A/BIN2 PIN*/
															motor_bool ifDrvInv);															/*If driver inverses rotating direction*/
void motor_setRotateDir_TB6612(motor_t * motor, rotateDir_t rotateDir);
#endif

/*Motor run in openloop mode*/
#if (MOTOR_CLOSEDLOOP == 0)
void motor_init_openloop(motor_t * motor, TIM_HandleTypeDef * phtim, uint32_t channel);
void motor_rotate_openloop(motor_t * motor, uint32_t compareVal, rotateDir_t rotateDir);
#endif

/*Motor run in closedloop mode*/
#if (MOTOR_CLOSEDLOOP == 1)
#if (LOWPASS_FILTER == 1)
/*velocity filter function*/
float motor_filter_calc(filter_t *f, float in);
#endif

/*motor tick functions*/
uint32_t motor_getTick(motor_t * motor);
float motor_getTime_s(motor_t * m);
void motor_timeTick(motor_t * motor);

float motor_get_velocity(motor_t * motor);
float motor_get_position(motor_t * motor);

void motor_init_closedloop (motor_t * motor, motor_dataType dataType,					/*Motor init*/
														pid_typedef * velPID, pid_typedef * posPID, 			/*PID init*/
														TIM_HandleTypeDef * htim, uint32_t channel,				/*PWM init*/
														TIM_HandleTypeDef * enchtim, motor_bool ifCalInv,	/*Timer-Encoder init*/
														uint32_t usTickPeriod,														/*Set the period of motor tick*/
														int gearRatio, int ppr, float radius);						/*Set motor parameters*/

void motor_update (motor_t * motor);
void motor_checkReload(motor_t * motor);

void motor_arrive_velocity(motor_t * m, float targetVel);
void motor_arrive_position(motor_t * m, float targetPos);

void motor_set_velocity(motor_t * m, float tV);
void motor_set_position(motor_t * m, float tP);
void motor_stop(motor_t * m);

void motor_run(motor_t * m);

#endif

#endif
