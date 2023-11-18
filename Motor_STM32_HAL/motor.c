/*
@file: motor.c
@author: Asaka(Yuzupi)
@time: 2023/11/15 21:20
@brief: The file uses STM32 HAL API to drive a Brush DC-Motor
*/

#include "motor.h"

/*--------------------------variable definition--------------------------*/
// motor_t rMotor;
// motor_t lMotor;

/*----------------------------Basic functions----------------------------*/
static inline float _abs(float in)
{
	return (in >= 0) ? in:(-in);
}
/*--------------------motor PWM peripherals functions--------------------*/
void motor_channelSetCompare(motor_t * motor, uint32_t compareVal)
{
	uint32_t period = motor_channelGetPeriod(motor);
	if (compareVal > period + 1)
	{
		compareVal = period + 1;
	}
	__HAL_TIM_SetCompare(motor->phtim, motor->motorChannel, compareVal);
}

inline uint32_t motor_channelGetCompare(motor_t * motor)
{
	return __HAL_TIM_GetCompare(motor->phtim, motor->motorChannel);
}

inline uint32_t motor_channelGetPeriod(motor_t * motor)
{
	return motor->phtim->Instance->ARR;
}
/*-------------------------motor driver function-------------------------*/
#if (TB6612_DRIVER == 1)
void motor_driverInit_TB6612 (motor_t * motor, 
															GPIO_TypeDef * drvGPIO,  uint16_t drvGPIO_Pin,
															GPIO_TypeDef * GPIO_IN1, uint16_t GPIO_IN1_PIN,
															GPIO_TypeDef * GPIO_IN2, uint16_t GPIO_IN2_PIN,
															motor_bool ifDrvInv)
{
	motor->driver.pDrvGPIO = drvGPIO;
	motor->driver.drvGPIO_Pin = drvGPIO_Pin;
	motor->driver.pGPIO_IN1 = GPIO_IN1;
	motor->driver.pGPIO_IN1_Pin = GPIO_IN1_PIN;
	motor->driver.pGPIO_IN2 = GPIO_IN2;
	motor->driver.pGPIO_IN2_Pin = GPIO_IN2_PIN;
	motor->ifDrvInv = ifDrvInv;
	HAL_GPIO_WritePin(drvGPIO, drvGPIO_Pin, GPIO_PIN_SET);
}

void motor_setRotateDir_TB6612(motor_t * motor, rotateDir_t rotateDir)
{
	if (rotateDir == CLOCKWISE) {
		if (motor->ifDrvInv == MOTOR_TRUE) {
			HAL_GPIO_WritePin(motor->driver.pGPIO_IN1, motor->driver.pGPIO_IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor->driver.pGPIO_IN2, motor->driver.pGPIO_IN2_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(motor->driver.pGPIO_IN1, motor->driver.pGPIO_IN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->driver.pGPIO_IN2, motor->driver.pGPIO_IN2_Pin, GPIO_PIN_SET);
		}
	} else {
		if (motor->ifDrvInv == MOTOR_TRUE) {
			HAL_GPIO_WritePin(motor->driver.pGPIO_IN1, motor->driver.pGPIO_IN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->driver.pGPIO_IN2, motor->driver.pGPIO_IN2_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(motor->driver.pGPIO_IN1, motor->driver.pGPIO_IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor->driver.pGPIO_IN2, motor->driver.pGPIO_IN2_Pin, GPIO_PIN_RESET);
		}
	}
}
#endif

/*--------------------------motor openloop mode--------------------------*/
#if (MOTOR_CLOSEDLOOP == 0)
void motor_init_openloop(motor_t * motor, TIM_HandleTypeDef * phtim, uint32_t channel)
{
	motor->phtim = phtim;
	motor->motorChannel = channel;
	HAL_TIM_PWM_Start(phtim, channel);
}

void motor_rotate_openloop(motor_t * motor, uint32_t compareVal, rotateDir_t rotateDir)
{
	motor_setRotateDir_TB6612(motor, rotateDir);
	motor_channelSetCompare(motor, compareVal);
}
#endif

/*-------------------------motor closedloop mode-------------------------*/
#if (MOTOR_CLOSEDLOOP == 1)

#if (LOWPASS_FILTER == 1)
/*lowpass filter functions*/
float motor_filter_calc(filter_t *f, float in)
{
	float sum = 0;
	for(uint8_t i=FILTER_NUM-1;i>0;i--)
	{
		f->filter[i] = f->filter[i-1];
		sum += f->filter[i];
	}
	f->filter[0] = in;
	sum += in;
	return sum / FILTER_NUM;
}
#endif

/*get motor data*/
inline void motor_timeTick(motor_t * motor)
{
	motor->tick.nowTick++;
}

inline uint32_t motor_getTick(motor_t * motor)
{
	return motor->tick.nowTick;
}

inline float motor_getTime_s(motor_t * m)
{
	return m->tick.nowTick * 1e-6 * m->tick.usTickPeriod;
}

inline float motor_get_velocity(motor_t * motor)
{
	return motor->data.velocity;
}

inline float motor_get_position(motor_t * motor)
{
	return motor->data.position;
}

void motor_init_closedloop (motor_t * motor, motor_dataType dataType, 
														pid_typedef * velPID, pid_typedef * posPID,
														TIM_HandleTypeDef * htim, uint32_t channel,
														TIM_HandleTypeDef * enchtim, motor_bool ifCalInv,
														uint32_t usTickPeriod,
														int gearRatio, int ppr, float radius)
{
	motor->data.dataType = dataType;
	motor->pVelPID = velPID;
	motor->pPosPID = posPID;
	motor->phtim = htim;
	motor->encoder.pEncHtim = enchtim;
	motor->ifCalInv = ifCalInv;
	motor->tick.usTickPeriod = usTickPeriod;
	motor->ctrlType = MOTOR_CTRL_NON;
	
	motor->param.gearRatio = gearRatio;
	motor->param.pulsePerRound = ppr;
	motor->param.rotateRadius_mm = radius;
	motor->motorChannel = channel;
	HAL_TIM_PWM_Start(htim, channel);
	__HAL_TIM_ENABLE_IT(enchtim, TIM_IT_UPDATE);
	HAL_TIM_Encoder_Start_IT(enchtim, TIM_CHANNEL_ALL);
	
	motor->initStatus = MOTOR_INIT_OK;
}

void motor_update (motor_t * motor)
{
	uint32_t nowTick = motor_getTick(motor);
	float ts = (nowTick - motor->tick.lastTick) * motor->tick.usTickPeriod * 1e-6;
	motor->encoder.nowEncCnt = __HAL_TIM_GetCounter(motor->encoder.pEncHtim)
									 + motor->encoder.encOverflowNum * __HAL_TIM_GetAutoreload(motor->encoder.pEncHtim);
	motor->encoder.totalEncCnt = motor->encoder.nowEncCnt - motor->encoder.lastEncCnt;
	/*Judge direction*/
	if(motor->encoder.totalEncCnt > 0) {
		if (motor->ifCalInv == MOTOR_TRUE) {
			motor->data.rotateDir = CLOCKWISE;
		} else {
			motor->data.rotateDir = ANTICLOCKWISE;
		}
	} else {
		if (motor->ifCalInv == MOTOR_FALSE) {
			motor->data.rotateDir = CLOCKWISE;
		} else {
			motor->data.rotateDir = ANTICLOCKWISE;
		}
	}
	
	float dRad = motor->data.rotateDir
						 * _abs(motor->encoder.totalEncCnt 
						 / (4.0 * motor->param.gearRatio * motor->param.pulsePerRound))
						 * 2 * _PI;
	if(motor->data.dataType == rad) {
		motor->data.position += dRad;
		motor->data.velocity = dRad / ts;
	} else if (motor->data.dataType == angle) {
		float dAng = dRad / (2*_PI) * 360;
		motor->data.position += dAng;
		motor->data.velocity = dAng / ts;
	} else if (motor->data.dataType == line_mm) {
		float dl = dRad * motor->param.rotateRadius_mm;
		motor->data.position += dl;
		motor->data.velocity = dl / ts;
	} else if (motor->data.dataType == line_m) {
		float dl = dRad * motor->param.rotateRadius_mm * 1e-3;
		motor->data.position += dl;
		motor->data.velocity = dl / ts;
	}
	
#if (LOWPASS_FILTER == 1)
	motor->data.velocity = motor_filter_calc(&motor->filter, motor->data.velocity);
#endif
	motor->tick.lastTick = nowTick;
	motor->encoder.lastEncCnt = motor->encoder.nowEncCnt;
}

void motor_checkReload(motor_t * motor)
{
	if(__HAL_TIM_GetCounter(motor->encoder.pEncHtim) 
		> 0.5 * __HAL_TIM_GetAutoreload(motor->encoder.pEncHtim)) {	/*Down overflow*/
		motor->encoder.encOverflowNum--;
	} else {					/*Up overflow*/
		motor->encoder.encOverflowNum++;
	}
	
	if(motor_getTime_s(motor) <= 5*1e-1) 	/*Prevent motor jittering at the initialization zero point.*/
	{
		motor->encoder.encOverflowNum = 0;
	}
}

void motor_arrive_velocity(motor_t * m, float targetVel)
{
	float out = 0;
	out = pid_calc(m->pVelPID, targetVel - m->data.velocity);
	if(out >= 0) {
		motor_setRotateDir_TB6612(m, CLOCKWISE);
	} else {
		motor_setRotateDir_TB6612(m, ANTICLOCKWISE);
	}
	motor_channelSetCompare(m, (uint32_t)_abs(out));
}

void motor_arrive_position(motor_t * m, float targetPos)
{
	float out = 0;
	out = pid_calc(m->pPosPID, targetPos - m->data.position);
	motor_arrive_velocity(m, out);
}

void motor_run(motor_t * m)
{
	if (m->ctrlType == MOTOR_CTRL_VEL) {
		motor_arrive_velocity(m, m->data.targetVelo);
	} else if (m->ctrlType == MOTOR_CTRL_POS) {
		motor_arrive_position(m, m->data.targetPos);
	} else {
	}
}

void motor_set_velocity(motor_t * m, float tV)
{
	m->data.targetVelo = tV;
	m->ctrlType = MOTOR_CTRL_VEL;
}

void motor_set_position(motor_t * m, float tP)
{
	m->data.targetPos = tP;
	m->ctrlType = MOTOR_CTRL_POS;
}

void motor_stop(motor_t * m)
{
	m->data.targetPos = 0;
	m->data.targetVelo = 0;
	m->ctrlType = MOTOR_CTRL_VEL;
}

#endif
