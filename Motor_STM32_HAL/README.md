# 有刷电机驱动（Driven by STM32(HAL) and TB6612）

## 简介

这是一个使用HAL库+cubeMX开发STM32并驱动有刷电机工作的一个小型驱动库，能够实现有刷电机的速度闭环、位置闭环、以及在一定的加速度下启动电机/切换速度。

## 使用说明

在motor.h中，可以根据实际需求修改下面的宏定义

```c
/*user custom configuration*/
#define TB6612_DRIVER 					1
#define MOTOR_CLOSEDLOOP 				1
#define LOWPASS_FILTER 					1

#if (LOWPASS_FILTER == 1)
#define FILTER_NUM 						5
#endif
```

外部变量声明可以放在这里

```c
/*declare extern variables*/
extern motor_t rMotor;
```

电机对象定义可以放在motor.c的头部：

```c
/*--------------------------variable definition--------------------------*/
motor_t rMotor;
```

pid对象定义以及声明可以放在pid.c/h中标注的地方

### 关键结构体

电机结构体定义如下：

```c
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
	motor_status_t status;
	/*motor mode*/
	motor_ctrl_t ctrlType;
#endif

#if (LOWPASS_FILTER == 1 && MOTOR_CLOSEDLOOP == 1)
	filter_t filter;
#endif
	/*test whether motor rotates correctly*/
	motor_bool ifDrvInv;
	motor_bool ifCalInv;
}motor_t;
```

### 关键函数

```c
void motor_channelSetCompare(motor_t * motor, uint32_t compareVal)
```

* 作用：更改电机所选Channel的CCR寄存器值

```c
uint32_t motor_channelGetCompare(motor_t * motor);
```

* 作用：获取电机所选Channel的CCR寄存器值

```c
uint32_t motor_channelGetPeriod(motor_t * motor);
```

* 作用：获取电机所选Timer的ARR寄存器值

```c
void motor_driverInit_TB6612 (	motor_t * motor, 
								GPIO_TypeDef * drvGPIO,  uint16_t drvGPIO_Pin,		/*STBY PIN*/
								GPIO_TypeDef * GPIO_IN1, uint16_t GPIO_IN1_PIN,		/*A/BIN1 PIN*/
								GPIO_TypeDef * GPIO_IN2, uint16_t GPIO_IN2_PIN,		/*A/BIN2 PIN*/
								motor_bool ifDrvInv);								/*If inverse rotating direction*/
```

* 作用：初始化电机驱动
* 参数：
  * motor：电机结构体
  * drvGPIO, drvGPIO_Pin: 绑定TB6612的STBY引脚
  * GPIO_IN1/2, GPIO_IN1/2_PIN: 绑定TB6612用于控制电机转动方向的的A/BIN1/2引脚
  * ifDrvInv: 根据实际情况调整：如果设置为MOTOR_TRUE，转的是反的，那么就改为MOTOR_FALSE

```c
void motor_setRotateDir_TB6612(motor_t * motor, rotateDir_t rotateDir);
```

* 作用：设置电机旋转方向

```c
float motor_filter_calc(filter_t *f, float in);
```

* 作用：对读取到的速度进行滤波

```c
uint32_t motor_getTick(motor_t * motor);
```

* 作用：获取电机初始化到现在的运行tick数

```c
void motor_timeTick(motor_t * motor);
```

* 作用：产生tick
* 说明：可以放在任何定时器中断里

```c
void motor_init_closedloop (motor_t * motor, motor_dataType dataType,			/*Motor init*/
							pid_typedef * velPID, pid_typedef * posPID, 		/*PID init*/
							TIM_HandleTypeDef * htim, uint32_t channel,			/*PWM init*/
							TIM_HandleTypeDef * enchtim, motor_bool ifCalInv,	/*Timer-Encoder init*/
							uint32_t usTickPeriod,								/*Set the period of motor tick*/
							int gearRatio, int ppr, float radius);				/*Set motor parameters*/
```

* 作用：电机闭环初始化

* 参数：

  * dataType：测得的速度、位置的单位（支持毫米、米、角度、弧度）

  * ifCalInv：根据实际情况修改，如设置为MOTOR_TRUE，与实际测得速度符号相反，就改为MOTOR_FALSE
  * usTickPeriod：**motor_timeTick**所在定时器的滴答周期（以微秒为单位）
  * gearRatio：电机减速比
  * ppr：电机编码器线数
  * radius：旋转半径（就是轮子半径，以毫米为单位）

```c
void motor_update (motor_t * motor);
```

* 作用：更新电机参数
* 说明：放在循环里即可，调用频率不能太高（否则时间间距过短，可能会出现n/0的情况等）

```c
void motor_checkReload(motor_t * motor);
```

* 作用：防止编码器计数值溢出
* 说明：放在定时器中断回调里的编码器中断部分

```c
void motor_set_velocity(motor_t * m, float tV);
```

* 作用：设置电机速度
* 说明：单位为rad/s（别的单位也可以，初始化记得改一下）

```c
void motor_set_position(motor_t * m, float tP);
```

* 作用：设置电机位置
* 说明：单位为rad

```c
float motor_get_velocity(motor_t * motor);
```

* 作用：获取电机速度
* 说明：单位为rad/s

```c
float motor_get_position(motor_t * motor);
```

* 作用：获取电机位置
* 说明：单位为rad

```c
void motor_run(motor_t * m);
```

* 作用：启动电机
* 说明：放在循环里调用即可（最好放在100Hz定时器中断里）

## 用法举例

### cubeMX（只显示必要部分）

1. 设置一个100Hz的定时器中断，同时开启pwm输出口用于驱动电机，PSC设置为0，ARR设置为1000即可（只是举个例子，参数合理即可）
2. 开启编码器模式，将Encoder Mode设置为 Encoder Mode TI1 and TI2，并使能编码器中断
3. 设置TB6612驱动接口（STBY, A/BIN1/2）

### 代码部分（举例）

1. 定时器中断初始化

```c
HAL_TIM_Base_Start_IT(&htim9);
```

2. 电机驱动部分初始化

```c
motor_driverInit_TB6612(&rMotor,
						GPIOB, GPIO_PIN_4,	/*STBY PIN*/
						GPIOB, GPIO_PIN_1,
						GPIOB, GPIO_PIN_0,
						MOTOR_FALSE);
```

3. 电机闭环初始化

```c
motor_init_closedloop ( &rMotor, rad,
					    &pid_rMotorVel,&pid_rMotorPos,		/*pid init*/
					    &htim2, TIM_CHANNEL_1,				/*peripherals init*/
					    &htim4, MOTOR_FALSE,				/*encoder init*/
					    10000,								/*tick period set*/
						30, 11, 33.5 );						/*motor parameters init*/
```

4. 速度环、位置环PID初始化

```c
pid_init ( &pid_rMotorVel,
			40, 420, 0,
			10000, 1000,
			10000);
pid_init ( &pid_rMotorPos,
			10, 0, 0.7,
			10000, 50,
			10000);
```

5. 定时器中断回调函数

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim9 && rMotor.initStatus == MOTOR_INIT_OK)
	{
		motor_timeTick(&rMotor);
		pid_timeTick(&pid_rMotorVel);
        
		motor_update(&rMotor);
        
        motor_run(&rMotor);
	}
	if (htim == rMotor.encoder.pEncHtim)
	{
		motor_checkReload(&rMotor);
	}
}
```

6. 调用motor_set_velocity/position/stop函数





































 
