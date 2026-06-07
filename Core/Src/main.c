/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
volatile int rawVal2 = 0;
volatile int rawVal3 = 0;
volatile int rawVal4 = 0;
volatile int rawVal5 = 0;
volatile int leftIr = 0;
volatile int middleIr = 0;
volatile int rightIr = 0;
typedef struct
{
	double intState;
	double drevState;
	int reached;
	int32_t totalPos;
	uint16_t prevPos;
} pidState;

typedef struct
{
	int pin1;
	int pin2;
	uint16_t rightEnPin;
	uint16_t leftEnPin;
	GPIO_TypeDef * rightEnPort;
	GPIO_TypeDef * leftEnPort;
	double kp;
	double ki;
	double kd;
	double kf;
	TIM_HandleTypeDef* encTimer;
} portsAndPins;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PCA9685_ADDRESS (0x40 << 1)
#define PCA9685_MODE1         0x0
#define PCA9685_PRE_SCALE     0xFE
#define PCA9685_LED0_ON_L     0x6
#define PCA9685_MODE1_SLEEP_BIT      4
#define PCA9685_MODE1_AI_BIT         5
#define PCA9685_MODE1_RESTART_BIT    7
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//first ku = 2.75 and tu is 4.50143 second ku = 3.45 and tu is 4.5162 third ku is 3.35 and tu is 4.730475 foruth ku = 3.15 and tu = 4.5153
//first ku = 3.85 and tu is 7.49915 second ku = 4.4 and tu is 7.34447 third ku is 4.35 and tu is 7.7961 foruth ku = 3.9 and tu = 7.39478
const portsAndPins motors[4] = {{1, 0, topLeftR_EN_Pin, topLeftL_EN_Pin, topLeftR_EN_GPIO_Port, topLeftL_EN_GPIO_Port, 1.7325, 27.723, 0, 0.256, &htim2},
							{2, 3, topRightR_EN_Pin, topRightL_EN_Pin, topRightR_EN_GPIO_Port, topRightL_EN_GPIO_Port, 1.98, 32.371, 0, 0.276,&htim3},
							{5, 4, bottomLeftR_EN_Pin, bottomLeftL_EN_Pin, bottomLeftR_EN_GPIO_Port, bottomLeftL_EN_GPIO_Port, 1.9575, 30.130, 0, 0.248,&htim4},
							{6, 7, bottomRightR_EN_Pin, bottomRightL_EN_Pin, bottomRightR_EN_GPIO_Port, bottomRightL_EN_GPIO_Port, 1.755, 28.480, 0, 0.276,&htim5}};
const int adjustedTargetRatios[3][4] = {{1, 1, 1, 1}, {1, -1, -1, 1}, {1, -1, 1, -1}};
const int FORWARD = 1, BACKWARDS = 0, RIGHT = 1, LEFT = 0;
const double kpp = 3.9, kii = 0, kdd = 0, period = 0.01, alpha = 0.3;
const int intMax = 40, maxPwm = 4095, maxSpeeeed = 15000, minSpeed = 2000, speedAdjust = 750, breakDistance = 2000, quarterTurn = 4875;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
	return len;
}

void PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value)
{
  uint8_t readValue;
  // Read all 8 bits and set only one bit to 0/1 and write all 8 bits back
  HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS, Register, I2C_MEMADD_SIZE_8BIT, &readValue, 1, 10);
  if (Value == 0) readValue &= ~(1 << Bit);
  else readValue |= (1 << Bit);
  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, Register, I2C_MEMADD_SIZE_8BIT, &readValue, 1, 10);
  HAL_Delay(1);
}

void PCA9685_SetPWMFrequency(uint16_t frequency)
{
  uint8_t prescale;
  if(frequency >= 1526) prescale = 0x03;
  else if(frequency <= 24) prescale = 0xFF;
  //  internal 25 MHz oscillator as in the datasheet page no 1/52
  else prescale = 25000000 / (4096 * frequency);
  // prescale changes 3 to 255 for 1526Hz to 24Hz as in the datasheet page no 1/52
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);
  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, PCA9685_PRE_SCALE, I2C_MEMADD_SIZE_8BIT, &prescale, 1, 10);
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 0);
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, 1);
}

void PCA9685_Init(uint16_t frequency)
{
  PCA9685_SetPWMFrequency(frequency); // 50 Hz for servo
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_AI_BIT, 1);
}

void PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime)
{
  uint8_t registerAddress;
  uint8_t pwm[4];
  registerAddress = PCA9685_LED0_ON_L + (4 * Channel);
  pwm[0] = OnTime & 0xFF;
  pwm[1] = OnTime>>8;
  pwm[2] = OffTime & 0xFF;
  pwm[3] = OffTime>>8;
  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, registerAddress, I2C_MEMADD_SIZE_8BIT, pwm, 4, 10);
}

double updateEncoder(pidState *positions, int index)
{
	int16_t current = __HAL_TIM_GET_COUNTER(motors[index].encTimer);
	int16_t diff = (int16_t)(current - positions->prevPos);
	positions->totalPos += diff;
	positions->prevPos = current;
	return (diff / period) * alpha + ((1 - alpha) * positions->drevState);
}

void oneWord(int speed, int direction, int index)
{
	PCA9685_SetPWM(motors[index].pin1, 0, direction == FORWARD ? speed : 0);
	PCA9685_SetPWM(motors[index].pin2, 0, direction == FORWARD ? 0 : speed);
}

void stop()
{
	for (int index = 0; index < 4; index++)
	{
		PCA9685_SetPWM(motors[index].pin1, 0, 0);
		PCA9685_SetPWM(motors[index].pin2, 0, 0);
	}
}

void breaking(int index)
{
	PCA9685_SetPWM(motors[index].pin1, 0, 4095);
	PCA9685_SetPWM(motors[index].pin2, 0, 4095);
}

//void sideways(int speed, int direction, int index)
//{
//	if (index == 0 || index == 3)
//	{
//		PCA9685_SetPWM(motors[index].pin1, 0, direction == RIGHT ? speed : 0);
//		PCA9685_SetPWM(motors[index].pin2, 0, direction == RIGHT ? 0 : speed);
//	}
//	else
//	{
//		PCA9685_SetPWM(motors[index].pin1, 0, direction == RIGHT ? 0 : speed);
//		PCA9685_SetPWM(motors[index].pin2, 0, direction == RIGHT ? speed : 0);
//	}
//}

//void rotate(int speed, int direction, int index)
//{
//	if (index == 0 || index == 2)
//	{
//		PCA9685_SetPWM(motors[index].pin1, 0, direction == RIGHT ? speed : 0);
//		PCA9685_SetPWM(motors[index].pin2, 0, direction == RIGHT ? 0 : speed);
//	}
//	else
//	{
//		PCA9685_SetPWM(motors[index].pin1, 0, direction == RIGHT ? 0 : speed);
//		PCA9685_SetPWM(motors[index].pin2, 0, direction == RIGHT ? speed : 0);
//	}
//}
//
//void Diagonal(const int speed, const int sidewayDir, const int oneWayDir)
//{
//	for (int index = 0; index < 4; index++)
//	{
//		__HAL_TIM_SET_COMPARE(motors[index].pwmTimer, motors[index].channel, speed);
//		if (sidewayDir == RIGHT && (index == 0 || index == 3))
//		{
//			HAL_GPIO_WritePin(motors[index].pin1Port, motors[index].pin1Pin, oneWayDir == FORWARD ? GPIO_PIN_SET : GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(motors[index].pin2Port, motors[index].pin2Pin, oneWayDir == FORWARD ? GPIO_PIN_RESET : GPIO_PIN_SET);
//		}
//		else if (sidewayDir == LEFT && (index == 1 || index == 2))
//		{
//			HAL_GPIO_WritePin(motors[index].pin1Port, motors[index].pin1Pin, oneWayDir == FORWARD ? GPIO_PIN_RESET : GPIO_PIN_SET);
//			HAL_GPIO_WritePin(motors[index].pin2Port, motors[index].pin2Pin, oneWayDir == FORWARD ? GPIO_PIN_SET : GPIO_PIN_RESET);
//		}
//    }
//}

double updatePid(pidState *pid, double error, double velocity, int index)
{
	double propVal = 0, intVal = 0, dervVal = 0;
	propVal = motors[index].kp * error;
	pid->intState += error * period;
	dervVal = motors[index].kd * ((velocity - pid->drevState) / period);
	pid->intState = pid->intState > intMax ? intMax : pid->intState;
	pid->intState = pid->intState < -intMax ? -intMax : pid->intState;
	intVal = pid->intState * motors[index].ki;
	pid->drevState = velocity;
	return propVal + intVal + dervVal;
}

void onewordPid(int target, int mode)
{
	int targetSpeed = maxSpeeeed;
	int adjustRatio = 0;
	uint32_t prevTick = HAL_GetTick();
	pidState motorState[4];
	for (int index = 0; index < 4; index++)
	{
		__HAL_TIM_SET_COUNTER(motors[index].encTimer, 0);
		motorState[index].drevState = 0;
		motorState[index].reached = 0;
		motorState[index].intState = 0;
		motorState[index].prevPos = 0;
		motorState[index].totalPos = 0;
		HAL_GPIO_WritePin(motors[index].rightEnPort, motors[index].rightEnPin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(motors[index].leftEnPort, motors[index].leftEnPin, GPIO_PIN_SET);

	}
	while (!(motorState[0].reached && motorState[1].reached && motorState[2].reached && motorState[3].reached))
	{
		if (HAL_GetTick() - prevTick >= (uint32_t)(period * 1000))
		{
			if (mode == 0)
			{
				leftIr = HAL_GPIO_ReadPin(leftIR_GPIO_Port, leftIR_Pin);
				middleIr = HAL_GPIO_ReadPin(middleIR_GPIO_Port, middleIR_Pin);
				rightIr = HAL_GPIO_ReadPin(rightIR_GPIO_Port, rightIR_Pin);
				if (leftIr && !middleIr) adjustRatio = -2;
				else if (leftIr && middleIr && !rightIr) adjustRatio = -1;
				else if (rightIr && !middleIr) adjustRatio = 2;
				else if (rightIr && middleIr && !leftIr) adjustRatio = 1;
				else adjustRatio = 0;
			}
			int32_t averagePosition = ((motorState[0].totalPos * adjustedTargetRatios[mode][0]) +
										(motorState[1].totalPos * adjustedTargetRatios[mode][1]) +
										(motorState[2].totalPos * adjustedTargetRatios[mode][2]) +
										(motorState[3].totalPos * adjustedTargetRatios[mode][3])) / 4;

			printf("%d,%d,%d,%d\n", rawVal2, rawVal3, rawVal4, rawVal5);

			for (int index = 0; index < 4; index++)
			{
				double currentSpeed = updateEncoder(&motorState[index], index);
				if (index == 0) rawVal2 = currentSpeed;
				if (index == 1) rawVal3 = currentSpeed;
				if (index == 2) rawVal4 = currentSpeed;
				if (index == 3) rawVal5 = currentSpeed;
				if (motorState[index].reached) continue;

				int distanceAway = adjustedTargetRatios[mode][index] * (target - averagePosition);
				if (distanceAway > breakDistance) targetSpeed = maxSpeeeed;
				else if (distanceAway < -breakDistance) targetSpeed = -maxSpeeeed;
				else
				{
					motorState[index].intState = 0;
					targetSpeed = (maxSpeeeed * distanceAway) / breakDistance;
					if (targetSpeed > 0 && targetSpeed < minSpeed) targetSpeed = minSpeed;
					if (targetSpeed < 0 && targetSpeed > -minSpeed) targetSpeed = -minSpeed;
				}

				if (adjustRatio && (index == 0 || index == 2)) targetSpeed += speedAdjust * adjustRatio;
				else if (adjustRatio) targetSpeed += -(speedAdjust * adjustRatio);
				targetSpeed = targetSpeed > maxSpeeeed ? maxSpeeeed : targetSpeed;
				targetSpeed = targetSpeed < -maxSpeeeed ? -maxSpeeeed : targetSpeed;

				int error = targetSpeed - currentSpeed;
				double pwmVal = updatePid(&motorState[index], error, currentSpeed, index) + (motors[index].kf * targetSpeed);
				int absPwm = (int)(pwmVal < 0 ? -pwmVal : pwmVal);
//				absPwm = maxPwm; pwmVal = maxPwm;
				oneWord(absPwm > maxPwm ? maxPwm : absPwm, pwmVal < 0 ? BACKWARDS : FORWARD, index);

				if (distanceAway < 50 && distanceAway > -50)
				{
					motorState[index].reached = 1;
					motorState[index].intState = 0;
					breaking(index);
				}
				else motorState[index].reached = 0;
			}
			prevTick += (uint32_t)(period * 1000);
		}
	}
	stop();
}

//void onewordPid(int target)
//{
//	uint32_t prevTick = HAL_GetTick();
//	pidState motorState[4];
//	for (int index = 0; index < 4; index++)
//	{
//		__HAL_TIM_SET_COUNTER(motors[index].encTimer, 0);
//		motorState[index].drevState = 0;
//		motorState[index].reached = 0;
//		motorState[index].intState = 0;
//		motorState[index].prevPos = 0;
//		motorState[index].totalPos = 0;
//	}
//	while (!(motorState[0].reached && motorState[1].reached && motorState[2].reached && motorState[3].reached))
//	{
//		rawVal2 = __HAL_TIM_GET_COUNTER(&htim2);
//		rawVal3 = __HAL_TIM_GET_COUNTER(&htim3);
//		rawVal4 = __HAL_TIM_GET_COUNTER(&htim4);
//		rawVal5 = __HAL_TIM_GET_COUNTER(&htim5);
//		if (HAL_GetTick() - prevTick >= 10)
//		{
//			for (int index = 0; index < 4; index++)
//			{
//				updateEncoder(&motorState[index], index);
//				int error = target - motorState[index].totalPos;
//				double speed = updatePid(&motorState[index], error, motorState[index].totalPos);
//				int absSpeed = speed < 0 ? speed * -1 : speed;
//				OneWord(absSpeed > maxSpeed ? maxSpeed : absSpeed, speed < 0 ? BACKWARDS : FORWARD, index);
//				if (error < 250 && error > -250) motorState[index].reached = 1;
//				else motorState[index].reached = 0;
//			}
//			prevTick = HAL_GetTick();
//		}
//	}
//	Stop();
//}

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  extern UART_HandleTypeDef huart2;
  PCA9685_Init(1526);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  for (int index = 0; index < 16; index++)
  {
	  if (index < 4)
	  {
		  HAL_GPIO_WritePin(motors[index].rightEnPort, motors[index].rightEnPin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(motors[index].leftEnPort, motors[index].leftEnPin, GPIO_PIN_SET);
	  }
	  PCA9685_SetPWM(index, 0, 0);
  }
  //29400 for rotation
  HAL_Delay(2000);
//  onewordPid(61123.13758, 0);
  onewordPid(20374.379, 0);
  onewordPid(2 * quarterTurn, 2);
  onewordPid(20374.379, 0);
  onewordPid(2 * quarterTurn, 2);
  onewordPid(20374.379, 0);
  onewordPid(-2 * quarterTurn, 2);
  onewordPid(61123.137, 0);
  onewordPid(-2 * quarterTurn, 2);
  onewordPid(10187.189, 0);
  onewordPid(-quarterTurn, 2);
  onewordPid(28813.7234, 0);
  while (1)
  {
	  leftIr = HAL_GPIO_ReadPin(leftIR_GPIO_Port, leftIR_Pin);
	  middleIr = HAL_GPIO_ReadPin(middleIR_GPIO_Port, middleIR_Pin);
	  rightIr = HAL_GPIO_ReadPin(rightIR_GPIO_Port, rightIR_Pin);
	  //	  __HAL_TIM_SET_COMPARE(motors[1].pwmTimer, motors[1].channel, 500);
	  //	  HAL_GPIO_WritePin(motors[1].pin1Port, motors[1].pin1Pin, GPIO_PIN_SET);
	  //	  HAL_GPIO_WritePin(motors[1].pin2Port, motors[1].pin2Pin, GPIO_PIN_RESET);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, topLeftR_EN_Pin|topLeftL_EN_Pin|topRightL_EN_Pin|topRightR_EN_Pin
                          |bottomRightL_EN_Pin|bottomLeftR_EN_Pin|bottomRightR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(bottomLeftL_EN_GPIO_Port, bottomLeftL_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : topLeftR_EN_Pin topLeftL_EN_Pin topRightL_EN_Pin topRightR_EN_Pin
                           bottomRightL_EN_Pin bottomLeftR_EN_Pin bottomRightR_EN_Pin */
  GPIO_InitStruct.Pin = topLeftR_EN_Pin|topLeftL_EN_Pin|topRightL_EN_Pin|topRightR_EN_Pin
                          |bottomRightL_EN_Pin|bottomLeftR_EN_Pin|bottomRightR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : middleIR_Pin leftIR_Pin */
  GPIO_InitStruct.Pin = middleIR_Pin|leftIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : rightIR_Pin */
  GPIO_InitStruct.Pin = rightIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(rightIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : bottomLeftL_EN_Pin */
  GPIO_InitStruct.Pin = bottomLeftL_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(bottomLeftL_EN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
