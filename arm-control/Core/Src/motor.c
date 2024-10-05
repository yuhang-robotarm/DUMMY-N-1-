#include "motor.h" 
#include "can.h" 
#include "freertos.h"
#include "task.h"
#include "tim.h"

// 定义每个电机的脉冲计数器
volatile uint32_t motor1_pulses = 0;
volatile uint32_t motor2_pulses = 0;
volatile uint32_t motor3_pulses = 0;
volatile uint32_t motor4_pulses = 0;

// 定义目标脉冲数
uint32_t motor1_target_pulses = 0;
uint32_t motor2_target_pulses = 0;
uint32_t motor3_target_pulses = 0;
uint32_t motor4_target_pulses = 0;

// 定义 PWM 占空比
uint32_t pwm_duty_cycle = 50; // 50%


void motor_driver(void *pvParameters)
{	
	struct motor *idata = pvParameters;
	
	
	if(idata -> addr == 1){
		motor_step1(idata);
	}else
	if(idata -> addr == 2)
	{
		motor_step2(idata);
	}else
	if(idata -> addr == 3)
	{
		motor_step3(idata);
	}else
	if(idata -> addr == 6)
	{
		motor_step6(idata);
	}else{
		motor_can(idata);
	}

}

void motor_step1(void *pvParameters)
{
	struct motor *idata = pvParameters;
	float xclk;
	static float rclk1 = 0;
	
	xclk = idata -> clk - rclk1;
	if(xclk <= 0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		motor1_target_pulses = -xclk;
	}else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		motor1_target_pulses = xclk;
	}
	start_motor(&htim3, TIM_CHANNEL_1, &motor1_pulses, motor1_target_pulses);
	
	rclk1 = idata -> clk;
}

void motor_step2(void *pvParameters)
{
	struct motor *idata = pvParameters;
	float xclk;
	static float rclk1 = 0;
	
	xclk = idata -> clk - rclk1;
	if(xclk <= 0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		motor2_target_pulses = -xclk;
	}else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		motor2_target_pulses = xclk;
	}	
	
	start_motor(&htim3, TIM_CHANNEL_2, &motor2_pulses, motor2_target_pulses);
	
	rclk1 = idata -> clk;
}

void motor_step3(void *pvParameters)
{
	struct motor *idata = pvParameters;
	float xclk;
	static float rclk1 = 0;
	
	xclk = idata -> clk - rclk1;
	
	if(xclk <= 0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		motor3_target_pulses = -xclk;
	}else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		motor3_target_pulses = xclk;
	}
	
	start_motor(&htim4, TIM_CHANNEL_1, &motor3_pulses, motor3_target_pulses);
	
	rclk1 = idata -> clk;
}

void motor_step6(void *pvParameters)
{
	struct motor *idata = pvParameters;
	float xclk;
	static float rclk1 = 0;
	
	xclk = idata -> clk - rclk1;
	
	if(xclk <= 0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		motor4_target_pulses = -xclk;
	}else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		motor4_target_pulses = xclk;
	}
	
	start_motor(&htim4, TIM_CHANNEL_2, &motor4_pulses, motor4_target_pulses);
	
	rclk1 = idata -> clk;
}

void start_motor(TIM_HandleTypeDef *htim, uint32_t channel, volatile uint32_t *pulses, uint32_t target_pulses) {
    *pulses = 0;
    HAL_TIM_PWM_Start_IT(htim, channel);

    while (*pulses < target_pulses) {
        // 等待目标脉冲数达到
    }

    HAL_TIM_PWM_Stop_IT(htim, channel); // 达到目标脉冲数后停止PWM
}

void motor_can(void *pvParameters)
{
	struct motor *idata = pvParameters;
  CAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[8];
  uint32_t TxMailbox;
  uint8_t cmd[16] = {0};
  uint8_t i = 0, j = 0, k = 0, l = 0, packNum = 0;
  uint8_t len = 13;

  // 初始化cmd数组
  cmd[0]  =  idata -> addr ;
  cmd[1]  =  0xFD;
  cmd[2]  =  idata -> dir;
  cmd[3]  =  (uint8_t)(idata -> vel >> 8);
  cmd[4]  =  (uint8_t)(idata -> vel >> 0);
  cmd[5]  =  idata -> acc;
  cmd[6]  =  (uint8_t)(idata -> clk >> 24);
  cmd[7]  =  (uint8_t)(idata -> clk >> 16);
  cmd[8]  =  (uint8_t)(idata -> clk >> 8);
  cmd[9]  =  (uint8_t)(idata -> clk >> 0);
  cmd[10] =  0;
  cmd[11] =  0;
  cmd[12] =  0x6B;

  j = len - 2;

  TxHeader.StdId = 0x00;
  TxHeader.IDE = CAN_ID_EXT;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.TransmitGlobalTime = DISABLE;


		while (i < j)
			{
				k = j - i;
				TxHeader.ExtId = ((uint32_t)cmd[0] << 8) | (uint32_t)packNum;
				TxData[0] = cmd[1];
				if (k < 8)
				{
						for (l = 0; l < k; l++, i++)
						{
								TxData[l + 1] = cmd[i + 2];
						}
						TxHeader.DLC = k + 1;
				}
				else
				{
						for (l = 0; l < 7; l++, i++)
						{
								TxData[l + 1] = cmd[i + 2];
						}
						TxHeader.DLC = 8;
				}

				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
				{
						Error_Handler();
				}
				++packNum;
    }	
	
	
}
