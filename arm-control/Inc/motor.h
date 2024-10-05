#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
#include "tim.h"
/**
  * @brief    位置模式
  * @param    addr：电机地址
  * @param    dir ：方向        ，0为CW，其余值为CCW
  * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
  * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
  * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
struct motor {
    uint8_t addr;  // 地址
    uint8_t dir;   // 方向
    uint16_t vel;  // 速度
    uint8_t acc;   // 加速度
    uint32_t clk;  // 脉冲数
};

// 定义每个电机的脉冲计数器
extern volatile uint32_t motor1_pulses;
extern volatile uint32_t motor2_pulses;
extern volatile uint32_t motor3_pulses;
extern volatile uint32_t motor4_pulses;

// 定义目标脉冲数
extern uint32_t motor1_target_pulses;
extern uint32_t motor2_target_pulses;
extern uint32_t motor3_target_pulses;
extern uint32_t motor4_target_pulses;

void start_motor(TIM_HandleTypeDef *htim, uint32_t channel, volatile uint32_t *pulses, uint32_t target_pulses);

void motor_driver(void *pvParameters);
void motor_step1(void *pvParameters);
void motor_step2(void *pvParameters);
void motor_step3(void *pvParameters);
void motor_step6(void *pvParameters);
void motor_can(void *pvParameters);
#endif //__MOTOR_H

