#include "game.h"  
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"
#include "FreeRTOS.h"                   // ARM.FreeRTOS::RTOS:Core
#include "task.h"                       // ARM.FreeRTOS::RTOS:Core
#include "event_groups.h"               // ARM.FreeRTOS::RTOS:Event Groups
#include "semphr.h"                     // ARM.FreeRTOS::RTOS:Core
#include "motor.h"  
#include "usart.h"
#include "kinematics_task.h"

struct motor g_motors[6] = {
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0}
};

static void MotorTask(void *params)
{
	struct motor *pmotor = params;
	struct motor idata;
	
	/* 创建自己的队列 */
	QueueHandle_t xQueuekt = xQueueCreate(10, sizeof(struct motor));
	
	/* 注册队列 */
	RegisterQueueHandle(xQueuekt);
	while(1)
	{
		/* 读取按键值:读队列 */
		xQueueReceive(xQueuekt, &idata, portMAX_DELAY);
		
		
		
		motor_driver(&idata);
		// 暂停任务调度
    vTaskSuspendAll();
    printf("addr: %d\n", idata.addr);
    printf("dir: %d\n", idata.dir);
    printf("vel: %d\n", idata.vel);
    printf("acc: %d\n", idata.acc);
    printf("clk: %f\r\n",(float)idata.clk/200/16/50*360);	
		// 恢复任务调度
    xTaskResumeAll();
	}
}

void motor_game(void)
{
	
	BaseType_t ret1;
	BaseType_t ret2;
	BaseType_t ret3;
	BaseType_t ret4;	
	BaseType_t ret5;
	BaseType_t ret6;	
	BaseType_t ret7;
	ret1 = xTaskCreate(MotorTask, "motor1", 128, &g_motors[0], osPriorityNormal, NULL);
  ret2 = xTaskCreate(MotorTask, "motor2", 128, &g_motors[1], osPriorityNormal, NULL);
  ret3 = xTaskCreate(MotorTask, "motor3", 128, &g_motors[2], osPriorityNormal, NULL);	
	ret4 = xTaskCreate(MotorTask, "motor4", 128, &g_motors[3], osPriorityNormal, NULL);
  ret5 = xTaskCreate(MotorTask, "motor5", 128, &g_motors[4], osPriorityNormal, NULL);
  ret6 = xTaskCreate(MotorTask, "motor6", 128, &g_motors[5], osPriorityNormal, NULL);
	ret7 = xTaskCreate(Kinematics_IK_Task, "Kinematics", 512, NULL, osPriorityNormal, NULL);
	if (ret1 != pdPASS) {
			// 任务创建失败，处理错误
			printf("Motor1 task creation failed!\n");
			Error_Handler();  // 可根据需要调用一个错误处理函数
	}

	if (ret2 != pdPASS) {
			// 任务创建失败，处理错误
			printf("Motor2 task creation failed!\n");
			Error_Handler();  // 可根据需要调用一个错误处理函数
	}	
		if (ret3 != pdPASS) {
			// 任务创建失败，处理错误
			printf("Motor3 task creation failed!\n");
			Error_Handler();  // 可根据需要调用一个错误处理函数
	}	
			if (ret4 != pdPASS) {
			// 任务创建失败，处理错误
			printf("Motor4 task creation failed!\n");
			Error_Handler();  // 可根据需要调用一个错误处理函数
	}	
				if (ret5 != pdPASS) {
			// 任务创建失败，处理错误
			printf("Motor5 task creation failed!\n");
			Error_Handler();  // 可根据需要调用一个错误处理函数
	}	
					if (ret6 != pdPASS) {
			// 任务创建失败，处理错误
			printf("Motor6 task creation failed!\n");
			Error_Handler();  // 可根据需要调用一个错误处理函数
	}	
					if (ret7 != pdPASS) {
			// 任务创建失败，处理错误
			printf("Kinematics task creation failed!\n");
			Error_Handler();  // 可根据需要调用一个错误处理函数
	}	
}
