//
// Created by 15368 on 2024/4/9.
//

#ifndef __KINEMATICS_TASK_H
#define __KINEMATICS_TASK_H


#include "main.h"
#include "math.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "usart.h"
#include "motor.h"
extern const double dl1;  
extern const double dl2;  
extern const double dl3;  
extern const double dl4;  
extern const double dl5;  
extern const double dl6;  

extern QueueHandle_t xQueueUS;

void Kinematics_IK_Task(void* pvParameters);

void RegisterQueueHandle(QueueHandle_t queueHandle);


void invtran(float* Titi, float* Titf);
void pos2tran(float* Xpt, float* Tpt);
void DH1line(float thetadh, float alfadh, float rdh, float ddh, float* Tdh);
void MatrixMultiply(float* A, float* B, int m, int p, int n, float* C);
void MatrixScale(float* A, int m, int n, float k);

void InverseK(float* Xik, float* Jik);

#endif //__KINEMATICS_TASK_H
