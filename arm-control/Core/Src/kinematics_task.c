#include "kinematics_task.h"
#include "freertos.h"
#include "usart.h"
#include <stdio.h>
#include "task.h"


const double r1 = 35.0;
const double r2 = 146.0;
const double r3 = 52;
const double d1 = 126.5;
const double d3 = 0.0;
const double d4 = 117.0;
const double d6 = 75.5;


const double dl[6] = {200*16*50/360,
											200*16*50/360,
											200*16*50/360,
											200*16*50/360,
											200*16*50/360,
											200*16*50/360};         


uint8_t buff[100]={0};

float Xbox[6] = {0,0,0,0,0,0};
float Jbox[6] = {0};

static QueueHandle_t g_xQueues[10];
static int g_queue_cnt = 0;

/* 创建自己的队列 */
QueueHandle_t xQueueUS;

void Kinematics_IK_Task(void* pvParameters)
{
	struct us_data *pmotor = pvParameters;
	struct us_data idata;
	struct motor mdata[10];
  xQueueUS = xQueueCreate(10, sizeof(struct us_data));

	while(1)
	{
		xQueueReceive(xQueueUS, &idata, portMAX_DELAY);
		
		if(idata.dec_x == 1 && idata.dec_y == 1 && idata.dec_z == 1 && idata.euler_Z1 == 1 && idata.euler_y == 1 && idata.euler_Z2 == 1)
		{
			Xbox[2] +=10;
		}else
		if(idata.dec_x == 2 && idata.dec_y == 2 && idata.dec_z == 2 && idata.euler_Z1 == 2 && idata.euler_y == 2 && idata.euler_Z2 == 2)
		{
			Xbox[2] -=10;
		}else
		if(idata.dec_x == 3 && idata.dec_y == 3 && idata.dec_z == 3 && idata.euler_Z1 == 3 && idata.euler_y == 3 && idata.euler_Z2 == 3)
		{
			Xbox[1] +=10;
		}else
		if(idata.dec_x == 4 && idata.dec_y == 4 && idata.dec_z == 4 && idata.euler_Z1 == 4 && idata.euler_y == 4 && idata.euler_Z2 == 4)
		{
			Xbox[1] -=10;
		}else
		{
		Xbox[0] = idata.dec_x;
		Xbox[1] = idata.dec_y;
		Xbox[2] = idata.dec_z;
		Xbox[3] = idata.euler_Z1;
		Xbox[4] = idata.euler_y;
		Xbox[5] = idata.euler_Z2;
		}
				// 暂停任务调度
    vTaskSuspendAll();
    printf(" Xbox[0]: %f\n", Xbox[0]);
    printf(" Xbox[1]: %f\n", Xbox[1]);
    printf(" Xbox[2]: %f\n", Xbox[2]);
    printf(" Xbox[3]: %f\n", Xbox[3]);
		printf(" Xbox[4]: %f\n", Xbox[4]);
    printf(" Xbox[5]: %f\r\n",Xbox[5]);	
		// 恢复任务调度
    xTaskResumeAll();
		InverseK(Xbox, Jbox);
		
		for (int i = 0; i < g_queue_cnt; i++)
		{
			mdata[i].acc = 0;
			mdata[i].addr = i+1	;
			mdata[i].clk = Jbox[i]*dl[i];
			mdata[i].dir = 0;
			mdata[i].vel = 500;
		}
		for (int i = 0; i < g_queue_cnt; i++)
		{
			xQueueSend(g_xQueues[i], &mdata[i], NULL);
		}
		
	}
}



void RegisterQueueHandle(QueueHandle_t queueHandle)
{
	if (g_queue_cnt < 10)
	{
		g_xQueues[g_queue_cnt] = queueHandle;
		g_queue_cnt++;
	}
}


void invtran(float* Titi, float* Titf)
{
  // finding the inverse of the homogeneous transformation matrix //求齐次变换矩阵的逆
  // first row  //第1行
  Titf[0*4 + 0] = Titi[0*4 + 0];
  Titf[0*4 + 1] = Titi[1*4 + 0];
  Titf[0*4 + 2] = Titi[2*4 + 0];
  Titf[0*4 + 3] = -Titi[0*4 + 0]*Titi[0*4 + 3]-Titi[1*4 + 0]*Titi[1*4 + 3]-Titi[2*4 + 0]*Titi[2*4 + 3];
  // second row //第2行
  Titf[1*4 + 0] = Titi[0*4 + 1];
  Titf[1*4 + 1] = Titi[1*4 + 1];
  Titf[1*4 + 2] = Titi[2*4 + 1];
  Titf[1*4 + 3] = -Titi[0*4 + 1]*Titi[0*4 + 3]-Titi[1*4 + 1]*Titi[1*4 + 3]-Titi[2*4 + 1]*Titi[2*4 + 3];
  // third row  //第3行
  Titf[2*4 + 0] = Titi[0*4 + 2];
  Titf[2*4 + 1] = Titi[1*4 + 2];
  Titf[2*4 + 2] = Titi[2*4 + 2];
  Titf[2*4 + 3] = -Titi[0*4 + 2]*Titi[0*4 + 3]-Titi[1*4 + 2]*Titi[1*4 + 3]-Titi[2*4 + 2]*Titi[2*4 + 3];
  // forth row  //第4行
  Titf[3*4 + 0] = 0.0;
  Titf[3*4 + 1] = 0.0;
  Titf[3*4 + 2] = 0.0;
  Titf[3*4 + 3] = 1.0;
}

void pos2tran(float* Xpt, float* Tpt)
{
  // pos to homogeneous transformation matrix //从点坐标到齐次变换矩阵
  // first row  //第1行
  Tpt[0*4 + 0] = cos(Xpt[3])*cos(Xpt[4])*cos(Xpt[5])-sin(Xpt[3])*sin(Xpt[5]);
  Tpt[0*4 + 1] = -cos(Xpt[3])*cos(Xpt[4])*sin(Xpt[5])-sin(Xpt[3])*cos(Xpt[5]);
  Tpt[0*4 + 2] = cos(Xpt[3])*sin(Xpt[4]);
  Tpt[0*4 + 3] = Xpt[0];
  // second row //第2行
  Tpt[1*4 + 0] = sin(Xpt[3])*cos(Xpt[4])*cos(Xpt[5])+cos(Xpt[3])*sin(Xpt[5]);
  Tpt[1*4 + 1] = -sin(Xpt[3])*cos(Xpt[4])*sin(Xpt[5])+cos(Xpt[3])*cos(Xpt[5]);
  Tpt[1*4 + 2] = sin(Xpt[3])*sin(Xpt[4]);
  Tpt[1*4 + 3] = Xpt[1];
  // third row  //第3行
  Tpt[2*4 + 0] = -sin(Xpt[4])*cos(Xpt[5]);
  Tpt[2*4 + 1] = sin(Xpt[4])*sin(Xpt[5]);
  Tpt[2*4 + 2] = cos(Xpt[4]);
  Tpt[2*4 + 3] = Xpt[2];
  // forth row  //第4行
  Tpt[3*4 + 0] = 0.0;
  Tpt[3*4 + 1] = 0.0;
  Tpt[3*4 + 2] = 0.0;
  Tpt[3*4 + 3] = 1.0;
}

void DH1line(float thetadh, float alfadh, float rdh, float ddh, float* Tdh)
{
  // creats Denavit-Hartenberg homogeneous transformation matrix  //建立德纳维-哈滕伯格齐次变换矩阵
  // first row  //第1行
  Tdh[0*4 + 0] = cos(thetadh);
  Tdh[0*4 + 1] = -sin(thetadh)*cos(alfadh);
  Tdh[0*4 + 2] = sin(thetadh)*sin(alfadh);
  Tdh[0*4 + 3] = rdh*cos(thetadh);
  // second row //第2行
  Tdh[1*4 + 0] = sin(thetadh);
  Tdh[1*4 + 1] = cos(thetadh)*cos(alfadh);
  Tdh[1*4 + 2] = -cos(thetadh)*sin(alfadh);
  Tdh[1*4 + 3] = rdh*sin(thetadh);
  // third row  //第3行
  Tdh[2*4 + 0] = 0.0;
  Tdh[2*4 + 1] = sin(alfadh);
  Tdh[2*4 + 2] = cos(alfadh);
  Tdh[2*4 + 3] = ddh;
  // forth row  //第4行
  Tdh[3*4 + 0] = 0.0;
  Tdh[3*4 + 1] = 0.0;
  Tdh[3*4 + 2] = 0.0;
  Tdh[3*4 + 3] = 1.0;
}


/*
 * MatrixMultiply
 * Matrix Multiplication Routine
 * C = A*B
 * 矩阵的乘法例程
 */
void MatrixMultiply(float* A, float* B, int m, int p, int n, float* C)
{
  // A = input matrix (m x p)
  // B = input matrix (p x n)
  // m = number of rows in A
  // p = number of columns in A = number of rows in B
  // n = number of columns in B
  // C = output matrix = A*B (m x n)
  int i, j, k;
  for (i = 0; i < m; i++)
    for(j = 0; j < n; j++)
    {
      C[n * i + j] = 0;
      for (k = 0; k < p; k++)
        C[n * i + j] = C[n * i + j] + A[p * i + k] * B[n * k + j];
    }
}

/*
 * MatrixScale
 * Matrix Scale
 * 矩阵比例变化程序
 */
void MatrixScale(float* A, int m, int n, float k)
{
  for (int i = 0; i < m; i++)
    for (int j = 0; j < n; j++)
      A[n * i + j] = A[n * i + j] * k;
}

void InverseK(float* Xik, float* Jik)
{
  // from deg to rad    //角度制转到弧度制
  // Xik(4:6)=Xik(4:6)*pi/180;
  Xik[3]=Xik[3]*PI/180.0;
  Xik[4]=Xik[4]*PI/180.0;
  Xik[5]=Xik[5]*PI/180.0;
  // Denavit-Hartenberg matrix  //DH坐标系矩阵
  float theta[6]={0.0, -90.0, 0.0, 0.0, 0.0, 0.0};      // theta=[0; -90+0; 0; 0; 0; 0];    //θ
  float alfa[6]={-90.0, 0.0, -90.0, 90.0, -90.0, 0.0};  // alfa=[-90; 0; -90; 90; -90; 0];  //a
  float r[6]={r1, r2, r3, 0.0, 0.0, 0.0};               // r=[47; 110; 26; 0; 0; 0];        //r
  float d[6]={d1, 0.0, d3, d4, 0.0, d6};                // d=[133; 0; 0; 117.5; 0; 28];     //d
  // from deg to rad  //角度制转到弧度制
  MatrixScale(theta, 6, 1, PI/180.0);   // theta=theta*pi/180;
  MatrixScale(alfa, 6, 1, PI/180.0);    // alfa=alfa*pi/180;
  // work frame   //工作坐标系
  float Xwf[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Xwf=[0; 0; 0; 0; 0; 0];
  // tool frame   //工具坐标系
  float Xtf[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Xtf=[0; 0; 0; 0; 0; 0];
  // work frame transformation matrix //工作坐标系变换矩阵
  float Twf[16];
  pos2tran(Xwf, Twf); // Twf=pos2tran(Xwf);
  // tool frame transformation matrix //工具坐标系变换矩阵
  float Ttf[16];
  pos2tran(Xtf, Ttf); // Ttf=pos2tran(Xtf);
  // total transformation matrix      //全部的变换矩阵
  float Twt[16];
  pos2tran(Xik, Twt); // Twt=pos2tran(Xik);
  // find T06 //找到 T06
  float inTwf[16], inTtf[16], Tw6[16], T06[16];
  invtran(Twf, inTwf); // inTwf=invtran(Twf);
  invtran(Ttf, inTtf); // inTtf=invtran(Ttf);
  MatrixMultiply(Twt, inTtf, 4, 4, 4, Tw6); // Tw6=Twt*inTtf;
  MatrixMultiply(inTwf, Tw6, 4, 4, 4, T06); // T06=inTwf*Tw6;
  // positon of the spherical wrist //球形手腕的位置
  float Xsw[3];
  // Xsw=T06(1:3,4)-d(6)*T06(1:3,3);
  Xsw[0]=T06[0*4 + 3]-d[5]*T06[0*4 + 2];
  Xsw[1]=T06[1*4 + 3]-d[5]*T06[1*4 + 2];
  Xsw[2]=T06[2*4 + 3]-d[5]*T06[2*4 + 2];
  // joints variable    //关节变量
  // Jik=zeros(6,1);
  // first joint        //第一个关节
  Jik[0]=atan2(Xsw[1],Xsw[0])-atan2(d[2],sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])); // Jik(1)=atan2(Xsw(2),Xsw(1))-atan2(d(3),sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2));
  // second joint       //第二个关节
  Jik[1] = PI/2.0
           -acos((r[1]*r[1]+(Xsw[2]-d[0])*(Xsw[2]-d[0])+(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])*(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])-(r[2]*r[2]+d[3]*d[3]))/(2.0*r[1]*sqrt((Xsw[2]-d[0])*(Xsw[2]-d[0])+(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])*(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0]))))
           -atan((Xsw[2]-d[0])/(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])); // Jik(2)=pi/2-acos((r(2)^2+(Xsw(3)-d(1))^2+(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1))^2-(r(3)^2+d(4)^2))/(2*r(2)*sqrt((Xsw(3)-d(1))^2+(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1))^2)))-atan((Xsw(3)-d(1))/(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1)));
  // third joint        //第三个关节
  Jik[2]=PI
         -acos((r[1]*r[1]+r[2]*r[2]+d[3]*d[3]-(Xsw[2]-d[0])*(Xsw[2]-d[0])-(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])*(sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0]))/(2*r[1]*sqrt(r[2]*r[2]+d[3]*d[3])))
         -atan(d[3]/r[2]); // Jik(3)=pi-acos((r(2)^2+r(3)^2+d(4)^2-(Xsw(3)-d(1))^2-(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1))^2)/(2*r(2)*sqrt(r(3)^2+d(4)^2)))-atan(d(4)/r(3));
  // last three joints  //最后三个关节
  float T01[16], T12[16], T23[16], T02[16], T03[16], inT03[16], T36[16];
  DH1line(theta[0]+Jik[0], alfa[0], r[0], d[0], T01); // T01=DH1line(theta(1)+Jik(1),alfa(1),r(1),d(1));
  DH1line(theta[1]+Jik[1], alfa[1], r[1], d[1], T12); // T12=DH1line(theta(2)+Jik(2),alfa(2),r(2),d(2));
  DH1line(theta[2]+Jik[2], alfa[2], r[2], d[2], T23); // T23=DH1line(theta(3)+Jik(3),alfa(3),r(3),d(3));
  MatrixMultiply(T01, T12, 4, 4, 4, T02); // T02=T01*T12;
  MatrixMultiply(T02, T23, 4, 4, 4, T03); // T03=T02*T23;
  invtran(T03, inT03); // inT03=invtran(T03);
  MatrixMultiply(inT03, T06, 4, 4, 4, T36); // T36=inT03*T06;
  // forth joint      //第四个关节
  Jik[3]=atan2(-T36[1*4+2], -T36[0*4+2]); // Jik(4)=atan2(-T36(2,3),-T36(1,3));
  // fifth joint      //第五个关节
  Jik[4]=atan2(sqrt(T36[0*4+2]*T36[0*4+2]+T36[1*4+2]*T36[1*4+2]), T36[2*4+2]); // Jik(5)=atan2(sqrt(T36(1,3)^2+T36(2,3)^2),T36(3,3));
  // sixth joints     //第六个关节
  Jik[5]=atan2(-T36[2*4+1], T36[2*4+0]); // Jik(6)=atan2(-T36(3,2),T36(3,1));
  // rad to deg       //角度制转弧度制
  MatrixScale(Jik, 6, 1, 180.0/PI); // Jik=Jik/pi*180;
}
