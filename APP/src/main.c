/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
 * File Name          : main.c
 * Author             : danceww
 * Version            : V0.0.1
 * Date               : 08/23/2010
 * Description        : Main program body
 *******************************************************************************/

/* Includes ------------------------------------------------------------------*/

//#include "includes.h"
#include "stm32f10x_lib.h"
#include "dynamixel.h"
#include "dxl_hal.h"
#include "math.h"
#include <string.h>
#include "rand.h"
#include <stdio.h>
#include <stdlib.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define P_GOAL_POSITION_L		30
#define P_GOAL_POSITION_H		31
#define P_MOVING_SPEED			32
#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37
#define P_MOVING				46

#define PORT_ENABLE_TXD			GPIOB
#define PORT_ENABLE_RXD			GPIOB
#define PORT_DXL_TXD			GPIOB
#define PORT_DXL_RXD			GPIOB
#define PORT_ZIGBEE_TXD			GPIOC
#define PORT_ZIGBEE_RXD			GPIOD
#define PORT_ZIGBEE_RESET		GPIOA

#define PIN_ENABLE_TXD			GPIO_Pin_4
#define PIN_ENABLE_RXD			GPIO_Pin_5
#define PIN_DXL_TXD				GPIO_Pin_6
#define PIN_DXL_RXD				GPIO_Pin_7
#define PIN_PC_TXD				GPIO_Pin_10
#define PIN_PC_RXD              GPIO_Pin_11
#define PIN_ZIGBEE_TXD			GPIO_Pin_12
#define PIN_ZIGBEE_RXD			GPIO_Pin_2
#define PIN_ZIGBEE_RESET		GPIO_Pin_12

#define USART_DXL			    0
#define USART_ZIGBEE		 	1
#define USART_PC			    2

#define word                    u16
#define byte                    u8

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile byte gbpRxInterruptBuffer[256]; // dxl buffer
volatile byte gbRxBufferWritePointer, gbRxBufferReadPointer;
volatile vu32 gwTimingDelay, gw1msCounter;
u32 Baudrate_DXL = 1000000;
u32 Baudrate_PC = 115200;
vu16 CCR1_Val = 100; // 1ms
vu32 capture = 0;
int UM6_recieve_state = 0;

int UM6_Roll_Recieve = 0;
int UM6_Roll = 0;

int UM6_Pitch_Recieve = 0;
int UM6_Pitch = 0;

int UM6_Yaw_Recieve = 0;
int UM6_Yaw = 0;
int HeadPos[2] = { 500, 500};
word GoalPos[43] = {
		0,0,0,0,0,0,0,0,0,0,
		0x080a,0x07dd,0x06ed,0x0849,0x0458,0x005a,0,0,0,0,
		0x085b,0x07df,0x06ef,0x088e,0x0b4e,0x0dab,0,0,0,0,
		0x0dff,0x04f1,0x0509,0,0,0,0,0,0,0,
		0x04cf,0x0ab6,0x02f0
};
word Position;
word wPresentPos;
byte INDEX = 0;
byte Voltage;
byte id = 16;
byte bMoving, CommStatus;

/* Private walking variables --------------------------------------------------*/

u32 millisec;
float scaler = 0;
float scalerR, scalerL, walkspeed = 0;
int sidecurve, curveR, curveL, rotcurve, walkcurve;
volatile u32 begin;
byte frames = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void RCC_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void SysTick_Configuration(void);
void Timer_configuration(void);
void TimerInterrupt_1ms(void);
void RxD0Interrupt(void);
void __ISR_DELAY(void);
void USART1_Configuration(u32);
void USART_Configuration(u8, u32);
void DisableUSART1(void);
void ClearBuffer256(void);
byte CheckNewArrive(void);
void PrintCommStatus(int);
void PrintErrorCode(void);
void TxDByte_DXL(byte);
byte RxDByte_DXL(void);
void TxDString(byte*);
void TxDWord16(word);
void TxDByte16(byte);
void TxDByte_PC(byte);
void TxDInt(int);
void Timer_Configuration(void);
void mDelay(u32);
void SetRunTime(u32);
int RunTiming(void);
void StartDiscount(s32);
void EnableZigbee(void);
void RxD2Interrupt(void);
byte CheckTimeOut(void);
volatile byte gbPacketWritePointer;
volatile byte gbPacketReadPointer;
volatile byte gbpPacketDataBuffer[16 + 1 + 16]; //PC buffer

// HaviMo
byte camType;

typedef struct {
	byte Index, Color;
	unsigned short int NumPix;
	unsigned int SumX, SumY;
	byte MaxX, MinX, MaxY, MinY;
} HaViMoRegion;

HaViMoRegion regions[16];

int BallX, BallY;
int GoalX, GoalY;
int noBall = 0;
int noGoal = 0;
int ballAtFeet=0;


int wantedHeadPos;
int currentBallPos[2];
int currentGoalPos[2];
int side, rot, forward, curve, distance;
int switchX = 0;
int switchY = 0;
int stepperiode;

int captureRegion(HaViMoRegion*);
int captureRegion(HaViMoRegion *regions);
int getDataFromCamera(const char*, int, unsigned char*, int, int);
int findBall(void);
int findGoal(void);
int headSearch(void);

// Constants

const int maxForward = 200;
const int maxCurve  = 200;
const int maxSide = 50;
const int stepPeriod = 600;
const int maxRot = 50;
const int ballColor = 1;
const int goalColor = 2;
const int fieldColor = 3;

const int maxHeadY = 500;
const int minHead = 310;
int MaxHeadX = 1000;
/*******************************************************************************
 * Function Name  : main
 * Description    : Main program
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/



#define FIR_HIST_DEPTH 100
int responseOCM[FIR_HIST_DEPTH];
int responseWalk[FIR_HIST_DEPTH];
int responseFront[FIR_HIST_DEPTH];

int histFWD[FIR_HIST_DEPTH];
int histptrFWD = 0;
int hist[FIR_HIST_DEPTH];
int histptr = 0;


void pushFIR(int val,int* hist_, int* ptr){
	hist_[*ptr] = val;
	(*ptr)--;
	if (*ptr < 0) *ptr = FIR_HIST_DEPTH-1;
}

int calcFIR(int* resp,int* hist_, int* ptr){
	int sum=0;
	int i;
	for (i=0; i < FIR_HIST_DEPTH; i++){
		//		int v = i< 35? 1 : i<55 ? -2 : i<65 ? 1 : 0;
		sum+=hist_[((*ptr)+i+1) % FIR_HIST_DEPTH] * resp[i];
	}
	return sum;
}



int main(void) {

	/* System Clocks Configuration */
	RCC_Configuration();

	/* NVIC configuration */
	NVIC_Configuration();

	/* GPIO configuration */
	GPIO_Configuration();

	SysTick_Configuration();

	Timer_Configuration();
	SysTick_CounterCmd(SysTick_Counter_Enable);


	/* Initialize USART configurations*/

	USART_Configuration(USART_PC, Baudrate_PC);

	EnableZigbee();

	USART_SendData(UART5,0xAC);

	mDelay(2000);



//	TxDString("Init");
//	TxDString(",");
//	TxDInt(millisec);
//	TxDString("\r\n");
	/*
	 * Initialize  DXL
	 */


	dxl_initialize(0, 1);


	dxl_write_byte(254, 16, 2);
	dxl_write_byte(254, 28, 64);
	dxl_write_byte(254, 27, 0);
	dxl_write_byte(254, 24, 1);

	int robotmodel = (dxl_read_word(11,0)==0x0136)?-1:1;
	if(robotmodel <0){
		dxl_write_word(11, 28, 32);
		dxl_write_word(21, 28, 32);
		dxl_write_word(12, 28, 32);
		dxl_write_word(13, 28, 32);
		dxl_write_word(22, 28, 32);
		dxl_write_word(23, 28, 32);
	}else{
		dxl_write_word(11, 28, 128);
		dxl_write_word(21, 28, 128);
	}


	dxl_write_word(15, 28, 32);
	dxl_write_word(25, 28, 32);

	dxl_write_word(30, 28, 16);
	dxl_write_word(31, 28, 16);
	dxl_write_word(40, 28, 16);
	dxl_write_word(41, 28, 16);

	/* Initialize positions */
	for (INDEX = 10; INDEX < 16; INDEX++) {
		do
			GoalPos[INDEX] = dxl_read_word(INDEX, P_PRESENT_POSITION_L);
		while (GoalPos[INDEX] == 0);
//		TxDWord16(GoalPos[INDEX]);
//		TxDByte_PC('\r');
//		TxDByte_PC('\n');
	}

	for (INDEX = 20; INDEX < 26; INDEX++) {
		do
			GoalPos[INDEX] = dxl_read_word(INDEX, P_PRESENT_POSITION_L);
		while (GoalPos[INDEX] == 0);
//		TxDWord16(GoalPos[INDEX]);
//		TxDByte_PC('\r');
//		TxDByte_PC('\n');
	}

	dxl_write_word(30, 30,3220 );
	dxl_write_word(31, 30, 800);
	dxl_write_word(32, 30, 664);
	dxl_write_word(40, 30, 1195);
	dxl_write_word(41, 30, 2770);
	dxl_write_word(42, 30, 1520);

//	for (INDEX = 30; INDEX < 33; INDEX++) {
//		do
//			GoalPos[INDEX] = dxl_read_word(INDEX, P_PRESENT_POSITION_L);
//		while (GoalPos[INDEX] == 0);
//		TxDWord16(GoalPos[INDEX]);
//		TxDByte_PC('\r');
//		TxDByte_PC('\n');
//	}
//	for (INDEX = 40; INDEX < 43; INDEX++) {
//		do
//			GoalPos[INDEX] = dxl_read_word(INDEX, P_PRESENT_POSITION_L);
//		while (GoalPos[INDEX] == 0);
//		TxDWord16(GoalPos[INDEX]);
//		TxDByte_PC('\r');
//		TxDByte_PC('\n');
//	}




	/* Head position initialize */
	dxl_write_word(50, 30, HeadPos[0]);
	dxl_write_word(51, 30, HeadPos[1]);


//	while (1){
////		int mx = (s16)dxl_read_word(101,0);
//		int my = dxl_read_word(101,10);
//		TxDWord16(my);
//		TxDByte_PC(' ');
//		TxDByte_PC(' ');
//	}

	int lastwalk = 0;
	int walk = 0;
	int corr = 0;
	int stepStart=millisec;

	int ax = (s16)dxl_read_word(15,36)-GoalPos[15] +(s16)dxl_read_word(25,36)-GoalPos[25];
	int ay = (s16)dxl_read_word(11,36)+(s16)dxl_read_word(21,36) - 2*2048;
	int iax = 0;
	int i;
	for (i=0; i<FIR_HIST_DEPTH; i++){
		responseOCM[i] =  i< 30? 1 : i<60 ? -1 : i<70 ? 0 : 0;
		responseWalk[i] = i< 30? 1 : i<60 ? -1 : i<70 ? 0 : 0;
		responseFront[i] = i< 1? 1 : i<2 ? -1 : i<100 ? 0 : 0;
		pushFIR(ay,hist,&histptr);
		pushFIR(ax,histFWD,&histptrFWD);
	}

	//0 = kickoff
	//1 = no kickoff
	int startState = 0;

	while (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3)==Bit_SET){

		TxDInt(UM6_Yaw);

		TxDString("\r\n");

		if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15)==Bit_RESET) startState=1;
		if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_14)==Bit_RESET) startState=0;
		if (startState == 0)
			if ((millisec % 1000)<500) GPIO_SetBits(GPIOB,GPIO_Pin_13); else GPIO_ResetBits(GPIOB,GPIO_Pin_13);
		else
			if ((millisec % 200)<100) GPIO_SetBits(GPIOB,GPIO_Pin_13); else GPIO_ResetBits(GPIOB,GPIO_Pin_13);

	}

	// Initialize variables

	int lastax;

	int frameCnt=0;

	int sumCorr=0;

//	int gain=40;

	int avgAmp = 0;

	int fwdcorr =0;

	int sumax=0;

	if(startState==0) mDelay(10000);
	else mDelay(25000);

		/* Find Ball and Goal */
	while(findBall()==0)
		headSearch();

	currentBallPos[0]= HeadPos[0]-(BallX - 80);
	//		currentBallPos[1]=HeadPos[1]-(BallY-60);
	currentBallPos[1]=HeadPos[1];

	//		dxl_write_word(50, 30, HeadPos[0]);
	//		dxl_write_word(51, 30, HeadPos[1]);

	//		while(findGoal()==0)
	//			headSearch();
	//


	dxl_write_word(50, 30, currentBallPos[0]);
	dxl_write_word(51, 30, currentBallPos[1]);

	SetRunTime(20000);


/* End Camera initialization */

	stepperiode = stepPeriod;
	curve = maxCurve;
	rot = 0;
	side = 0;
	forward = 0;
	int ballFrames =0;
	int j;


	int startKick =0;
	currentGoalPos[0] = UM6_Yaw;
	currentGoalPos[1] = 512;
	while (1) {

		/*
		 * Walk code
		 */

//		TxDString("Forward");
//		TxDInt(forward);
//		TxDString("\r\n");

		for (j = 0; j < 30; j++) {

//			mDelay(5);
//			int mx = (s16)dxl_read_word(101,0);
//			int my = 0;//(s16)dxl_read_word(101,10) - 0x22C; //10,0x22c
			ay = (s16)dxl_read_word(11,36)+(s16)dxl_read_word(21,36) - 2*2048;//(s16)dxl_read_word(101,4) - 0x22C;
			ax = (s16)dxl_read_word(15,36)-GoalPos[15] +(s16)dxl_read_word(25,36)-GoalPos[25];

//			ax+= forward / 40;
			lastax=ax;

//			iax = ax>0? iax+1 : iax-1;

//			if(RunTiming()>0){
//				iax+=ax;
//			}
//			else{
				iax+=ax/2;
//			}
			pushFIR(ax,histFWD,&histptrFWD);
			//			int d = calcFIR(responseFront,histFWD,&histptrFWD)*4;

			int frontCorr =  ax/2 + iax/2000 - forward/40;//sqrt(abs(iax))* (iax>0?1:-1)/10;// -d  + ax / 10;

			if (frontCorr>100) frontCorr=100;
			if (frontCorr<-100) frontCorr=-100;

			fwdcorr = 0;//ax*5;

			if (fwdcorr>200) fwdcorr=200;
			if (fwdcorr<-200) fwdcorr=-200;

			lastwalk = walk;
			pushFIR(ay, hist, &histptr);
			corr = 0;//calcFIR(responseOCM, hist, &histptr) * gain / 100;
			walk = calcFIR(responseWalk, hist, &histptr);

//			TxDInt(ax);
//			TxDString(",");
//			TxDInt(fwdcorr);
//			TxDString(",");
//			TxDInt(frontCorr);
//			TxDString(",");
//			TxDInt(0);
//			TxDString("\r\n");

			sumCorr += abs(corr);
			sumax+=ax;

			frameCnt++;

			if (corr>200) corr=200;
			if (corr<-200) corr=-200;

			if (walk * lastwalk <=0 ){
				int temp = millisec - stepStart;
				if (temp>500){
//				  stepperiode = (stepperiode+temp)/2;
				}
				stepStart=millisec;
//				TxDString("stepperiode:");
//				TxDWord16(stepperiode);
//				TxDString(" FPS:");
//				TxDWord16(1000*frameCnt/temp);
//				TxDByte_PC('\r');
//				TxDByte_PC('\n');


				avgAmp = sumCorr/frameCnt;
//
//				if (avgAmp<100) gain += 1; else gain -= 1;
//				if (gain<10) gain=10;
//				if (gain>50) gain=50;

//				curve=(curve + (avgAmp + 100)) / 2;
//				if (curve>250) curve=250;

				frameCnt = 0;
				sumCorr = 0;
				sumax = 0;

//				forward = curve/2;
			}



			scaler = ((float) (millisec % stepperiode) / stepperiode);
			scaler = scaler>1? 1 : scaler;

			scalerR = (scaler > 0.5) ? scaler * 2 - 1 : 0;
			scalerL = (scaler < 0.5) ? scaler * 2 : 1;

			curveR = curve * sin(3.14 * sqrt(scalerR)) - 50;
			curveL = curve * sin(3.14 * sqrt(scalerL)) - 50;

			sidecurve = 0;//side*sin(2 * 3.14 * scaler)+ abs(side);
			rotcurve = rot* sin(2 * 3.14 * scaler);
			walkcurve = (forward+fwdcorr) * sin(2 * 3.14 * scaler - 0);


			// Write values to motors

			dxl_write_word(10, 30, GoalPos[10] + rotcurve);
			dxl_write_word(20, 30, GoalPos[20] - rotcurve);

			//11,12 applied on the other leg to minimize weight effect
			dxl_write_word(11, 30, GoalPos[11]+robotmodel*(-60 - sidecurve) );// - corr/10);


			dxl_write_word(21, 30, GoalPos[21] +robotmodel*( 60 + sidecurve));// - corr/10);

			dxl_write_word(12, 30, GoalPos[12] - curveL - frontCorr);
			dxl_write_word(13, 30, GoalPos[13] + curveL + frontCorr);

			dxl_write_word(22, 30, GoalPos[22] - curveR - frontCorr);
			dxl_write_word(23, 30, GoalPos[23] + curveR + frontCorr);

			dxl_write_word(14, 30, GoalPos[14] - walkcurve - frontCorr * 6);

			dxl_write_word(24, 30, GoalPos[24] + walkcurve - frontCorr * 6);

			dxl_write_word(15, 30, GoalPos[15] - walkspeed + frontCorr * 0);

			dxl_write_word(25, 30, GoalPos[25] - walkspeed + frontCorr * 0);


		}

//		if () forward++;

		/*
		 * End of Walk code
		 */


		/*
		 * Camera code
		 */

		switch (ballAtFeet){
		case 0:
			TxDInt(currentGoalPos[0]);
			TxDString(",");
			TxDInt(UM6_Yaw);
			TxDString(",");
			TxDInt(rot);
			TxDString("\r\n");

			if (findBall()>0) {

				HeadPos[0] -= (BallX - 80);
				HeadPos[1] -= (BallY -60);

				if(HeadPos[0]<0) HeadPos[0] = 0;
				if(HeadPos[0]>MaxHeadX) HeadPos[0]=MaxHeadX;

				if(HeadPos[1]<minHead) HeadPos[1]=minHead;
				if(HeadPos[1]>maxHeadY) HeadPos[1] =maxHeadY;


				float cameraAngle = (90 - (512- HeadPos[1])*270.0/1023)*3.14/180;

				float ballAngle = (BallY-60)*(40.0/120)*(3.14/180);

				distance = (int) 90*tan(cameraAngle-ballAngle);
				if (distance <0) distance = 1000;

				float headAngle = (HeadPos[0]-512)*270.0/1023*(3.14/180);
				int BallRevalPosX = distance*cos(headAngle);
				int BallRevalPosY = distance*sin(headAngle);

				currentBallPos[0] = HeadPos[0];
				currentBallPos[1] = HeadPos[1];

				dxl_write_word(50, 30, currentBallPos[0]);
				dxl_write_word(51, 30, currentBallPos[1]);

				/*
				 * Player Code
				 */


				curve = maxCurve; // Curve init

				//forward = (forward <2*(BallRevalPosX))?forward+10:2*BallRevalPosX;  // Forward init

				int goalDiff = (currentGoalPos[0]-UM6_Yaw);
				if (goalDiff>180) goalDiff-=360;
				if (goalDiff<-180) goalDiff+=360;

				side = 0;
				rot = (512 - currentBallPos[0])/2 - goalDiff/2;

				int fwd = maxForward;
				if(BallRevalPosX<100)fwd = maxForward-(abs(rot)*maxForward/maxRot);

				if (fwd>forward) forward+=10;
				if (fwd<forward) forward-=10;

				if(forward > maxForward) forward = maxForward;
				if(forward < -maxForward) forward = -maxForward;


//				// If ball is close to us
//				if(BallRevalPosX<80 && BallRevalPosX>10){
//					// Not directly in front of us
//					if(abs(BallRevalPosY)>20){
//						rot = -BallRevalPosY*2; // ?
////						rot = -(currentGoalPos[0]-UM6_Yaw);
////						if (rot>180) rot-=360;
////						if (rot<-180) rot+=360;
////						side = -BallRevalPosY;
////						side += side<0? -20 : 20;
////						if(side>maxSide) side = maxSide;
////						if(side<-maxSide) side = -maxSide;
//					}
//
//					// close to us
//					if(abs(BallRevalPosY)<20){
//						side =0;
//						forward = (forward < maxForward) ? forward +10: maxForward;
//						rot = -(currentGoalPos[0]-UM6_Yaw)/4;
////						if (rot>180) rot-=360;
////						if (rot<-180) rot+=360;
//
////						TxDInt(rot);
////						TxDString("\r\n");
////						rot = (512 - currentGoalPos[0])/4;  // Rotate towards goal
////						currentGoalPos[0]=(currentGoalPos[0]*9+512*1)/10;
////						ballFrames +=1;
//					}
//				}else{ // Ball not close to us
//					side=0;
//					rot = (512 - currentBallPos[0])/4; // Rotate towards ball
////					rot = -(currentGoalPos[0]-UM6_Yaw);
////					if (rot>180) rot-=360;
////					if (rot<-180) rot+=360;
//					}

				if(BallRevalPosX<40 && BallRevalPosX>0 && abs(BallRevalPosY)<20 && abs(goalDiff)<20){
					// MiniKick
					if (millisec-startKick<3000){
						side = 0;
						rot = 0;
						curve = maxCurve;//+150;
						forward = maxForward;//+100;

					}else startKick = millisec;
				}


				//  Ball Behind us
//				if(BallRevalPosX<0) side=0;

//				if(BallRevalPosX<50 && abs(BallRevalPosY) > 20) {
//					rot = 0;
////					rot = -(currentGoalPos[0]-UM6_Yaw);
////					if (rot>180) rot-=360;
////					if (rot<-180) rot+=360;					side = 0;
//					forward=-maxForward;
//				}else{
////			rot = currentGoalPos[0]-UM6_Yaw;
					if(rot > maxRot) rot = maxRot;
					if(rot <-maxRot) rot = -maxRot;
//				}



				/*
				 * End Player Code
				 */


				/*
				 * Goal Keaper Code
				 *
				 */
//				side= -BallRevalPosY;
//				if(side>maxSide) side = maxSide;
//				if(side<-maxSide) side = -maxSide;
//				if(abs(side) <15){
//					side =0;
//					rot =0;
//					curve = curve >0? curve-10: 0;
//
//				}else{
//				curve = curve < maxCurve? curve +10:maxCurve;
//				rot = side/5;
//				}
//				forward = 0;

				/*
				 * End GoalKeaper Code
				 */

			}else if(noBall==5){
				HeadPos[0]=MaxHeadX/2;
				HeadPos[1] = minHead;
				dxl_write_word(50, 30, HeadPos[0]);
				dxl_write_word(51, 30, HeadPos[1]);
			}
			else if (noBall > 10){
				headSearch();
				rot=0;
				side=0;
				forward=-maxForward;

			}

			// ball vor mir und nah und lagen gesehen.
			if(ballFrames >10){
//				TxDString("Search for goal \r\n");
				ballAtFeet = 1;
				HeadPos[1]=maxHeadY;
				dxl_write_word(51, 30, 500);
			}
			break;

		case 1:
			if(findGoal()>0){
				HeadPos[0] -= (GoalX - 80); // Put the goal in the center of image
				HeadPos[1] -= (GoalY - 60);

				if(HeadPos[1]<minHead) HeadPos[1]= minHead;
				if(HeadPos[1]>maxHeadY) HeadPos[1] = maxHeadY;

				currentGoalPos[0] = HeadPos[0];
				currentGoalPos[1] = HeadPos[1];

				dxl_write_word(50, 30, currentBallPos[0]);
				dxl_write_word(51, 30, currentBallPos[1]);

				ballAtFeet = 0;
				ballFrames=0;

			}else if(noGoal>5 && noGoal<20){
				headSearch();
				forward=0;
			}
			else if (noGoal>30){
				dxl_write_word(50, 30, currentBallPos[0]);
				dxl_write_word(51, 30, currentBallPos[1]);
				noGoal=0;
				ballAtFeet=0;
			}
			break;
		}

		/*
		 * End of Camera Code
		 *
		 */

	}

	return 0;
}


int headSearch(void){
	switch (switchX) {
	case 0:
		HeadPos[0] -= 25;
		break;
	case 1:
		HeadPos[0] += 25;
		break;
	}

	if(ballAtFeet==0){
		switch (switchY) {
		case 0:
			HeadPos[1] -= 45;
			break;
		case 1:
			HeadPos[1] += 45;
			break;
		}
	}
	dxl_write_word(50, 30, HeadPos[0]);
	dxl_write_word(51, 30, HeadPos[1]);

	 // Extreme checks for head position

	if (HeadPos[1] > maxHeadY-50) switchY = 0;
	else if (HeadPos[1] < minHead)switchY = 1;

	if (HeadPos[0] > MaxHeadX-50)switchX = 0;
	else if (HeadPos[0] < 100)switchX = 1;
}

/*******************************************************************************
 * Function Name  : captureRegions
 * Description    : Capture Regions
 * Input          : Havimo region structure
 * Output         : Havimo region structure
 * Return         : Boolean
 *******************************************************************************/

int captureRegions(HaViMoRegion *regions) {

	//TxDString(" captureRegion");
	//TxDByte_PC('\r');
	//TxDByte_PC('\n');
	unsigned char buff[100];

	while (!getDataFromCamera("\xFF\xFF\x64\x02\x01\x98", 6, buff, 6, 1));

	int j;
	for (j = 1; j < 16; j++) {
		char s[] = "\xFF\xFF\x64\x04\x02\x00\x10\x98";
		s[5] = j * 16;
		if (!getDataFromCamera(s, 8, buff, 22, 30)) {
			TxDString("no Region data");
			TxDByte_PC('\r');
			TxDByte_PC('\n');

			return 0;
		}

		memcpy((unsigned char*) (regions + j), buff + 5, 16);
	}

	getDataFromCamera("\xFF\xFF\x64\x02\x0E\x8B", 7, buff, 0, 300);

	//TxDString("Reading Region success");
	//TxDByte_PC('\r');
	//TxDByte_PC('\n');

	return 1;
}

int getDataFromCamera(const char* cmd, int length, unsigned char* readBuf,
		int numBytesToRead, int timeOut) {

	dxl_hal_clear();

	unsigned char s[100];
	memcpy(s, cmd, 100);

	int num_FF_s = 0;
	unsigned char checkSum = 0;
	int i;
	for (i = 0; i < length - 1; i++) {
		if (num_FF_s >= 2) {
			checkSum += s[i];
		}
		if (s[i] == 0xff)
			num_FF_s++;
	}
	s[length - 1] = ~checkSum;

	dxl_hal_set_timeout(timeOut);
	dxl_hal_tx(s, length);

	int readPtr = 0;
	do {
		readPtr += dxl_hal_rx(readBuf + readPtr, numBytesToRead);

	} while (readPtr < numBytesToRead && !dxl_hal_timeout());
	if (readPtr < numBytesToRead)
		return 0;

	return 1;

}

int findBall(void) {
	captureRegions(regions);

	int j;
	int MaxBallCandidate = 0;
	byte fieldMinX=255, fieldMaxX=0, fieldMaxY=0, fieldMinY=255;

	//if(currentBallPos[1]>325){
	for( j=1; j<16; j++){
		if(regions[j].Index > 0 && regions[j].Color ==fieldColor && regions[j].NumPix>200){
			//			maxField = regions[j].NumPix;
			fieldMinX = (regions[j].MinX<fieldMinX)?regions[j].MinX:fieldMinX;
			fieldMinY = (regions[j].MinY<fieldMinY)?regions[j].MinY:fieldMinY;
			fieldMaxX = (regions[j].MaxX>fieldMaxX)?regions[j].MaxX:fieldMaxX;
			fieldMaxY = (regions[j].MaxY>fieldMaxY)?regions[j].MaxY:fieldMaxY;

		}
	}
	for (j = 1; j < 16; j++) {

		if(regions[j].Index > 0 && regions[j].Color ==ballColor ) {

			if(fieldMinX < regions[j].MinX && fieldMaxX > regions[j].MaxX && fieldMinY < regions[j].MinY && fieldMaxY > regions[j].MaxY){
				if (regions[j].NumPix > MaxBallCandidate) {
					MaxBallCandidate = regions[j].NumPix;
					BallX = regions[j].SumX / regions[j].NumPix; //Average X
					//					BallY = regions[j].SumY / regions[j].NumPix; // Average Y
					BallY = regions[j].MaxY;
				}
			}
		}
	}

	if (MaxBallCandidate > 0) {
		noBall = 0;
		return 1;
	}

	noBall += 1;

	return 0;

}


int findGoal(void) {
	captureRegions(regions);
	int j ;
	//noGoal =0;
	int Goal=50;
	GoalX=0;
	GoalY=0;

	//	byte minX=100, maxX=0;
	for (j = 1; j < 16; j++) {
		if(regions[j].Index > 0 && regions[j].Color==goalColor) {
			if(Goal<regions[j].NumPix){
				Goal=regions[j].NumPix;
				GoalX = regions[j].SumX / regions[j].NumPix; //Average X
				GoalY = regions[j].SumY / regions[j].NumPix; // Average Y
				//			minX=(regions[j].MinX<minX)?regions[j].MinX:minX;
				//			maxX=(regions[j].MaxX>maxX)?regions[j].MaxX:maxX;
			}
		}

	}
	if (Goal>51){
		noGoal=0;
		return 1;
	}

	noGoal++;
	return 0;
}

/*******************************************************************************
 * Function Name  : RCC_Configuration
 * Description    : Configures the different system clocks.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void RCC_Configuration(void) {
	ErrorStatus HSEStartUpStatus;
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if (HSEStartUpStatus == SUCCESS) {
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		/* Flash 2 wait state */
		FLASH_SetLatency(FLASH_Latency_2);

		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);

		/* PCLK2 = HCLK */
		RCC_PCLK2Config(RCC_HCLK_Div1);

		/* PCLK1 = HCLK/2 */
		RCC_PCLK1Config(RCC_HCLK_Div2);

		/* PLLCLK = 8MHz * 9 = 72 MHz */
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

		/* Enable PLL */
		RCC_PLLCmd(ENABLE);

		/* Wait till PLL is ready */
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {
		}

		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* Wait till PLL is used as system clock source */
		while (RCC_GetSYSCLKSource() != 0x08) {
		}
	}

	/* Enable peripheral clocks --------------------------------------------------*/

	/* Enable USART1 and GPIOB clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOB,
			ENABLE);

	/* Enable USART3+5 clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5 | RCC_APB1Periph_USART3 | RCC_APB1Periph_TIM2, ENABLE);

	PWR_BackupAccessCmd(ENABLE);
}

/*******************************************************************************
 * Function Name  : NVIC_Configuration
 * Description    : Configures Vector Table base location.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void NVIC_Configuration(void) {
	NVIC_InitTypeDef NVIC_InitStructure;

#ifdef  VECT_TAB_RAM
	// Set the Vector Table base location at 0x20000000
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  // VECT_TAB_FLASH
	// Set the Vector Table base location at 0x08003000
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x3000);
#endif

	// Configure the NVIC Preemption Priority Bits
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	// Enable the USART1 Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Enable the TIM2 Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Enable the UART5 Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
 * Function Name  : GPIO_Configuration
 * Description    : Configures the different GPIO ports.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void GPIO_Configuration(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);

	// PORTB CONFIG
	GPIO_InitStructure.GPIO_Pin = PIN_ENABLE_TXD | PIN_ENABLE_RXD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_DXL_RXD | PIN_PC_RXD;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_DXL_TXD | PIN_PC_TXD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

	GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD); // TX Disable
	GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD); // RX Enable

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// PORTA CONFIG
//	GPIO_InitStructure.GPIO_Pin = 	PIN_ZIGBEE_RESET;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//	GPIO_InitStructure.GPIO_Pin =  PIN_ZIGBEE_TXD;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);
//
//	// PORTD CONFIG
//	GPIO_InitStructure.GPIO_Pin = PIN_ZIGBEE_RXD;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(GPIOD, &GPIO_InitStructure);


}

void USART1_Configuration(u32 baudrate) {
	USART_Configuration(USART_DXL, baudrate);
}

void USART_Configuration(u8 PORT, u32 baudrate) {

	USART_InitTypeDef USART_InitStructure;

	USART_StructInit(&USART_InitStructure);

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	if (PORT == USART_DXL) {
		USART_DeInit(USART1);
		mDelay(10);
		/* Configure the USART1 */
		USART_Init(USART1, &USART_InitStructure);

		/* Enable USART1 Receive and Transmit interrupts */
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		//USART_ITConfig(USART1, USART_IT_TC, ENABLE);

		/* Enable the USART1 */
		USART_Cmd(USART1, ENABLE);
	}

	else if (PORT == USART_PC) {
		USART_DeInit(USART3);
		mDelay(10);
		/* Configure the USART3 */
		USART_Init(USART3, &USART_InitStructure);

		/* Enable USART3 Receive and Transmit interrupts */
		//USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
		//USART_ITConfig(USART3, USART_IT_TC, ENABLE);
		/* Enable the USART3 */
		USART_Cmd(USART3, ENABLE);
	}
	else if (PORT == USART_ZIGBEE) {
		USART_DeInit(UART5);
		mDelay(10);
		/* Configure the UART5 */
		USART_Init(UART5, &USART_InitStructure);

		/* Enable UART5 Receive and Transmit interrupts */
		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);

		/* Enable the UART5 */
		USART_Cmd(UART5, ENABLE);
	}
}

void DisableUSART1(void) {
	USART_Cmd(USART1, DISABLE);
}

void ClearBuffer256(void) {
	gbRxBufferReadPointer = gbRxBufferWritePointer = 0;
}

byte CheckNewArrive(void) {
	if (gbRxBufferReadPointer != gbRxBufferWritePointer)
		return 1;
	else
		return 0;
}

void TxDByte_DXL(byte bTxdData) {
	GPIO_ResetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD); // RX Disable
	GPIO_SetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD); // TX Enable

	USART_SendData(USART1, bTxdData);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
		;

	GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD); // TX Disable
	GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD); // RX Enable
}

byte RxDByte_DXL(void) {
	byte bTemp;

	while (1) {
		if (gbRxBufferReadPointer != gbRxBufferWritePointer)
			break;
	}

	bTemp = gbpRxInterruptBuffer[gbRxBufferReadPointer];
	gbRxBufferReadPointer++;

	return bTemp;
}

void TxDString(byte *bData) {
	while (*bData)
		TxDByte_PC(*bData++);
}

void TxDInt(int wSentData){
	if (wSentData<0) {
		TxDByte_PC('-');
		wSentData = -wSentData;
	} else if (wSentData==0){
		TxDByte_PC('0');
	}
	char s[10];
	int ptr=0;
	while (wSentData != 0){
		s[ptr++]=(wSentData % 10) + '0';
		wSentData/=10;
	}
	int i;
	for (i=0; i<ptr; i++){
		TxDByte_PC(s[ptr-1-i]);
	}
}

void TxDWord16(word wSentData) {
	TxDByte16((wSentData >> 8) & 0xff);
	TxDByte16(wSentData & 0xff);
}

void TxDByte16(byte bSentData) {
	byte bTmp;

	bTmp = ((byte) (bSentData >> 4) & 0x0f) + (byte) '0';
	if (bTmp > '9')
		bTmp += 7;
	TxDByte_PC(bTmp);
	bTmp = (byte) (bSentData & 0x0f) + (byte) '0';
	if (bTmp > '9')
		bTmp += 7;
	TxDByte_PC(bTmp);
}

void TxDByte_PC(byte bTxdData) {
	USART_SendData(USART3, bTxdData);
	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
		;
}

void Timer_Configuration(void) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_OCStructInit(&TIM_OCInitStructure);

	TIM_DeInit(TIM2);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* Prescaler configuration */
	TIM_PrescalerConfig(TIM2, 722, TIM_PSCReloadMode_Immediate);

	/* Output Compare Timing Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;

	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);

	/* TIM IT enable */
	TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);

	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);
}

void TimerInterrupt_1ms(void) //OLLO CONTROL
{
	if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET) // 1ms//
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

		capture = TIM_GetCapture1(TIM2);
		TIM_SetCompare1(TIM2, capture + CCR1_Val);

		if (gw1msCounter > 0)
			gw1msCounter--;
	}
}

/*__interrupt*/
void RxD0Interrupt(void) {
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
		gbpRxInterruptBuffer[gbRxBufferWritePointer++] = USART_ReceiveData(
				USART1);
}

void SysTick_Configuration(void) {
	/* SysTick end of count event each 1ms with input clock equal to 9MHz (HCLK/8, default) */
	SysTick_SetReload(9000);

	/* Enable SysTick interrupt */
	SysTick_ITConfig(ENABLE);
}

void __ISR_DELAY(void) {
	if (gwTimingDelay != 0x00)
		gwTimingDelay--;
	millisec++;
}

void mDelay(u32 nTime) {
	/* Enable the SysTick Counter */
	//	SysTick_CounterCmd(SysTick_Counter_Enable);

	gwTimingDelay = nTime;

	while (gwTimingDelay != 0)
		;

	//	/* Disable SysTick Counter */
	//	SysTick_CounterCmd(SysTick_Counter_Disable);
	//	/* Clear SysTick Counter */
	//	SysTick_CounterCmd(SysTick_Counter_Clear);
}

void SetRunTime(u32 nTime){
	gwTimingDelay = nTime;
}
int RunTiming(void) {
	if(gwTimingDelay ==0)
		return 0;

	return 1;
}

void StartDiscount(s32 StartTime) {
	gw1msCounter = StartTime;
}

u8 CheckTimeOut(void) {
	// Check timeout
	// Return: 0 is false, 1 is true(timeout occurred)

	if (gw1msCounter == 0)
		return 1;
	else
		return 0;
}

void EnableZigbee(void)
{
	USART_Configuration(USART_ZIGBEE, 115200);
	GPIO_ResetBits(PORT_ZIGBEE_RESET, PIN_ZIGBEE_RESET);


}

void RxD2Interrupt(void){

	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
	{
		if (GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_14)==RESET) GPIO_SetBits(GPIOB,GPIO_Pin_14); else GPIO_ResetBits(GPIOB,GPIO_Pin_14);

		word temp;
		temp = USART_ReceiveData(UART5);

		switch(UM6_recieve_state){
		case 0:
			if(temp == 's') UM6_recieve_state = 1;
			break;
		case 1:
			if(temp =='n') UM6_recieve_state =2;
			else UM6_recieve_state =0;
			break;
		case 2:
			if(temp =='p') UM6_recieve_state =3;
			else UM6_recieve_state =0;
			break;
		case 3:
			UM6_recieve_state=4;
			break;
		case 4:
			if(temp ==0x62) UM6_recieve_state =5;
			else UM6_recieve_state =0;
			break;
		case 5:
			UM6_recieve_state++;
			UM6_Roll_Recieve= (temp << 8);
			break;
		case 6:
			UM6_recieve_state++;
			UM6_Roll_Recieve |= temp;
			UM6_Roll = 0.0109863 * (s16)UM6_Roll_Recieve;
			break;
		case 7:
			UM6_recieve_state++;
			UM6_Pitch_Recieve = (temp << 8);
			break;
		case 8:
			UM6_recieve_state++;
			UM6_Pitch_Recieve |= temp;
			UM6_Pitch = 0.0109863 * (s16)UM6_Pitch_Recieve;
			break;
		case 9:
			UM6_recieve_state++;
			UM6_Yaw_Recieve = (temp << 8);
			break;
		case 10:
			UM6_recieve_state = 0;
			UM6_Yaw_Recieve |= temp;
			UM6_Yaw = 0.0109863 * (s16)UM6_Yaw_Recieve;
			break;
		default:
			UM6_recieve_state =0;
			break;
		}


	}
}

