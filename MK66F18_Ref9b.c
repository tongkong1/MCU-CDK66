 /*
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    MK66F18_Ref2a.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include <string.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK66F18.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"
#include "fsl_uart.h"
#include "fsl_rtc.h"
#include "CDK66_OLED.h"
#include "pin_mux.h"

/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */
#define clip(x, xmin, xmax) (((x)<(xmin))?(xmin):(((x)>(xmax))?(xmax):(x)))

#define SW1() 			GPIO_PinRead(BOARD_INITPINS_SW1_GPIO, BOARD_INITPINS_SW1_PIN)
#define SW2() 			GPIO_PinRead(BOARD_INITPINS_SW2_GPIO, BOARD_INITPINS_SW2_PIN)
#define SW3() 			GPIO_PinRead(BOARD_INITPINS_SW3_GPIO, BOARD_INITPINS_SW3_PIN)
#define SW4() 			GPIO_PinRead(BOARD_INITPINS_SW4_GPIO, BOARD_INITPINS_SW4_PIN)
#define KEY1() 			!GPIO_PinRead(BOARD_INITPINS_KEY1_GPIO, BOARD_INITPINS_KEY1_PIN)
#define KEY2() 			!GPIO_PinRead(BOARD_INITPINS_KEY2_GPIO, BOARD_INITPINS_KEY2_PIN)
#define QESA() 			GPIO_PinRead(BOARD_INITPINS_QEPA_GPIO, BOARD_INITPINS_QEPA_PIN)
#define QESB() 			GPIO_PinRead(BOARD_INITPINS_QEPB_GPIO, BOARD_INITPINS_QEPB_PIN)
#define BTN() 			!GPIO_PinRead(BOARD_INITPINS_BTN_GPIO, BOARD_INITPINS_BTN_PIN)

#define LED1_ON() 		GPIO_PortClear(BOARD_INITPINS_LED1_GPIO, BOARD_INITPINS_LED1_GPIO_PIN_MASK)
#define LED1_OFF() 		GPIO_PortSet(BOARD_INITPINS_LED1_GPIO, BOARD_INITPINS_LED1_GPIO_PIN_MASK)
#define LED1_TOGGLE() 	GPIO_PortToggle(BOARD_INITPINS_LED1_GPIO, BOARD_INITPINS_LED1_GPIO_PIN_MASK)
#define LED2_ON() 		GPIO_PortClear(BOARD_INITPINS_LED2_GPIO, BOARD_INITPINS_LED2_GPIO_PIN_MASK)
#define LED2_OFF() 		GPIO_PortSet(BOARD_INITPINS_LED2_GPIO, BOARD_INITPINS_LED2_GPIO_PIN_MASK)
#define LED2_TOGGLE() 	GPIO_PortToggle(BOARD_INITPINS_LED2_GPIO, BOARD_INITPINS_LED2_GPIO_PIN_MASK)
#define BEEP_ON() 		GPIO_PortSet(BOARD_INITPINS_BEEP_GPIO, BOARD_INITPINS_BEEP_GPIO_PIN_MASK)
#define BEEP_OFF() 		GPIO_PortClear(BOARD_INITPINS_BEEP_GPIO, BOARD_INITPINS_BEEP_GPIO_PIN_MASK)
#define BEEP_TOGGLE() 	GPIO_PortToggle(BOARD_INITPINS_BEEP_GPIO, BOARD_INITPINS_BEEP_GPIO_PIN_MASK)
#define M1_ENABLE() 	GPIO_PortSet(BOARD_INITPINS_MLEN_GPIO, BOARD_INITPINS_MLEN_GPIO_PIN_MASK)
#define M1_DISABLE() 	GPIO_PortClear(BOARD_INITPINS_MLEN_GPIO, BOARD_INITPINS_MLEN_GPIO_PIN_MASK)
#define M2_ENABLE() 	GPIO_PortSet(BOARD_INITPINS_MREN_GPIO, BOARD_INITPINS_MREN_GPIO_PIN_MASK)
#define M2_DISABLE() 	GPIO_PortClear(BOARD_INITPINS_MREN_GPIO, BOARD_INITPINS_MREN_GPIO_PIN_MASK)

#define CCD_SI_HIGH		GPIO_PinWrite(BOARD_INITPINS_CCDSI_GPIO, BOARD_INITPINS_CCDSI_PIN, 1)
#define CCD_SI_LOW		GPIO_PinWrite(BOARD_INITPINS_CCDSI_GPIO, BOARD_INITPINS_CCDSI_PIN, 0)
#define CCD_CLK_HIGH	GPIO_PinWrite(BOARD_INITPINS_CCDCLK_GPIO, BOARD_INITPINS_CCDCLK_PIN, 1)
#define CCD_CLK_LOW		GPIO_PinWrite(BOARD_INITPINS_CCDCLK_GPIO, BOARD_INITPINS_CCDCLK_PIN, 0)

#define I2C_CAT9555_BASEADDR I2C3
#define I2C_CAT9555_SLAVE_ADDR_7BIT 0x22U
#define I2C_CAT9555_DATA_LENGTH 2U
#define USB_RING_BUFFER_SIZE	(128U)
#define cmdDataLenMax			(100U)

typedef struct _Board_Analog
{
	 uint32_t	x;			// MMA7260 x
	 uint32_t	y;			// MMA7260 y
	 uint32_t	z;			// MMA7260 z
	 uint32_t	IRV1;		// IR sensor #1
	 uint32_t	IRV2;		// IR sensor #2
	 uint32_t	HALL1;		// Hall sensor #1
	 uint32_t	HALL2;		// Hall sensor #2
	 uint32_t	VSENSE;		// Battery Voltage
	 uint32_t	Temp;		// On-chip temperature sensor
	 uint32_t	CCD;		// TSL1401 Linear CCD
} Board_Analog_t;

// Variables for HMI
static uint8_t DSPTable[] = { 0x40, 0x79, 0x24, 0x30, 0x19, 0x12, 0x02, 0x78, 0x00, 0x10, 0x08, 0x03, 0x27, 0x21, 0x06, 0x0E };
uint16_t QESVar=0;

// Variables for I2C3
uint8_t g_master_txBuff[I2C_CAT9555_DATA_LENGTH];
uint8_t g_master_rxBuff[I2C_CAT9555_DATA_LENGTH];
i2c_master_transfer_t masterXfer;

// Variable for RTC
rtc_datetime_t appDateTime;
uint32_t ElapsedSeconds = 0;

// Variables for ADCs
Board_Analog_t AnalogIn;

// Variables for UART
bool CmdDataReady;		// new cmd data flag, 1: new cmd received
uint8_t CmdDataLen;		// PC cmd data length counter
uint8_t CmdDataBuffer[cmdDataLenMax];	// buffer to hold cmd data from PC
uint8_t UsbRingBuffer[USB_RING_BUFFER_SIZE];
uint16_t g_bufferWriteByteCnt;
volatile uint16_t txIndexUSB=0;		// Index of the debug port data to send out.
volatile uint16_t rxIndexUSB=0;		// Index of the memory to save new arrived debug port data.

// Variables for MMA7260
uint16_t MemsX, MemsY, MemsZ;
float MemsX1, MemsY1, MemsZ1, VBat;

// Variables for Servo & Motors
bool servo_update_flag = true;
uint16_t ServoDC1, ServoDC2;
uint16_t ServoDC1ref = 1510;
uint16_t ServoDC2ref = 1510;
uint16_t ServoLimitUpper = 1960;	// Adjust for a better value on your car
uint16_t ServoLimitLower = 1060;	// Adjust for a better value on your car
uint8_t MotorSpeedCmd1=0, MotorSpeedCmd2=0;
uint32_t MotorSpeedTarget=0;		// 用于实现电机pid控制的编码器值参考量
uint16_t MotorSpeedCal=0;			// debug purpose
uint32_t MotorSpeedFdbk1=0, MotorSpeedFdbk2=0;
uint32_t MotorSpeedRef = 330;
bool MotorDir1, MotorDir2;
uint32_t EnterCurveSpeed = 0;
// Variables for Linear CCD TSL1401
uint16_t CCDData[128];	// Linear CCD raw data
uint16_t TrackIndex;	// Target tracking point index from 1 to 128

// Variables for MT9V034
uint8_t Image_Data[120][188];	// MT9V034 DMA data receiver ROW*COL: 120*188
uint8_t Image_Use[60][94 ];	// resized CAM data to fit OLED
uint8_t Pixle[60][94];		// 0/1 CAM data
uint16_t CAM_DMA_Counter=0;
uint8_t CamLineCounter=0;
bool CamDataReady=0;	// MT9V034 new data frame ready in buffer

// Variables for autorun
uint16_t TrackErrorCounter=0;	// Invalid detection counter for off-track protection
uint8_t TrackFlag=0;	// 0: Invalid track detection
						// 1: Valid/Normal track found
						// 2: Start/End Mark detected
						// 3: Cross-road detected
						// 4: Undefined
uint8_t TrackOffsetCounter = 0;		// 巡线偏差过大值计数器，用于判断前方为弯道/直道
uint8_t BlackCounter = 0;		//黑点计数器，主要用于判别十字路
uint8_t EdgeCounter = 0;		//边沿计数器，主要用于判别断续线
uint8_t LapCounter = 0;
uint8_t Exposure_Time = 4;

// Variables for pid
typedef struct  PID_Parameter   //PID参数
{
    float input;
    float error;
    float lastError;
    float kp;
    float ki;
    float kd;
    float integral;
    float derivative;
    float proportionMax;
    float integralMax;
    float derivativeMax;
    float output;
    float outputMax;
} PID_Parameter;

PID_Parameter speed_Parameter =     //速度闭环控制  //TODO:电机驱动是否更适合增量式PID？
{
    .integral = 0,   //初始化，勿改动
    .lastError = 0,  //初始化，勿改动
	.error = 0,		//初始化，勿改动
    .kp = 0.75,
    .ki = 0,
    .kd = 2.5,
    .proportionMax = 60,
    .integralMax = 60,
    .derivativeMax = 60,
    .outputMax = 60
};
PID_Parameter angle_Parameter =     //速度闭环控制  //TODO:电机驱动是否更适合增量式PID？
{
    .integral = 0,   //初始化，勿改动
    .lastError = 0,  //初始化，勿改动
	.error = 0,		//初始化，勿改动
    .kp = 7.5,
    .ki = 0,
    .kd = 40,
    .proportionMax = 2500,
    .integralMax = 0,
    .derivativeMax = 2500,
    .outputMax = 2500
};
PID_Parameter* motor_PIDInfo = &speed_Parameter;
PID_Parameter* servo_PIDInfo = &angle_Parameter;


// Variables for Debug
uint8_t Second_Cnt = 0;	//秒计时器，用于调试
uint8_t Button_Cnt = 0;	//按键按下数，用于制作参数调整
uint8_t PIT_20ms_status = 0;	//20ms状态，用于蜂鸣器鸣叫
uint8_t PIT_500ms_status = 0;	//100ms状态
uint8_t PIT_500ms_counter = 0;  //20ms*5=100ms，用于断续线元素定时保护
uint8_t BEEP_inteval_counter = 15;	//20ms*15=300ms


void DMA_CAM_Repeat(uint8_t LineNumber)
{
	DMA_DMA_BASEADDR->TCD[DMA_CH0_DMA_CHANNEL].DADDR         = (uint32_t) &Image_Data[LineNumber][0];
	DMA_DMA_BASEADDR->TCD[DMA_CH0_DMA_CHANNEL].CITER_ELINKNO = DMA_CITER_ELINKNO_CITER(188);
	DMA_DMA_BASEADDR->TCD[DMA_CH0_DMA_CHANNEL].BITER_ELINKNO = DMA_BITER_ELINKNO_BITER(188);
	//EDMA_EnableChannelRequest(DMA_DMA_BASEADDR, DMA_CH0_DMA_CHANNEL);
	DMA_DMA_BASEADDR->SERQ = DMA_SERQ_SERQ(DMA_CH0_DMA_CHANNEL);
}

/* DMA0_DMA16_IRQn interrupt handler */
void DMA_DMA_CH_INT_DONE_0_IRQHANDLER(void) {
  uint32_t CH0_status;

  /* Reading all flags of status register */
  CH0_status = EDMA_GetChannelStatusFlags(DMA_DMA_BASEADDR, DMA_CH0_DMA_CHANNEL);
  /* Clearing all flags of the channel status register */
  EDMA_ClearChannelStatusFlags(DMA_DMA_BASEADDR, DMA_CH0_DMA_CHANNEL, CH0_status);

  /* Place your code here */
  //CamDataReady = 1;
  //DMA_CAM_Repeat();

  /*
  CAM_DMA_Counter++;
  if ((CAM_DMA_Counter%50)==0)
	  LED2_TOGGLE();
	*/

  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}

float PIDCalc(float target, PID_Parameter* PIDInfo)
{
    float componentKp, componentKi, componentKd;
    PIDInfo->error = target - PIDInfo->input;

    PIDInfo->integral += PIDInfo->error;
    PIDInfo->derivative = PIDInfo->error - PIDInfo->lastError;
    //限值，注意单独限制integral
    if(PIDInfo->ki != 0)    //防止除数为0
        PIDInfo->integral = clip(PIDInfo->integral, -PIDInfo->integralMax/PIDInfo->ki, PIDInfo->integralMax/PIDInfo->ki);
    componentKi = PIDInfo->integral * PIDInfo->ki;

    componentKp = clip(PIDInfo->error * PIDInfo->kp, -PIDInfo->proportionMax, PIDInfo->proportionMax);
    componentKd = clip(PIDInfo->derivative * PIDInfo->kd, -PIDInfo->derivativeMax, PIDInfo->derivativeMax);

    PIDInfo->output = clip(componentKp + componentKi + componentKd, -PIDInfo->outputMax, PIDInfo->outputMax);
    // PIDInfo->output = clip(componentKp + componentKi + componentKd, 0, PIDInfo->outputMax);

    PIDInfo->lastError =  PIDInfo->error;  //更新误差

    return PIDInfo->output;
}

void ShowNumHEX(uint8_t num) {
	uint16_t l;
	l = DSPTable[num & 0x0F];

	// DSP: PD9~PD15
	GPIO_PortSet(BOARD_INITPINS_SEG_A_GPIO, 0x7F << BOARD_INITPINS_SEG_A_PIN);	// turn off all DSP segments
	GPIO_PortClear(BOARD_INITPINS_SEG_A_GPIO, ((~l)&0x7F) << BOARD_INITPINS_SEG_A_PIN);	// turn on the code segments
}

void ShowNumDEC(uint8_t num) {
	uint16_t l;
	l = DSPTable[num % 10];

	// DSP: PD9~PD15
	GPIO_PortSet(BOARD_INITPINS_SEG_A_GPIO, 0x7F << BOARD_INITPINS_SEG_A_PIN);	// turn off all DSP segments
	GPIO_PortClear(BOARD_INITPINS_SEG_A_GPIO, ((~l)&0x7F) << BOARD_INITPINS_SEG_A_PIN);	// turn on the code segments
}

void ShowNumOFF(void) {
	// DSP: PD9~PD15
	GPIO_PortSet(BOARD_INITPINS_SEG_A_GPIO, 0x7F << BOARD_INITPINS_SEG_A_PIN);	// turn off all DSP segments
}

/* PORTB_IRQn interrupt handler */
void GPIOB_IRQHANDLER(void) {
  /* Get pin flags */
  uint32_t pin_flags = GPIO_PortGetInterruptFlags(GPIOB);

  /* Place your interrupt code here */
  // Rotary Encoder Switch
  if (pin_flags & BOARD_INITPINS_QEPA_GPIO_PIN_MASK)
  {
	  delay();
  	  GPIO_PortClearInterruptFlags(GPIOB, BOARD_INITPINS_QEPA_GPIO_PIN_MASK);
	  if (!GPIO_PinRead(BOARD_INITPINS_QEPB_GPIO, BOARD_INITPINS_QEPB_PIN))
	  {
		  QESVar++;
	  }
	  else
	  {
		  if (QESVar>0) QESVar--;
	  }
  }
  // HREF on PTB11
  if (pin_flags & BOARD_INITPINS_CAMHREF_GPIO_PIN_MASK)
  {
	  GPIO_PortClearInterruptFlags(GPIOB, BOARD_INITPINS_CAMHREF_GPIO_PIN_MASK);
	  // Start DMA transfer on new line
	  DMA_CAM_Repeat(CamLineCounter);
	  CamLineCounter++;
	  if (CamLineCounter >= 120)
	  {
		  CamLineCounter = 0;
	  }
	  return;
  }
  // VSYN on PTB16
  if (pin_flags & BOARD_INITPINS_CAMVSYN_GPIO_PIN_MASK)
  {
	  GPIO_PortClearInterruptFlags(GPIOB, BOARD_INITPINS_CAMVSYN_GPIO_PIN_MASK);
	  //CAM_DMA_Counter = EDMA_GetRemainingMajorLoopCount(DMA_DMA_BASEADDR, DMA_CH0_DMA_CHANNEL);
	  //DMA_CAM_Repeat();
	  //DMALeftSize = EDMA_GetRemainingMajorLoopCount(DMA_DMA_BASEADDR, DMA_CH0_DMA_CHANNEL);
	  CamDataReady = 1;
	  CamLineCounter = 0;
	  //DMA_CAM_Repeat();
	  CAM_DMA_Counter++;
	  if ((CAM_DMA_Counter%50)==0)
		  LED2_TOGGLE();
	  return;
  }

  /* Clear pin flags */
  GPIO_PortClearInterruptFlags(GPIOB, pin_flags);

  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}

/* PC command string parsing for RTC synchronizing */
void PcCmdProcess(void) {
    int year;	/*!< Range from 1970 to 2099.*/
    int month;	/*!< Range from 1 to 12.*/
    int day;	/*!< Range from 1 to 31 (depending on month).*/
    int hour;	/*!< Range from 0 to 23.*/
    int minute;	/*!< Range from 0 to 59.*/
    int second;	/*!< Range from 0 to 59.*/
    rtc_datetime_t PCTIME;


	if (CmdDataLen == 27U)	// "#cmd01data20230405-180000\r\n"
	{
		PRINTF("Command from PC received!\r\n");
		sscanf(CmdDataBuffer, "#cmd01data%4d%2d%2d-%2d%2d%2d",
				&year, &month, &day, &hour, &minute, &second);
		PRINTF("RTC: %04hd-%02hd-%02hd %02hd:%02hd:%02hd\r\n",
				year, month, day, hour, minute, second);

	    /* Set DATE TIME registers and start RTC */
		PCTIME.year   = year;
		PCTIME.month  = month;
		PCTIME.day    = day;
		PCTIME.hour   = hour;
		PCTIME.minute = minute;
		PCTIME.second = second;
	    RTC_StopTimer(RTC);
	    RTC_SetDatetime(RTC, &PCTIME);
	    RTC_StartTimer(RTC);
	}
	else
	{
		PRINTF("Invalid RTC synchronizing command!\r\n");
	}
}

/* UART0_RX_TX_IRQn interrupt handler */
void UART0_SERIAL_RX_TX_IRQHANDLER(void) {
	uint8_t data;
	uint32_t intStatus;

	/* Reading all interrupt flags of status registers */
	intStatus = UART_GetStatusFlags(UART0_PERIPHERAL);

	/* Place your code here */
	if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & intStatus)	// If new data arrived?
	{
		data = UART_ReadByte(UART0_PERIPHERAL);
		//UART_WriteByte(UART0_PERIPHERAL, data);	// send data back for loop test

		/* PC command data framing */
		/* RTC calibrate example: "#cmd01data20230405-180000\r\n" */
		if (data == '#')		// start of a data frame?
		{
			CmdDataLen = 0;		// reset cmd data counter
			CmdDataBuffer[0] = data;
		}
		else
		{
			CmdDataLen++;
			if (CmdDataLen < cmdDataLenMax)
			{
				CmdDataBuffer[CmdDataLen] = data;
				if (data == '\n')	// end of a data frame?
				{
					CmdDataReady = 1;	// a complete PC cmd data frame received
					CmdDataLen++;
					PcCmdProcess();		// cmd processing
				}
			}
			else	// no cmd data frame received when data buffer overflow occur
			{
				CmdDataLen = 0;
			}
		}

		/* If ring buffer is not full, add data to ring buffer. */
		if (((rxIndexUSB + 1) % USB_RING_BUFFER_SIZE) != txIndexUSB)
		{
			UsbRingBuffer[rxIndexUSB] = data;
			rxIndexUSB++;
			rxIndexUSB %= USB_RING_BUFFER_SIZE;
		}
	}

	UART_ClearStatusFlags(UART0_PERIPHERAL, intStatus);
	/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
	Store immediate overlapping exception return operation might vector to incorrect interrupt. */
	#if defined __CORTEX_M && (__CORTEX_M == 4U)
	__DSB();
	#endif
}

/* RTC_Seconds_IRQn interrupt handler */
void RTC_SECONDS_IRQHANDLER(void) {
  /* Get status flags */
  uint32_t status_flags = RTC_GetStatusFlags(RTC_PERIPHERAL);

  /* Place your interrupt code here */
  ElapsedSeconds++;
  if( Second_Cnt < 2 )	Second_Cnt++;

  /* Clear status flags */
  RTC_ClearStatusFlags(RTC_PERIPHERAL, status_flags);

  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}

void delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 800000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

void delay_short(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 20000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

uint8_t InverseByteBits(uint8_t data)
{
	uint8_t tempChar;

	tempChar = data;
	tempChar=(tempChar<<4)|(tempChar>>4);
	tempChar=((tempChar<<2)&0xcc)|((tempChar>>2)&0x33);
	tempChar=((tempChar<<1)&0xaa)|((tempChar>>1)&0x55);

	return tempChar;
}

void BOARD_InitI2C(void) {
    uint8_t deviceAddress;

    /* CAT9555 communication protocal */
	// Polarity
    g_master_txBuff[0] = 0x0U;
    g_master_txBuff[1] = 0x0U;
    deviceAddress     = 0x04U;
    masterXfer.slaveAddress   = I2C_CAT9555_SLAVE_ADDR_7BIT;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = (uint32_t)deviceAddress;
    masterXfer.subaddressSize = 1;
    masterXfer.data           = g_master_txBuff;
    masterXfer.dataSize       = I2C_CAT9555_DATA_LENGTH;
    masterXfer.flags          = kI2C_TransferDefaultFlag;
    I2C_MasterTransferBlocking(I2C_CAT9555_BASEADDR, &masterXfer);
    delay();

    // IO Configuration
    g_master_txBuff[0] = 0U;
    g_master_txBuff[1] = 0U;
    deviceAddress     = 0x06U;
    masterXfer.subaddress     = (uint32_t)deviceAddress;
    masterXfer.subaddressSize = 1;
    masterXfer.data           = g_master_txBuff;
    masterXfer.dataSize       = I2C_CAT9555_DATA_LENGTH;
    I2C_MasterTransferBlocking(I2C_CAT9555_BASEADDR, &masterXfer);
    delay();
}

void BOARD_I2C_GPIO(uint16_t ctrlValue) {
    uint8_t deviceAddress;

    g_master_txBuff[0] = ~(ctrlValue&0xFFU);
    g_master_txBuff[1] = ~(InverseByteBits((ctrlValue>>8)&0xFFU));
    deviceAddress     = 0x02U;
    masterXfer.slaveAddress   = I2C_CAT9555_SLAVE_ADDR_7BIT;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = (uint32_t)deviceAddress;
    masterXfer.subaddressSize = 1;
    masterXfer.data           = g_master_txBuff;
    masterXfer.dataSize       = I2C_CAT9555_DATA_LENGTH;
    masterXfer.flags          = kI2C_TransferDefaultFlag;
    I2C_MasterTransferBlocking(I2C_CAT9555_BASEADDR, &masterXfer);
}

void CDK66_Analog_Input(Board_Analog_t *AnalogInput) {
    //adc16_channel_config_t adc16ChannelConfigStruct;
    adc16_channel_mux_mode_t ADCMUX0, ADCMUX1;

    ADCMUX0 = kADC16_ChannelMuxA;
    ADCMUX1 = kADC16_ChannelMuxA;
    ADC16_SetChannelMuxMode(ADC0, ADCMUX0);
    ADC16_SetChannelMuxMode(ADC1, ADCMUX1);

    // ADC0
    // MEMSx: ADC0, se19
    ADC16_SetChannelConfig(ADC0_PERIPHERAL, ADC0_CH1_CONTROL_GROUP, &ADC0_channelsConfig[1]);
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(ADC0_PERIPHERAL, ADC0_CH1_CONTROL_GROUP))) {}
    AnalogInput->x = ADC16_GetChannelConversionValue(ADC0_PERIPHERAL, ADC0_CH0_CONTROL_GROUP) & 0xFFF;
    // MEMSy: ADC0, se3
    ADC16_SetChannelConfig(ADC0_PERIPHERAL, ADC0_CH0_CONTROL_GROUP, &ADC0_channelsConfig[0]);
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(ADC0_PERIPHERAL, ADC0_CH0_CONTROL_GROUP))) {}
    AnalogInput->y = ADC16_GetChannelConversionValue(ADC0_PERIPHERAL, ADC0_CH0_CONTROL_GROUP) & 0xFFF;
    // HALL2: ADC0, se17
    ADC16_SetChannelConfig(ADC0_PERIPHERAL, ADC0_CH2_CONTROL_GROUP, &ADC0_channelsConfig[2]);
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(ADC0_PERIPHERAL, ADC0_CH2_CONTROL_GROUP))) {}
    AnalogInput->HALL2 = ADC16_GetChannelConversionValue(ADC0_PERIPHERAL, ADC0_CH2_CONTROL_GROUP) & 0xFFF;
    // IRV2: ADC0, se23
    ADC16_SetChannelConfig(ADC0_PERIPHERAL, ADC0_CH3_CONTROL_GROUP, &ADC0_channelsConfig[3]);
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(ADC0_PERIPHERAL, ADC0_CH3_CONTROL_GROUP))) {}
    AnalogInput->IRV2 = ADC16_GetChannelConversionValue(ADC0_PERIPHERAL, ADC0_CH3_CONTROL_GROUP) & 0xFFF;
    // CCD: ADC0, se14
    ADC16_SetChannelConfig(ADC0_PERIPHERAL, ADC0_CH4_CONTROL_GROUP, &ADC0_channelsConfig[4]);
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(ADC0_PERIPHERAL, ADC0_CH4_CONTROL_GROUP))) {}
    AnalogInput->CCD = ADC16_GetChannelConversionValue(ADC0_PERIPHERAL, ADC0_CH4_CONTROL_GROUP) & 0xFFF;

    // ADC1
    // MEMSz: ADC1, se19
    ADC16_SetChannelConfig(ADC1_PERIPHERAL, ADC1_CH0_CONTROL_GROUP, &ADC1_channelsConfig[0]);
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(ADC1_PERIPHERAL, ADC1_CH0_CONTROL_GROUP))) {}
    AnalogInput->z = ADC16_GetChannelConversionValue(ADC1_PERIPHERAL, ADC1_CH0_CONTROL_GROUP) & 0xFFF;
    // IRV1: ADC1, se18
    ADC16_SetChannelConfig(ADC1_PERIPHERAL, ADC1_CH1_CONTROL_GROUP, &ADC1_channelsConfig[1]);
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(ADC1_PERIPHERAL, ADC1_CH1_CONTROL_GROUP))) {}
    AnalogInput->IRV1 = ADC16_GetChannelConversionValue(ADC1_PERIPHERAL, ADC1_CH1_CONTROL_GROUP) & 0xFFF;
    // HALL1: ADC1, se23
    ADC16_SetChannelConfig(ADC1_PERIPHERAL, ADC1_CH2_CONTROL_GROUP, &ADC1_channelsConfig[2]);
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(ADC1_PERIPHERAL, ADC1_CH2_CONTROL_GROUP))) {}
    AnalogInput->HALL1 = ADC16_GetChannelConversionValue(ADC1_PERIPHERAL, ADC1_CH2_CONTROL_GROUP) & 0xFFF;
    // Temperature: ADC1, se26
    ADC16_SetChannelConfig(ADC1_PERIPHERAL, ADC1_CH3_CONTROL_GROUP, &ADC1_channelsConfig[3]);
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(ADC1_PERIPHERAL, ADC1_CH3_CONTROL_GROUP))) {}
    AnalogInput->Temp = ADC16_GetChannelConversionValue(ADC1_PERIPHERAL, ADC1_CH3_CONTROL_GROUP) & 0xFFF;

    return;
}

/* PIT0_IRQn interrupt handler */
void PIT_CHANNEL_0_IRQHANDLER(void) {
  uint32_t intStatus;
  /* Reading all interrupt flags of status register */
  intStatus = PIT_GetStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_0);
  PIT_ClearStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_0, intStatus);

  /* Place your code here */
  FTM_SetSoftwareTrigger(FTM3_PERIPHERAL, true);
  //servo_update_flag = true;

  if( PIT_20ms_status == 0 )
  {
		PIT_20ms_status = 1;
  }
  else 	PIT_20ms_status = 0;

  if( PIT_500ms_counter == 25 )
  {
		PIT_500ms_status = 1;
  }
  else
  {
		PIT_500ms_status = 0;
		PIT_500ms_counter++;
  }

  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}

// open-loop speed control
void MotorCtrlOL(uint8_t SpeedCmd)
{
	if(SpeedCmd > 60)	//速度限制
		SpeedCmd = 60;
	if(SpeedCmd < 0)
		SpeedCmd = 0;

	FTM_UpdatePwmDutycycle(FTM0_PERIPHERAL, kFTM_Chnl_3, kFTM_EdgeAlignedPwm, SpeedCmd);
	FTM_SetSoftwareTrigger(FTM0_PERIPHERAL, true);
}

// Closed-loop speed control
// feedback available on MotorSpeedFdbk1
void MotorCtrlCL(void)
{
	uint16_t CtrlValue=0;

	CtrlValue = MotorSpeedCal/10;		// MotorSpeedCal updated in PIT IRQ at an interval of 5ms
	if (CtrlValue > 60)					//限制CtrlValue,即限制最大速度
		CtrlValue = 60;

	//FTM_UpdatePwmDutycycle(FTM0_PERIPHERAL, kFTM_Chnl_3, kFTM_EdgeAlignedPwm, SpeedCmd);
	FTM_UpdatePwmDutycycle(FTM0_PERIPHERAL, kFTM_Chnl_3, kFTM_EdgeAlignedPwm, CtrlValue);
	FTM_SetSoftwareTrigger(FTM0_PERIPHERAL, true);

}

// appTicks increasing at an interval of 5ms
void PIT_CHANNEL_1_IRQHANDLER(void)
{
	uint32_t intStatus;
	/* Reading all interrupt flags of status register */
	intStatus = PIT_GetStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_1);
	PIT_ClearStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_1, intStatus);

	/* Place your code here */
    /* Read Encoder#1 pulses value */
    MotorSpeedFdbk1 = FTM_GetQuadDecoderCounterValue(FTM1_PERIPHERAL);		//获得编码器feedback
    /* Clear Encoder#1 pulses counter */
    FTM_ClearQuadDecoderCounterValue(FTM1_PERIPHERAL);		//清除编码器脉冲数寄存器
    /* Read Motor#1 running direction */
    if (FTM_GetQuadDecoderFlags(FTM1_PERIPHERAL) & kFTM_QuadDecoderCountingIncreaseFlag)	//根据编码器方向确定电机运动方向【不确定底层如何实现】
    {
        MotorDir1 = 1;		//【TODO:测试这个是正转还是反转】
    }
    else if (MotorSpeedFdbk1 != 0)		//如果是反转且保证不是静止状态
    {
        MotorDir1 = 0;
        MotorSpeedFdbk1 = 5000 - MotorSpeedFdbk1;		//编码器反馈量取反
    }
    motor_PIDInfo->input = MotorSpeedFdbk1;
  	MotorCtrlOL( (uint8_t)PIDCalc( MotorSpeedTarget, motor_PIDInfo ) );		//5ms更新电机数值

  		//老师原本的闭环控制
	//用MotorSpeedFdbk1/20作为反馈，更新MotorSpeedCal
    /*if (MotorSpeedFdbk1 < MotorSpeedCmd1*20)		//实际速度小于设定速度
    {
    	MotorSpeedCal++;
    }
    else						//实际速度大于或等于预期速度
    {
    	if (MotorSpeedCal>0)	//保证MotorSpeedCal>0
    		MotorSpeedCal--;
    }*/


    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
       Store immediate overlapping exception return operation might vector to incorrect interrupt. */
    #if defined __CORTEX_M && (__CORTEX_M == 4U)
      __DSB();
    #endif
}

/*ServoMotor controller*/
void static inline Update_ServoUS(uint8_t ChanNo, uint32_t DutyCycleUS)
{
	uint32_t cv, mod;

	if (DutyCycleUS > ServoLimitUpper)
		DutyCycleUS = ServoLimitUpper;
	if (DutyCycleUS < ServoLimitLower)
		DutyCycleUS = ServoLimitLower;
	//if ((ChanNo<2) && (DutyCycleUS<=2500) && (DutyCycleUS>=500))
	if (ChanNo < 2)
	{
		mod = FTM3_PERIPHERAL->MOD;
		cv = (mod*DutyCycleUS) / 20000U;
		if (cv >= mod)
		{
			cv = mod + 1U;
		}
		FTM3_PERIPHERAL->CONTROLS[ChanNo].CnV = cv;
		//FTM_SetSoftwareTrigger(FTM3_PERIPHERAL, true);
	}
	return;
}

// Count value calibrated with 1 second timing to be 142
void delay1ms(uint16_t delayTime) {
	uint32_t delayCount;
	delayCount = delayTime * USEC_TO_COUNT(142, CLOCK_GetFreq(SYS_CLK));
	//delayCount = delayTime * USEC_TO_COUNT(100, CLOCK_GetFreq(kCLOCK_BusClk));

	while(delayCount--) {}
}

void CCD_delay()
{
	uint16_t delayCount;
	delayCount = 20;
	while(delayCount--) {}
}

void CCD_IntegratingDelay(uint16_t delayCount)
{
	if (delayCount==0) return;
	while(delayCount--) {}
}


/* Drive to and read from TSL1401CL */
void CollectCCD(void)
{
    uint8_t index=0;

    CCD_CLK_HIGH;
    CCD_delay();
    CCD_SI_LOW;
    CCD_delay();

    CCD_SI_HIGH;
    CCD_delay();
    CCD_CLK_LOW;
    CCD_delay();

    CCD_CLK_HIGH;
    CCD_delay();
    CCD_SI_LOW;
    CCD_delay();

    for(index=0; index<128; index++)
    {
        CCD_CLK_LOW;
        CCD_delay();

        ADC16_SetChannelConfig(ADC0_PERIPHERAL, ADC0_CH4_CONTROL_GROUP, &ADC0_channelsConfig[4]);
        while (0U == (kADC16_ChannelConversionDoneFlag &
                      ADC16_GetChannelStatusFlags(ADC0_PERIPHERAL, ADC0_CH4_CONTROL_GROUP))) {}
        CCDData[index] = ADC16_GetChannelConversionValue(ADC0_PERIPHERAL, ADC0_CH4_CONTROL_GROUP) & 0xFFF;

        CCD_CLK_HIGH;
        CCD_delay();
    }
}

// simply generate SI & CLK pulses to flush the previously integrated frame
// while integrating new frame to be read out
void LinearCameraFlush(void)
{
    uint8_t index=0;

    CCD_SI_HIGH;
    CCD_delay();
    CCD_CLK_HIGH;	// 1st Pulse
    CCD_delay();
    CCD_SI_LOW;
    CCD_delay();
    CCD_CLK_LOW;

    for(index=0; index<128; index++)
    {
        CCD_CLK_HIGH;
        //CCD_delay();
        CCD_IntegratingDelay(150);
        CCD_CLK_LOW;
        //CCD_delay();
        CCD_IntegratingDelay(150);
    }
}

void LinearCameraOneShot(void)
{
    uint8_t index=0;

    // flush previously integrated frame before capturing new frame
    LinearCameraFlush();
    // wait for TSL1401 to integrate new frame, exposure time control by delay
    delay1ms(Exposure_Time);

    CCD_SI_HIGH;
    CCD_delay();
    CCD_CLK_HIGH;
    CCD_delay();
    CCD_SI_LOW;
    CCD_delay();
    CCD_CLK_LOW;

    // ADC mux to CCD channel
    ADC16_SetChannelMuxMode(ADC0, kADC16_ChannelMuxA);
    for(index=0; index<128; index++)
    {
    	// read AO value using ADC
        ADC16_SetChannelConfig(ADC0_PERIPHERAL, ADC0_CH4_CONTROL_GROUP, &ADC0_channelsConfig[4]);
        while (0U == (kADC16_ChannelConversionDoneFlag &
                      ADC16_GetChannelStatusFlags(ADC0_PERIPHERAL, ADC0_CH4_CONTROL_GROUP))) {}
        CCDData[index] = 2*ADC16_GetChannelConversionValue(ADC0_PERIPHERAL, ADC0_CH4_CONTROL_GROUP) & 0xFFF;
        // clock pulse to read next pixel output
        CCD_CLK_HIGH;
        CCD_delay();
        CCD_CLK_LOW;
    }

    // 129th pulse to terminate output of 128th pixel
    CCD_CLK_HIGH;
    CCD_delay();
    CCD_CLK_LOW;
}

void Draw_LinearView(void)
{
    uint8_t index=0, tempData;
    uint8_t i=0, j=0, k=0;

    for(index=0; index<128; index++)
	{
    	i = 7 - (CCDData[index]>>6)/8;
    	j = 7- (CCDData[index]>>6)%8;
    	for (k=0; k<8; k++)
    	{
    		OLED_Set_Pos(index, k);
    		if (i==k)
    		{
    			tempData = (1U << j);
    		}
    		else
    		{
    			tempData = 0;
    		}
    		OLED_WrDat(tempData);
    	}
	}
}

void Draw_LinearViewHalf(void)
{
    uint8_t index=0, tempData;
    uint8_t i=0, j=0, k=0;

    for(index=0; index<128; index++)
	{
    	i = 7 - (CCDData[index]>>10);
    	j = 7- (CCDData[index]>>7)%8;
    	for (k=4; k<8; k++)
    	{
    		OLED_Set_Pos(index, k);
    		if (i==k)
    		{
    			tempData = (1U << j);
    		}
    		else
    		{
    			tempData = 0;
    		}
    		OLED_WrDat(tempData);
    	}
	}
}

void KC4D(void)
{
    OLED_KC4D(16,2);
    BEEP_ON();
    delay_short();
    BEEP_OFF();
    delay();
    BEEP_ON();
    delay();
    BEEP_OFF();
    delay();
    delay();
}

void Get_Use_Image()
{
	uint16_t i=0, j=0, row=0, col=0;

	for(i=0; i<120; i+=2)		// ROW
	{
		for(j=0; j<188; j+=2)	// COL
		{
			Image_Use[row][col] = Image_Data[i][j];
			col++;
		}
		col = 0;
		row++;
	}
}

uint8_t GetOTSU(uint8_t image[60][94])
{
	uint16_t i, j;
	uint32_t Amount = 0;
	uint32_t PixelBack = 0;
	uint32_t PixelIntegralBack = 0;
	uint32_t PixelIntegral = 0;
	uint32_t PixelIntegralFore = 0;
	uint32_t PixelFore = 0;
	float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
	uint16_t MinValue, MaxValue;
	uint8_t Threshold = 0;
	uint8_t HistoGram[256];

	for (j=0; j<256; j++)  HistoGram[j] = 0; //初始化灰度直方图

	for (j=0; j<60; j++)
	{
		for (i = 0; i < 94; i++)
		{
			HistoGram[image[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
		}
	}

	for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
	for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值

	if (MaxValue == MinValue)     return MaxValue;         // 图像中只有一个颜色
	if (MinValue + 1 == MaxValue)  return MinValue;        // 图像中只有二个颜色

	for (j = MinValue; j <= MaxValue; j++)    Amount += HistoGram[j];        //  像素总数

	PixelIntegral = 0;
	for (j = MinValue; j <= MaxValue; j++)
	{
		PixelIntegral += HistoGram[j] * j;//灰度值总数
	}
	SigmaB = -1;
	for (j = MinValue; j < MaxValue; j++)
	{
		PixelBack = PixelBack + HistoGram[j];   //前景像素点数
		PixelFore = Amount - PixelBack;         //背景像素点数
		OmegaBack = (float)PixelBack / Amount;//前景像素百分比
		OmegaFore = (float)PixelFore / Amount;//背景像素百分比
		PixelIntegralBack += HistoGram[j] * j;  //前景灰度值
		PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
		MicroBack = (float)PixelIntegralBack / PixelBack;   //前景灰度百分比
		MicroFore = (float)PixelIntegralFore / PixelFore;   //背景灰度百分比
		Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//计算类间方差
		if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
		{
			SigmaB = Sigma;
			Threshold = j;
		}
	}
	return Threshold;                        //返回最佳阈值;

}

// threshold mode: 0 - OTSU; 1 - average
void Get_01_Value(uint8_t mode)
{
	uint16_t i=0, j=0;
	uint8_t Threshold;
	uint32_t tv=0;
	char txt[16];

	if (mode)
	{
		for (i=0; i<60; i++)
		{
			for (j=0; j<94; j++)
				{
					tv += Image_Use[i][j];   //累加
				}
		}
		Threshold = tv/60/94;        	//求平均值,光线越暗越小，全黑约35，对着屏幕约160，一般情况下大约100
		Threshold = Threshold*8/10+10;	//此处阈值设置，根据环境的光线来设定
	}
	else
	{
		Threshold = GetOTSU(Image_Use);	//大津法阈值
		Threshold = (uint8_t)(Threshold*0.5)+70;
	}
	sprintf(txt,"%03d", Threshold);
	OLED_P6x8Str(80, 1, (uint8_t *)txt);

	for (i=0; i<60; i++)
	{
		for(j=0; j<94; j++)
		{
			if(Image_Use[i][j] > Threshold) //数值越大，显示的内容越多，较浅的图像也能显示出来
				Pixle[i][j] = 1;
			else
				Pixle[i][j] = 0;
		}
	}
}

void Draw_CameraView()
{
	uint8_t i=0, j=0, temp=0;

	for(i=8;i<56;i+=8)//6*8=48行
	{
		OLED_Set_Pos(18, i/8+1);	//起始位置
		for(j=0; j<94; j++)  	//列数
		{
			temp = 0;
			if(Pixle[0+i][j]) temp |= 1;
			if(Pixle[1+i][j]) temp |= 2;
			if(Pixle[2+i][j]) temp |= 4;
			if(Pixle[3+i][j]) temp |= 8;
			if(Pixle[4+i][j]) temp |= 0x10;
			if(Pixle[5+i][j]) temp |= 0x20;
			if(Pixle[6+i][j]) temp |= 0x40;
			if(Pixle[7+i][j]) temp |= 0x80;
			OLED_WrDat(temp);
		}
	}
}

void SendImage(void)
{
	uint8_t PcImageHeader[4];

	PcImageHeader[0] = 0x00;
	PcImageHeader[1] = 0xFF;
	PcImageHeader[2] = 0x01;
	PcImageHeader[3] = 0x01;

	//发送帧头标志
	UART_WriteBlocking(UART0_PERIPHERAL, (uint8_t *)PcImageHeader, 4U);
	//发送灰度图像
	UART_WriteBlocking(UART0_PERIPHERAL, (uint8_t *)Image_Data, 22560U);
}

/*
// open-loop speed control
void MotorCtrlOL(uint8_t SpeedCmd)
{
	if(SpeedCmd>100)	//速度限制
		return;

	FTM_UpdatePwmDutycycle(FTM0_PERIPHERAL, kFTM_Chnl_3, kFTM_EdgeAlignedPwm, SpeedCmd);
	FTM_SetSoftwareTrigger(FTM0_PERIPHERAL, true);
}

// Closed-loop speed control
// feedback available on MotorSpeedFdbk1
void MotorCtrlCL(void)
{
	uint16_t CtrlValue=0;

	CtrlValue = MotorSpeedCal/10;		// MotorSpeedCal updated in PIT IRQ at an interval of 5ms
	if (CtrlValue > 60)					//限制CtrlValue,即限制最大速度
		CtrlValue = 60;

	//FTM_UpdatePwmDutycycle(FTM0_PERIPHERAL, kFTM_Chnl_3, kFTM_EdgeAlignedPwm, SpeedCmd);
	FTM_UpdatePwmDutycycle(FTM0_PERIPHERAL, kFTM_Chnl_3, kFTM_EdgeAlignedPwm, CtrlValue);
	FTM_SetSoftwareTrigger(FTM0_PERIPHERAL, true);

}
*/

// Your code here to find the target tracking point
// Result: update TrackIndex value
// uint8_t TrackFlag=0;	// 0: Invalid track detection
//						// 1: Valid/Normal track found
//						// 2: Start/End Mark detected
//						// 3: Cross-road detected
//						// 4: Undefined
void FindTrackPoint(void)
{
	int16_t DMax, DMin, DV;		//记录像素灰度值从左边到右边的落差
	uint8_t MaxIndex, MinIndex, MinPointIndex;
	uint16_t MinPoint = 4096;
	uint16_t i;

	//寻找最大落差DMax和最低灰度值DMin
	DMax = 0;
	DMin = 0;
	//【For Debug Sake】 全局计数器清零
	BlackCounter = 0;
	EdgeCounter = 0;

	//【For Debug Sake】现在暂且设置为全局变量
	//uint8_t BlackCounter = 0;		//黑点计数器，主要用于判别十字路
	//uint8_t EdgeCounter = 0;		//边沿计数器，主要用于判别断续线

	for (i=0; i<126; i++)		//【TODO:这里是取用0~125是否会有影响】
	{
		//跳变沿检测
		DV = CCDData[i+2] - CCDData[i];		//DV为像素灰度值从左边到右边的落差【!!注意!! 中间滤去一个像素点】
		if (DV > DMax)		// Raising edge：灰度值从左向右爬升
		{
			DMax = DV;
			MaxIndex = i+1;
		}
		if (DV < DMin)		// Falling edge：灰度值从左向右下落
		{
			DMin = DV;
			MinIndex = i+1;
		}

		//最小值
		if( i > 3 && i < 124 )		//滤除边界点影响，对过弯中线处于边界可能有些影响
		{
			if( CCDData[i] <= MinPoint )
			{
				MinPoint = CCDData[i];
				MinPointIndex = i;
			}
		}

		//黑点计数
		if( CCDData[i] < 300 )
		{
			BlackCounter++;
		}

		//跳变沿计数
		if( abs(DV) > 100 )   //阈值可以适当调大
		{
			EdgeCounter++;
		}
	}

	// 特殊元素处理
	if( EdgeCounter >= 7 && BlackCounter > 22 )		//如果跳变沿超过某个阈值，识别为断续线
	{
		TrackIndex = 64;
		TrackFlag = 2;
		return;
	}

	if( BlackCounter > 50 )		//如果黑点个数超过阈值，识别为十字路
	{
		TrackIndex = 64;
		TrackFlag = 3;
		return;
	}

	// 中线识别前预处理,阳光滤波
	for (i=0; i<128; i++)
	{

		if( CCDData[i] > MinPoint + 200 )
		{
			CCDData[i] = MinPoint + 200;
		}

		/*
		if( CCDData[i] > MinPoint + 250 )
		{
			CCDData[i] = MinPoint + 250;
		}
		*/
	}

	// 中线识别
	DMax = DMin = 0;	// 重新初始化
	for (i=0; i<126; i++)
	{
		//跳变沿检测
		DV = CCDData[i+2] - CCDData[i];		//DV为像素灰度值从左边到右边的落差【!!注意!! 中间滤去一个像素点】
		if (DV > DMax)		// Raising edge：灰度值从左向右爬升
		{
			DMax = DV;
			MaxIndex = i+1;
		}
		if (DV < DMin)		// Falling edge：灰度值从左向右下落
		{
			DMin = DV;
			MinIndex = i+1;
		}
	}
	// Normal v.s. Invalid
		//if ((MaxIndex<MinIndex) || (DMax<120) || (DMin>(-120)))
	if( MaxIndex<MinIndex )
	{
		TrackFlag = 0;
		TrackErrorCounter++;
		return;
	}
	if ((DMax<70) && (DMin>(-70)))	// Invalid cases 限制上下跳变位置与跳变量【!!注意!! 120为自己定义的参数】
	{
		TrackFlag = 0;
		TrackErrorCounter++;		//错误数计数器，用于丢线保护装置
	}
	else
	{
		/*if ((MaxIndex-MinIndex) > 30)	// One of invalid cases		//限制山谷宽度【不知道适用于什么情况？】
		{
			TrackFlag = 0;
			TrackErrorCounter++;
		}
		else*/

		TrackFlag = 1;		// Normal track found
		if( DMax >= 70 && DMin <= -70)   //正常
		{
			TrackIndex = (MaxIndex+MinIndex)/2;
		}

		if(DMax >= 70 && DMin > -70)
		{
			TrackIndex = 0;
		}

		if(DMax <= 70 && DMin <= -70)
		{
			TrackIndex = 127;
		}

		TrackErrorCounter = 0;		//清除错误数计数器
	}
}

// indicate TrackIndex on OLED
void Draw_Mark(TrackIndex)
{
	uint8_t i=0;
	for (i=4; i<8; i++)
	{
		OLED_Set_Pos(TrackIndex, i);
		OLED_WrDat(0xFF);
	}
}

// 判别前方是否为弯道
bool isCurve(void)
{
	// 当中线出现在偏左或偏右位置时，说明可能有弯道出现，记录TrackOffsetCounter计数器；否则，说明进入直道，直接清零计数
	if( TrackIndex > 85 || TrackIndex < 40 )	//或许需要再测试下
	// if( TrackIndex > 80 || TrackIndex < 45 )
	{
		TrackOffsetCounter++;
	}
	else TrackOffsetCounter = 0;

	// 对TrackOffsetCounter设定阈值来确定直道/弯道
	if( TrackOffsetCounter > 5 )	return true;
	else 	return false;
}

/*
 * @brief   Application entry point.
 */
int main(void) {
 	uint8_t TaskID = 0;		// loop switch
    uint32_t LastSecond=0;
    //uint16_t ServoDutyCounter=0;
    //bool ServoDutyCountDir=1;		// 0: count down; 1: count up
	uint32_t tempInt = 0;
	uint16_t CCDMin = 0;
	uint8_t index = 0;
	bool BeepFlag = 0;	//【For Debug Sake】
	uint16_t LastQESVar = 0;
	bool StartRace = false;
	bool PastCrossroad = false;
	bool PastBeginLine = false;
	bool servoInitialed = false;
	uint8_t BeepCounter = 0;		// 十字线蜂鸣计数

	uint8_t CCD2PC[260];	// data to be sent to PC

	char OLEDLine1[] = "0123456789abcdef";
	char OLEDLine2[] = "0123456789abcdef";
	char OLEDLine3[] = "0123456789abcdef";
	char OLEDLine4[] = "0123456789abcdef";

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    BOARD_InitI2C();
    OLED_Init();

    KC4D();		// Welcome message
    // Header of CCD data frame, seekfree protocol
    CCD2PC[0] = 0x00;
	CCD2PC[1] = 0xFF;
	CCD2PC[2] = 0x01;
	CCD2PC[3] = 0x00;


#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

	TaskID = (~((SW1()<<3) + (SW2()<<2) + (SW3()<<1) + SW4())) & 0x0F;
    PRINTF("Task ID: %d\r\n", TaskID);

    LastSecond = ElapsedSeconds;
    LED1_ON();    LED2_OFF();

    volatile static int i = 0 ;
    //ServoDutyCountDir = 1;	// count up
    //ServoDutyCounter = 500;	// Lower Limit

	if (TaskID == 1U)		// Test RC/Servo
	{
		QESVar = ServoDC1ref;
		ServoDC1 = ServoDC1ref;
		ServoDC2 = ServoDC2ref;
	}
    if (TaskID == 7U)		// using MT9V034?
		MT9V034_Init(50);
	if ((TaskID == 4U) || (TaskID == 9U))		// Test Motor driver?
	{
		M1_DISABLE();
		M2_DISABLE();
		FTM_UpdatePwmDutycycle(FTM0_PERIPHERAL, kFTM_Chnl_2, kFTM_EdgeAlignedPwm, 0);
		FTM_UpdatePwmDutycycle(FTM0_PERIPHERAL, kFTM_Chnl_3, kFTM_EdgeAlignedPwm, 0);
    	FTM_UpdatePwmDutycycle(FTM0_PERIPHERAL, kFTM_Chnl_4, kFTM_EdgeAlignedPwm, 0);
    	FTM_UpdatePwmDutycycle(FTM0_PERIPHERAL, kFTM_Chnl_7, kFTM_EdgeAlignedPwm, 0);
		M1_ENABLE();
	}

    OLED_Init();

    while(1) {
    	switch (TaskID)
    	{
    	case 1U:		// Tune ServoDC1 to find ServoLimitUpper and ServoLimitLower
						// 【TODO:电机控制闭环后】
        	//delay_short();
    		if (BTN())
    		{
    			//QESVar = ServoDC1ref;
				ServoDC1 = ServoDC1ref;
				ServoDC2 = ServoDC1ref;
    		}
    		if (KEY1())
    		{
    			if (ServoDC1 > 530)
    				ServoDC1 -= 5;
    			while (KEY1()) {}
    		}
    		if (KEY2())
    		{
    			if (ServoDC1 <= 2510)
    				ServoDC1 += 5;
    			while (KEY2()) {}
    		}
    		if (tempInt != ServoDC1)
    		{
    			tempInt = ServoDC1;
    			Update_ServoUS(kFTM_Chnl_0, tempInt);
    			Update_ServoUS(kFTM_Chnl_1, tempInt);
                sprintf (OLEDLine2, "PWM: %04hd", tempInt);
				OLED_P8x16Str(0,2,(uint8_t *)OLEDLine2);
				ShowNumDEC( abs(tempInt-ServoDC1ref) );
    		}
    		break;
    	case 2U:		// MEMS data acquisition and processing
            CDK66_Analog_Input(&AnalogIn);		// Update all analog inputs
            MemsX = AnalogIn.x;
            MemsY = AnalogIn.y;
            MemsZ = AnalogIn.z;
            //MemsX1 = ((float)MemsX-2048.0)/101.3234;
            //MemsY1 = ((float)MemsY-2048.0)/101.3234;
            //MemsZ1 = ((float)MemsZ-2048.0)/101.3234;
    		break;
    	case 3U:		// MEMS data acquisition and processing
            CDK66_Analog_Input(&AnalogIn);		// Update all analog inputs
            if (KEY1())
            {
                sprintf (OLEDLine2, "x:%04hd  IR1:%04hd", AnalogIn.x, AnalogIn.IRV1);
                sprintf (OLEDLine3, "y:%04hd  IR2:%04hd", AnalogIn.y, AnalogIn.IRV2);
            }
            else
            {
                sprintf (OLEDLine2, "x:%04hd  Mg1:%04hd", AnalogIn.x, AnalogIn.HALL1);
                sprintf (OLEDLine3, "y:%04hd  Mg2:%04hd", AnalogIn.y, AnalogIn.HALL2);
            }
            sprintf (OLEDLine4, "z:%04hd  ct:%04hd", AnalogIn.z, AnalogIn.Temp);
            //sprintf (OLEDLine4, "z:%04hd  ct:%4.1fC", AnalogIn.z, ICTemp);
            OLED_P8x16Str(0,0,(uint8_t *)"## Cyber-K66 ###");
    	    OLED_P8x16Str(0,2,(uint8_t *)OLEDLine2);
    	    OLED_P8x16Str(0,4,(uint8_t *)OLEDLine3);
    	    OLED_P8x16Str(0,6,(uint8_t *)OLEDLine4);
    		break;
    	case 4U:		// TSL1401 Linear CCD acquisition and display
        	//CollectCCD();
        	LinearCameraOneShot();
        	// draw linear CCD 128 pixel on OLED
    		//Draw_LinearView();
    		Draw_LinearViewHalf();
			FindTrackPoint();

    		if (KEY1())		// 把预处理前/后的像素灰度值传给上位机【很漂亮的设计，只有按下按键时才会进行上位机通讯，不会影响实际调试时程序运行效率】
    		{
    	    	//send 128 points CCD data (128*2byte) to UART0, using seekfree protocol
    			for(tempInt=0; tempInt<128; tempInt++)
    			{
    				CCD2PC[tempInt*2+4] = CCDData[tempInt] >> 8;	// Upper byte
    				CCD2PC[tempInt*2+5] = CCDData[tempInt] & 0XFF;	// Lower byte
    			}
    			UART_WriteBlocking(UART0_PERIPHERAL, CCD2PC, 260U);
    		}
    		// FindTrackPoint();		// your code to find the tracking point
    		Draw_Mark(TrackIndex);	// indicate TrackIndex on OLED

    		// 特殊道路元素控制
    		// For Debug Sake:

    		switch( TrackFlag )
    		{
			case 2: 	// 断续线识别
				BEEP_ON();
				PastBeginLine = true;

				if( PIT_500ms_status == 1 )		// 100ms到了
				{
					LapCounter++;				// 记录已经跑了一圈
					PIT_500ms_counter = 0;		// 重置计时，作为元素保护
				}
				break;

			case 3: 	// 十字路识别
				BEEP_ON();
				PastCrossroad = true;
				break;

			default:
				if( PastBeginLine )	// 断续线蜂鸣器操作
				{
					if( PIT_20ms_status )	// 20ms到了
					{
						if( ++BEEP_inteval_counter >= 10 )
						{
							BEEP_inteval_counter = 0;
							BEEP_OFF();
							PastBeginLine = false;
						}
					}
				}
				if( PastCrossroad )	// 十字线蜂鸣器操作
				{
					if( PIT_20ms_status )	// 20ms到了
					{
						if( ++BEEP_inteval_counter >= 5 )
						{
							BEEP_inteval_counter = 0;
							switch( BeepCounter )
							{
								case 0:
									BEEP_OFF();
									break;
								case 1:
									BEEP_ON();
									break;
								case 2:
									PastCrossroad = false;
									BEEP_OFF();
									break;
								default: 	break;
							}

							BeepCounter = (BeepCounter+1) % 3;
						}
					}
				}
				break;
    		}

 			//预瞄点位置与舵机pwm占空比换算，更新舵机角度
			/*
			if(TrackIndex >= 45 && TrackIndex <= 82)
			{
				tempInt = 1183.5 + TrackIndex*5.141;
			}
			else if(TrackIndex < 45)
			{
				tempInt = 186.4 + TrackIndex*27.299;      //TrackIndex为32时达到极限
			}
			else
			{
				tempInt = -633.4 + TrackIndex*27.299;   //TrackIndex为95时达到极限
			}
			if(tempInt <= ServoLimitLower)   //舵机超限位保护
			{
				tempInt = ServoLimitLower;
			}
			if(tempInt >= ServoLimitUpper)
			{
				tempInt = ServoLimitUpper;
			}   //5.22版本,63.5为中心
			*/

			// PID策略
			servo_PIDInfo->input = 63.5-TrackIndex;
			tempInt = ServoDC1ref + (int32_t)PIDCalc( 0, servo_PIDInfo );
    		Update_ServoUS(kFTM_Chnl_0, (int)(tempInt));
			Update_ServoUS(kFTM_Chnl_1, (int)(tempInt));

			// 策略2：匀速

			if (BTN())
			{
			    MotorSpeedTarget = MotorSpeedRef;
				if( !servoInitialed )
				{
					servo_PIDInfo->error = 0;
					servo_PIDInfo->integral = 0;
					servo_PIDInfo->derivative = 0;
					servoInitialed = true;
				}
			}
			if (KEY1())
	        {
			    if (MotorSpeedRef >= 110)
			        MotorSpeedRef -= 10;
			    while (KEY1()) {}
			}
			if (KEY2())
			{
			    if (MotorSpeedRef <= 500)
			        MotorSpeedRef += 10;
			    while (KEY2()) {}
			}

			if( LapCounter >= 3 )
			{
				MotorSpeedTarget = 0;
				if( MotorSpeedTarget == 0)
				{
					delay();
					BEEP_OFF();
				}
			}

			// 策略1：直道加速，弯道减速
			/*
			if( BTN() )
			{
				StartRace = true;
			}
			if( StartRace )
			{
				if(LapCounter < 3)
				{
					// 根据前方道路是直道还是弯道来调节运行速度
					if( isCurve() )			//如果是弯道，则迅速减速/慢慢减速？
					{
						//BEEP_ON();
						// MotorSpeedTarget = 140;//-= 10
						// if( MotorSpeedTarget >= 150 )	MotorSpeedTarget -= 10;
						// else	MotorSpeedTarget = 140;

						if( MotorSpeedTarget >= 110 )	MotorSpeedTarget -= 10;
						else	MotorSpeedTarget = 100;
						if( MotorSpeedTarget >= 100 )	MotorSpeedTarget -= 30;
						else	MotorSpeedTarget = 70;

						// 下面这段代码是弯道分段
						if( EnterCurveSpeed > 300 )		// 高速入弯
						{
							if( MotorSpeedTarget >= 110 )	MotorSpeedTarget -= 20;
							else	MotorSpeedTarget = 100;
						}
						else	// 低速入弯
						{
							BEEP_ON();		//【For Debug Sake】
							if( MotorSpeedTarget >= 180 )	MotorSpeedTarget -= 10;
							else	MotorSpeedTarget =170;
						}
						// 上面这段代码是弯道分段

					}
					else					//如果是直道，则迅速加速/慢慢加速？
					{
						//BEEP_OFF();
						//MotorSpeedTarget = 300;//-= 10

						// if( MotorSpeedTarget <= 290 )	MotorSpeedTarget += 10;
						// else	MotorSpeedTarget = 300;
						if( MotorSpeedTarget <= 220 )	MotorSpeedTarget += 45;
						else if( MotorSpeedTarget <= 360 ) MotorSpeedTarget += 25;
						else MotorSpeedTarget = 370;
						EnterCurveSpeed = MotorSpeedFdbk1;
					}

				}
				else
				{
					if( MotorSpeedTarget > 100)  MotorSpeedTarget -= 100;
					else 	MotorSpeedTarget = 0;

					if( MotorSpeedTarget == 0)
					{
						delay();
						BEEP_OFF();
					}
				}
			}
			*/

			//显示屏显示项目，便于调试
			sprintf (OLEDLine1, "Tar: %03hd,Ref:%03hd", MotorSpeedTarget, MotorSpeedRef);
            sprintf (OLEDLine2, "Target: %03hd  %01hd", TrackIndex, TrackFlag);
			OLED_P8x16Str(0,0,(uint8_t *)OLEDLine1);
			OLED_P8x16Str(0,2,(uint8_t *)OLEDLine2);

			//如果错误场景出现多次，判定为丢线，进行丢线保护

			if (TrackErrorCounter >= 40)		// adjust the counter limit to meet your expectation
			{
				while(1)
				{
					MotorSpeedTarget = 0;		// 强制停车
					Update_ServoUS(kFTM_Chnl_0, ServoDC1ref);

					BEEP_ON();
					delay_short();
					BEEP_OFF();
					delay_short();
					BEEP_ON();
					delay_short();
					BEEP_OFF();
				}
			}


    		LED1_TOGGLE();
            break;
    	case 7U:		// MT9V034 image capture and display
        	if (CamDataReady)
        	{
        		Get_Use_Image();		// resize data to fit OLED
        		if (KEY1())				// key S1 pressed down? If yes then use OTSU
        			Get_01_Value(1);	// 1 - OTSU; 0 - average
        		else
        			Get_01_Value(0);	// 1 - OTSU; 0 - average
        		Draw_CameraView();		// display on OLED
        		if (KEY2()) SendImage();	// key S2 pressed down? If yes send ImageData to PC
        		CamDataReady = 0;
        	}
    		break;
    	case 8U:		// RTC: "#cmd01data20230405-180000\r\n"
        	if (ElapsedSeconds != LastSecond)	// ElapsedSeconds increased in RTC_Seconds_IRQn
        	{
        		LED1_TOGGLE();	LED2_TOGGLE();
        		LastSecond = ElapsedSeconds;
        		RTC_GetDatetime(RTC, &appDateTime);
    			sprintf (OLEDLine1, "Date: %04hd-%02hd-%02hd", appDateTime.year, appDateTime.month, appDateTime.day);
    			sprintf (OLEDLine2, "Time: %02hd:%02hd:%02hd", appDateTime.hour, appDateTime.minute, appDateTime.second);
    			OLED_P8x16Str(0,0,(uint8_t *)OLEDLine1);
    			OLED_P8x16Str(0,2,(uint8_t *)OLEDLine2);
        	}
    		break;
    	case 9U:		// Test motor drive //【TODO:增加一个pid控制菜单】
    		if (BTN())
    		{
    			MotorSpeedTarget = 300;
				Button_Cnt = (Button_Cnt + 1) % 4;		//【这里修复过了一个bug】
				while (BTN()) {}
    		}
    		if (KEY1())
    		{
    			if (MotorSpeedTarget >= 10)
    				MotorSpeedTarget -= 10;
    			while (KEY1()) {}
    		}
    		if (KEY2())
    		{
    			if (MotorSpeedTarget <= 600)
    				MotorSpeedTarget += 10;
    			while (KEY2()) {}
    		}

    		//PID参数调节面板
    		switch( Button_Cnt )
			{
				case 0:		//不调节参数
					sprintf (OLEDLine4, "Now for nothing");
					break;
				case 1:		//调节Kp
					sprintf (OLEDLine4, "Now for Kp     ");
					break;
				case 2:		//调节Ki
					sprintf (OLEDLine4, "Now for Ki     ");
					break;
				case 3:		//调节Kd
					sprintf (OLEDLine4, "Now for Kd     ");
					break;
				default:
					break;
			}

			//PID参数调节
			if( QESVar != LastQESVar )		//编码器旋转了
			{
				switch( Button_Cnt )
				{
					case 0:		//不调节参数
						//sprintf (OLEDLine4, "Now for nothing");
						break;
					case 1:		//调节Kp
						//sprintf (OLEDLine4, "Now for Kp     ");
						if( QESVar>LastQESVar )		motor_PIDInfo->kp += 0.05;
						else 			motor_PIDInfo->kp -= 0.05;
						break;
					case 2:		//调节Ki
						//sprintf (OLEDLine4, "Now for Ki     ");
						if( QESVar>LastQESVar )		motor_PIDInfo->ki += 0.05;
						else 			motor_PIDInfo->ki -= 0.05;
						break;
					case 3:		//调节Kd
						//sprintf (OLEDLine4, "Now for Kd     ");
						if( QESVar>LastQESVar )		motor_PIDInfo->kd += 0.05;
						else 			motor_PIDInfo->kd -= 0.05;
						break;
					default:
						break;
				}
				LastQESVar = QESVar;
			}

			// OLED菜单显示
    		if (ElapsedSeconds != LastSecond)	// ElapsedSeconds increased in RTC_Seconds_IRQn
    		{
    			LastSecond = ElapsedSeconds;
				sprintf(OLEDLine1, "Target: %03hd", MotorSpeedTarget);
				//sprintf(OLEDLine2, "Feedback: %03hd", MotorSpeedFdbk1);
				sprintf(OLEDLine2, "p: %.2f, i:%.2f", motor_PIDInfo->kp, motor_PIDInfo->ki);
				sprintf(OLEDLine3, "d:%.2f", motor_PIDInfo->kd);
				OLED_P8x16Str(0,0,(uint8_t *)OLEDLine1);
				OLED_P8x16Str(0,2,(uint8_t *)OLEDLine2);
				OLED_P8x16Str(0,4,(uint8_t *)OLEDLine3);
				OLED_P8x16Str(0,6,(uint8_t *)OLEDLine4);
    		}

    		break;
    	case 14U:
    		if (KEY1())
    	    {
    	        if (Exposure_Time >= 1)
    	        {
    	        	Exposure_Time += 1;
    	        }
    			while (KEY1()) {}
    	    }
    		if (KEY2())
    		{
    		    if (Exposure_Time > 1)
    		    {
    		    	Exposure_Time -= 1;
    		    }
    		    while (KEY2()) {}
    		}
    		//CollectCCD();
			LinearCameraOneShot();
			// draw linear CCD 128 pixel on OLED
			//Draw_LinearView();
			Draw_LinearViewHalf();
			FindTrackPoint();

			if (BTN())		// 把预处理前/后的像素灰度值传给上位机【很漂亮的设计，只有按下按键时才会进行上位机通讯，不会影响实际调试时程序运行效率】
			{
				//send 128 points CCD data (128*2byte) to UART0, using seekfree protocol
				for(tempInt=0; tempInt<128; tempInt++)
				{
					CCD2PC[tempInt*2+4] = CCDData[tempInt] >> 8;	// Upper byte
					CCD2PC[tempInt*2+5] = CCDData[tempInt] & 0XFF;	// Lower byte
				}
				UART_WriteBlocking(UART0_PERIPHERAL, CCD2PC, 260U);
			}
			//FindTrackPoint();		// your code to find the tracking point
			Draw_Mark(TrackIndex);	// indicate TrackIndex on OLED

			sprintf(OLEDLine2, "ET: %03hd", Exposure_Time);
    	    OLED_P8x16Str(0,2,(uint8_t *)OLEDLine2);

    	case 15U:
			if( Second_Cnt == 1 && !BeepFlag )	//没响过
			{
				BEEP_ON();
				Second_Cnt = 0;	//开始计时
				BeepFlag = 1;
			}
			if( Second_Cnt == 1 && BeepFlag )	//响过且过了2s（大约）
			{
				BEEP_OFF();
			}

			break;

    	default:		// HMI basics
    		i++;
    		delay();
            if (KEY1()) LED1_ON();
            else LED1_OFF();
            if (KEY2()) LED2_ON();
            else LED2_OFF();
    		if (BTN()) BEEP_ON();
            else BEEP_OFF();
	        ShowNumDEC(i);
	        BOARD_I2C_GPIO(i);
    		break;
    	}
    }
    return 0 ;
}
