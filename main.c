/*
 * VGA Generator example program using the STM32F4 Discovery board
 * available from STMicroelectronics
 *
 * Author: Harris Shallcross
 * Year: 21/06/2015
 *
 *Interfacing an VGA with timers and SPI
 *
 *Code and example descriptions can be found on my blog at:
 *www.hsel.co.uk
 *
 *The MIT License (MIT)
 *Copyright (c) 2015 Harris Shallcross
 *
 *Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 */

#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_dma.h>
#include <misc.h>
#include "GFX.h"
#include "ImO.h"

//VGA Pin and timing definitions
#define V_HSYNC		GPIO_Pin_10
#define V_HSPS		GPIO_PinSource10
#define V_HSAF		GPIO_AF_TIM1
#define V_HSTIM		TIM1
#define V_HSGPIO	GPIOA

#define V_VSYNC		GPIO_Pin_0
#define V_VSPS		GPIO_PinSource0
#define V_VSAF		GPIO_AF_TIM2
#define V_VSTIM		TIM2
#define V_VSGPIO	GPIOA

#define V_DAT		GPIO_Pin_7
#define V_DPS		GPIO_PinSource7
#define V_DAF		GPIO_AF_SPI1
#define V_DGPIO		GPIOA
#define V_SPI		SPI1

//VGA timing in pixel clocks
#define V_HFP		40
#define V_HSP		128
#define V_HBP		88
#define V_VFP		1
#define V_VSP		4
#define V_VBP		23

#define GBufS		((XPix*YPix)/16)

//16 pixels per word
volatile uint16_t GBuf[GBufS] = {0};


//Typedefs
TIM_TimeBaseInitTypeDef TB;
TIM_OCInitTypeDef TO;
NVIC_InitTypeDef N;
GPIO_InitTypeDef G;
SPI_InitTypeDef S;
DMA_InitTypeDef D;

//Graphic buffer pointer and position counter
volatile uint32_t *GBufPos = GBuf, GBC = 0;

//HSync interrupt - this is fired after the back porch has finished, ready
//to initialize the DMA and stream out the SPI data
void TIM1_CC_IRQHandler(void){
	static uint8_t A = 0;

	if(TIM_GetITStatus(V_HSTIM, TIM_IT_CC2) == SET){
		TIM_ClearITPendingBit(V_HSTIM, TIM_IT_CC2);
		//Make sure we're in the visible region of the screen! Ensure
		//vertical counter is more than sync pulse and back porch
		if(TIM_GetCounter(V_VSTIM)>V_VSP+V_VBP){
			V_DGPIO->MODER &= ~GPIO_MODER_MODER7;
			V_DGPIO->MODER |= GPIO_MODER_MODER7_1;
			SPI_ITConfig(V_SPI, SPI_I2S_IT_RXNE, DISABLE);

			//Disable the DMA stream
			DMA_Cmd(DMA2_Stream3, DISABLE);

			//Rewrite the graphic buffer position address
			D.DMA_Memory0BaseAddr = GBufPos;

			//Initialize DMA stream
			DMA_Init(DMA2_Stream3, &D);

			//Enable DMA stream
			DMA_Cmd(DMA2_Stream3, ENABLE);
		}
	}
	//Set SPI to output and set low
	else if(TIM_GetITStatus(V_HSTIM, TIM_IT_CC4) == SET){
		TIM_ClearITPendingBit(V_HSTIM, TIM_IT_CC4);

		V_DGPIO->MODER &= ~GPIO_MODER_MODER7;
		V_DGPIO->MODER |= GPIO_MODER_MODER7_0;
	}
}

//End of VSync Sync pulse - Back porch from now to next
//frame
void TIM2_IRQHandler(void){
	//Check for CC1 interrupt bit
	if(TIM_GetITStatus(V_VSTIM, TIM_IT_CC1) == SET){
		//If set, clear the interrupt but
		TIM_ClearITPendingBit(V_VSTIM, TIM_IT_CC1);
		//Set the graphic buffer pointer to start of graphic
		//buffer
		GBufPos = GBuf;

		//Reset the graphic buffer position counter
		GBC = 0;
	}
}

//Advance graphic buffer pointer
void DMA2_Stream3_IRQHandler(void){
	//Check if DMA interrupt is set
	if(DMA2->LISR & (1<<27)){
		//Clear DMA interrupt bit
		DMA2->LIFCR |= (uint32_t) (1<<27);

		//Advance graphic buffer counter
		GBC += XPix/16;

		//Reset graphic buffer position pointer
		GBufPos = &GBuf[GBC];
	}
}

//Link to ImO.h image file
const uint16_t Image[];

int main(void)
{
	//Enable clocks
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	//Configure HSync output
	G.GPIO_Pin = V_HSYNC;
	G.GPIO_Mode = GPIO_Mode_AF;
	G.GPIO_OType = GPIO_OType_PP;
	G.GPIO_PuPd = GPIO_PuPd_NOPULL;
	G.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(V_HSGPIO, &G);

	//Configure VSync output
	G.GPIO_Pin = V_VSYNC;
	G.GPIO_Mode = GPIO_Mode_AF;
	G.GPIO_OType = GPIO_OType_PP;
	G.GPIO_PuPd = GPIO_PuPd_NOPULL;
	G.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(V_VSGPIO, &G);

	//Configure SPI data output
	G.GPIO_Pin = V_DAT;
	G.GPIO_Mode = GPIO_Mode_AF;
	G.GPIO_OType = GPIO_OType_PP;
	G.GPIO_PuPd = GPIO_PuPd_NOPULL;
	G.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(V_DGPIO, &G);
	GPIO_ResetBits(V_DGPIO, V_DAT);

	//Assign all outputs to alternate functions
	GPIO_PinAFConfig(V_HSGPIO, V_HSPS, V_HSAF);
	GPIO_PinAFConfig(V_VSGPIO, V_VSPS, V_VSAF);
	GPIO_PinAFConfig(V_DGPIO, V_DPS, V_DAF);

	//Initialize HSync timer
	TB.TIM_ClockDivision = TIM_CKD_DIV1;
	TB.TIM_CounterMode = TIM_CounterMode_Up;
	TB.TIM_Period = XPix + V_HFP + V_HSP + V_HBP;
	TB.TIM_Prescaler = 3;
	TB.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(V_HSTIM, &TB);

	//Initialize HSync pulse output. HSync pulse is negative from
	//zero to V_HSP.
	TO.TIM_OCIdleState = TIM_OCIdleState_Set;
	TO.TIM_OCMode = TIM_OCMode_PWM2;
	TO.TIM_OCPolarity = TIM_OCPolarity_High;
	TO.TIM_OutputState = TIM_OutputState_Enable;
	TO.TIM_Pulse = V_HSP;
	TIM_OC3Init(V_HSTIM, &TO);

	//Allow preloads and enable output
	TIM_OC3PreloadConfig(V_HSTIM, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(V_HSTIM, ENABLE);
	TIM_CtrlPWMOutputs(V_HSTIM, ENABLE);

	//Start of frame - aim for 2.2us, this is the back porch!
	//TIM_SetCompare2(V_HSTIM, V_HFP+V_HSP-53); //Adjust for correct timing!
	TIM_SetCompare2(V_HSTIM, V_HFP+V_HSP-30); //Adjust for correct timing!
	TIM_CCxCmd(V_HSTIM, TIM_Channel_2, TIM_CCx_Enable);
	TIM_ClearITPendingBit(V_HSTIM, TIM_IT_CC2);
	TIM_ITConfig(V_HSTIM, TIM_IT_CC2, ENABLE);

	//Use compare4 to set the SPI pin as an output and reset the state. This
	//is done during the front porch
	TIM_SetCompare4(V_HSTIM, XPix+V_HSP+V_HBP+10);
	TIM_CCxCmd(V_HSTIM, TIM_Channel_4, TIM_CCx_Enable);
	TIM_ClearITPendingBit(V_HSTIM, TIM_IT_CC4);
	TIM_ITConfig(V_HSTIM, TIM_IT_CC4, ENABLE);

	//End of frame. From here to the timer overflow is the front
	//porch
	TIM_SetCompare1(V_HSTIM, XPix+V_HSP+V_HBP);
	TIM_CCxCmd(V_HSTIM, TIM_Channel_1, TIM_CCx_Enable);

	//Enable timer 1 as the clock to timer 2. Clock timer 2 after every
	//frame to keep synchronised. Use OC1 as timer 2s clock
	TIM_SelectMasterSlaveMode(V_HSTIM, TIM_MasterSlaveMode_Enable);
	TIM_SelectOutputTrigger(V_HSTIM, TIM_TRGOSource_OC1);

	//Initialize the VSync timer
	TB.TIM_ClockDivision = TIM_CKD_DIV1;
	TB.TIM_CounterMode = TIM_CounterMode_Up;
	TB.TIM_Period = YPix + V_VFP + V_VSP + V_VBP;
	TB.TIM_Prescaler = 0;
	TB.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(V_VSTIM, &TB);

	//Initialize VSync output compare. VSync pulse is negative
	//going from zero to V_VSP.
	TO.TIM_OCIdleState = TIM_OCIdleState_Set;
	TO.TIM_OCMode = TIM_OCMode_PWM2;
	TO.TIM_OCPolarity = TIM_OCPolarity_High;
	TO.TIM_OutputState = TIM_OutputState_Enable;
	TO.TIM_Pulse = V_VSP;
	TIM_OC1Init(V_VSTIM, &TO);

	//Enable preload and the interrupt.
	TIM_OC1PreloadConfig(V_VSTIM, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(V_VSTIM, ENABLE);
	TIM_ClearITPendingBit(V_VSTIM, TIM_IT_CC1);
	TIM_ITConfig(V_VSTIM, TIM_IT_CC1, ENABLE);

	//Set timer 2 clock input to timer 1 OC1
	TIM_SelectMasterSlaveMode(V_VSTIM, TIM_MasterSlaveMode_Enable);
	TIM_SelectInputTrigger(V_VSTIM, TIM_TS_ITR0);
	TIM_SelectSlaveMode(V_VSTIM, TIM_SlaveMode_External1);

	TO.TIM_OCIdleState = TIM_OCIdleState_Set;
	TO.TIM_OCMode = TIM_OCMode_PWM1;
	TO.TIM_OCPolarity = TIM_OCPolarity_High;
	TO.TIM_OutputState = TIM_OutputState_Enable;
	TO.TIM_Pulse = 3;
	TIM_OC1Init(TIM3, &TO);

	//Allow preloads and enable output
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM3, ENABLE);

	//Initialize SPI. A baudrate of 2 means data is shifted out
	//at SysClk/4 which in this instance is 160/4 = 40MHz
	S.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	S.SPI_CPHA = SPI_CPHA_1Edge;
	S.SPI_CPOL = SPI_CPOL_Low;
	S.SPI_DataSize = SPI_DataSize_16b;
	S.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	S.SPI_FirstBit = SPI_FirstBit_MSB;
	S.SPI_Mode = SPI_Mode_Master;
	S.SPI_NSS = SPI_NSS_Soft;
	SPI_Init(V_SPI, &S);
	SPI_I2S_DMACmd(V_SPI, SPI_I2S_DMAReq_Tx, ENABLE);
	SPI_Cmd(V_SPI, ENABLE);

	//Initialize DMA from memory to SPI1 TX
	DMA_StructInit(&D);
	DMA_Cmd(DMA2_Stream3, DISABLE);
	D.DMA_BufferSize = XPix/16;
	D.DMA_Channel = DMA_Channel_3;
	D.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	D.DMA_FIFOMode = DMA_FIFOMode_Disable;
	D.DMA_Memory0BaseAddr = GBuf;
	D.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	D.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	D.DMA_MemoryInc = DMA_MemoryInc_Enable;
	D.DMA_Mode = DMA_Mode_Normal;
	D.DMA_PeripheralBaseAddr = (uint32_t) &(SPI1->DR);
	D.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	D.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	D.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	D.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_Init(DMA2_Stream3, &D);
	DMA_ClearITPendingBit(DMA2_Stream3, DMA_IT_TCIF3);
	DMA_ITConfig(DMA2_Stream3, DMA_IT_TC, ENABLE);

	//Enable timer 1 interrupt
	N.NVIC_IRQChannel = TIM1_CC_IRQn;
	N.NVIC_IRQChannelCmd = ENABLE;
	N.NVIC_IRQChannelPreemptionPriority = 0;
	N.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&N);

	//Enable timer 2 interrupt
	N.NVIC_IRQChannel = TIM2_IRQn;
	N.NVIC_IRQChannelCmd = ENABLE;
	N.NVIC_IRQChannelPreemptionPriority = 0;
	N.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&N);

	//Enable DMA interrupt
	N.NVIC_IRQChannel = DMA2_Stream3_IRQn;
	N.NVIC_IRQChannelCmd = ENABLE;
	N.NVIC_IRQChannelPreemptionPriority = 0;
	N.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&N);

	//Enable SPI interrupt
	N.NVIC_IRQChannel = SPI1_IRQn;
	N.NVIC_IRQChannelCmd = ENABLE;
	N.NVIC_IRQChannelPreemptionPriority = 0;
	N.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&N);

	//Clear graphics buffer
	uint32_t Cnt;
	for(Cnt = 0; Cnt<GBufS; Cnt++){
		GBuf[Cnt] = 0x0000;
	}

	//Copy image into graphics buffer
	memcpy(GBuf, Image, 30000*sizeof(uint16_t));

	//Write Hello over image. PixNorm has a black background,
	//PixInv is the opposite of PixNorm and PixNB has no
	//background at all.
	PStr("No background!", 0, 0, 3, PixNB);
	PStr("Black background!", 0, 256, 4, PixNorm);

	//Enable timers. As timer 2 is clocked off timer 1,
	//enable this timer first
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM1, ENABLE);

	while(1);
}
