#ifndef GFX_H
#define GFX_H

#include <stm32f4xx.h>

/*
 * GFX.h
 * Author: Harris Shallcross
 * Year: 2014-ish
 *
 * General GFX library, uses a framebuffer (1024byte) for all functions.
 * Circle, ellipse and semicircle functions use a sine wavetable (256byte).
 *
 * This code is provided AS IS and no warranty is included!
 */

extern volatile uint16_t GBuf[];

#define XPix	800
#define YPix	600
#define P_XPix	XPix
#define P_YPix	YPix
#define P_GBuf	GBuf

#define LetterSpace 1

#define CHAR_LINE 	(0x80)
#define LEFT_ARROW 	(0x81)
#define RIGHT_ARROW (0x82)
#define DOWN_ARROW 	(0x83)
#define UP_ARROW 	(0x84)
#define SAVE_FILE 	(0x85)
#define CLR_ALL 	(0x86)

typedef enum {
	PixNorm = 0,
	PixInv,
	PixNB
} PixT;

uint8_t WritePix(int16_t, int16_t, PixT);
uint8_t SetPix(uint8_t, uint8_t);
uint8_t ClrPix(uint8_t, uint8_t);

uint8_t Circle(uint8_t, uint8_t, uint8_t, PixT);
uint8_t Semicircle(uint8_t, uint8_t, uint8_t, uint8_t, PixT);

uint8_t Square(uint8_t, uint8_t, uint8_t, uint8_t, PixT);
uint8_t LineP(uint8_t, uint8_t, uint8_t, int16_t, PixT);
uint8_t LineL(uint8_t, uint8_t, uint8_t, uint8_t, PixT);

void DrawBMP(uint8_t [], uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);

int PChar(uint16_t, int16_t, int16_t, uint8_t, PixT);
int PStr(const char*, int16_t, int16_t, uint8_t, PixT);
int PNum(int32_t Num, int16_t X, int16_t Y, int8_t Pad, uint8_t Size, PixT Inv);
int PNumF(float, int16_t, int16_t, uint8_t, uint8_t, PixT);
void FillRow(uint32_t XPos, uint32_t Y);

int32_t FPow(int32_t Num, uint32_t Pow);
uint8_t CheckNumLength(int32_t Num);
uint32_t Strlen(const char*);

#endif
