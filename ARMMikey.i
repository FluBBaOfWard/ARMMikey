//
//  ARMMikey.i
//  Atari Lynx Mikey emulation for ARM32.
//
//  Created by Fredrik Ahlström on 2024-10-14.
//  Copyright © 2024 Fredrik Ahlström. All rights reserved.
//

#define HW_AUTO       (0)
#define HW_LYNX       (1)
#define HW_LYNX_II    (2)
#define HW_SELECT_END (3)

#define SOC_HOWARD    (0)
#define SOC_HOWARD2   (1)

/** Game screen width in pixels */
#define GAME_WIDTH  (160)
/** Game screen height in pixels */
#define GAME_HEIGHT (102)



	.struct 0
mikTimerStart:
BKUP:			.long
ENABLE_RELOAD:	.long
ENABLE_COUNT:	.long
LINKING:		.long
CURRENT:		.long
TIMER_DONE:		.long
LAST_CLOCK:		.long
BORROW_IN:		.long
BORROW_OUT:		.long
LAST_LINK_CARRY:.long
LAST_COUNT:		.long
mikTimerEnd:
mikTimerSize = 4*11


	mikptr		.req r12
						;@ SVVideo.s
	.struct 0
scanline:			.long 0		;@ These 3 must be first in state.
nextLineChange:		.long 0
lineState:			.long 0

windowData:			.long 0
mikeyState:					;@
mikRegs:

timer0:				.space mikTimerSize
timer1:				.space mikTimerSize
timer2:				.space mikTimerSize
timer3:				.space mikTimerSize
timer4:				.space mikTimerSize
timer5:				.space mikTimerSize
timer6:				.space mikTimerSize
timer7:				.space mikTimerSize

mikLCDHSize:		.byte 0		;@ 0x00 LCD Horizontal Size
mikLCDVSize:		.byte 0		;@ 0x01 LCD Vertical Size
mikHScroll:			.byte 0		;@ 0x02 Horizontal Scroll
mikVScroll:			.byte 0		;@ 0x03 Vertical Scroll
mikMirr00:			.byte 0		;@ 0x04 Mirror of reg 0x00
mikMirr01:			.byte 0		;@ 0x05 Mirror of reg 0x01
mikMirr02:			.byte 0		;@ 0x06 Mirror of reg 0x02
mikMirr03:			.byte 0		;@ 0x07 Mirror of reg 0x03

mikDMACBus:
mikDMACBusLow:		.byte 0		;@ 0x08 DMA CBus Low
mikDMACBusHigh:		.byte 0		;@ 0x09 DMA CBus High
mikDMAVBus:
mikDMAVBusLow:		.byte 0		;@ 0x0A DMA VBus Low
mikDMAVBusHigh:		.byte 0		;@ 0x0B DMA VBus High
mikDMALen:			.byte 0		;@ 0x0C DMA Length
mikDMACtrl:			.byte 0		;@ 0x0D DMA Control

mikPadding0:		.space 2	;@ 0x0E-0x0F ??

mikCh1Freq:						;@ Channel 1 (Right only)
mikCh1FreqLow:		.byte 0		;@ 0x10 Channel 1 Frequency Low
mikCh1FreqHigh:		.byte 0		;@ 0x11 Channel 1 Frequency High
mikCh1Ctrl:			.byte 0		;@ 0x12 Channel 1 Volume/Duty cycle
mikCh1Len:			.byte 0		;@ 0x13 Channel 1 Length
mikCh2Freq:						;@ Channel 2 (Left only)
mikCh2FreqLow:		.byte 0		;@ 0x14 Channel 2 Frequency Low
mikCh2FreqHigh:		.byte 0		;@ 0x15 Channel 2 Frequency High
mikCh2Ctrl:			.byte 0		;@ 0x16 Channel 2 Volume/Duty cycle
mikCh2Len:			.byte 0		;@ 0x17 Channel 2 Length

mikCh3Adr:
mikCh3AdrLow:		.byte 0		;@ 0x18 Channel 3 Address Low
mikCh3AdrHigh:		.byte 0		;@ 0x19 Channel 3 Address High
mikCh3Len:			.byte 0		;@ 0x1A Channel 3 Length
mikCh3Ctrl:			.byte 0		;@ 0x1B Channel 3 Control
mikCh3Trigg:		.byte 0		;@ 0x1C Channel 3 Trigger
mikPadding1:		.space 3	;@ 0x1D - 0x1F ???

mikController:		.byte 0		;@ 0x20 Controller
mikLinkPortDDR:		.byte 0		;@ 0x21 Link Port DDR
mikLinkPortData:	.byte 0		;@ 0x22 Link Port Data
mikIRQTimer:		.byte 0		;@ 0x23 IRQ Timer
mikTimerIRQReset:	.byte 0		;@ 0x24 Timer IRQ Reset
mikSndDMAIRQReset:	.byte 0		;@ 0x25 Sound DMA IRQ Reset
mikSystemControl:	.byte 0		;@ 0x26 System Control
mikIRQStatus:		.byte 0		;@ 0x27 IRQ Status
mikCh4FreqVol:		.byte 0		;@ 0x28 Channel 4 Frequency and volume
mikCh4Len:			.byte 0		;@ 0x29 Channel 4 Length
mikCh4Ctrl:			.byte 0		;@ 0x2A Channel 4 Control
mikPadding2:		.byte 0		;@ 0x2B ???
mikMirr028:			.byte 0		;@ 0x2C Mirror of Reg 0x28
mikMirr029:			.byte 0		;@ 0x2D Mirror of Reg 0x29
mikMirr02A:			.byte 0		;@ 0x2E Mirror of Reg 0x2A
mikPadding3:		.byte 0		;@ 0x2F ???

;@----------------------------------------------------------------------------
mikNMITimer:		.long 0
mikTimerValue:		.long 0

mikCh1Counter:		.long 0		;@ Ch1 Counter
mikCh2Counter:		.long 0		;@ Ch2 Counter
mikCh3Counter:		.long 0		;@ Ch3 Counter
mikCh4Counter:		.long 0		;@ Ch4 Counter
mikCh4LFSR:			.long 0		;@ Ch4 Noise LFSR
mikCh3Address:		.long 0		;@ Ch3 sample address (physical)
mikCh4Feedback:		.long 0		;@ Ch4 Noise Feedback

mikNMIStatus:		.byte 0		;@ NMI Status
mikSOC:				.byte 0		;@ HOWARD or HOWARD2
mikPadding4:		.space 2

mikeyStateEnd:

mikNmiFunction:		.long 0		;@ NMI function
mikIrqFunction:		.long 0		;@ IRQ function

mikGfxRAM:			.long 0		;@ 0x10000

mikeySize:

;@----------------------------------------------------------------------------

