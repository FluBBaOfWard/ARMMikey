//
//  ARMMikey.h
//  Atari Lynx Mikey emulation for ARM32.
//
//  Created by Fredrik Ahlström on 2024-10-14.
//  Copyright © 2024 Fredrik Ahlström. All rights reserved.
//

#ifndef MIKEY_HEADER
#define MIKEY_HEADER

#ifdef __cplusplus
extern "C" {
#endif

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

typedef struct
{
	u32 BKUP;
	u32 ENABLE_RELOAD;
	u32 ENABLE_COUNT;
	u32 LINKING;
	u32 CURRENT;
	u32 TIMER_DONE;
	u32 LAST_CLOCK;
	u32 BORROW_IN;
	u32 BORROW_OUT;
	u32 LAST_LINK_CARRY;
	u32 LAST_COUNT;
} MTIMER;

typedef struct
{
	u32 BKUP;
	u32 ENABLE_RELOAD;
	u32 ENABLE_COUNT;
	u32 LINKING;
	u32 CURRENT;
	u32 TIMER_DONE;
	u32 LAST_CLOCK;
	u32 BORROW_IN;
	u32 BORROW_OUT;
	u32 LAST_LINK_CARRY;
	u32 LAST_COUNT;
	s8  VOLUME;
	s8  OUTPUT;
	u32 INTEGRATE_ENABLE;
	u32 WAVESHAPER;
} MAUDIO;


typedef struct {
	u32 scanline;
	u32 nextLineChange;
	u32 lineState;

	u32 windowData;
//mikState:
	MTIMER timer0;
	MTIMER timer1;
	MTIMER timer2;
	MTIMER timer3;
	MTIMER timer4;
	MTIMER timer5;
	MTIMER timer6;
	MTIMER timer7;

//mikRegs:

	u8 mikLCDVSize;				// 0x01 LCD Y Size
	u8 mikXScroll;				// 0x02 X Scroll
	u8 mikYScroll;				// 0x03 Y Scroll
	u8 mikMirr00;				// 0x04 Mirror of reg 0x00
	u8 mikMirr01;				// 0x05 Mirror of reg 0x01
	u8 mikMirr02;				// 0x06 Mirror of reg 0x02
	u8 mikMirr03;				// 0x07 Mirror of reg 0x03

	u8 mikDMASrcLow;			// 0x08 DMA Source Low
	u8 mikDMASrcHigh;			// 0x09 DMA Source High
	u8 mikDMADstLow;			// 0x0A DMA Destination Low
	u8 mikDMADstHigh;			// 0x0B DMA Destination High
	u8 mikDMALen;				// 0x0C DMA Length
	u8 mikDMACtrl;				// 0x0D DMA Control

	u8 mikPadding0[2];			// 0x0E-0x0F ??

	u8 mikCh1FreqLow;			// 0x10 Channel 1 Frequency Low (Right only)
	u8 mikCh1FreqHigh;			// 0x11 Channel 1 Frequency High
	u8 mikCh1Duty;				// 0x12 Channel 1 Duty cycle
	u8 mikCh1Len;				// 0x13 Channel 1 Length
	u8 mikCh2FreqLow;			// 0x14 Channel 2 Frequency Low (Left only)
	u8 mikCh2FreqHigh;			// 0x15 Channel 2 Frequency High
	u8 mikCh2Duty;				// 0x16 Channel 2 Duty cycle
	u8 mikCh2Len;				// 0x17 Channel 2 Length

	u8 mikCh3AdrLow;			// 0x18 Channel 3 Address Low
	u8 mikCh3AdrHigh;			// 0x19 Channel 3 Address High
	u8 mikCh3Len;				// 0x1A Channel 3 Length
	u8 mikCh3Ctrl;				// 0x1B Channel 3 Control
	u8 mikCh3Trigg;				// 0x1C Channel 3 Trigger
	u8 mikPadding1[3];			// 0x1D - 0x1F ???

	u8 mikController;			// 0x20 Controller
	u8 mikLinkPortDDR;			// 0x21 Link Port DDR
	u8 mikLinkPortData;			// 0x22 Link Port Data
	u8 mikIRQTimer;				// 0x23 IRQ Timer
	u8 mikTimerIRQReset;		// 0x24 Timer IRQ Reset
	u8 mikSndDMAIRQReset;		// 0x25 Sound DMA IRQ Reset
	u8 mikSystemControl;		// 0x26 System Control
	u8 mikIRQStatus;			// 0x27 IRQ Status
	u8 mikCh4FreqVol;			// 0x28 Channel 4 Frquency and volume
	u8 mikCh4Len;				// 0x29 Channel 4 Length
	u8 mikCh4Ctrl;				// 0x2A Channel 4 Control
	u8 mikPadding2;				// 0x2B ???
	u8 mikMirr028;				// 0x2C Mirror of Reg 0x28
	u8 mikMirr029;				// 0x2D Mirror of Reg 0x29
	u8 mikMirr02A;				// 0x2E Mirror of Reg 0x2A
	u8 mikPadding3;				// 0x2F ???

//------------------------------
	u32 mikNMITimer;
	u32 mikTimerValue;

	u32 mikCh1Counter;			// Ch1 Counter
	u32 mikCh2Counter;			// Ch2 Counter
	u32 mikCh3Counter;			// Ch3 Counter
	u32 mikCh4Counter;			// Ch4 Counter
	u32 mikCh4LFSR;				// Ch4 Noise LFSR
	u32 mikCh3Address;			// Ch3 sample address (physical)
	u32 mikCh4Feedback;			// Ch4 Noise Feedback

	u8 mikNMIStatus;			// NMI pin out status
	u8 mikSOC;					// HOWARD or HOWARD2
	u8 mikPadding4[2];

	void *mikNmiFunction;			// NMI callback
	void *mikIrqFunction;			// IRQ callback

	void *mikGfxRAM;

} MIKEY;

void svVideoReset(void *irqFunction(), void *ram, int soc);

/**
 * Saves the state of the chip to the destination.
 * @param  *destination: Where to save the state.
 * @param  *chip: The SUZY chip to save.
 * @return The size of the state.
 */
int miVideoSaveState(void *destination, const MIKEY *chip);

/**
 * Loads the state of the chip from the source.
 * @param  *chip: The SUZY chip to load a state into.
 * @param  *source: Where to load the state from.
 * @return The size of the state.
 */
int miVideoLoadState(MIKEY *chip, const void *source);

/**
 * Gets the state size of a SUZY chip.
 * @return The size of the state.
 */
int miVideoGetStateSize(void);

void miDoScanline(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // SVVIDEO_HEADER
