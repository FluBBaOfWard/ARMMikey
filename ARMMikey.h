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

#include "ARM6502/M6502.h"

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

#define BORROW_OUT (1<<0)
#define BORROW_IN (1<<1)
#define LAST_CLOCK (1<<2)
#define TIMER_DONE (1<<3)

#define CLOCK_SEL (7<<0)
#define LINKING (7<<0)
#define ENABLE_COUNT (1<<3)
#define ENABLE_RELOAD (1<<4)
#define INTEGRATE (1<<5)

typedef struct
{
	u32 CURRENT;
	u32 LAST_COUNT;
} MTIMER;

typedef struct
{
	u32 BKUP;
	u32 CURRENT;
	u32 LAST_COUNT;
	u32 WAVESHAPER;
} MAUDIO;


typedef struct {
	M6502Core m6502;
	// mikeyState:					;@
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
	u8 tim0Bkup;			// 0x00 Timer 0 Backup
	u8 tim0CtlA;			// 0x01 Timer 0 Control A
	u8 tim0Cnt;				// 0x02 Timer 0 Count
	u8 tim0CtlB;			// 0x03 Timer 0 Control B
	u8 tim1Bkup;			// 0x04 Timer 1 Backup
	u8 tim1CtlA;			// 0x05 Timer 1 Control A
	u8 tim1Cnt;				// 0x06 Timer 1 Count
	u8 tim1CtlB;			// 0x07 Timer 1 Control B
	u8 tim2Bkup;			// 0x08 Timer 2 Backup
	u8 tim2CtlA;			// 0x09 Timer 2 Control A
	u8 tim2Cnt;				// 0x0A Timer 2 Count
	u8 tim2CtlB;			// 0x0B Timer 2 Control B
	u8 tim3Bkup;			// 0x0C Timer 3 Backup
	u8 tim3CtlA;			// 0x0D Timer 3 Control A
	u8 tim3Cnt;				// 0x0E Timer 3 Count
	u8 tim3CtlB;			// 0x0F Timer 3 Control B
	u8 tim4Bkup;			// 0x10 Timer 4 Backup
	u8 tim4CtlA;			// 0x11 Timer 4 Control A
	u8 tim4Cnt;				// 0x12 Timer 4 Count
	u8 tim4CtlB;			// 0x13 Timer 4 Control B
	u8 tim5Bkup;			// 0x14 Timer 5 Backup
	u8 tim5CtlA;			// 0x15 Timer 5 Control A
	u8 tim5Cnt;				// 0x16 Timer 5 Count
	u8 tim5CtlB;			// 0x17 Timer 5 Control B
	u8 tim6Bkup;			// 0x18 Timer 6 Backup
	u8 tim6CtlA;			// 0x19 Timer 6 Control A
	u8 tim6Cnt;				// 0x1A Timer 6 Count
	u8 tim6CtlB;			// 0x1B Timer 6 Control B
	u8 tim7Bkup;			// 0x1C Timer 7 Backup
	u8 tim7CtlA;			// 0x1D Timer 7 Control A
	u8 tim7Cnt;				// 0x1E Timer 7 Count
	u8 tim7CtlB;			// 0x1F Timer 7 Control B

	s8 aud0Vol;				// 0x20 Audio 0 2's Complement Volume Control
	u8 aud0ShftFb;			// 0x21 Audio 0 Shift Register Feedback Enable
	s8 aud0OutVal;			// 0x22 Audio 0 Output Value
	u8 aud0L8Shft;			// 0x23 Audio 0 Lower 8 Bits of Shift Register
	u8 aud0TBack;			// 0x24 Audio 0 Audio Timer Backup Value
	u8 aud0Ctl;				// 0x25 Audio 0 Audio Control Bits
	u8 aud0Count;			// 0x26 Audio 0 Counter
	u8 aud0Misc;			// 0x27 Audio 0 Other Bits

	s8 aud1Vol;				// 0x28 Audio 1 2's Complement Volume Control
	u8 aud1ShftFb;			// 0x29 Audio 1 Shift Register Feedback Enable
	s8 aud1OutVal;			// 0x2A Audio 1 Output Value
	u8 aud1L8Shft;			// 0x2B Audio 1 Lower 8 Bits of Shift Register
	u8 aud1TBack;			// 0x2C Audio 1 Audio Timer Backup Value
	u8 aud1Ctl;				// 0x2D Audio 1 Audio Control Bits
	u8 aud1Count;			// 0x2E Audio 1 Counter
	u8 aud1Misc;			// 0x2F Audio 1 Other Bits

	s8 aud2Vol;				// 0x30 Audio 2 2's Complement Volume Control
	u8 aud2ShftFb;			// 0x31 Audio 2 Shift Register Feedback Enable
	s8 aud2OutVal;			// 0x32 Audio 2 Output Value
	u8 aud2L8Shft;			// 0x33 Audio 2 Lower 8 Bits of Shift Register
	u8 aud2TBack;			// 0x34 Audio 2 Audio Timer Backup Value
	u8 aud2Ctl;				// 0x35 Audio 2 Audio Control Bits
	u8 aud2Count;			// 0x36 Audio 2 Counter
	u8 aud2Misc;			// 0x37 Audio 2 Other Bits

	s8 aud3Vol;				// 0x38 Audio 3 2's Complement Volume Control
	u8 aud3ShftFb;			// 0x39 Audio 3 Shift Register Feedback Enable
	s8 aud3OutVal;			// 0x3A Audio 3 Output Value
	u8 aud3L8Shft;			// 0x3B Audio 3 Lower 8 Bits of Shift Register
	u8 aud3TBack;			// 0x3C Audio 3 Audio Timer Backup Value
	u8 aud3Ctl;				// 0x3D Audio 3 Audio Control Bits
	u8 aud3Count;			// 0x3E Audio 3 Counter
	u8 aud3Misc;			// 0x3F Audio 3 Other Bits

			// Lynx2 Regs
	u8 attenA;				// 0x40
	u8 attenB;				// 0x41
	u8 attenC;				// 0x42
	u8 attenD;				// 0x43
	u8 mPan;				// 0x44
	u8 padding0[0x0B];		// 0x45-0x4F

	u8 stereo;				// 0x50 STEREO
	u8 padding1[0x2F];		// 0x51-0x7F

	u8 intRst;				// 0x80 Interrupt Reset
	u8 intSet;				// 0x81 Interrupt Set
	u8 padding2[0x02];		// 0x82-0x83
	u8 magRdy0;				// 0x84 Mag Tape Channel 0 Ready bit
	u8 magRdy1;				// 0x85 Mag Tape Channel 1 Ready bit
	u8 audIn;				// 0x86 Audio In
	u8 sysCtl1;				// 0x87 System Control 1
	u8 mikeyHRev;			// 0x88 Mikey Hardware Revision
	u8 mikeySRev;			// 0x89 Mikey Software Revision
	u8 ioDir;				// 0x8A IO Direction
	u8 ioDat;				// 0x8B IO Data
	u8 serCtl;				// 0x8C Serial Control
	u8 serDat;				// 0x8D Serial Data
	u8 padding3[0x02];		// 0x8E-0x8F

	u8 sDoneAck;			// 0x90 Suzy Done Acknowledge
	u8 cpuSleep;			// 0x91 CPU Sleep, Bus Request Disable
	u8 dispCtl;				// 0x92 Display Control,  Video Bus Reguest Enable.
	u8 pBkup;				// 0x93 P Backup, Magic 'P' count
	u16 dispAdr;			// 0x94, 0x95 Display Address
	u8 padding4[0x06];		// 0x96-0x9B
	u8 mtest0;				// 0x9C Mtest0
	u8 mtest1;				// 0x9D Mtest1
	u8 mtest2;				// 0x9E Mtest2
	u8 padding5;			// 0x9F

	u8 paletteG[0x10];		// 0xA0-0xAF Green palette
	u8 paletteBR[0x10];		// 0xB0-0xBF Blue/Red palette


//------------------------------
	u32 palette[16];		// Merged palette
	u8 paletteChanged;		// Palette updated since last render.
	u8 mikLCDVSize;			// 0x01 LCD Y Size
	u8 mikSOC;				// HOWARD or HOWARD2
	u8 serCablePresent;
	u8 ioDatRestSignal;
	u8 timerStatusFlags;
	u8 timerInterruptMask;
	u8 systemCPUSleep;
	u8 memSelector;
	u8 padding6[3];

	u32 lynxLineDMACounter;
	u32 lynxLine;
	u32 lynxAddr;
	u32 systemCycleCount;
	u32 nextTimerEvent;
	u32 suzieDoneTime;
	u32 audioLastUpdateCycle;

	MAUDIO audio0;
	MAUDIO audio1;
	MAUDIO audio2;
	MAUDIO audio3;

	void *mikCartPtr;		// Pointer to LynxCart object.
	void (*mikNmiFunction)(bool pin);	// NMI callback
	void (*mikIrqFunction)(bool pin);	// IRQ callback
	void (*mikLineCallback)(u8 *ram, u32 *palette, bool flip);
	void (*mikFrameCallback)(void);

	void *mikGfxRAM;

} MIKEY;

void mikeyReset(void *irqFunction(), void *ram, int soc);

/**
 * Saves the state of the chip to the destination.
 * @param  *destination: Where to save the state.
 * @param  *chip: The MIKEY chip to save.
 * @return The size of the state.
 */
int mikeySaveState(void *destination, const MIKEY *chip);

/**
 * Loads the state of the chip from the source.
 * @param  *chip: The MIKEY chip to load a state into.
 * @param  *source: Where to load the state from.
 * @return The size of the state.
 */
int mikeyLoadState(MIKEY *chip, const void *source);

/**
 * Gets the state size of a MIKEY chip.
 * @return The size of the state.
 */
int mikeyGetStateSize(void);

void mikSysUpdate(void);

void miDoScanline(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // SVVIDEO_HEADER
