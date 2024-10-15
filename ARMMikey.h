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
	u8 mikTim0Bkup;		// 0x00 Timer 0 Backup
	u8 mikTim0CtlA;		// 0x01 Timer 0 Control A
	u8 mikTim0Cnt;		// 0x02 Timer 0 Count
	u8 mikTim0CtlB;		// 0x03 Timer 0 Control B
	u8 mikTim1Bkup;		// 0x04 Timer 1 Backup
	u8 mikTim1CtlA;		// 0x05 Timer 1 Control A
	u8 mikTim1Cnt;		// 0x06 Timer 1 Count
	u8 mikTim1CtlB;		// 0x07 Timer 1 Control B
	u8 mikTim2Bkup;		// 0x08 Timer 2 Backup
	u8 mikTim2CtlA;		// 0x09 Timer 2 Control A
	u8 mikTim2Cnt;		// 0x0A Timer 2 Count
	u8 mikTim2CtlB;		// 0x0B Timer 2 Control B
	u8 mikTim3Bkup;		// 0x0C Timer 3 Backup
	u8 mikTim3CtlA;		// 0x0D Timer 3 Control A
	u8 mikTim3Cnt;		// 0x0E Timer 3 Count
	u8 mikTim3CtlB;		// 0x0F Timer 3 Control B
	u8 mikTim4Bkup;		// 0x10 Timer 4 Backup
	u8 mikTim4CtlA;		// 0x11 Timer 4 Control A
	u8 mikTim4Cnt;		// 0x12 Timer 4 Count
	u8 mikTim4CtlB;		// 0x13 Timer 4 Control B
	u8 mikTim5Bkup;		// 0x14 Timer 5 Backup
	u8 mikTim5CtlA;		// 0x15 Timer 5 Control A
	u8 mikTim5Cnt;		// 0x16 Timer 5 Count
	u8 mikTim5CtlB;		// 0x17 Timer 5 Control B
	u8 mikTim6Bkup;		// 0x18 Timer 6 Backup
	u8 mikTim6CtlA;		// 0x19 Timer 6 Control A
	u8 mikTim6Cnt;		// 0x1A Timer 6 Count
	u8 mikTim6CtlB;		// 0x1B Timer 6 Control B
	u8 mikTim7Bkup;		// 0x1C Timer 7 Backup
	u8 mikTim7CtlA;		// 0x1D Timer 7 Control A
	u8 mikTim7Cnt;		// 0x1E Timer 7 Count
	u8 mikTim7CtlB;		// 0x1F Timer 7 Control B

	u8 mikAud0Vol;		// 0x20 Audio 0 2's Complement Volume Control
	u8 mikAud0ShftFb;	// 0x21 Audio 0 Shift Register Feedback Enable
	u8 mikAud0OutVal;	// 0x22 Audio 0 Output Value
	u8 mikAud0L8Shft;	// 0x23 Audio 0 Lower 8 Bits of Shift Register
	u8 mikAud0TBack;	// 0x24 Audio 0 Audio Timer Backup Value
	u8 mikAud0Ctl;		// 0x25 Audio 0 Audio Control Bits
	u8 mikAud0Count;	// 0x26 Audio 0 Counter
	u8 mikAud0Misc;		// 0x27 Audio 0 Other Bits

	u8 mikAud1Vol;		// 0x28 Audio 1 2's Complement Volume Control
	u8 mikAud1ShftFb;	// 0x29 Audio 1 Shift Register Feedback Enable
	u8 mikAud1OutVal;	// 0x2A Audio 1 Output Value
	u8 mikAud1L8Shft;	// 0x2B Audio 1 Lower 8 Bits of Shift Register
	u8 mikAud1TBack;	// 0x2C Audio 1 Audio Timer Backup Value
	u8 mikAud1Ctl;		// 0x2D Audio 1 Audio Control Bits
	u8 mikAud1Count;	// 0x2E Audio 1 Counter
	u8 mikAud1Misc;		// 0x2F Audio 1 Other Bits

	u8 mikAud2Vol;		// 0x30 Audio 2 2's Complement Volume Control
	u8 mikAud2ShftFb;	// 0x31 Audio 2 Shift Register Feedback Enable
	u8 mikAud2OutVal;	// 0x32 Audio 2 Output Value
	u8 mikAud2L8Shft;	// 0x33 Audio 2 Lower 8 Bits of Shift Register
	u8 mikAud2TBack;	// 0x34 Audio 2 Audio Timer Backup Value
	u8 mikAud2Ctl;		// 0x35 Audio 2 Audio Control Bits
	u8 mikAud2Count;	// 0x36 Audio 2 Counter
	u8 mikAud2Misc;		// 0x37 Audio 2 Other Bits

	u8 mikAud3Vol;		// 0x38 Audio 3 2's Complement Volume Control
	u8 mikAud3ShftFb;	// 0x39 Audio 3 Shift Register Feedback Enable
	u8 mikAud3OutVal;	// 0x3A Audio 3 Output Value
	u8 mikAud3L8Shft;	// 0x3B Audio 3 Lower 8 Bits of Shift Register
	u8 mikAud3TBack;	// 0x3C Audio 3 Audio Timer Backup Value
	u8 mikAud3Ctl;		// 0x3D Audio 3 Audio Control Bits
	u8 mikAud3Count;	// 0x3E Audio 3 Counter
	u8 mikAud3Misc;		// 0x3F Audio 3 Other Bits

			// Lynx2 Regs
	u8 mikAttenA;		// 0x40
	u8 mikAttenB;		// 0x41
	u8 mikAttenC;		// 0x42
	u8 mikAttenD;		// 0x43
	u8 mikMPAN;			// 0x44
	u8 mikPadding0[0x0B];// 0x45-0x4F

	u8 mikMStereo;		// 0x50 MSTEREO
	u8 mikPadding1[0x2F];// 0x51-0x7F

	u8 mikIntRst;		// 0x80 Interrupt Reset
	u8 mikIntSet;		// 0x81 Interrupt Set
	u8 mikPadding2[0x02];// 0x82-0x83
	u8 mikMagRdy0;		// 0x84 Mag Tape Channel 0 ready bit
	u8 mikMagRdy1;		// 0x85 Mag Tape Channel 1 ready bit
	u8 mikAudIn;		// 0x86 Audio In
	u8 mikSysCtl1;		// 0x87 System Control 1
	u8 mikMikeyHRev;	// 0x88 Mikey Hardware Revision
	u8 mikMikeySRev;	// 0x89 Mikey Software Revision
	u8 mikIODir;		// 0x8A IO Direction
	u8 mikIODat;		// 0x8B IO Data
	u8 mikSerCtl;		// 0x8C Serial Control
	u8 mikSerDat;		// 0x8D Serial Data
	u8 mikPadding3[0x02];// 0x8E-0x8F

	u8 mikSDoneAck;		// 0x90 Suzy Done Acknowledge
	u8 mikCpuSleep;		// 0x91 CPU Sleep, Bus Request Disable
	u8 mikDispCtl;		// 0x92 Display Control,  Video Bus Reguest Enable.
	u8 mikPBkup;		// 0x93 P Backup, Magic 'P' count
	//u8 DispAdr;
	u8 DispAdrL;		// 0x94 Display Adress Low
	u8 DispAdrH;		// 0x95 Display Adress High
	u8 mikPadding4[0x06];// 0x96-0x9B
	u8 mikMtest0;		// 0x9C Mtest0
	u8 mikMtest1;		// 0x9D Mtest1
	u8 mikMtest2;		// 0x9E Mtest2
	u8 mikPadding5;		// 0x9F

	u8 mikGreen[0x10];	// 0xA0-0xAF Green palette
	u8 mikBlueRed[0x10];// 0xB0-0xBF Blue/Red palette


//------------------------------
	u8 mikLCDVSize;				// 0x01 LCD Y Size
	u8 mikSOC;					// HOWARD or HOWARD2
	u8 sdfsdfsf[2];

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
