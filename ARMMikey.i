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
ENABLE_RELOAD:	.long
ENABLE_COUNT:	.long
LINKING:		.long
CURRENT:		.long
TIMER_DONE:		.long
unused2:		.long
unused:			.long
CTLB:			.long
LAST_LINK_CARRY:.long
LAST_COUNT:		.long
mikTimerEnd:
mikTimerSize = 4*10


	mikptr		.req r12
						;@ SVVideo.s
	.struct 0
scanline:			.long 0		;@ These 3 must be first in state.
nextLineChange:		.long 0
lineState:			.long 0

windowData:			.long 0
mikeyState:					;@

timer0:				.space mikTimerSize
timer1:				.space mikTimerSize
timer2:				.space mikTimerSize
timer3:				.space mikTimerSize
timer4:				.space mikTimerSize
timer5:				.space mikTimerSize
timer6:				.space mikTimerSize
timer7:				.space mikTimerSize

mikRegs:

mikTim0Bkup:		.byte 0		;@ 0x00 Timer 0 Backup
mikTim0CtlA:		.byte 0		;@ 0x01 Timer 0 Control A
mikTim0Cnt:			.byte 0		;@ 0x02 Timer 0 Count
mikTim0CtlB:		.byte 0		;@ 0x03 Timer 0 Control B
mikTim1Bkup:		.byte 0		;@ 0x04 Timer 1 Backup
mikTim1CtlA:		.byte 0		;@ 0x05 Timer 1 Control A
mikTim1Cnt:			.byte 0		;@ 0x06 Timer 1 Count
mikTim1CtlB:		.byte 0		;@ 0x07 Timer 1 Control B
mikTim2Bkup:		.byte 0		;@ 0x08 Timer 2 Backup
mikTim2CtlA:		.byte 0		;@ 0x09 Timer 2 Control A
mikTim2Cnt:			.byte 0		;@ 0x0A Timer 2 Count
mikTim2CtlB:		.byte 0		;@ 0x0B Timer 2 Control B
mikTim3Bkup:		.byte 0		;@ 0x0C Timer 3 Backup
mikTim3CtlA:		.byte 0		;@ 0x0D Timer 3 Control A
mikTim3Cnt:			.byte 0		;@ 0x0E Timer 3 Count
mikTim3CtlB:		.byte 0		;@ 0x0F Timer 3 Control B
mikTim4Bkup:		.byte 0		;@ 0x10 Timer 4 Backup
mikTim4CtlA:		.byte 0		;@ 0x11 Timer 4 Control A
mikTim4Cnt:			.byte 0		;@ 0x12 Timer 4 Count
mikTim4CtlB:		.byte 0		;@ 0x13 Timer 4 Control B
mikTim5Bkup:		.byte 0		;@ 0x14 Timer 5 Backup
mikTim5CtlA:		.byte 0		;@ 0x15 Timer 5 Control A
mikTim5Cnt:			.byte 0		;@ 0x16 Timer 5 Count
mikTim5CtlB:		.byte 0		;@ 0x17 Timer 5 Control B
mikTim6Bkup:		.byte 0		;@ 0x18 Timer 6 Backup
mikTim6CtlA:		.byte 0		;@ 0x19 Timer 6 Control A
mikTim6Cnt:			.byte 0		;@ 0x1A Timer 6 Count
mikTim6CtlB:		.byte 0		;@ 0x1B Timer 6 Control B
mikTim7Bkup:		.byte 0		;@ 0x1C Timer 7 Backup
mikTim7CtlA:		.byte 0		;@ 0x1D Timer 7 Control A
mikTim7Cnt:			.byte 0		;@ 0x1E Timer 7 Count
mikTim7CtlB:		.byte 0		;@ 0x1F Timer 7 Control B

mikAud0Vol:			.byte 0		;@ 0x20 Audio 0 2's Complement Volume Control
mikAud0ShftFb:		.byte 0		;@ 0x21 Audio 0 Shift Register Feedback Enable
mikAud0OutVal:		.byte 0		;@ 0x22 Audio 0 Output Value
mikAud0L8Shft:		.byte 0		;@ 0x23 Audio 0 Lower 8 Bits of Shift Register
mikAud0TBack:		.byte 0		;@ 0x24 Audio 0 Audio Timer Backup Value
mikAud0Ctl:			.byte 0		;@ 0x25 Audio 0 Audio Control Bits
mikAud0Count:		.byte 0		;@ 0x26 Audio 0 Counter
mikAud0Misc:		.byte 0		;@ 0x27 Audio 0 Other Bits

mikAud1Vol:			.byte 0		;@ 0x28 Audio 1 2's Complement Volume Control
mikAud1ShftFb:		.byte 0		;@ 0x29 Audio 1 Shift Register Feedback Enable
mikAud1OutVal:		.byte 0		;@ 0x2A Audio 1 Output Value
mikAud1L8Shft:		.byte 0		;@ 0x2B Audio 1 Lower 8 Bits of Shift Register
mikAud1TBack:		.byte 0		;@ 0x2C Audio 1 Audio Timer Backup Value
mikAud1Ctl:			.byte 0		;@ 0x2D Audio 1 Audio Control Bits
mikAud1Count:		.byte 0		;@ 0x2E Audio 1 Counter
mikAud1Misc:		.byte 0		;@ 0x2F Audio 1 Other Bits

mikAud2Vol:			.byte 0		;@ 0x30 Audio 2 2's Complement Volume Control
mikAud2ShftFb:		.byte 0		;@ 0x31 Audio 2 Shift Register Feedback Enable
mikAud2OutVal:		.byte 0		;@ 0x32 Audio 2 Output Value
mikAud2L8Shft:		.byte 0		;@ 0x33 Audio 2 Lower 8 Bits of Shift Register
mikAud2TBack:		.byte 0		;@ 0x34 Audio 2 Audio Timer Backup Value
mikAud2Ctl:			.byte 0		;@ 0x35 Audio 2 Audio Control Bits
mikAud2Count:		.byte 0		;@ 0x36 Audio 2 Counter
mikAud2Misc:		.byte 0		;@ 0x37 Audio 2 Other Bits

mikAud3Vol:			.byte 0		;@ 0x38 Audio 3 2's Complement Volume Control
mikAud3ShftFb:		.byte 0		;@ 0x39 Audio 3 Shift Register Feedback Enable
mikAud3OutVal:		.byte 0		;@ 0x3A Audio 3 Output Value
mikAud3L8Shft:		.byte 0		;@ 0x3B Audio 3 Lower 8 Bits of Shift Register
mikAud3TBack:		.byte 0		;@ 0x3C Audio 3 Audio Timer Backup Value
mikAud3Ctl:			.byte 0		;@ 0x3D Audio 3 Audio Control Bits
mikAud3Count:		.byte 0		;@ 0x3E Audio 3 Counter
mikAud3Misc:		.byte 0		;@ 0x3F Audio 3 Other Bits

		// Lynx2 Regs
mikAttenA:			.byte 0		;@ 0x40
mikAttenB:			.byte 0		;@ 0x41
mikAttenC:			.byte 0		;@ 0x42
mikAttenD:			.byte 0		;@ 0x43
mikMPAN:			.byte 0		;@ 0x44
mikPadding0:		.space 0x0B	;@ 0x45-0x4F

mikMStereo:			.byte 0		;@ 0x50 MSTEREO
mikPadding1:		.space 0x2F	;@ 0x51-0x7F

mikIntRst:			.byte 0		;@ 0x80 Interrupt Reset
mikIntSet:			.byte 0		;@ 0x81 Interrupt Set
mikPadding2:		.space 0x02	;@ 0x82-0x83
mikMagRdy0:			.byte 0		;@ 0x84 Mag Tape Channel 0 Ready bit
mikMagRdy1:			.byte 0		;@ 0x85 Mag Tape Channel 1 Ready bit
mikAudIn:			.byte 0		;@ 0x86 Audio In
mikSysCtl1:			.byte 0		;@ 0x87 System Control 1
mikMikeyHRev:		.byte 0		;@ 0x88 Mikey Hardware Revision
mikMikeySRev:		.byte 0		;@ 0x89 Mikey Software Revision
mikIODir:			.byte 0		;@ 0x8A IO Direction
mikIODat:			.byte 0		;@ 0x8B IO Data
mikSerCtl:			.byte 0		;@ 0x8C Serial Control
mikSerDat:			.byte 0		;@ 0x8D Serial Data
mikPadding3:		.space 0x02	;@ 0x8E-0x8F

mikSDoneAck:		.byte 0		;@ 0x90 Suzy Done Acknowledge
mikCpuSleep:		.byte 0		;@ 0x91 CPU Sleep, Bus Request Disable
mikDispCtl:			.byte 0		;@ 0x92 Display Control,  Video Bus Reguest Enable.
mikPBkup:			.byte 0		;@ 0x93 P Backup, Magic 'P' count
mikDispAdr:
mikDispAdrL:		.byte 0		;@ 0x94 Display Address Low
mikDispAdrH:		.byte 0		;@ 0x95 Display Address High
mikPadding4:		.space 0x06	;@ 0x96-0x9B
mikMtest0:			.byte 0		;@ 0x9C Mtest0
mikMtest1:			.byte 0		;@ 0x9D Mtest1
mikMtest2:			.byte 0		;@ 0x9E Mtest2
mikPadding5:		.space 0x01	;@ 0x9F

mikPaletteG:		.space 0x10	;@ 0xA0-0xAF Green palette
mikPaletteBR:		.space 0x10	;@ 0xB0-0xBF Blue/Red palette

;@----------------------------------------------------------------------------
mikPalette:			.space 16*4	;@ Merged palette
mikLCDVSize:		.byte 0		;@ 0x01 LCD Vertical Size
mikSOC:				.byte 0		;@ HOWARD or HOWARD2
					.space 2

mikeyStateEnd:

mikNmiFunction:		.long 0		;@ NMI function
mikIrqFunction:		.long 0		;@ IRQ function

mikGfxRAM:			.long 0		;@ 0x10000

mikeySize:

;@----------------------------------------------------------------------------

