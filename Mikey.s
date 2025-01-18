//
//  Mikey.s
//  Atari Lynx Mikey emulation for ARM32.
//
//  Created by Fredrik Ahlström on 2024-10-14.
//  Copyright © 2024-2025 Fredrik Ahlström. All rights reserved.
//

#ifdef __arm__

#include "ARMMikey.i"
#include "ARM6502/M6502mac.h"
#include "../LynxCart/LynxCart.i"

	.global mikeyInit
	.global mikeyReset
	.global mikeySaveState
	.global mikeyLoadState
	.global mikeyGetStateSize
	.global ComLynxCable
	.global ComLynxRxData
	.global ComLynxTxCallback
	.global mikSysUpdate
	.global mikeyRead
	.global mikeyWrite
	.global miPBackupW

	.syntax unified
	.arm

#if GBA
	.section .ewram, "ax", %progbits	;@ For the GBA
#else
	.section .text						;@ For anything else
#endif
	.align 2
;@----------------------------------------------------------------------------
mikeyInit:				;@ Only need to be called once
;@----------------------------------------------------------------------------
	mov r0,mikptr
	ldr r1,=mikeySize/4
	b memclr_					;@ Clear Mikey object
	bx lr
;@----------------------------------------------------------------------------
mikeyReset:				;@ r10=mikptr
;@----------------------------------------------------------------------------
	stmfd sp!,{r0-r4,lr}

	add r0,mikptr,#mikeyState
	ldr r1,=mikeyStateSize/4
	bl memclr_					;@ Clear Mikey state

	mov r4,#0x1F
palClrLoop:
	ldr r0,=0xFDA0
	mov r1,#0xFF
	add r0,r0,r4
	bl mikeyWrite
	subs r4,r4,#1
	bpl palClrLoop

	mov r0,#0xFF
	strb r0,[mikptr,#mikStereo]	;@ All channels enabled, (reg is inverted)

	mov r0,#UART_TX_INACTIVE
	str r0,[mikptr,#uart_TX_COUNTDOWN]
	mov r0,#UART_RX_INACTIVE
	str r0,[mikptr,#uart_RX_COUNTDOWN]

	ldmfd sp!,{r0-r4,lr}
	cmp r0,#0
	adreq r0,dummyFunc
	str r0,[mikptr,#mikLineCallback]
	cmp r1,#0
	adreq r1,dummyFunc
	str r1,[mikptr,#mikFrameCallback]

	str r2,[mikptr,#mikGfxRAM]

	strb r3,[mikptr,#mikSOC]

	ldr r0,=159*105*16			;@ Cycle count for 60Hz frame.
	str r0,[mikptr,#mikCyclesPerFrame]

	ldr r0,=suzy_0
	str r0,[mikptr,#mikSuzyPtr]
	ldr r0,=cart_0
	str r0,[mikptr,#mikCartPtr]

dummyFunc:
	bx lr
;@----------------------------------------------------------------------------
_debugIOUnmappedR:
;@----------------------------------------------------------------------------
	ldr r3,=debugIOUnmappedR
	bx r3
;@----------------------------------------------------------------------------
_debugIOUnimplR:
;@----------------------------------------------------------------------------
	ldr r3,=debugIOUnimplR
	bx r3
;@----------------------------------------------------------------------------
_debugIOUnmappedW:
;@----------------------------------------------------------------------------
	ldr r3,=debugIOUnmappedW
	bx r3
;@----------------------------------------------------------------------------
_debugIOUnimplW:
;@----------------------------------------------------------------------------
	ldr r3,=debugIOUnimplW
	bx r3
;@----------------------------------------------------------------------------
memCopy:
;@----------------------------------------------------------------------------
	ldr r3,=memcpy
;@----------------------------------------------------------------------------
thumbCallR3:
;@----------------------------------------------------------------------------
	bx r3

;@----------------------------------------------------------------------------
mikeySaveState:			;@ In r0=destination, r1=mikptr. Out r0=state size.
	.type	mikeySaveState STT_FUNC
;@----------------------------------------------------------------------------
	stmfd sp!,{r4,r5,lr}
	mov r4,r0					;@ Store destination
	mov r5,r1					;@ Store mikptr (r1)

	add r1,r5,#mikeyState
	mov r2,#mikeyStateSize
	bl memCopy

	ldmfd sp!,{r4,r5,lr}
	mov r0,#mikeyStateSize
	bx lr
;@----------------------------------------------------------------------------
mikeyLoadState:			;@ In r0=mikptr, r1=source. Out r0=state size.
	.type	mikeyLoadState STT_FUNC
;@----------------------------------------------------------------------------
	stmfd sp!,{r4,r5,lr}
	mov r5,r0					;@ Store mikptr (r0)
	mov r4,r1					;@ Store source

	add r0,r5,#mikeyState
	mov r2,#mikeyStateSize
	bl memCopy

	mov r0,#1
	strb r0,[r5,paletteChanged]

	ldmfd sp!,{r4,r5,lr}
;@----------------------------------------------------------------------------
mikeyGetStateSize:		;@ Out r0=state size.
	.type	mikeyGetStateSize STT_FUNC
;@----------------------------------------------------------------------------
	mov r0,#mikeyStateSize
	bx lr

	.pool
;@----------------------------------------------------------------------------
mikeyRead:					;@ I/O read (0xFD00-0xFDC0)
;@----------------------------------------------------------------------------
	sub r2,r0,#0xFD00
	cmp r2,#0xC0
	ldrmi pc,[pc,r2,lsl#2]
	b miUnmappedR
io_read_tbl:
	.long miRegR				;@ 0xFD00 TIM0BKUP
	.long miRegR				;@ 0xFD01 TIM0CTLA
	.long miTimXCntR			;@ 0xFD02 TIM0CNT
	.long miRegR				;@ 0xFD03 TIM0CTLB
	.long miRegR				;@ 0xFD04 TIM1BKUP
	.long miRegR				;@ 0xFD05 TIM1CTLA
	.long miTimXCntR			;@ 0xFD06 TIM1CNT
	.long miRegR				;@ 0xFD07 TIM1CTLB
	.long miRegR				;@ 0xFD08 TIM2BKUP
	.long miRegR				;@ 0xFD09 TIM2CTLA
	.long miTimXCntR			;@ 0xFD0A TIM2CNT
	.long miRegR				;@ 0xFD0B TIM2CTLB
	.long miRegR				;@ 0xFD0C TIM3BKUP
	.long miRegR				;@ 0xFD0D TIM3CTLA
	.long miTimXCntR			;@ 0xFD0E TIM3CNT
	.long miRegR				;@ 0xFD0F TIM3CTLB

	.long miRegR				;@ 0xFD10 TIM4BKUP
	.long miRegR				;@ 0xFD11 TIM4CTLA
	.long miTimXCntR			;@ 0xFD12 TIM4CNT
	.long miRegR				;@ 0xFD13 TIM4CTLB
	.long miRegR				;@ 0xFD14 TIM5BKUP
	.long miRegR				;@ 0xFD15 TIM5CTLA
	.long miTimXCntR			;@ 0xFD16 TIM5CNT
	.long miRegR				;@ 0xFD17 TIM5CTLB
	.long miRegR				;@ 0xFD18 TIM6BKUP
	.long miRegR				;@ 0xFD19 TIM6CTLA
	.long miTimXCntR			;@ 0xFD1A TIM6CNT
	.long miRegR				;@ 0xFD1B TIM6CTLB
	.long miRegR				;@ 0xFD1C TIM7BKUP
	.long miRegR				;@ 0xFD1D TIM7CTLA
	.long miTimXCntR			;@ 0xFD1E TIM7CNT
	.long miRegR				;@ 0xFD1F TIM7CTLB

	.long miRegR				;@ 0xFD20 AUD0VOL
	.long miRegR				;@ 0xFD21 AUD0SHFTFB
	.long miRegR				;@ 0xFD22 AUD0OUTVAL
	.long miAud0L8ShftR			;@ 0xFD23 AUD0L8SHFT
	.long miRegR				;@ 0xFD24 AUD0TBACK
	.long miRegR				;@ 0xFD25 AUD0CTL
	.long miRegR				;@ 0xFD26 AUD0COUNT
	.long miAud0MiscR			;@ 0xFD27 AUD0MISC
	.long miRegR				;@ 0xFD28 AUD1VOL
	.long miRegR				;@ 0xFD29 AUD1SHFTFB
	.long miRegR				;@ 0xFD2A AUD1OUTVAL
	.long miAud1L8ShftR			;@ 0xFD2B AUD1L8SHFT
	.long miRegR				;@ 0xFD2C AUD1TBACK
	.long miRegR				;@ 0xFD2D AUD1CTL
	.long miRegR				;@ 0xFD2E AUD1COUNT
	.long miAud1MiscR			;@ 0xFD2F AUD1MISC

	.long miRegR				;@ 0xFD30 AUD2VOL
	.long miRegR				;@ 0xFD31 AUD2SHFTFB
	.long miRegR				;@ 0xFD32 AUD2OUTVAL
	.long miAud2L8ShftR			;@ 0xFD33 AUD2L8SHFT
	.long miRegR				;@ 0xFD34 AUD2TBACK
	.long miRegR				;@ 0xFD35 AUD2CTL
	.long miRegR				;@ 0xFD36 AUD2COUNT
	.long miAud2MiscR			;@ 0xFD37 AUD2MISC
	.long miRegR				;@ 0xFD38 AUD3VOL
	.long miRegR				;@ 0xFD39 AUD3SHFTFB
	.long miRegR				;@ 0xFD3A AUD3OUTVAL
	.long miAud3L8ShftR			;@ 0xFD3B AUD3L8SHFT
	.long miRegR				;@ 0xFD3C AUD3TBACK
	.long miRegR				;@ 0xFD3D AUD3CTL
	.long miRegR				;@ 0xFD3E AUD3COUNT
	.long miAud3MiscR			;@ 0xFD3F AUD3MISC

		// Lynx2 Regs
	.long miImportantR			;@ 0xFD40 ATTEN_A
	.long miImportantR			;@ 0xFD41 ATTEN_B
	.long miImportantR			;@ 0xFD42 ATTEN_C
	.long miImportantR			;@ 0xFD43 ATTEN_D
	.long miImportantR			;@ 0xFD44 PAN
	.long miUnmappedR			;@ 0xFD45
	.long miUnmappedR			;@ 0xFD46
	.long miUnmappedR			;@ 0xFD47
	.long miUnmappedR			;@ 0xFD48
	.long miUnmappedR			;@ 0xFD49
	.long miUnmappedR			;@ 0xFD4A
	.long miUnmappedR			;@ 0xFD4B
	.long miUnmappedR			;@ 0xFD4C
	.long miUnmappedR			;@ 0xFD4D
	.long miUnmappedR			;@ 0xFD4E
	.long miUnmappedR			;@ 0xFD4F

	.long miStereoR				;@ 0xFD50 STEREO
	.long miUnmappedR			;@ 0xFD51
	.long miUnmappedR			;@ 0xFD52
	.long miUnmappedR			;@ 0xFD53
	.long miUnmappedR			;@ 0xFD54
	.long miUnmappedR			;@ 0xFD55
	.long miUnmappedR			;@ 0xFD56
	.long miUnmappedR			;@ 0xFD57
	.long miUnmappedR			;@ 0xFD58
	.long miUnmappedR			;@ 0xFD59
	.long miUnmappedR			;@ 0xFD5A
	.long miUnmappedR			;@ 0xFD5B
	.long miUnmappedR			;@ 0xFD5C
	.long miUnmappedR			;@ 0xFD5D
	.long miUnmappedR			;@ 0xFD5E
	.long miUnmappedR			;@ 0xFD5F

	.long miUnmappedR			;@ 0xFD60
	.long miUnmappedR			;@ 0xFD61
	.long miUnmappedR			;@ 0xFD62
	.long miUnmappedR			;@ 0xFD63
	.long miUnmappedR			;@ 0xFD64
	.long miUnmappedR			;@ 0xFD65
	.long miUnmappedR			;@ 0xFD66
	.long miUnmappedR			;@ 0xFD67
	.long miUnmappedR			;@ 0xFD68
	.long miUnmappedR			;@ 0xFD69
	.long miUnmappedR			;@ 0xFD6A
	.long miUnmappedR			;@ 0xFD6B
	.long miUnmappedR			;@ 0xFD6C
	.long miUnmappedR			;@ 0xFD6D
	.long miUnmappedR			;@ 0xFD6E
	.long miUnmappedR			;@ 0xFD6F

	.long miUnmappedR			;@ 0xFD70
	.long miUnmappedR			;@ 0xFD71
	.long miUnmappedR			;@ 0xFD72
	.long miUnmappedR			;@ 0xFD73
	.long miUnmappedR			;@ 0xFD74
	.long miUnmappedR			;@ 0xFD75
	.long miUnmappedR			;@ 0xFD76
	.long miUnmappedR			;@ 0xFD77
	.long miUnmappedR			;@ 0xFD78
	.long miUnmappedR			;@ 0xFD79
	.long miUnmappedR			;@ 0xFD7A
	.long miUnmappedR			;@ 0xFD7B
	.long miUnmappedR			;@ 0xFD7C
	.long miUnmappedR			;@ 0xFD7D
	.long miUnmappedR			;@ 0xFD7E
	.long miUnmappedR			;@ 0xFD7F

	.long miIntRstR				;@ 0xFD80 INTRST
	.long miIntSetR				;@ 0xFD81 INTSET
	.long miUnmappedR			;@ 0xFD82
	.long miUnmappedR			;@ 0xFD83
	.long miMagRdy0R			;@ 0xFD84 MAGRDY0
	.long miMagRdy1R			;@ 0xFD85 MAGRDY1
	.long miAudInR				;@ 0xFD86 AUDIN
	.long miWriteOnlyR			;@ 0xFD87 SYSCTL1
	.long miMikeyHRevR			;@ 0xFD88 MIKEYHREV
	.long miWriteOnlyR			;@ 0xFD89 MIKEYSREV
	.long miWriteOnlyR			;@ 0xFD8A IODIR
	.long miIODatR				;@ 0xFD8B IODAT
	.long miSerCtlR				;@ 0xFD8C SERCTL
	.long miSerDatR				;@ 0xFD8D SERDAT
	.long miUnmappedR			;@ 0xFD8E
	.long miUnmappedR			;@ 0xFD8F

	.long miWriteOnlyR			;@ 0xFD90 SDONEACK
	.long miWriteOnlyR			;@ 0xFD91 CPUSLEEP
	.long miWriteOnlyR			;@ 0xFD92 DISPCTL
	.long miWriteOnlyR			;@ 0xFD93 PBKUP
	.long miWriteOnlyR			;@ 0xFD94 DISPADR/DISPADRL
	.long miWriteOnlyR			;@ 0xFD95 DISPADRH
	.long miUnmappedR			;@ 0xFD96
	.long miHandyDetectR		;@ 0xFD97 Handy Detect
	.long miUnmappedR			;@ 0xFD98
	.long miUnmappedR			;@ 0xFD99
	.long miUnmappedR			;@ 0xFD9A
	.long miUnmappedR			;@ 0xFD9B
	.long miWriteOnlyR			;@ 0xFD9C Mtest0
	.long miWriteOnlyR			;@ 0xFD9D Mtest1
	.long miWriteOnlyR			;@ 0xFD9E Mtest2
	.long miUnmappedR			;@ 0xFD9F

	.long miRegR				;@ 0xFDA0 GREEN0
	.long miRegR				;@ 0xFDA1 GREEN1
	.long miRegR				;@ 0xFDA2 GREEN2
	.long miRegR				;@ 0xFDA3 GREEN3
	.long miRegR				;@ 0xFDA4 GREEN4
	.long miRegR				;@ 0xFDA5 GREEN5
	.long miRegR				;@ 0xFDA6 GREEN6
	.long miRegR				;@ 0xFDA7 GREEN7
	.long miRegR				;@ 0xFDA8 GREEN8
	.long miRegR				;@ 0xFDA9 GREEN9
	.long miRegR				;@ 0xFDAA GREENA
	.long miRegR				;@ 0xFDAB GREENB
	.long miRegR				;@ 0xFDAC GREENC
	.long miRegR				;@ 0xFDAD GREEND
	.long miRegR				;@ 0xFDAE GREENE
	.long miRegR				;@ 0xFDAF GREENF

	.long miRegR				;@ 0xFDB0 BLUERED0
	.long miRegR				;@ 0xFDB1 BLUERED1
	.long miRegR				;@ 0xFDB2 BLUERED2
	.long miRegR				;@ 0xFDB3 BLUERED3
	.long miRegR				;@ 0xFDB4 BLUERED4
	.long miRegR				;@ 0xFDB5 BLUERED5
	.long miRegR				;@ 0xFDB6 BLUERED6
	.long miRegR				;@ 0xFDB7 BLUERED7
	.long miRegR				;@ 0xFDB8 BLUERED8
	.long miRegR				;@ 0xFDB9 BLUERED9
	.long miRegR				;@ 0xFDBA BLUEREDA
	.long miRegR				;@ 0xFDBB BLUEREDB
	.long miRegR				;@ 0xFDBC BLUEREDC
	.long miRegR				;@ 0xFDBD BLUEREDD
	.long miRegR				;@ 0xFDBE BLUEREDE
	.long miRegR				;@ 0xFDBF BLUEREDF

;@----------------------------------------------------------------------------
miWriteOnlyR:
;@----------------------------------------------------------------------------
;@----------------------------------------------------------------------------
miUnmappedR:
;@----------------------------------------------------------------------------
	mov r11,r11					;@ No$GBA breakpoint
	stmfd sp!,{mikptr,lr}
	bl _debugIOUnmappedR
	ldmfd sp!,{mikptr,lr}
	mov r0,#0xFF
	bx lr
;@----------------------------------------------------------------------------
miUnknownR:
;@----------------------------------------------------------------------------
	ldr r1,=0x826EBAD0
;@----------------------------------------------------------------------------
miImportantR:
	mov r11,r11					;@ No$GBA breakpoint
	stmfd sp!,{r2,mikptr,lr}
	bl _debugIOUnimplR
	ldmfd sp!,{r2,mikptr,lr}
;@----------------------------------------------------------------------------
miRegR:
	add r2,r2,#mikRegs
	ldrb r0,[mikptr,r2]
	bx lr
	.pool

;@----------------------------------------------------------------------------
miTimXCntR:					;@ Timer X Count (0xFDX2/6/A/E)
;@----------------------------------------------------------------------------
	stmfd sp!,{r2,r4,lr}
	ldr r4,[mikptr,#systemCycleCount]
	bl mikUpdate				;@ returns consumed cycles
	mov r0,r0,lsr#2
	sub cycles,cycles,r0,lsl#CYC_SHIFT
	ldmfd sp!,{r2,r4,lr}
	b miRegR

;@----------------------------------------------------------------------------
miAud0L8ShftR:				;@ Audio 0 L8 Shift (0xFD23)
;@----------------------------------------------------------------------------
	ldrb r0,[mikptr,#audio0+WAVESHAPER+2]
	bx lr
;@----------------------------------------------------------------------------
miAud1L8ShftR:				;@ Audio 1 L8 Shift (0xFD2B)
;@----------------------------------------------------------------------------
	ldrb r0,[mikptr,#audio1+WAVESHAPER+2]
	bx lr
;@----------------------------------------------------------------------------
miAud2L8ShftR:				;@ Audio 2 L8 Shift (0xFD33)
;@----------------------------------------------------------------------------
	ldrb r0,[mikptr,#audio2+WAVESHAPER+2]
	bx lr
;@----------------------------------------------------------------------------
miAud3L8ShftR:				;@ Audio 3 L8 Shift (0xFD3B)
;@----------------------------------------------------------------------------
	ldrb r0,[mikptr,#audio3+WAVESHAPER+2]
	bx lr

;@----------------------------------------------------------------------------
miAud0MiscR:				;@ Audio 0 Misc (0xFD27)
;@----------------------------------------------------------------------------
	ldrb r0,[mikptr,#mikAud0Misc]
	ldrb r1,[mikptr,#audio0+WAVESHAPER+3]
	and r0,r0,#0x0F
	and r1,r1,#0x0F
	orr r0,r0,r1,lsl#4
	bx lr
;@----------------------------------------------------------------------------
miAud1MiscR:				;@ Audio 1 Misc (0xFD2F)
;@----------------------------------------------------------------------------
	ldrb r0,[mikptr,#mikAud1Misc]
	ldrb r1,[mikptr,#audio1+WAVESHAPER+3]
	and r0,r0,#0x0F
	and r1,r1,#0x0F
	orr r0,r0,r1,lsl#4
	bx lr
;@----------------------------------------------------------------------------
miAud2MiscR:				;@ Audio 2 Misc (0xFD37)
;@----------------------------------------------------------------------------
	ldrb r0,[mikptr,#mikAud2Misc]
	ldrb r1,[mikptr,#audio2+WAVESHAPER+3]
	and r0,r0,#0x0F
	and r1,r1,#0x0F
	orr r0,r0,r1,lsl#4
	bx lr
;@----------------------------------------------------------------------------
miAud3MiscR:				;@ Audio 3 Misc (0xFD3F)
;@----------------------------------------------------------------------------
	ldrb r0,[mikptr,#mikAud3Misc]
	ldrb r1,[mikptr,#audio3+WAVESHAPER+3]
	and r0,r0,#0x0F
	and r1,r1,#0x0F
	orr r0,r0,r1,lsl#4
	bx lr

;@----------------------------------------------------------------------------
miStereoR:					;@ Stereo (0xFD50)
;@----------------------------------------------------------------------------
	ldrb r0,[mikptr,#mikStereo]
	eor r0,r0,#0xFF				;@ Stereo is inverted
	bx lr
;@----------------------------------------------------------------------------
miIntRstR:					;@ Interrupt bits (0xFD80)
;@----------------------------------------------------------------------------
;@----------------------------------------------------------------------------
miIntSetR:					;@ Interrupt bits (0xFD81)
;@----------------------------------------------------------------------------
	ldrb r0,[mikptr,#timerStatusFlags]
	bx lr
;@----------------------------------------------------------------------------
miMagRdy0R:					;@ Magnetic Tape ready 0 (0xFD84)
miMagRdy1R:					;@ Magnetic Tape ready 1 (0xFD85)
;@----------------------------------------------------------------------------
	mov r0,#0
	bx lr
;@----------------------------------------------------------------------------
miAudInR:					;@ (0xFD86)
;@----------------------------------------------------------------------------
	mov r0,#0x80				;@ bit 7 = audio comparator result. magnetic tape?
	bx lr
;@----------------------------------------------------------------------------
miMikeyHRevR:				;@ (0xFD88)
;@----------------------------------------------------------------------------
	mov r0,#1
	bx lr
;@----------------------------------------------------------------------------
miIODatR:					;@ IO-Data (0xFD8B)
;@----------------------------------------------------------------------------
	ldrb r0,[mikptr,#mikIODat]
	ldrb r1,[mikptr,#mikIODir]
	and r0,r0,r1				;@ Keep bits set for output.
	ldrb r2,[mikptr,#mikSerCablePresent]
	cmp r2,#0
	mov r2,#0x11				;@ Default inputs audio in & power in
	orrne r2,r2,#0x04
	bic r2,r2,r1
	orr r0,r0,r2
	ldrb r2,[mikptr,#ioDatRestSignal]
	cmp r2,#0
	bicne r0,r0,#0x08

	bx lr
;@----------------------------------------------------------------------------
miSerCtlR:					;@ Serial Control (0xFD8C)
;@----------------------------------------------------------------------------
	ldr r0,[mikptr,#uart_TX_COUNTDOWN]
	ands r0,r0,#UART_TX_INACTIVE
	movne r0,#0xA0				;@ Indicate TxDone & TxAllDone
	ldrb r1,[mikptr,#uart_RX_READY]
	cmp r1,#0
	orrne r0,r0,#0x40			;@ Indicate Rx data ready
	ldrb r1,[mikptr,#uart_Rx_overun_error]
	cmp r1,#0
	orrne r0,r0,#0x08			;@  Framing error
	ldrb r1,[mikptr,#uart_Rx_framing_error]
	cmp r1,#0
	orrne r0,r0,#0x04			;@  Rx overrun
	ldr r1,[mikptr,#uart_RX_DATA]
	tst r1,#UART_BREAK_CODE
	orrne r0,r0,#0x02			;@ Indicate break received
	tst r1,#0x100
	orrne r0,r0,#0x01			;@ Add parity bit
	bx lr
;@----------------------------------------------------------------------------
miSerDatR:					;@ Serial Data (0xFD8D)
;@----------------------------------------------------------------------------
	mov r1,#0
	ldrb r0,[mikptr,#uart_RX_DATA]
	strb r1,[mikptr,#uart_RX_READY]
	bx lr
;@----------------------------------------------------------------------------
miHandyDetectR:				;@ Handy detection register (0xFD97)
;@----------------------------------------------------------------------------
	mov r0,#42
	bx lr
;@----------------------------------------------------------------------------
mikeyWrite:					;@ I/O write (0xFD00-0xFDC0)
;@----------------------------------------------------------------------------
	sub r2,r0,#0xFD00
	cmp r2,#0xC0
	ldrmi pc,[pc,r2,lsl#2]
	b miUnmappedW
io_write_tbl:

	.long miRegW				;@ 0xFD00 TIM0BKUP
	.long miTim0CtlAW			;@ 0xFD01 TIM0CTLA
	.long miTim0CntW			;@ 0xFD02 TIM0CNT
	.long miTimCtlBW			;@ 0xFD03 TIM0CTLB
	.long miRegW				;@ 0xFD04 TIM1BKUP
	.long miTim1CtlAW			;@ 0xFD05 TIM1CTLA
	.long miTim1CntW			;@ 0xFD06 TIM1CNT
	.long miTimCtlBW			;@ 0xFD07 TIM1CTLB
	.long miRegW				;@ 0xFD08 TIM2BKUP
	.long miTim2CtlAW			;@ 0xFD09 TIM2CTLA
	.long miTim2CntW			;@ 0xFD0A TIM2CNT
	.long miTimCtlBW			;@ 0xFD0B TIM2CTLB
	.long miRegW				;@ 0xFD0C TIM3BKUP
	.long miTim3CtlAW			;@ 0xFD0D TIM3CTLA
	.long miTim3CntW			;@ 0xFD0E TIM3CNT
	.long miTimCtlBW			;@ 0xFD0F TIM3CTLB

	.long miRegW				;@ 0xFD10 TIM4BKUP
	.long miTim4CtlAW			;@ 0xFD11 TIM4CTLA
	.long miTim4CntW			;@ 0xFD12 TIM4CNT
	.long miTimCtlBW			;@ 0xFD13 TIM4CTLB
	.long miRegW				;@ 0xFD14 TIM5BKUP
	.long miTim5CtlAW			;@ 0xFD15 TIM5CTLA
	.long miTim5CntW			;@ 0xFD16 TIM5CNT
	.long miTimCtlBW			;@ 0xFD17 TIM5CTLB
	.long miRegW				;@ 0xFD18 TIM6BKUP
	.long miTim6CtlAW			;@ 0xFD19 TIM6CTLA
	.long miTim6CntW			;@ 0xFD1A TIM6CNT
	.long miTimCtlBW			;@ 0xFD1B TIM6CTLB
	.long miRegW				;@ 0xFD1C TIM7BKUP
	.long miTim7CtlAW			;@ 0xFD1D TIM7CTLA
	.long miTim7CntW			;@ 0xFD1E TIM7CNT
	.long miTimCtlBW			;@ 0xFD1F TIM7CTLB

	.long miAud0VolW			;@ 0xFD20 AUD0VOL
	.long miAud0ShftFbW			;@ 0xFD21 AUD0SHFTFB
	.long miRegW				;@ 0xFD22 AUD0OUTVAL
	.long miAud0L8ShftW			;@ 0xFD23 AUD0L8SHFT
	.long miRegW				;@ 0xFD24 AUD0TBACK
	.long miAud0CtlW			;@ 0xFD25 AUD0CTL
	.long miAud0CountW			;@ 0xFD26 AUD0COUNT
	.long miAud0MiscW			;@ 0xFD27 AUD0MISC
	.long miAud1VolW			;@ 0xFD28 AUD1VOL
	.long miAud1ShftFbW			;@ 0xFD29 AUD1SHFTFB
	.long miRegW				;@ 0xFD2A AUD1OUTVAL
	.long miAud1L8ShftW			;@ 0xFD2B AUD1L8SHFT
	.long miRegW				;@ 0xFD2C AUD1TBACK
	.long miAud1CtlW			;@ 0xFD2D AUD1CTL
	.long miAud1CountW			;@ 0xFD2E AUD1COUNT
	.long miAud1MiscW			;@ 0xFD2F AUD1MISC

	.long miAud2VolW			;@ 0xFD30 AUD2VOL
	.long miAud2ShftFbW			;@ 0xFD31 AUD2SHFTFB
	.long miRegW				;@ 0xFD32 AUD2OUTVAL
	.long miAud2L8ShftW			;@ 0xFD33 AUD2L8SHFT
	.long miRegW				;@ 0xFD34 AUD2TBACK
	.long miAud2CtlW			;@ 0xFD35 AUD2CTL
	.long miAud2CountW			;@ 0xFD36 AUD2COUNT
	.long miAud2MiscW			;@ 0xFD37 AUD2MISC
	.long miAud3VolW			;@ 0xFD38 AUD3VOL
	.long miAud3ShftFbW			;@ 0xFD39 AUD3SHFTFB
	.long miRegW				;@ 0xFD3A AUD3OUTVAL
	.long miAud3L8ShftW			;@ 0xFD3B AUD3L8SHFT
	.long miRegW				;@ 0xFD3C AUD3TBACK
	.long miAud3CtlW			;@ 0xFD3D AUD3CTL
	.long miAud3CountW			;@ 0xFD3E AUD3COUNT
	.long miAud3MiscW			;@ 0xFD3F AUD3MISC

	// Lynx2 Regs
	.long miImportantW			;@ 0xFD40 ATTEN_A
	.long miImportantW			;@ 0xFD41 ATTEN_B
	.long miImportantW			;@ 0xFD42 ATTEN_C
	.long miImportantW			;@ 0xFD43 ATTEN_D
	.long miImportantW			;@ 0xFD44 PAN
	.long miUnmappedW			;@ 0xFD45
	.long miUnmappedW			;@ 0xFD46
	.long miUnmappedW			;@ 0xFD47
	.long miUnmappedW			;@ 0xFD48
	.long miUnmappedW			;@ 0xFD49
	.long miUnmappedW			;@ 0xFD4A
	.long miUnmappedW			;@ 0xFD4B
	.long miUnmappedW			;@ 0xFD4C
	.long miUnmappedW			;@ 0xFD4D
	.long miUnmappedW			;@ 0xFD4E
	.long miUnmappedW			;@ 0xFD4F

	.long miStereoW				;@ 0xFD50 STEREO
	.long miUnmappedW			;@ 0xFD51
	.long miUnmappedW			;@ 0xFD52
	.long miUnmappedW			;@ 0xFD53
	.long miUnmappedW			;@ 0xFD54
	.long miUnmappedW			;@ 0xFD55
	.long miUnmappedW			;@ 0xFD56
	.long miUnmappedW			;@ 0xFD57
	.long miUnmappedW			;@ 0xFD58
	.long miUnmappedW			;@ 0xFD59
	.long miUnmappedW			;@ 0xFD5A
	.long miUnmappedW			;@ 0xFD5B
	.long miUnmappedW			;@ 0xFD5C
	.long miUnmappedW			;@ 0xFD5D
	.long miUnmappedW			;@ 0xFD5E
	.long miUnmappedW			;@ 0xFD5F

	.long miUnmappedW			;@ 0xFD60
	.long miUnmappedW			;@ 0xFD61
	.long miUnmappedW			;@ 0xFD62
	.long miUnmappedW			;@ 0xFD63
	.long miUnmappedW			;@ 0xFD64
	.long miUnmappedW			;@ 0xFD65
	.long miUnmappedW			;@ 0xFD66
	.long miUnmappedW			;@ 0xFD67
	.long miUnmappedW			;@ 0xFD68
	.long miUnmappedW			;@ 0xFD69
	.long miUnmappedW			;@ 0xFD6A
	.long miUnmappedW			;@ 0xFD6B
	.long miUnmappedW			;@ 0xFD6C
	.long miUnmappedW			;@ 0xFD6D
	.long miUnmappedW			;@ 0xFD6E
	.long miUnmappedW			;@ 0xFD6F

	.long miUnmappedW			;@ 0xFD70
	.long miUnmappedW			;@ 0xFD71
	.long miUnmappedW			;@ 0xFD72
	.long miUnmappedW			;@ 0xFD73
	.long miUnmappedW			;@ 0xFD74
	.long miUnmappedW			;@ 0xFD75
	.long miUnmappedW			;@ 0xFD76
	.long miUnmappedW			;@ 0xFD77
	.long miUnmappedW			;@ 0xFD78
	.long miUnmappedW			;@ 0xFD79
	.long miUnmappedW			;@ 0xFD7A
	.long miUnmappedW			;@ 0xFD7B
	.long miUnmappedW			;@ 0xFD7C
	.long miUnmappedW			;@ 0xFD7D
	.long miUnmappedW			;@ 0xFD7E
	.long miUnmappedW			;@ 0xFD7F

	.long miIntRstW				;@ 0xFD80 INTRST
	.long miIntSetW				;@ 0xFD81 INTSET
	.long miUnmappedW			;@ 0xFD82
	.long miUnmappedW			;@ 0xFD83
	.long miReadOnlyW			;@ 0xFD84 MAGRDY0
	.long miReadOnlyW			;@ 0xFD85 MAGRDY1
	.long miReadOnlyW			;@ 0xFD86 AUDIN
	.long miSysCtl1W			;@ 0xFD87 SYSCTL1
	.long miReadOnlyW			;@ 0xFD88 MIKEYHREV
	.long miRegW				;@ 0xFD89 MIKEYSREV
	.long miRegW				;@ 0xFD8A IODIR
	.long miIODatW				;@ 0xFD8B IODAT
	.long miSerCtlW				;@ 0xFD8C SERCTL
	.long miSerDatW				;@ 0xFD8D SERDAT
	.long miUnmappedW			;@ 0xFD8E
	.long miUnmappedW			;@ 0xFD8F

	.long miRegW				;@ 0xFD90 SDONEACK
	.long miCpuSleepW			;@ 0xFD91 CPUSLEEP
	.long miRegW				;@ 0xFD92 DISPCTL
	.long miPBackupW			;@ 0xFD93 PBKUP
	.long miRegW				;@ 0xFD94 DISPADR/DISPADRL
	.long miRegW				;@ 0xFD95 DISPADRH
	.long miUnmappedW			;@ 0xFD96
	.long miUnmappedW			;@ 0xFD97
	.long miUnmappedW			;@ 0xFD98
	.long miUnmappedW			;@ 0xFD99
	.long miUnmappedW			;@ 0xFD9A
	.long miUnmappedW			;@ 0xFD9B
	.long miImportantW			;@ 0xFD9C Mtest0
	.long miImportantW			;@ 0xFD9D Mtest1
	.long miImportantW			;@ 0xFD9E Mtest2
	.long miUnmappedW			;@ 0xFD9F

	.long miPaletteGW			;@ 0xFDA0 GREEN0
	.long miPaletteGW			;@ 0xFDA1 GREEN1
	.long miPaletteGW			;@ 0xFDA2 GREEN2
	.long miPaletteGW			;@ 0xFDA3 GREEN3
	.long miPaletteGW			;@ 0xFDA4 GREEN4
	.long miPaletteGW			;@ 0xFDA5 GREEN5
	.long miPaletteGW			;@ 0xFDA6 GREEN6
	.long miPaletteGW			;@ 0xFDA7 GREEN7
	.long miPaletteGW			;@ 0xFDA8 GREEN8
	.long miPaletteGW			;@ 0xFDA9 GREEN9
	.long miPaletteGW			;@ 0xFDAA GREENA
	.long miPaletteGW			;@ 0xFDAB GREENB
	.long miPaletteGW			;@ 0xFDAC GREENC
	.long miPaletteGW			;@ 0xFDAD GREEND
	.long miPaletteGW			;@ 0xFDAE GREENE
	.long miPaletteGW			;@ 0xFDAF GREENF

	.long miPaletteBRW			;@ 0xFDB0 BLUERED0
	.long miPaletteBRW			;@ 0xFDB1 BLUERED1
	.long miPaletteBRW			;@ 0xFDB2 BLUERED2
	.long miPaletteBRW			;@ 0xFDB3 BLUERED3
	.long miPaletteBRW			;@ 0xFDB4 BLUERED4
	.long miPaletteBRW			;@ 0xFDB5 BLUERED5
	.long miPaletteBRW			;@ 0xFDB6 BLUERED6
	.long miPaletteBRW			;@ 0xFDB7 BLUERED7
	.long miPaletteBRW			;@ 0xFDB8 BLUERED8
	.long miPaletteBRW			;@ 0xFDB9 BLUERED9
	.long miPaletteBRW			;@ 0xFDBA BLUEREDA
	.long miPaletteBRW			;@ 0xFDBB BLUEREDB
	.long miPaletteBRW			;@ 0xFDBC BLUEREDC
	.long miPaletteBRW			;@ 0xFDBD BLUEREDD
	.long miPaletteBRW			;@ 0xFDBE BLUEREDE
	.long miPaletteBRW			;@ 0xFDBF BLUEREDF

;@----------------------------------------------------------------------------
miUnknownW:
;@----------------------------------------------------------------------------
miImportantW:
;@----------------------------------------------------------------------------
	add r2,r2,#mikRegs
	strb r1,[mikptr,r2]
	b _debugIOUnimplW
;@----------------------------------------------------------------------------
miReadOnlyW:
;@----------------------------------------------------------------------------
miUnmappedW:
;@----------------------------------------------------------------------------
	b _debugIOUnmappedW
;@----------------------------------------------------------------------------
miRegW:
	add r2,r2,#mikRegs
	strb r1,[mikptr,r2]
	bx lr

;@----------------------------------------------------------------------------
miTim0CtlAW:				;@ Timer 0 Control A (0xFD01)
;@----------------------------------------------------------------------------
	and r0,r1,#0xBF				;@ "Reset Timer Done" should not be preserved?
	strb r0,[mikptr,#mikTim0CtlA]
	ldrb r0,[mikptr,#timerInterruptMask]
	tst r1,#0x80				;@ Enable interrupt?
	biceq r0,r0,#1<<0
	orrne r0,r0,#1<<0
	strb r0,[mikptr,#timerInterruptMask]
	tst r1,#0x40				;@ Check "Reset Timer Done".
	ldrbne r0,[mikptr,#mikTim0CtlB]
	bicne r0,r0,#0x08			;@ Timer done, in CtlB.
	strbne r0,[mikptr,#mikTim0CtlB]
	tst r1,#0x48				;@ Enable Count/ Reset Timer Done?
	bxeq lr
	ldr r0,[mikptr,#systemCycleCount]
	str r0,[mikptr,#timer0+LAST_COUNT]
	str r0,[mikptr,#nextTimerEvent]
	m6502BailOut
	bx lr
;@----------------------------------------------------------------------------
miTim1CtlAW:				;@ Timer 1 Control A (0xFD05)
;@----------------------------------------------------------------------------
	and r0,r1,#0xBF				;@ "Reset Timer Done" should not be preserved?
	strb r0,[mikptr,#mikTim1CtlA]
	ldrb r0,[mikptr,#timerInterruptMask]
	tst r1,#0x80				;@ Enable interrupt?
	biceq r0,r0,#1<<1
	orrne r0,r0,#1<<1
	strb r0,[mikptr,#timerInterruptMask]
	tst r1,#0x40				;@ Check "Reset Timer Done".
	ldrbne r0,[mikptr,#mikTim1CtlB]
	bicne r0,r0,#0x08			;@ Timer done, in CtlB.
	strbne r0,[mikptr,#mikTim1CtlB]
	tst r1,#0x48				;@ Enable Count/ Reset Timer Done?
	bxeq lr
	ldr r0,[mikptr,#systemCycleCount]
	str r0,[mikptr,#timer1+LAST_COUNT]
	str r0,[mikptr,#nextTimerEvent]
	m6502BailOut
	bx lr
;@----------------------------------------------------------------------------
miTim2CtlAW:				;@ Timer 2 Control A (0xFD09)
;@----------------------------------------------------------------------------
	and r0,r1,#0xBF				;@ "Reset Timer Done" should not be preserved?
	strb r0,[mikptr,#mikTim2CtlA]
	ldrb r0,[mikptr,#timerInterruptMask]
	tst r1,#0x80				;@ Enable interrupt?
	biceq r0,r0,#1<<2
	orrne r0,r0,#1<<2
	strb r0,[mikptr,#timerInterruptMask]
	tst r1,#0x40				;@ Check "Reset Timer Done".
	ldrbne r0,[mikptr,#mikTim2CtlB]
	bicne r0,r0,#0x08			;@ Timer done, in CtlB.
	strbne r0,[mikptr,#mikTim2CtlB]
	tst r1,#0x48				;@ Enable Count/ Reset Timer Done?
	bxeq lr
	ldr r0,[mikptr,#systemCycleCount]
	str r0,[mikptr,#timer2+LAST_COUNT]
	str r0,[mikptr,#nextTimerEvent]
	m6502BailOut
	bx lr
;@----------------------------------------------------------------------------
miTim3CtlAW:				;@ Timer 3 Control A (0xFD0D)
;@----------------------------------------------------------------------------
	and r0,r1,#0xBF				;@ "Reset Timer Done" should not be preserved?
	strb r0,[mikptr,#mikTim3CtlA]
	ldrb r0,[mikptr,#timerInterruptMask]
	tst r1,#0x80				;@ Enable interrupt?
	biceq r0,r0,#1<<3
	orrne r0,r0,#1<<3
	strb r0,[mikptr,#timerInterruptMask]
	tst r1,#0x40				;@ Check "Reset Timer Done".
	ldrbne r0,[mikptr,#mikTim3CtlB]
	bicne r0,r0,#0x08			;@ Timer done, in CtlB.
	strbne r0,[mikptr,#mikTim3CtlB]
	tst r1,#0x48				;@ Enable Count/ Reset Timer Done?
	bxeq lr
	ldr r0,[mikptr,#systemCycleCount]
	str r0,[mikptr,#timer3+LAST_COUNT]
	str r0,[mikptr,#nextTimerEvent]
	m6502BailOut
	bx lr
;@----------------------------------------------------------------------------
miTim4CtlAW:				;@ Timer 4 Control A (0xFD11)
;@----------------------------------------------------------------------------
	and r0,r1,#0xBF				;@ "Reset Timer Done" should not be preserved?
	strb r0,[mikptr,#mikTim4CtlA]
	ldrb r0,[mikptr,#timerInterruptMask]
	tst r1,#0x80				;@ Enable interrupt?
	biceq r0,r0,#1<<4
	orrne r0,r0,#1<<4
	strb r0,[mikptr,#timerInterruptMask]
	tst r1,#0x40				;@ Check "Reset Timer Done".
	ldrbne r0,[mikptr,#mikTim4CtlB]
	bicne r0,r0,#0x08			;@ Timer done, in CtlB.
	strbne r0,[mikptr,#mikTim4CtlB]
	tst r1,#0x48				;@ Enable Count/ Reset Timer Done?
	bxeq lr
	ldr r0,[mikptr,#systemCycleCount]
	str r0,[mikptr,#timer4+LAST_COUNT]
	str r0,[mikptr,#nextTimerEvent]
	m6502BailOut
	bx lr
;@----------------------------------------------------------------------------
miTim5CtlAW:				;@ Timer 5 Control A (0xFD15)
;@----------------------------------------------------------------------------
	and r0,r1,#0xBF				;@ "Reset Timer Done" should not be preserved?
	strb r0,[mikptr,#mikTim5CtlA]
	ldrb r0,[mikptr,#timerInterruptMask]
	tst r1,#0x80				;@ Enable interrupt?
	biceq r0,r0,#1<<5
	orrne r0,r0,#1<<5
	strb r0,[mikptr,#timerInterruptMask]
	tst r1,#0x40				;@ Check "Reset Timer Done".
	ldrbne r0,[mikptr,#mikTim5CtlB]
	bicne r0,r0,#0x08			;@ Timer done, in CtlB.
	strbne r0,[mikptr,#mikTim5CtlB]
	tst r1,#0x48				;@ Enable Count/ Reset Timer Done?
	bxeq lr
	ldr r0,[mikptr,#systemCycleCount]
	str r0,[mikptr,#timer5+LAST_COUNT]
	str r0,[mikptr,#nextTimerEvent]
	m6502BailOut
	bx lr
;@----------------------------------------------------------------------------
miTim6CtlAW:				;@ Timer 6 Control A (0xFD19)
;@----------------------------------------------------------------------------
	and r0,r1,#0xBF				;@ "Reset Timer Done" should not be preserved?
	strb r0,[mikptr,#mikTim6CtlA]
	ldrb r0,[mikptr,#timerInterruptMask]
	tst r1,#0x80				;@ Enable interrupt?
	biceq r0,r0,#1<<6
	orrne r0,r0,#1<<6
	strb r0,[mikptr,#timerInterruptMask]
	tst r1,#0x40				;@ Check "Reset Timer Done".
	ldrbne r0,[mikptr,#mikTim6CtlB]
	bicne r0,r0,#0x08			;@ Timer done, in CtlB.
	strbne r0,[mikptr,#mikTim6CtlB]
	tst r1,#0x48				;@ Enable Count/ Reset Timer Done?
	bxeq lr
	ldr r0,[mikptr,#systemCycleCount]
	str r0,[mikptr,#timer6+LAST_COUNT]
	str r0,[mikptr,#nextTimerEvent]
	m6502BailOut
	bx lr
;@----------------------------------------------------------------------------
miTim7CtlAW:				;@ Timer 7 Control A (0xFD1D)
;@----------------------------------------------------------------------------
	and r0,r1,#0xBF				;@ "Reset Timer Done" should not be preserved?
	strb r0,[mikptr,#mikTim7CtlA]
	ldrb r0,[mikptr,#timerInterruptMask]
	tst r1,#0x80				;@ Enable interrupt?
	biceq r0,r0,#1<<7
	orrne r0,r0,#1<<7
	strb r0,[mikptr,#timerInterruptMask]
	tst r1,#0x40				;@ Check "Reset Timer Done".
	ldrbne r0,[mikptr,#mikTim7CtlB]
	bicne r0,r0,#0x08			;@ Timer done, in CtlB.
	strbne r0,[mikptr,#mikTim7CtlB]
	tst r1,#0x48				;@ Enable Count/ Reset Timer Done?
	bxeq lr
	ldr r0,[mikptr,#systemCycleCount]
	str r0,[mikptr,#timer7+LAST_COUNT]
	str r0,[mikptr,#nextTimerEvent]
	m6502BailOut
	bx lr
;@----------------------------------------------------------------------------
miTimCtlBW:					;@ Timer X Control B (0xFDX3)
;@----------------------------------------------------------------------------
	and r1,r1,#0x0F
	add r2,r2,#mikRegs
	strb r1,[mikptr,r2]
	bx lr

;@----------------------------------------------------------------------------
miTim0CntW:					;@ Timer 0 Count (0xFD02)
;@----------------------------------------------------------------------------
	and r1,r1,#0xFF
	strb r1,[mikptr,#mikTim0Cnt]
	str r1,[mikptr,#timer0+CURRENT]
	ldr r1,[mikptr,#systemCycleCount]
	str r1,[mikptr,#nextTimerEvent]
	m6502BailOut
	bx lr
;@----------------------------------------------------------------------------
miTim1CntW:					;@ Timer 1 Count (0xFD06)
;@----------------------------------------------------------------------------
	and r1,r1,#0xFF
	strb r1,[mikptr,#mikTim1Cnt]
	str r1,[mikptr,#timer1+CURRENT]
	ldr r1,[mikptr,#systemCycleCount]
	str r1,[mikptr,#nextTimerEvent]
	m6502BailOut
	bx lr
;@----------------------------------------------------------------------------
miTim2CntW:					;@ Timer 2 Count (0xFD0A)
;@----------------------------------------------------------------------------
	and r1,r1,#0xFF
	strb r1,[mikptr,#mikTim2Cnt]
	str r1,[mikptr,#timer2+CURRENT]
	ldr r1,[mikptr,#systemCycleCount]
	str r1,[mikptr,#nextTimerEvent]
	m6502BailOut
	bx lr
;@----------------------------------------------------------------------------
miTim3CntW:					;@ Timer 3 Count (0xFD0E)
;@----------------------------------------------------------------------------
	and r1,r1,#0xFF
	strb r1,[mikptr,#mikTim3Cnt]
	str r1,[mikptr,#timer3+CURRENT]
	ldr r1,[mikptr,#systemCycleCount]
	str r1,[mikptr,#nextTimerEvent]
	m6502BailOut
	bx lr
;@----------------------------------------------------------------------------
miTim4CntW:					;@ Timer 4 Count (0xFD12)
;@----------------------------------------------------------------------------
	and r1,r1,#0xFF
	strb r1,[mikptr,#mikTim4Cnt]
	str r1,[mikptr,#timer4+CURRENT]
	ldr r1,[mikptr,#systemCycleCount]
	str r1,[mikptr,#nextTimerEvent]
	m6502BailOut
	bx lr
;@----------------------------------------------------------------------------
miTim5CntW:					;@ Timer 5 Count (0xFD16)
;@----------------------------------------------------------------------------
	and r1,r1,#0xFF
	strb r1,[mikptr,#mikTim5Cnt]
	str r1,[mikptr,#timer5+CURRENT]
	ldr r1,[mikptr,#systemCycleCount]
	str r1,[mikptr,#nextTimerEvent]
	m6502BailOut
	bx lr
;@----------------------------------------------------------------------------
miTim6CntW:					;@ Timer 6 Count (0xFD1A)
;@----------------------------------------------------------------------------
	and r1,r1,#0xFF
	strb r1,[mikptr,#mikTim6Cnt]
	str r1,[mikptr,#timer6+CURRENT]
	ldr r1,[mikptr,#systemCycleCount]
	str r1,[mikptr,#nextTimerEvent]
	m6502BailOut
	bx lr
;@----------------------------------------------------------------------------
miTim7CntW:					;@ Timer 7 Count (0xFD1E)
;@----------------------------------------------------------------------------
	and r1,r1,#0xFF
	strb r1,[mikptr,#mikTim7Cnt]
	str r1,[mikptr,#timer7+CURRENT]
	ldr r1,[mikptr,#systemCycleCount]
	str r1,[mikptr,#nextTimerEvent]
	m6502BailOut
	bx lr

;@----------------------------------------------------------------------------
miAud0VolW:					;@ Audio 0 Volume (0xFD20)
;@----------------------------------------------------------------------------
	strb r1,[mikptr,#mikAud0Vol]
	bx lr
;@----------------------------------------------------------------------------
miAud1VolW:					;@ Audio 1 Volume (0xFD28)
;@----------------------------------------------------------------------------
	strb r1,[mikptr,#mikAud1Vol]
	bx lr
;@----------------------------------------------------------------------------
miAud2VolW:					;@ Audio 2 Volume (0xFD30)
;@----------------------------------------------------------------------------
	strb r1,[mikptr,#mikAud2Vol]
	bx lr
;@----------------------------------------------------------------------------
miAud3VolW:					;@ Audio 3 Volume (0xFD38)
;@----------------------------------------------------------------------------
	strb r1,[mikptr,#mikAud3Vol]
	bx lr

;@----------------------------------------------------------------------------
miAud0ShftFbW:				;@ Audio 0 Shift Feedback (0xFD21)
;@----------------------------------------------------------------------------
	ldr r0,[mikptr,#audio0+WAVESHAPER]
	strb r1,[mikptr,#mikAud0ShftFb]
	mov r1,r1,ror#6
	bic r0,r0,#0x03F				;@ Clear bits 0-5
	bic r0,r0,#0xC00				;@ Clear bits 10-11
	orr r0,r0,r1,lsr#26				;@ Set bits 0-5
	orr r0,r0,r1,lsl#10				;@ Set bits 10-11
	str r0,[mikptr,#audio0+WAVESHAPER]
	bx lr
;@----------------------------------------------------------------------------
miAud1ShftFbW:				;@ Audio 1 Shift Feedback (0xFD29)
;@----------------------------------------------------------------------------
	ldr r0,[mikptr,#audio1+WAVESHAPER]
	strb r1,[mikptr,#mikAud1ShftFb]
	mov r1,r1,ror#6
	bic r0,r0,#0x03F				;@ Clear bits 0-5
	bic r0,r0,#0xC00				;@ Clear bits 10-11
	orr r0,r0,r1,lsr#26				;@ Set bits 0-5
	orr r0,r0,r1,lsl#10				;@ Set bits 10-11
	str r0,[mikptr,#audio1+WAVESHAPER]
	bx lr
;@----------------------------------------------------------------------------
miAud2ShftFbW:				;@ Audio 2 Shift Feedback (0xFD31)
;@----------------------------------------------------------------------------
	ldr r0,[mikptr,#audio2+WAVESHAPER]
	strb r1,[mikptr,#mikAud2ShftFb]
	mov r1,r1,ror#6
	bic r0,r0,#0x03F				;@ Clear bits 0-5
	bic r0,r0,#0xC00				;@ Clear bits 10-11
	orr r0,r0,r1,lsr#26				;@ Set bits 0-5
	orr r0,r0,r1,lsl#10				;@ Set bits 10-11
	str r0,[mikptr,#audio2+WAVESHAPER]
	bx lr
;@----------------------------------------------------------------------------
miAud3ShftFbW:				;@ Audio 3 Shift Feedback (0xFD39)
;@----------------------------------------------------------------------------
	ldr r0,[mikptr,#audio3+WAVESHAPER]
	strb r1,[mikptr,#mikAud3ShftFb]
	mov r1,r1,ror#6
	bic r0,r0,#0x03F				;@ Clear bits 0-5
	bic r0,r0,#0xC00				;@ Clear bits 10-11
	orr r0,r0,r1,lsr#26				;@ Set bits 0-5
	orr r0,r0,r1,lsl#10				;@ Set bits 10-11
	str r0,[mikptr,#audio3+WAVESHAPER]
	bx lr

;@----------------------------------------------------------------------------
miAud0L8ShftW:				;@ Audio 0 L8 Shift (0xFD23)
;@----------------------------------------------------------------------------
	strb r1,[mikptr,#audio0+WAVESHAPER+2]
	bx lr
;@----------------------------------------------------------------------------
miAud1L8ShftW:				;@ Audio 1 L8 Shift (0xFD2B)
;@----------------------------------------------------------------------------
	strb r1,[mikptr,#audio1+WAVESHAPER+2]
	bx lr
;@----------------------------------------------------------------------------
miAud2L8ShftW:				;@ Audio 2 L8 Shift (0xFD33)
;@----------------------------------------------------------------------------
	strb r1,[mikptr,#audio2+WAVESHAPER+2]
	bx lr
;@----------------------------------------------------------------------------
miAud3L8ShftW:				;@ Audio 3 L8 Shift (0xFD3B)
;@----------------------------------------------------------------------------
	strb r1,[mikptr,#audio3+WAVESHAPER+2]
	bx lr

;@----------------------------------------------------------------------------
miAud0CtlW:					;@ Audio 0 Control (0xFD25)
;@----------------------------------------------------------------------------
	and r0,r1,#0xBF				;@ "Reset Timer Done" should not be preserved?
	strb r0,[mikptr,#mikAud0Ctl]
	ldrb r0,[mikptr,#audio0+WAVESHAPER]
	tst r1,#0x80				;@ Waveshaper bit on/off
	biceq r0,r0,#0x80
	orrne r0,r0,#0x80
	strb r0,[mikptr,#audio0+WAVESHAPER]
	tst r1,#0x40				;@ Check "Reset Timer Done".
	ldrbne r0,[mikptr,#mikAud0Misc]
	bicne r0,r0,#0x08			;@ Timer done, in CtlB.
	strbne r0,[mikptr,#mikAud0Misc]
	bx lr
;@----------------------------------------------------------------------------
miAud1CtlW:					;@ Audio 1 Control (0xFD2D)
;@----------------------------------------------------------------------------
	and r0,r1,#0xBF				;@ "Reset Timer Done" should not be preserved?
	strb r0,[mikptr,#mikAud1Ctl]
	ldrb r0,[mikptr,#audio1+WAVESHAPER]
	tst r1,#0x80				;@ Waveshaper bit on/off
	biceq r0,r0,#0x80
	orrne r0,r0,#0x80
	strb r0,[mikptr,#audio1+WAVESHAPER]
	tst r1,#0x40				;@ Check "Reset Timer Done".
	ldrbne r0,[mikptr,#mikAud1Misc]
	bicne r0,r0,#0x08			;@ Timer done, in CtlB.
	strbne r0,[mikptr,#mikAud1Misc]
	bx lr
;@----------------------------------------------------------------------------
miAud2CtlW:					;@ Audio 2 Control (0xFD35)
;@----------------------------------------------------------------------------
	and r0,r1,#0xBF				;@ "Reset Timer Done" should not be preserved?
	strb r0,[mikptr,#mikAud2Ctl]
	ldrb r0,[mikptr,#audio2+WAVESHAPER]
	tst r1,#0x80				;@ Waveshaper bit on/off
	biceq r0,r0,#0x80
	orrne r0,r0,#0x80
	strb r0,[mikptr,#audio2+WAVESHAPER]
	tst r1,#0x40				;@ Check "Reset Timer Done".
	ldrbne r0,[mikptr,#mikAud2Misc]
	bicne r0,r0,#0x08			;@ Timer done, in CtlB.
	strbne r0,[mikptr,#mikAud2Misc]
	bx lr
;@----------------------------------------------------------------------------
miAud3CtlW:					;@ Audio 3 Control (0xFD3D)
;@----------------------------------------------------------------------------
	and r0,r1,#0xBF				;@ "Reset Timer Done" should not be preserved?
	strb r0,[mikptr,#mikAud3Ctl]
	ldrb r0,[mikptr,#audio3+WAVESHAPER]
	tst r1,#0x80				;@ Waveshaper bit on/off
	biceq r0,r0,#0x80
	orrne r0,r0,#0x80
	strb r0,[mikptr,#audio3+WAVESHAPER]
	tst r1,#0x40				;@ Check "Reset Timer Done".
	ldrbne r0,[mikptr,#mikAud3Misc]
	bicne r0,r0,#0x08			;@ Timer done, in CtlB.
	strbne r0,[mikptr,#mikAud3Misc]
	bx lr

;@----------------------------------------------------------------------------
miAud0CountW:				;@ Audio 0 Count (0xFD26)
;@----------------------------------------------------------------------------
	strb r1,[mikptr,#mikAud0Count]
	bx lr
;@----------------------------------------------------------------------------
miAud1CountW:				;@ Audio 1 Count (0xFD2E)
;@----------------------------------------------------------------------------
	strb r1,[mikptr,#mikAud1Count]
	bx lr
;@----------------------------------------------------------------------------
miAud2CountW:				;@ Audio 2 Count (0xFD36)
;@----------------------------------------------------------------------------
	strb r1,[mikptr,#mikAud2Count]
	bx lr
;@----------------------------------------------------------------------------
miAud3CountW:				;@ Audio 3 Count (0xFD3E)
;@----------------------------------------------------------------------------
	strb r1,[mikptr,#mikAud3Count]
	bx lr

;@----------------------------------------------------------------------------
miAud0MiscW:				;@ Audio 0 Misc (0xFD27)
;@----------------------------------------------------------------------------
	and r0,r1,#0x0F				;@ BORROW_OUT, BORROW_IN, LAST_CLOCK & TIMER_DONE
	strb r0,[mikptr,#mikAud0Misc]
	mov r1,r1,lsr#4
	strb r1,[mikptr,#audio0+WAVESHAPER+3]
	bx lr
;@----------------------------------------------------------------------------
miAud1MiscW:				;@ Audio 1 Misc (0xFD2F)
;@----------------------------------------------------------------------------
	and r0,r1,#0x0F				;@ BORROW_OUT, BORROW_IN, LAST_CLOCK & TIMER_DONE
	strb r0,[mikptr,#mikAud1Misc]
	mov r1,r1,lsr#4
	strb r1,[mikptr,#audio1+WAVESHAPER+3]
	bx lr
;@----------------------------------------------------------------------------
miAud2MiscW:				;@ Audio 2 Misc (0xFD37)
;@----------------------------------------------------------------------------
	and r0,r1,#0x0F				;@ BORROW_OUT, BORROW_IN, LAST_CLOCK & TIMER_DONE
	strb r0,[mikptr,#mikAud2Misc]
	mov r1,r1,lsr#4
	strb r1,[mikptr,#audio2+WAVESHAPER+3]
	bx lr
;@----------------------------------------------------------------------------
miAud3MiscW:				;@ Audio 3 Misc (0xFD3F)
;@----------------------------------------------------------------------------
	and r0,r1,#0x0F				;@ BORROW_OUT, BORROW_IN, LAST_CLOCK & TIMER_DONE
	strb r0,[mikptr,#mikAud3Misc]
	mov r1,r1,lsr#4
	strb r1,[mikptr,#audio3+WAVESHAPER+3]
	bx lr

;@----------------------------------------------------------------------------
miStereoW:					;@ 0xFD50
;@----------------------------------------------------------------------------
	eor r0,r0,#0xFF				;@ Stereo is inverted
	strb r0,[mikptr,#mikStereo]
	bx lr
;@----------------------------------------------------------------------------
miIntRstW:					;@ Interrupt Reset (0xFD80)
;@----------------------------------------------------------------------------
	ldrb r0,[mikptr,#timerStatusFlags]
//	bic r1,r1,#1<<4				;@ Serial can not be cleared this way.
	bic r0,r0,r1
	strb r0,[mikptr,#timerStatusFlags]
	b m6502SetIRQPin
;@----------------------------------------------------------------------------
miIntSetW:					;@ Interrupt Set (0xFD81)
;@----------------------------------------------------------------------------
	ldrb r0,[mikptr,#timerStatusFlags]
	orr r0,r0,r1
	strb r0,[mikptr,#timerStatusFlags]
	b m6502SetIRQPin

;@----------------------------------------------------------------------------
miSysCtl1W:					;@ System Control 1 (0xFD87)
;@----------------------------------------------------------------------------
//	tst r1,#0x02
//	beq PowerOff
	ldr r0,[mikptr,#mikCartPtr]
	and r1,r1,#0x01
	b cartAddressStrobe
;@----------------------------------------------------------------------------
miIODatW:					;@ IO-Data (0xFD8B)
;@----------------------------------------------------------------------------
	ldrb r2,[mikptr,#mikIODir]
	ldr r0,[mikptr,#mikCartPtr]
	strb r1,[mikptr,#mikIODat]
	;@ Enable cart writes to BANK1 on AUDIN if AUDIN is set to output
	tst r2,#0x10
	andne r2,r1,#0x10
	strbne r2,[r0,#cartWriteEnable1]
	and r1,r1,#0x02
	b cartAddressData

;@----------------------------------------------------------------------------
miSerCtlW:					;@ Serial Control (0xFD8C)
;@----------------------------------------------------------------------------
	strb r1,[mikptr,#mikSerCtl]
	tst r1,#0x08				;@ Reset Errors?
	movne r0,#0
	strbne r0,[mikptr,#uart_Rx_framing_error]
	strbne r0,[mikptr,#uart_Rx_overun_error]

	tst r1,#0x02
	bxeq lr
	;@ Trigger send break, it will self sustain as long as sendbreak is set
	mov r0,#UART_TX_TIME_PERIOD
	str r0,[mikptr,#uart_TX_COUNTDOWN]
	;@ Loop back what we transmitted
	mov r1,#UART_BREAK_CODE
	b ComLynxTxLoopback
;@----------------------------------------------------------------------------
miSerDatW:					;@ Serial Data (0xFD8D)
;@----------------------------------------------------------------------------
	;@ Fake transmission, set counter to be decremented by Timer 4
	;@
	;@ ComLynx only has one output pin, hence Rx & Tx are shorted
	;@ therefore any transmitted data will loopback
	str r1,[mikptr,#uart_TX_DATA]
	;@ Calculate Parity data
	ldrb r0,[mikptr,#mikSerCtl]
	tst r0,#0x10					;@ uart_PARITY_ENABLE
	beq noSerParity
	;@ Calc parity value
	;@ Leave at zero !!
	b serParity
noSerParity:
	;@ If disabled then the PAREVEN bit is sent
	ldrb r2,[mikptr,#mikSerCtl]
	tst r2,#0x01					;@ uart_PARITY_EVEN
	orrne r1,r1,#0x100
serParity:
	;@ Set countdown to transmission
	mov r0,#UART_TX_TIME_PERIOD
	str r0,[mikptr,#uart_TX_COUNTDOWN]
	;@ Loop back what we transmitted
	b ComLynxTxLoopback
;@----------------------------------------------------------------------------
miCpuSleepW:				;@ CPU Sleep (0xFD91)
;@----------------------------------------------------------------------------
	mov r0,#1
	strb r0,[mikptr,#systemCPUSleep]
	m6502BailOut
	bx lr
;@----------------------------------------------------------------------------
miPBackupW:					;@ PBKUP (0xFD93)
;@----------------------------------------------------------------------------
	strb r1,[mikptr,#mikPBkup]
	ldrb r0,[mikptr,#mikTim0Bkup]
	ldrb r1,[mikptr,#mikTim2Bkup]
	add r0,r0,#1
	add r1,r1,#1
	mul r0,r1,r0
	mov r0,r0,lsl#4
	str r0,[mikptr,#mikCyclesPerFrame]
	b setScreenRefresh
;@----------------------------------------------------------------------------
miPaletteGW:				;@ Green Palette (0xFDAX)
;@----------------------------------------------------------------------------
	strb r0,[mikptr,#paletteChanged]	;@ Mark palette changed
	and r0,r0,#0xF
	and r1,r1,#0xF
	add r2,mikptr,#mikPaletteG
	strb r1,[r2,r0]
	add r2,mikptr,#mikPalette
	add r2,r2,r0,lsl#2
	strb r1,[r2,#1]
	bx lr
;@----------------------------------------------------------------------------
miPaletteBRW:				;@ Blue & Red Palette (0xFDBX)
;@----------------------------------------------------------------------------
	strb r0,[mikptr,#paletteChanged]	;@ Mark palette changed
	and r0,r0,#0xF
	add r2,mikptr,#mikPaletteBR
	strb r1,[r2,r0]
	add r2,mikptr,#mikPalette
	strb r1,[r2,r0,lsl#2]
	bx lr

;@----------------------------------------------------------------------------
ComLynxCable:				;@ In r0=MIKEY, r1=inserted
	.type	ComLynxCable STT_FUNC
;@----------------------------------------------------------------------------
	strb r1,[r0,#mikSerCablePresent]
	bx lr
;@----------------------------------------------------------------------------
ComLynxRxData:				;@ In r0=MIKEY, r1=data
	.type	ComLynxRxData STT_FUNC
;@----------------------------------------------------------------------------
	;@ Copy over the data
	ldr r2,[r0,#uart_Rx_waiting]
	cmp r2,#UART_MAX_RX_QUEUE
	bxcs lr						;@ UART RX Overun
	;@ Trigger incoming receive IF none waiting otherwise
	;@ we NEVER get to receive it!!!
	cmp r2,#0
	moveq r3,#UART_RX_TIME_PERIOD
	streq r3,[r0,#uart_RX_COUNTDOWN]
	add r2,r2,#1
	str r2,[r0,#uart_Rx_waiting]
	;@ Receive the byte
	ldrb r3,[r0,#uart_Rx_input_ptr]
	add r2,r0,#uart_Rx_input_queue
	str r1,[r2,r3,lsl#2]
	add r3,r3,#1
	and r3,r3,#UART_MAX_RX_QUEUE-1
	strb r3,[r0,#uart_Rx_input_ptr]
	bx lr
;@----------------------------------------------------------------------------
ComLynxTxLoopback:			;@ In r1=data
;@----------------------------------------------------------------------------
	;@ Copy over the data
	ldr r2,[mikptr,#uart_Rx_waiting]
	cmp r2,#UART_MAX_RX_QUEUE
	bxcs lr						;@ UART RX Overun
	;@ Trigger incoming receive IF none waiting otherwise
	;@ we NEVER get to receive it!!!
	cmp r2,#0
	moveq r3,#UART_RX_TIME_PERIOD
	streq r3,[mikptr,#uart_RX_COUNTDOWN]
	add r2,r2,#1
	str r2,[mikptr,#uart_Rx_waiting]
	;@ Receive the byte
	ldrb r3,[mikptr,#uart_Rx_output_ptr]
	add r2,mikptr,#uart_Rx_input_queue
	sub r3,r3,#1
	and r3,r3,#UART_MAX_RX_QUEUE-1
	str r1,[r2,r3,lsl#2]
	strb r3,[mikptr,#uart_Rx_output_ptr]
	bx lr
;@----------------------------------------------------------------------------
ComLynxTxCallback:			;@ In r0=MIKEY, r1=function, r2=objref
;@----------------------------------------------------------------------------
	str r1,[r0,#mikTxFunction]
	str r2,[r0,#mikTxCallbackObj]
	bx lr


;@----------------------------------------------------------------------------
mikDisplayEndOfFrame:
;@----------------------------------------------------------------------------
	mov r0,#1
	strb r0,[mikptr,#mikFrameFinnished]
	// Stop any further line rendering
	mov r0,#0
	str r0,[mikptr,#lynxLineDMACounter]
	str r0,[mikptr,#lynxLine]
	// Trigger the callback to the display sub-system to render the
	// display.
	ldr pc,[mikptr,#mikFrameCallback]

#ifdef NDS
	.section .itcm						;@ For the NDS ARM9
#elif GBA
	.section .iwram, "ax", %progbits	;@ For the GBA
#endif
;@----------------------------------------------------------------------------
mikSysUpdate:
;@----------------------------------------------------------------------------
	stmfd sp!,{r4-r11,lr}
	mov r0,#0
	strb r0,[mikptr,#mikFrameFinnished]
	ldr r4,[mikptr,#systemCycleCount]
	ldr r5,[mikptr,#mikCyclesPerFrame]
	add r5,r5,r4
sysLoop:
	mov r0,#0
	ldr r1,[mikptr,#nextTimerEvent]
	cmp r4,r1
	blpl mikUpdate				;@ returns consumed cycles
	add r4,r4,r0				;@ This updates sysCycleCnt!!!
	ldrb r0,[mikptr,#systemCPUSleep]
	cmp r0,#0
	// systemCycleCount = nextTimerEvent;
	ldrne r4,[mikptr,#nextTimerEvent]	;@ This updates sysCycleCnt!!!
	bne sysUpdExit

;@------------------------------------
	str r4,[mikptr,#systemCycleCount]	;@ This stores sysCycleCnt!!!
	ldr r0,[mikptr,#nextTimerEvent]
	sub r0,r0,r4
	cmp r0,#8*5
	movmi r0,#8*5
	cmp r0,#159*16
	movcs r0,#159*16
	ldr r2,=(0x100000000/5)+1
	umull r1,r0,r2,r0
//	mov r0,#8
	stmfd sp!,{r0,r5}
	add r1,m6502ptr,#m6502Regs
	ldmia r1,{m6502nz-m6502pc,m6502zpage}	;@ Restore M6502 state
	clearCycles
	bl m6502RunXCycles
	m6502FixCycles
	add r1,m6502ptr,#m6502Regs
	stmia r1,{m6502nz-m6502pc}	;@ Save M6502 state
	ldmfd sp!,{r0,r5}
	ldr r4,[mikptr,#systemCycleCount]
;@------------------------------------
	sub r0,r0,cycles,asr#CYC_SHIFT
	// systemCycleCount += (1+(cyc*CPU_RDWR_CYC));
	add r0,r0,r0,lsl#2	// x5
	add r0,r0,#1
	add r4,r4,r0
sysUpdExit:
	ldrb r0,[mikptr,#systemCPUSleep]
	cmp r0,#0
	beq noSpritePaint
	ldr r2,[mikptr,#nextTimerEvent]
	sub r0,r2,r4
	ldr r12,[mikptr,#mikSuzyPtr]	;@ r12=suzptr
	bl suzPaintSprites
	add r4,r4,r0
	ldr r2,[mikptr,#nextTimerEvent]
	str r4,[mikptr,#suzieDoneTime]
	cmp r4,r2
	movpl r4,r2
	strmi r4,[mikptr,#nextTimerEvent]
noSpritePaint:
	ldrb r0,[mikptr,#mikFrameFinnished]
	cmp r0,#1
	cmpmi r4,r5
	bmi sysLoop
	str r4,[mikptr,#systemCycleCount]	;@ This stores sysCycleCnt!!!
	ldmfd sp!,{r4-r11,lr}
	bx lr
;@----------------------------------------------------------------------------

//	Timer updates, rolled out flat in group order
//
//	Group A:
//	Timer 0 -> Timer 2 -> Timer 4.
//
//	Group B:
//	Timer 1 -> Timer 3 -> Timer 5 -> Timer 7 -> Audio 0 -> Audio 1-> Audio 2 -> Audio 3 -> Timer 1.
//

//
// Within each timer code block we will predict the cycle count number of
// the next timer event
//
// We don't need to count linked timers as the timer they are linked
// from will always generate earlier events.
//
// As Timer 4 (UART) will generate many events we will ignore it
//
// We set the next event to the end of time at first and let the timers
// overload it. Any writes to timer controls will force next event to
// be immediate and hence a new prediction will be done. The prediction
// causes overflow as opposed to zero i.e. current+1
// (In reality T0 line counter should always be running.)
//
;@----------------------------------------------------------------------------
mikUpdate:				;@ in r4=systemCycleCount, out r0=consumed (16MHz) cycles.
;@----------------------------------------------------------------------------
	stmfd sp!,{r5,lr}

	//gNextTimerEvent = 0xffffffff;
	add r2,r4,#0x40000000
	// Check if the CPU needs to be woken up from sleep mode
	ldr r0,[mikptr,#suzieDoneTime]
	cmp r0,#0
	beq noSuzy
	cmp r4,r0
	movpl r0,#0
	strpl r0,[mikptr,#suzieDoneTime]
	strbpl r0,[mikptr,#systemCPUSleep]
	movmi r2,r0
noSuzy:
	str r2,[mikptr,#nextTimerEvent]


	bl miRunTimer0
	cmp r0,#0
	blne mikDisplayLine
	mov r5,r0

	bl miRunTimer2
	cmp r0,#0
	blne mikDisplayEndOfFrame

	bl miRunTimer4
	bl miRunTimer6

	bl miRunTimer1
	bl miRunTimer3
	bl miRunTimer5
	bl miRunTimer7

//	if (gAudioEnabled)
//	bl updateSound

	ldrb r0,[mikptr,#timerStatusFlags]
	cmp r0,#0
	movne r1,#0
	strbne r1,[mikptr,#systemCPUSleep]
	bl m6502SetIRQPin

	mov r0,r5
	ldmfd sp!,{r5,lr}
	bx lr
;@----------------------------------------------------------------------------
mikDisplayLine:
;@----------------------------------------------------------------------------
	mov r0,#0
	ldrb r1,[mikptr,#mikDispCtl]
	tst r1,#1					;@ Display DMA on?
	bxeq lr
	ldrb r3,[mikptr,#mikTim2Bkup]
	sub r3,r3,#GAME_HEIGHT
	ldr r2,[mikptr,#lynxLine]
	cmp r2,r3
	movcc r3,#1
	movcs r3,#0
	strb r3,[mikptr,#ioDatRestSignal]

	cmp r2,#3
	bne noLatch
	ldrb r3,[mikptr,#mikDispAdrL]
	ldrb r0,[mikptr,#mikDispAdrH]
	orr r3,r3,r0,lsl#8
	tst r1,#2					;@ Screen flip?
	biceq r3,r3,#3
	orrne r3,r3,#3
	str r3,[mikptr,#lynxAddr]
	mov r3,#GAME_HEIGHT
	str r3,[mikptr,#lynxLineDMACounter]
noLatch:
	add r2,r2,#1
	str r2,[mikptr,#lynxLine]

	ldr r0,[mikptr,#lynxLineDMACounter]
	cmp r0,#0
	bxeq lr
	sub r0,r0,#1
	str r0,[mikptr,#lynxLineDMACounter]

	ldr r0,[mikptr,#mikGfxRAM]
	ldr r3,[mikptr,#lynxAddr]
	add r0,r0,r3
	ands r2,r1,#2				;@ Screen flip?
	addeq r3,r3,#GAME_WIDTH/2
	subne r3,r3,#GAME_WIDTH/2
	str r3,[mikptr,#lynxAddr]
	add r1,mikptr,#mikPalette
	ldrb r3,[mikptr,#paletteChanged]	;@ Palette changed?
	stmfd sp!,{lr}
	mov lr,pc
	ldr pc,[mikptr,#mikLineCallback]
	ldmfd sp!,{lr}
	mov r0,#0
	strb r0,[mikptr,#paletteChanged]	;@ Clear Palette changed.
	mov r0,#80 * 4				;@ 80 * DMA_RDWR_CYC
	bx lr

;@----------------------------------------------------------------------------
miRunTimer0:				;@ in r4=systemCycleCount
;@----------------------------------------------------------------------------
	mov r0,#0
	ldr r2,[mikptr,#mikTim0Bkup]
	movs r1,r2,lsl#21
	bxcc lr						;@ CtlA Count Enabled?
	stmfd sp!,{r6-r7}
	bic r2,r2,#0x0F000000		;@ CtlB clear borrow in/out, last clock
	// Ordinary clocked mode as opposed to linked mode
	// 16MHz clock downto 1us == cyclecount >> 4
	//divide = 4 + (CtlA & CLOCK_SEL);
	mov r1,r1,lsr#29			;@ CtlA Clock Select
	add r1,r1,#4
	ldr r6,[mikptr,#timer0+CURRENT]
	ldr r7,[mikptr,#timer0+LAST_COUNT]
	//decval = (gSystemCycleCount - LAST_COUNT) >> divide;
	sub r3,r4,r7
	movs r3,r3,lsr r1
	beq tim0NoCount				;@ decval?
	mov r2,r2,ror#24
	orr r2,r2,#2				;@ CtlB Borrow in, because we count
	add r7,r7,r3,lsl r1
	sub r2,r2,r3,lsl#24
	subs r6,r6,r3
	orreq r2,r2,#4				;@ CtlB Last clock
	bpl tim0NoIrq
	orr r2,r2,#1				;@ CtlB Borrow Out
	//tst r2,#0x00100000		;@ CtlA & ENABLE_RELOAD
	and r3,r2,#0xFF00
	add r3,r3,#0x0100
	add r2,r2,r3,lsl#16
	add r6,r6,r3,lsr#8
//	biceq r2,r2,#0xFF000000		;@ No reload, clear count.
//	orreq r2,r2,#8				;@ CtlB Timer Done
//	moveq r6,#0
	tst r2,#0x800000			;@ CtlA Interrupt Enable?
	ldrbne r0,[mikptr,#timerStatusFlags]
	orrne r0,r0,#1<<0
	strbne r0,[mikptr,#timerStatusFlags]
//	bl mikDisplayLine
	mov r0,#1
tim0NoIrq:
	str r6,[mikptr,#timer0+CURRENT]
	str r7,[mikptr,#timer0+LAST_COUNT]
	mov r2,r2,ror#8
tim0NoCount:
	str r2,[mikptr,#mikTim0Bkup]
	// Prediction for next timer event cycle number
	// Sometimes timeupdates can be >2x rollover in which case
	// then CURRENT may still be negative and we can use it to
	// calc the next timer value, we just want another update ASAP
	//tmp = gSystemCycleCount;
	//tmp += (CURRENT & 0x80000000) ? 1 : ((CURRENT + 1) << divide);
	tst r6,#0x80000000
	addne r2,r4,#1
	addeq r6,r6,#1
	addeq r2,r4,r6,lsl r1
	//if (tmp < gNextTimerEvent) {
	//	gNextTimerEvent = tmp;
	//}
	ldr r1,[mikptr,#nextTimerEvent]
	cmp r2,r1
	strmi r2,[mikptr,#nextTimerEvent]
	ldmfd sp!,{r6-r7}
	bx lr

;@----------------------------------------------------------------------------
miRunTimer2:				;@ in r4=systemCycleCount
;@----------------------------------------------------------------------------
	mov r0,#0
	ldr r2,[mikptr,#mikTim2Bkup]
	movs r1,r2,lsl#21
	bxcc lr						;@ CtlA Count Enabled?
	stmfd sp!,{r6-r7}
	bic r2,r2,#0x0F000000		;@ CtlB clear borrow in/out, last clock
	mov r1,r1,lsr#29			;@ CtlA Clock Select
	// 16MHz clock downto 1us == cyclecount >> 4
	//divide = 4 + (CtlA & CLOCK_SEL);
	cmp r1,#7					;@ Link mode?
	moveq r1,#0
	ldrbeq r3,[mikptr,#mikTim0CtlB]
	and r3,r3,#1
	addne r1,r1,#4
	ldr r6,[mikptr,#timer2+CURRENT]
	ldr r7,[mikptr,#timer2+LAST_COUNT]
	//decval = (gSystemCycleCount - LAST_COUNT) >> divide;
	subne r3,r4,r7
	movs r3,r3,lsr r1
	beq tim2NoCount				;@ decval?
	mov r2,r2,ror#24
	orr r2,r2,#2				;@ CtlB Borrow in, because we count
	add r7,r7,r3,lsl r1
	sub r2,r2,r3,lsl#24
	subs r6,r6,r3
	orreq r2,r2,#4				;@ CtlB Last clock
	bpl tim2NoIrq
	orr r2,r2,#1				;@ CtlB Borrow Out
	//tst r2,#0x00100000		;@ CtlA & ENABLE_RELOAD
	and r3,r2,#0xFF00
	add r3,r3,#0x0100
	add r2,r2,r3,lsl#16
	add r6,r6,r3,lsr#8
//	biceq r2,r2,#0xFF000000		;@ No reload, clear count.
//	orreq r2,r2,#8				;@ CtlB Timer Done
//	moveq r6,#0
	tst r2,#0x800000			;@ CtlA Interrupt Enable?
	ldrbne r0,[mikptr,#timerStatusFlags]
	orrne r0,r0,#1<<2
	strbne r0,[mikptr,#timerStatusFlags]
//	bl mikDisplayEndOfFrame
	mov r0,#1
tim2NoIrq:
	str r6,[mikptr,#timer2+CURRENT]
	str r7,[mikptr,#timer2+LAST_COUNT]
	mov r2,r2,ror#8
tim2NoCount:
	str r2,[mikptr,#mikTim2Bkup]
	// Prediction for next timer event cycle number
	// We dont need to predict next timer event as its the frame timer
	// and will always be beaten by the line timer on Timer 0
	ldmfd sp!,{r6-r7}
	bx lr

;@----------------------------------------------------------------------------
miRunTimer4:				;@ in r4=systemCycleCount
;@----------------------------------------------------------------------------
	mov r0,#0
	ldr r2,[mikptr,#mikTim4Bkup]
	movs r1,r2,lsl#21
	bxcc lr						;@ CtlA Count Enabled?
	stmfd sp!,{r6-r7}
	bic r2,r2,#0x0F000000		;@ CtlB clear borrow in/out, last clock
	// Ordinary clocked mode as opposed to linked mode
	// 16MHz clock downto 1us == cyclecount >> 4
	// Additional /8 (+3) for 8 clocks per bit transmit
	mov r1,r1,lsr#29			;@ CtlA Clock Select
	add r1,r1,#4 + 3
	ldr r6,[mikptr,#timer4+CURRENT]
	ldr r7,[mikptr,#timer4+LAST_COUNT]
	// decval = (gSystemCycleCount - LAST_COUNT) >> divide;
	sub r3,r4,r7
	movs r3,r3,lsr r1
	beq tim4NoCount				;@ decval?
	mov r2,r2,ror#24
	orr r2,r2,#2				;@ CtlB Borrow in, because we count
	add r7,r7,r3,lsl r1
	sub r2,r2,r3,lsl#24
	subs r6,r6,r3
	orreq r2,r2,#4				;@ CtlB Last clock
	bpl tim4NoIrq
;@------------------------------------
	orr r2,r2,#1				;@ CtlB Borrow Out

	//tst r2,#0x00100000		;@ CtlA & ENABLE_RELOAD
	and r3,r2,#0xFF00
	add r3,r3,#0x0100
	add r2,r2,r3,lsl#16
	add r6,r6,r3,lsr#8			;@ !!! adds
//	addcc r6,r6,r3,lsr#8		;@ !!! Might be needed?
//	movcc r7,r4					;@ - || -
//	biceq r2,r2,#0xFF000000		;@ No reload, clear count.
//	orreq r2,r2,#8				;@ CtlB Timer Done
//	moveq r6,#0

// Handle UART RX here
	ldr r0,[mikptr,#uart_RX_COUNTDOWN]
	cmp r0,#0
	subpl r0,r0,#1
	strpl r0,[mikptr,#uart_RX_COUNTDOWN]
	bne noUartRx
	ldr r0,[mikptr,#uart_Rx_waiting]
	cmp r0,#0
	ble noUartFetch
	add r12,mikptr,#uart_Rx_input_queue
	ldrb r3,[mikptr,#uart_Rx_output_ptr]
	ldr r12,[r12,r3,lsl#2]
	str r12,[mikptr,#uart_RX_DATA]
	add r3,r3,#1
	and r3,r3,#UART_MAX_RX_QUEUE-1
	strb r3,[mikptr,#uart_Rx_output_ptr]
	subs r0,r0,#1
	str r0,[mikptr,#uart_Rx_waiting]
noUartFetch:
	movhi r0,#UART_RX_TIME_PERIOD + UART_RX_NEXT_DELAY
	movle r0,#UART_RX_INACTIVE
	str r0,[mikptr,#uart_RX_COUNTDOWN]
	ldrb r0,[mikptr,#uart_RX_READY]
	cmp r0,#0
	mov r0,#1
	strbne r0,[mikptr,#uart_Rx_overun_error]
	strbeq r0,[mikptr,#uart_RX_READY]
noUartRx:

// Handle UART TX here
	ldr r0,[mikptr,#uart_TX_COUNTDOWN]
	cmp r0,#0
	subpl r0,r0,#1
	strpl r0,[mikptr,#uart_TX_COUNTDOWN]
	bne noUartTx
	ldrb r0,[mikptr,#mikSerCtl]
	tst r0,#0x02				;@ uart_SENDBREAK
	moveq r3,#UART_TX_INACTIVE
	movne r3,#UART_TX_TIME_PERIOD
	str r3,[mikptr,#uart_TX_COUNTDOWN]
	beq noLoopBack
	stmfd sp!,{r1-r2,lr}
	mov r1,#UART_BREAK_CODE
	str r1,[mikptr,#uart_TX_DATA]
	bl ComLynxTxLoopback
	ldmfd sp!,{r1-r2,lr}
noLoopBack:
	ldr r3,[mikptr,#mikTxFunction]
	cmp r3,#0
	beq noUartTx
	stmfd sp!,{r1-r2,lr}
	ldr r0,[mikptr,#uart_RX_DATA]
	ldr r1,[mikptr,#mikTxCallbackObj]
#ifdef __ARM_ARCH_5TE__
	blx r3
#else
	mov lr,pc
	bx r3
#endif
	ldmfd sp!,{r1-r2,lr}
noUartTx:
;@------------------------------------
tim4NoIrq:
	str r6,[mikptr,#timer4+CURRENT]
	str r7,[mikptr,#timer4+LAST_COUNT]
	mov r2,r2,ror#8
tim4NoCount:
	str r2,[mikptr,#mikTim4Bkup]
	// Prediction for next timer event cycle number
	// Sometimes timeupdates can be >2x rollover in which case
	// then CURRENT may still be negative and we can use it to
	// calc the next timer value, we just want another update ASAP
	//tmp = gSystemCycleCount;
	//tmp += (CURRENT & 0x80000000) ? 1 : ((CURRENT + 1) << divide);
	tst r6,#0x80000000
	addne r2,r4,#1
	addeq r6,r6,#1
	addeq r2,r4,r6,lsl r1
	//if (tmp < gNextTimerEvent) {
	//	gNextTimerEvent = tmp;
	//}
	ldr r1,[mikptr,#nextTimerEvent]
	cmp r2,r1
	strmi r2,[mikptr,#nextTimerEvent]

	ldmfd sp!,{r6-r7}

;@----------------------------------------------------------------------------
miUpdateUartIrq:
;@----------------------------------------------------------------------------
	ldrb r1,[mikptr,#mikSerCtl]
	ldrb r0,[mikptr,#timerStatusFlags]

	// Is data waiting and the interrupt enabled
	tst r1,#0x40				;@ RX int enabled?
	ldrbne r2,[mikptr,#uart_RX_READY]
	cmpne r2,#0
	orrne r0,r0,#1<<4			;@

	// If Tx is inactive i.e ready for a byte to eat and the
	// IRQ is enabled then generate it always
	tst r1,#0x80				;@ TX int enabled?
	ldrne r2,[mikptr,#uart_TX_COUNTDOWN]
	tstne r2,#UART_TX_INACTIVE
	orrne r0,r0,#1<<4			;@

	strb r0,[mikptr,#timerStatusFlags]
	bx lr

;@----------------------------------------------------------------------------
miRunTimer6:				;@ in r4=systemCycleCount
;@----------------------------------------------------------------------------
	mov r0,#0
	ldr r2,[mikptr,#mikTim6Bkup]
	movs r1,r2,lsl#21
	bxcc lr						;@ CtlA Count Enabled?
	tst r2,#0x08000000			;@ CtlB Timer done?
	bxne lr
	mov r1,r1,lsr#29			;@ CtlA Clock Select
	cmp r1,#7					;@ Link mode?
	bxeq lr
	stmfd sp!,{r6-r7}
	bic r2,r2,#0x0F000000		;@ CtlB clear borrow in/out, last clock
	// Ordinary clocked mode as opposed to linked mode
	// 16MHz clock downto 1us == cyclecount >> 4
	//divide = 4 + (CtlA & CLOCK_SEL);
	add r1,r1,#4
	ldr r6,[mikptr,#timer6+CURRENT]
	ldr r7,[mikptr,#timer6+LAST_COUNT]
	//decval = (gSystemCycleCount - LAST_COUNT) >> divide;
	sub r3,r4,r7
	movs r3,r3,lsr r1
	beq tim6NoCount				;@ decval?
	mov r2,r2,ror#24
	orr r2,r2,#2				;@ CtlB Borrow in, because we count
	add r7,r7,r3,lsl r1
	sub r2,r2,r3,lsl#24
	subs r6,r6,r3
	orreq r2,r2,#4				;@ CtlB Last clock
	bpl tim6NoIrq
	orr r2,r2,#1				;@ CtlB Borrow Out
	tst r2,#0x00100000			;@ CtlA & ENABLE_RELOAD
	andne r3,r2,#0xFF00
	addne r3,r3,#0x0100
	addne r2,r2,r3,lsl#16
	addne r6,r6,r3,lsr#8
	biceq r2,r2,#0xFF000000		;@ No reload, clear count.
	orreq r2,r2,#8				;@ CtlB Timer Done
	moveq r6,#0
	tst r2,#0x800000			;@ CtlA Interrupt Enable?
	ldrbne r0,[mikptr,#timerStatusFlags]
	orrne r0,r0,#1<<6
	strbne r0,[mikptr,#timerStatusFlags]
	mov r0,#1
tim6NoIrq:
	str r6,[mikptr,#timer6+CURRENT]
	str r7,[mikptr,#timer6+LAST_COUNT]
	mov r2,r2,ror#8
tim6NoCount:
	str r2,[mikptr,#mikTim6Bkup]
	// Prediction for next timer event cycle number
	// Sometimes timeupdates can be >2x rollover in which case
	// then CURRENT may still be negative and we can use it to
	// calc the next timer value, we just want another update ASAP
	//tmp = gSystemCycleCount;
	//tmp += (CURRENT & 0x80000000) ? 1 : ((CURRENT + 1) << divide);
	tst r6,#0x80000000
	addne r2,r4,#1
	addeq r6,r6,#1
	addeq r2,r4,r6,lsl r1
	//if (tmp < gNextTimerEvent) {
	//	gNextTimerEvent = tmp;
	//}
	ldr r1,[mikptr,#nextTimerEvent]
	cmp r2,r1
	strmi r2,[mikptr,#nextTimerEvent]
	ldmfd sp!,{r6-r7}
	bx lr

;@----------------------------------------------------------------------------
miRunTimer1:				;@ in r4=systemCycleCount
;@----------------------------------------------------------------------------
	mov r0,#0
	ldr r2,[mikptr,#mikTim1Bkup]
	movs r1,r2,lsl#21
	bxcc lr						;@ CtlA Count Enabled?
	tst r2,#0x08000000			;@ CtlB Timer done?
	bxne lr
	mov r1,r1,lsr#29			;@ CtlA Clock Select
	cmp r1,#7					;@ Link mode?
	bxeq lr
	stmfd sp!,{r6-r7}
	bic r2,r2,#0x0F000000		;@ CtlB clear borrow in/out, last clock
	// Ordinary clocked mode as opposed to linked mode
	// 16MHz clock downto 1us == cyclecount >> 4
	//divide = 4 + (CtlA & CLOCK_SEL);
	add r1,r1,#4
	ldr r6,[mikptr,#timer1+CURRENT]
	ldr r7,[mikptr,#timer1+LAST_COUNT]
	//decval = (gSystemCycleCount - LAST_COUNT) >> divide;
	sub r3,r4,r7
	movs r3,r3,lsr r1
	beq tim1NoCount				;@ decval?
	mov r2,r2,ror#24
	orr r2,r2,#2				;@ CtlB Borrow in, because we count
	add r7,r7,r3,lsl r1
	sub r2,r2,r3,lsl#24
	subs r6,r6,r3
	orreq r2,r2,#4				;@ CtlB Last clock
	bpl tim1NoIrq
	orr r2,r2,#1				;@ CtlB Borrow Out
	tst r2,#0x00100000			;@ CtlA & ENABLE_RELOAD
	andne r3,r2,#0xFF00
	addne r3,r3,#0x0100
	addne r2,r2,r3,lsl#16
	addne r6,r6,r3,lsr#8
	biceq r2,r2,#0xFF000000		;@ No reload, clear count.
	orreq r2,r2,#8				;@ CtlB Timer Done
	moveq r6,#0
	tst r2,#0x800000			;@ CtlA Interrupt Enable?
	ldrbne r0,[mikptr,#timerStatusFlags]
	orrne r0,r0,#1<<1
	strbne r0,[mikptr,#timerStatusFlags]
	mov r0,#1
tim1NoIrq:
	str r6,[mikptr,#timer1+CURRENT]
	str r7,[mikptr,#timer1+LAST_COUNT]
	mov r2,r2,ror#8
tim1NoCount:
	str r2,[mikptr,#mikTim1Bkup]
	// Prediction for next timer event cycle number
	// Sometimes timeupdates can be >2x rollover in which case
	// then CURRENT may still be negative and we can use it to
	// calc the next timer value, we just want another update ASAP
	//tmp = gSystemCycleCount;
	//tmp += (CURRENT & 0x80000000) ? 1 : ((CURRENT + 1) << divide);
	tst r6,#0x80000000
	addne r2,r4,#1
	addeq r6,r6,#1
	addeq r2,r4,r6,lsl r1
	//if (tmp < gNextTimerEvent) {
	//	gNextTimerEvent = tmp;
	//}
	ldr r1,[mikptr,#nextTimerEvent]
	cmp r2,r1
	strmi r2,[mikptr,#nextTimerEvent]
	ldmfd sp!,{r6-r7}
	bx lr

;@----------------------------------------------------------------------------
miRunTimer3:				;@ in r4=systemCycleCount
;@----------------------------------------------------------------------------
	mov r0,#0
	ldr r2,[mikptr,#mikTim3Bkup]
	movs r1,r2,lsl#21
	bxcc lr						;@ CtlA Count Enabled?
	tst r2,#0x08000000			;@ CtlB Timer done?
	bxne lr
	stmfd sp!,{r6-r7}
	bic r2,r2,#0x0F000000		;@ CtlB clear borrow in/out, last clock
	mov r1,r1,lsr#29			;@ CtlA Clock Select
	// Ordinary clocked mode as opposed to linked mode
	// 16MHz clock downto 1us == cyclecount >> 4
	//divide = 4 + (CtlA & CLOCK_SEL);
	cmp r1,#7					;@ Link mode?
	moveq r1,#0
	ldrbeq r3,[mikptr,#mikTim1CtlB]
	and r3,r3,#1
	addne r1,r1,#4
	ldr r6,[mikptr,#timer3+CURRENT]
	ldr r7,[mikptr,#timer3+LAST_COUNT]
	//decval = (gSystemCycleCount - LAST_COUNT) >> divide;
	subne r3,r4,r7
	movs r3,r3,lsr r1
	beq tim3NoCount				;@ decval?
	mov r2,r2,ror#24
	orr r2,r2,#2				;@ CtlB Borrow in, because we count
	add r7,r7,r3,lsl r1
	sub r2,r2,r3,lsl#24
	subs r6,r6,r3
	orreq r2,r2,#4				;@ CtlB Last clock
	bpl tim3NoIrq
	orr r2,r2,#1				;@ CtlB Borrow Out
	tst r2,#0x00100000			;@ CtlA & ENABLE_RELOAD
	andne r3,r2,#0xFF00
	addne r3,r3,#0x0100
	addne r2,r2,r3,lsl#16
	addne r6,r6,r3,lsr#8
	biceq r2,r2,#0xFF000000		;@ No reload, clear count.
	orreq r2,r2,#8				;@ CtlB Timer Done
	moveq r6,#0
	tst r2,#0x800000			;@ CtlA Interrupt Enable?
	ldrbne r0,[mikptr,#timerStatusFlags]
	orrne r0,r0,#1<<3
	strbne r0,[mikptr,#timerStatusFlags]
	mov r0,#1
tim3NoIrq:
	str r6,[mikptr,#timer3+CURRENT]
	str r7,[mikptr,#timer3+LAST_COUNT]
	mov r2,r2,ror#8
tim3NoCount:
	str r2,[mikptr,#mikTim3Bkup]
	cmp r1,#0
	beq tim3Exit
	// Prediction for next timer event cycle number
	// Sometimes timeupdates can be >2x rollover in which case
	// then CURRENT may still be negative and we can use it to
	// calc the next timer value, we just want another update ASAP
	//tmp = gSystemCycleCount;
	//tmp += (CURRENT & 0x80000000) ? 1 : ((CURRENT + 1) << divide);
	tst r6,#0x80000000
	addne r2,r4,#1
	addeq r6,r6,#1
	addeq r2,r4,r6,lsl r1
	//if (tmp < gNextTimerEvent) {
	//	gNextTimerEvent = tmp;
	//}
	ldr r1,[mikptr,#nextTimerEvent]
	cmp r2,r1
	strmi r2,[mikptr,#nextTimerEvent]
tim3Exit:
	ldmfd sp!,{r6-r7}
	bx lr

;@----------------------------------------------------------------------------
miRunTimer5:				;@ in r4=systemCycleCount
;@----------------------------------------------------------------------------
	mov r0,#0
	ldr r2,[mikptr,#mikTim5Bkup]
	movs r1,r2,lsl#21
	bxcc lr						;@ CtlA Count Enabled?
	tst r2,#0x08000000			;@ CtlB Timer done?
	bxne lr
	stmfd sp!,{r6-r7}
	bic r2,r2,#0x0F000000		;@ CtlB clear borrow in/out, last clock
	mov r1,r1,lsr#29			;@ CtlA Clock Select
	// Ordinary clocked mode as opposed to linked mode
	// 16MHz clock downto 1us == cyclecount >> 4
	//divide = 4 + (CtlA & CLOCK_SEL);
	cmp r1,#7					;@ Link mode?
	moveq r1,#0
	ldrbeq r3,[mikptr,#mikTim3CtlB]
	and r3,r3,#1
	addne r1,r1,#4
	ldr r6,[mikptr,#timer5+CURRENT]
	ldr r7,[mikptr,#timer5+LAST_COUNT]
	//decval = (gSystemCycleCount - LAST_COUNT) >> divide;
	subne r3,r4,r7
	movs r3,r3,lsr r1
	beq tim5NoCount				;@ decval?
	mov r2,r2,ror#24
	orr r2,r2,#2				;@ CtlB Borrow in, because we count
	add r7,r7,r3,lsl r1
	sub r2,r2,r3,lsl#24
	subs r6,r6,r3
	orreq r2,r2,#4				;@ CtlB Last clock
	bpl tim5NoIrq
	orr r2,r2,#1				;@ CtlB Borrow Out
	tst r2,#0x00100000			;@ CtlA & ENABLE_RELOAD
	andne r3,r2,#0xFF00
	addne r3,r3,#0x0100
	addne r2,r2,r3,lsl#16
	addne r6,r6,r3,lsr#8
	biceq r2,r2,#0xFF000000		;@ No reload, clear count.
	orreq r2,r2,#8				;@ CtlB Timer Done
	moveq r6,#0
	tst r2,#0x800000			;@ CtlA Interrupt Enable?
	ldrbne r0,[mikptr,#timerStatusFlags]
	orrne r0,r0,#1<<5
	strbne r0,[mikptr,#timerStatusFlags]
	mov r0,#1
tim5NoIrq:
	str r6,[mikptr,#timer5+CURRENT]
	str r7,[mikptr,#timer5+LAST_COUNT]
	mov r2,r2,ror#8
tim5NoCount:
	str r2,[mikptr,#mikTim5Bkup]
	cmp r1,#0
	beq tim5Exit
	// Prediction for next timer event cycle number
	// Sometimes timeupdates can be >2x rollover in which case
	// then CURRENT may still be negative and we can use it to
	// calc the next timer value, we just want another update ASAP
	//tmp = gSystemCycleCount;
	//tmp += (CURRENT & 0x80000000) ? 1 : ((CURRENT + 1) << divide);
	tst r6,#0x80000000
	addne r2,r4,#1
	addeq r6,r6,#1
	addeq r2,r4,r6,lsl r1
	//if (tmp < gNextTimerEvent) {
	//	gNextTimerEvent = tmp;
	//}
	ldr r1,[mikptr,#nextTimerEvent]
	cmp r2,r1
	strmi r2,[mikptr,#nextTimerEvent]
tim5Exit:
	ldmfd sp!,{r6-r7}
	bx lr

;@----------------------------------------------------------------------------
miRunTimer7:				;@ in r4=systemCycleCount
;@----------------------------------------------------------------------------
	mov r0,#0
	ldr r2,[mikptr,#mikTim7Bkup]
	movs r1,r2,lsl#21
	bxcc lr						;@ CtlA Count Enabled?
	tst r2,#0x08000000			;@ CtlB Timer done?
	bxne lr
	stmfd sp!,{r6-r7}
	bic r2,r2,#0x0F000000		;@ CtlB clear borrow in/out, last clock
	mov r1,r1,lsr#29			;@ CtlA Clock Select
	// Ordinary clocked mode as opposed to linked mode
	// 16MHz clock downto 1us == cyclecount >> 4
	//divide = 4 + (CtlA & CLOCK_SEL);
	cmp r1,#7					;@ Link mode?
	moveq r1,#0
	ldrbeq r3,[mikptr,#mikTim5CtlB]
	and r3,r3,#1
	addne r1,r1,#4
	ldr r6,[mikptr,#timer7+CURRENT]
	ldr r7,[mikptr,#timer7+LAST_COUNT]
	//decval = (gSystemCycleCount - LAST_COUNT) >> divide;
	subne r3,r4,r7
	movs r3,r3,lsr r1
	beq tim7NoCount				;@ decval?
	mov r2,r2,ror#24
	orr r2,r2,#2				;@ CtlB Borrow in, because we count
	add r7,r7,r3,lsl r1
	sub r2,r2,r3,lsl#24
	subs r6,r6,r3
	orreq r2,r2,#4				;@ CtlB Last clock
	bpl tim7NoIrq
	orr r2,r2,#1				;@ CtlB Borrow Out
	tst r2,#0x00100000			;@ CtlA & ENABLE_RELOAD
	andne r3,r2,#0xFF00
	addne r3,r3,#0x0100
	addne r2,r2,r3,lsl#16
	addne r6,r6,r3,lsr#8
	biceq r2,r2,#0xFF000000		;@ No reload, clear count.
	orreq r2,r2,#8				;@ CtlB Timer Done
	moveq r6,#0
	tst r2,#0x800000			;@ CtlA Interrupt Enable?
	ldrbne r0,[mikptr,#timerStatusFlags]
	orrne r0,r0,#1<<7
	strbne r0,[mikptr,#timerStatusFlags]
	mov r0,#1
tim7NoIrq:
	str r6,[mikptr,#timer7+CURRENT]
	str r7,[mikptr,#timer7+LAST_COUNT]
	mov r2,r2,ror#8
tim7NoCount:
	str r2,[mikptr,#mikTim7Bkup]
	cmp r1,#0
	beq tim7Exit
	// Prediction for next timer event cycle number
	// Sometimes timeupdates can be >2x rollover in which case
	// then CURRENT may still be negative and we can use it to
	// calc the next timer value, we just want another update ASAP
	//tmp = gSystemCycleCount;
	//tmp += (CURRENT & 0x80000000) ? 1 : ((CURRENT + 1) << divide);
	tst r6,#0x80000000
	addne r2,r4,#1
	addeq r6,r6,#1
	addeq r2,r4,r6,lsl r1
	//if (tmp < gNextTimerEvent) {
	//	gNextTimerEvent = tmp;
	//}
	ldr r1,[mikptr,#nextTimerEvent]
	cmp r2,r1
	strmi r2,[mikptr,#nextTimerEvent]
tim7Exit:
	ldmfd sp!,{r6-r7}
	bx lr

;@----------------------------------------------------------------------------
#endif // #ifdef __arm__
