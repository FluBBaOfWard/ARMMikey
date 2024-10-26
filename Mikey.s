//
//  Mikey.s
//  Atari Lynx Mikey emulation for ARM32.
//
//  Created by Fredrik Ahlström on 2024-10-14.
//  Copyright © 2024 Fredrik Ahlström. All rights reserved.
//

#ifdef __arm__

#ifdef GBA
	#include "../Shared/gba_asm.h"
#elif NDS
	#include "../Shared/nds_asm.h"
#endif
#include "ARMMikey.i"
#include "../ARM6502/M6502.i"

#define CYCLE_PSL (246*2)

	.global miVideoInit
	.global miVideoReset
	.global miVideoSaveState
	.global miVideoLoadState
	.global miVideoGetStateSize
	.global miRunTimer0
	.global miRunTimer1
	.global miRunTimer2
	.global miRunTimer3
	.global miRunTimer4
	.global miRunTimer5
	.global miRunTimer6
	.global miRunTimer7
	.global miDoScanline
	.global mikeyRead
	.global mikeyWrite
	.global miRefW

	.syntax unified
	.arm

#if GBA
	.section .ewram, "ax", %progbits	;@ For the GBA
#else
	.section .text						;@ For anything else
#endif
	.align 2
;@----------------------------------------------------------------------------
miVideoInit:				;@ Only need to be called once
;@----------------------------------------------------------------------------

	bx lr
;@----------------------------------------------------------------------------
miVideoReset:				;@ r12=mikptr
;@----------------------------------------------------------------------------
	stmfd sp!,{r0-r3,lr}

	mov r0,mikptr
	ldr r1,=mikeySize/4
	bl memclr_					;@ Clear Suzy state

	ldr r2,=lineStateTable
	ldr r1,[r2],#4
	mov r0,#-1
	stmia mikptr,{r0-r2}		;@ Reset scanline, nextChange & lineState

	ldmfd sp!,{r0-r3,lr}
	cmp r0,#0
	adreq r0,dummyIrqFunc
	str r0,[mikptr,#mikNmiFunction]
	cmp r1,#0
	adreq r1,dummyIrqFunc
	str r1,[mikptr,#mikIrqFunction]

	str r2,[mikptr,#mikGfxRAM]

	strb r3,[mikptr,#mikSOC]

//	b miRegistersReset

dummyIrqFunc:
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
memCopy:
;@----------------------------------------------------------------------------
	ldr r3,=memcpy
;@----------------------------------------------------------------------------
thumbCallR3:
;@----------------------------------------------------------------------------
	bx r3
;@----------------------------------------------------------------------------
miRegistersReset:			;@ in r3=SOC
;@----------------------------------------------------------------------------
	adr r1,IO_Default
	mov r2,#0x30
	add r0,mikptr,#mikRegs
	stmfd sp!,{mikptr,lr}
	bl memCopy
	ldmfd sp!,{mikptr,lr}
	ldrb r1,[mikptr,#mikLCDVSize]
	b miRefW

;@----------------------------------------------------------------------------
IO_Default:
	.byte 0xA0, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.byte 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	.byte 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

;@----------------------------------------------------------------------------
miVideoSaveState:			;@ In r0=destination, r1=mikptr. Out r0=state size.
	.type	miVideoSaveState STT_FUNC
;@----------------------------------------------------------------------------
	stmfd sp!,{r4,r5,lr}
	mov r4,r0					;@ Store destination
	mov r5,r1					;@ Store mikptr (r1)

	add r1,r5,#mikeyState
	mov r2,#mikeyStateEnd-mikeyState
	bl memCopy

	ldmfd sp!,{r4,r5,lr}
	mov r0,#mikeyStateEnd-mikeyState
	bx lr
;@----------------------------------------------------------------------------
miVideoLoadState:			;@ In r0=mikptr, r1=source. Out r0=state size.
	.type	miVideoLoadState STT_FUNC
;@----------------------------------------------------------------------------
	stmfd sp!,{r4,r5,r10,lr}
	mov r5,r0					;@ Store mikptr (r0)
	mov r4,r1					;@ Store source

	add r0,r5,#mikeyState
	mov r2,#mikeyStateEnd-mikeyState
	bl memCopy

	bl clearDirtyTiles

	ldmfd sp!,{r4,r5,r10,lr}
;@----------------------------------------------------------------------------
miVideoGetStateSize:		;@ Out r0=state size.
	.type	miVideoGetStateSize STT_FUNC
;@----------------------------------------------------------------------------
	mov r0,#mikeyStateEnd-mikeyState
	bx lr

	.pool
;@----------------------------------------------------------------------------
mikeyRead:					;@ I/O read
;@----------------------------------------------------------------------------
	sub r2,r0,#0xFD00
	cmp r2,#0xC0
	ldrmi pc,[pc,r2,lsl#2]
	b miUnmappedR
io_read_tbl:
	.long miRegR				;@ 0xFD00 TIM0BKUP
	.long miRegR				;@ 0xFD01 TIM0CTLA
	.long miRegR				;@ 0xFD02 TIM0CNT
	.long miRegR				;@ 0xFD03 TIM0CTLB
	.long miRegR				;@ 0xFD04 TIM1BKUP
	.long miRegR				;@ 0xFD05 TIM1CTLA
	.long miRegR				;@ 0xFD06 TIM1CNT
	.long miRegR				;@ 0xFD07 TIM1CTLB
	.long miRegR				;@ 0xFD08 TIM2BKUP
	.long miRegR				;@ 0xFD09 TIM2CTLA
	.long miRegR				;@ 0xFD0A TIM2CNT
	.long miRegR				;@ 0xFD0B TIM2CTLB
	.long miRegR				;@ 0xFD0C TIM3BKUP
	.long miRegR				;@ 0xFD0D TIM3CTLA
	.long miRegR				;@ 0xFD0E TIM3CNT
	.long miRegR				;@ 0xFD0F TIM3CTLB

	.long miRegR				;@ 0xFD10 TIM4BKUP
	.long miRegR				;@ 0xFD11 TIM4CTLA
	.long miRegR				;@ 0xFD12 TIM4CNT
	.long miRegR				;@ 0xFD13 TIM4CTLB
	.long miRegR				;@ 0xFD14 TIM5BKUP
	.long miRegR				;@ 0xFD15 TIM5CTLA
	.long miRegR				;@ 0xFD16 TIM5CNT
	.long miRegR				;@ 0xFD17 TIM5CTLB
	.long miRegR				;@ 0xFD18 TIM6BKUP
	.long miRegR				;@ 0xFD19 TIM6CTLA
	.long miRegR				;@ 0xFD1A TIM6CNT
	.long miRegR				;@ 0xFD1B TIM6CTLB
	.long miRegR				;@ 0xFD1C TIM7BKUP
	.long miRegR				;@ 0xFD1D TIM7CTLA
	.long miRegR				;@ 0xFD1E TIM7CNT
	.long miRegR				;@ 0xFD1F TIM7CTLB

	.long miRegR				;@ 0xFD20 AUD0VOL
	.long miRegR				;@ 0xFD21 AUD0SHFTFB
	.long miRegR				;@ 0xFD22 AUD0OUTVAL
	.long miRegR				;@ 0xFD23 AUD0L8SHFT
	.long miRegR				;@ 0xFD24 AUD0TBACK
	.long miRegR				;@ 0xFD25 AUD0CTL
	.long miRegR				;@ 0xFD26 AUD0COUNT
	.long miRegR				;@ 0xFD27 AUD0MISC
	.long miRegR				;@ 0xFD28 AUD1VOL
	.long miRegR				;@ 0xFD29 AUD1SHFTFB
	.long miRegR				;@ 0xFD2A AUD1OUTVAL
	.long miRegR				;@ 0xFD2B AUD1L8SHFT
	.long miRegR				;@ 0xFD2C AUD1TBACK
	.long miRegR				;@ 0xFD2D AUD1CTL
	.long miRegR				;@ 0xFD2E AUD1COUNT
	.long miRegR				;@ 0xFD2F AUD1MISC

	.long miRegR				;@ 0xFD30 AUD2VOL
	.long miRegR				;@ 0xFD31 AUD2SHFTFB
	.long miRegR				;@ 0xFD32 AUD2OUTVAL
	.long miRegR				;@ 0xFD33 AUD2L8SHFT
	.long miRegR				;@ 0xFD34 AUD2TBACK
	.long miRegR				;@ 0xFD35 AUD2CTL
	.long miRegR				;@ 0xFD36 AUD2COUNT
	.long miRegR				;@ 0xFD37 AUD2MISC
	.long miRegR				;@ 0xFD38 AUD3VOL
	.long miRegR				;@ 0xFD39 AUD3SHFTFB
	.long miRegR				;@ 0xFD3A AUD3OUTVAL
	.long miRegR				;@ 0xFD3B AUD3L8SHFT
	.long miRegR				;@ 0xFD3C AUD3TBACK
	.long miRegR				;@ 0xFD3D AUD3CTL
	.long miRegR				;@ 0xFD3E AUD3COUNT
	.long miRegR				;@ 0xFD3F AUD3MISC

		// Lynx2 Regs
	.long miImportantR			;@ 0xFD40 ATTEN_A
	.long miImportantR			;@ 0xFD41 ATTEN_B
	.long miImportantR			;@ 0xFD42 ATTEN_C
	.long miImportantR			;@ 0xFD43 ATTEN_D
	.long miImportantR			;@ 0xFD44 MPAN
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

	.long miRegR				;@ 0xFD50 MSTEREO
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
	.long miRegR				;@ 0xFD8B IODAT
	.long miRegR				;@ 0xFD8C SERCTL
	.long miRegR				;@ 0xFD8D SERDAT
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
	ldr r2,=0x826EBAD0
;@----------------------------------------------------------------------------
miImportantR:
	mov r11,r11					;@ No$GBA breakpoint
	stmfd sp!,{r0,mikptr,lr}
	bl _debugIOUnimplR
	ldmfd sp!,{r0,mikptr,lr}
;@----------------------------------------------------------------------------
miRegR:
	and r0,r0,#0xFF
	add r2,mikptr,#mikRegs
	ldrb r0,[r2,r0]
	bx lr
	.pool

;@----------------------------------------------------------------------------
miIntRstR:					;@ 0xFD80
;@----------------------------------------------------------------------------
;@----------------------------------------------------------------------------
miIntSetR:					;@ 0xFD81
;@----------------------------------------------------------------------------
	ldrb r0,[mikptr,#timerStatusFlags]
	bx lr
;@----------------------------------------------------------------------------
miMagRdy0R:					;@ 0xFD84
miMagRdy1R:					;@ 0xFD85
;@----------------------------------------------------------------------------
	mov r0,#0
	bx lr
;@----------------------------------------------------------------------------
miAudInR:					;@ 0xFD86
;@----------------------------------------------------------------------------
	mov r0,#0x80				;@ bit 7 = audio comparator result
	bx lr
;@----------------------------------------------------------------------------
miMikeyHRevR:				;@ 0xFD88
;@----------------------------------------------------------------------------
	mov r0,#1
	bx lr
;@----------------------------------------------------------------------------
miHandyDetectR:				;@ 0xFD97
;@----------------------------------------------------------------------------
	mov r0,#42
	bx lr
;@----------------------------------------------------------------------------
mikeyWrite:					;@ I/O write
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

	.long miRegW				;@ 0xFD20 AUD0VOL
	.long miRegW				;@ 0xFD21 AUD0SHFTFB
	.long miRegW				;@ 0xFD22 AUD0OUTVAL
	.long miRegW				;@ 0xFD23 AUD0L8SHFT
	.long miRegW				;@ 0xFD24 AUD0TBACK
	.long miRegW				;@ 0xFD25 AUD0CTL
	.long miRegW				;@ 0xFD26 AUD0COUNT
	.long miRegW				;@ 0xFD27 AUD0MISC
	.long miRegW				;@ 0xFD28 AUD1VOL
	.long miRegW				;@ 0xFD29 AUD1SHFTFB
	.long miRegW				;@ 0xFD2A AUD1OUTVAL
	.long miRegW				;@ 0xFD2B AUD1L8SHFT
	.long miRegW				;@ 0xFD2C AUD1TBACK
	.long miRegW				;@ 0xFD2D AUD1CTL
	.long miRegW				;@ 0xFD2E AUD1COUNT
	.long miRegW				;@ 0xFD2F AUD1MISC

	.long miRegW				;@ 0xFD30 AUD2VOL
	.long miRegW				;@ 0xFD31 AUD2SHFTFB
	.long miRegW				;@ 0xFD32 AUD2OUTVAL
	.long miRegW				;@ 0xFD33 AUD2L8SHFT
	.long miRegW				;@ 0xFD34 AUD2TBACK
	.long miRegW				;@ 0xFD35 AUD2CTL
	.long miRegW				;@ 0xFD36 AUD2COUNT
	.long miRegW				;@ 0xFD37 AUD2MISC
	.long miRegW				;@ 0xFD38 AUD3VOL
	.long miRegW				;@ 0xFD39 AUD3SHFTFB
	.long miRegW				;@ 0xFD3A AUD3OUTVAL
	.long miRegW				;@ 0xFD3B AUD3L8SHFT
	.long miRegW				;@ 0xFD3C AUD3TBACK
	.long miRegW				;@ 0xFD3D AUD3CTL
	.long miRegW				;@ 0xFD3E AUD3COUNT
	.long miRegW				;@ 0xFD3F AUD3MISC

	// Lynx2 Regs
	.long miImportantW			;@ 0xFD40 ATTEN_A
	.long miImportantW			;@ 0xFD41 ATTEN_B
	.long miImportantW			;@ 0xFD42 ATTEN_C
	.long miImportantW			;@ 0xFD43 ATTEN_D
	.long miImportantW			;@ 0xFD44 MPAN
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

	.long miRegW				;@ 0xFD50 MSTEREO
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
	.long miRegW				;@ 0xFD87 SYSCTL1
	.long miReadOnlyW			;@ 0xFD88 MIKEYHREV
	.long miRegW				;@ 0xFD89 MIKEYSREV
	.long miRegW				;@ 0xFD8A IODIR
	.long miRegW				;@ 0xFD8B IODAT
	.long miRegW				;@ 0xFD8C SERCTL
	.long miRegW				;@ 0xFD8D SERDAT
	.long miUnmappedW			;@ 0xFD8E
	.long miUnmappedW			;@ 0xFD8F

	.long miRegW				;@ 0xFD90 SDONEACK
	.long miRegW				;@ 0xFD91 CPUSLEEP
	.long miRegW				;@ 0xFD92 DISPCTL
	.long miImportantW			;@ 0xFD93 PBKUP
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
	and r0,r0,#0xFF
	add r2,mikptr,#mikRegs
	strb r1,[r2,r0]
	ldr r2,=debugIOUnimplW
	bx r2
;@----------------------------------------------------------------------------
miReadOnlyW:
;@----------------------------------------------------------------------------
miUnmappedW:
;@----------------------------------------------------------------------------
	b _debugIOUnmappedW
;@----------------------------------------------------------------------------
miRegW:
	and r0,r0,#0xFF
	add r2,mikptr,#mikRegs
	strb r1,[r2,r0]
	bx lr

;@----------------------------------------------------------------------------
miTimCtlAW:					;@ Timer X Control A
;@----------------------------------------------------------------------------
	tst r1,#0x40				;@ Check "Reset Timer Done".
	and r1,r1,#0xBF				;@ "Reset Timer Done" should not be preserved?
	and r0,r0,#0xFF
	add r2,mikptr,#mikRegs
	strb r1,[r2,r0]
	ldr r1,[mikptr,#systemCycleCount]
	str r1,[mikptr,#nextTimerEvent]
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
	ldrne r0,[mikptr,#systemCycleCount]
	strne r0,[mikptr,#timer0+LAST_COUNT]
	strne r0,[mikptr,#nextTimerEvent]
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
	ldrne r0,[mikptr,#systemCycleCount]
	strne r0,[mikptr,#timer1+LAST_COUNT]
	strne r0,[mikptr,#nextTimerEvent]
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
	ldrne r0,[mikptr,#systemCycleCount]
	strne r0,[mikptr,#timer2+LAST_COUNT]
	strne r0,[mikptr,#nextTimerEvent]
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
	ldrne r0,[mikptr,#systemCycleCount]
	strne r0,[mikptr,#timer3+LAST_COUNT]
	strne r0,[mikptr,#nextTimerEvent]
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
	ldrne r0,[mikptr,#systemCycleCount]
	strne r0,[mikptr,#timer4+LAST_COUNT]
	strne r0,[mikptr,#nextTimerEvent]
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
	ldrne r0,[mikptr,#systemCycleCount]
	strne r0,[mikptr,#timer5+LAST_COUNT]
	strne r0,[mikptr,#nextTimerEvent]
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
	ldrne r0,[mikptr,#systemCycleCount]
	strne r0,[mikptr,#timer6+LAST_COUNT]
	strne r0,[mikptr,#nextTimerEvent]
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
	ldrne r0,[mikptr,#systemCycleCount]
	strne r0,[mikptr,#timer7+LAST_COUNT]
	strne r0,[mikptr,#nextTimerEvent]
	bx lr
;@----------------------------------------------------------------------------
miTimCtlBW:					;@ Timer X Control B (0xFDX3)
;@----------------------------------------------------------------------------
	and r1,r1,#0x0F
	and r0,r0,#0xFF
	add r2,mikptr,#mikRegs
	strb r1,[r2,r0]
	bx lr

;@----------------------------------------------------------------------------
miTim0CntW:					;@ Timer 0 Count (0xFD02)
;@----------------------------------------------------------------------------
	and r1,r1,#0xFF
	strb r1,[mikptr,#mikTim0Cnt]
	str r1,[mikptr,#timer0+CURRENT]
	ldr r1,[mikptr,#systemCycleCount]
	str r1,[mikptr,#nextTimerEvent]
	bx lr
;@----------------------------------------------------------------------------
miTim1CntW:					;@ Timer 1 Count (0xFD06)
;@----------------------------------------------------------------------------
	and r1,r1,#0xFF
	strb r1,[mikptr,#mikTim1Cnt]
	str r1,[mikptr,#timer1+CURRENT]
	ldr r1,[mikptr,#systemCycleCount]
	str r1,[mikptr,#nextTimerEvent]
	bx lr
;@----------------------------------------------------------------------------
miTim2CntW:					;@ Timer 2 Count (0xFD0A)
;@----------------------------------------------------------------------------
	and r1,r1,#0xFF
	strb r1,[mikptr,#mikTim2Cnt]
	str r1,[mikptr,#timer2+CURRENT]
	ldr r1,[mikptr,#systemCycleCount]
	str r1,[mikptr,#nextTimerEvent]
	bx lr
;@----------------------------------------------------------------------------
miTim3CntW:					;@ Timer 3 Count (0xFD0E)
;@----------------------------------------------------------------------------
	and r1,r1,#0xFF
	strb r1,[mikptr,#mikTim3Cnt]
	str r1,[mikptr,#timer3+CURRENT]
	ldr r1,[mikptr,#systemCycleCount]
	str r1,[mikptr,#nextTimerEvent]
	bx lr
;@----------------------------------------------------------------------------
miTim4CntW:					;@ Timer 4 Count (0xFD12)
;@----------------------------------------------------------------------------
	and r1,r1,#0xFF
	strb r1,[mikptr,#mikTim4Cnt]
	str r1,[mikptr,#timer4+CURRENT]
	ldr r1,[mikptr,#systemCycleCount]
	str r1,[mikptr,#nextTimerEvent]
	bx lr
;@----------------------------------------------------------------------------
miTim5CntW:					;@ Timer 5 Count (0xFD16)
;@----------------------------------------------------------------------------
	and r1,r1,#0xFF
	strb r1,[mikptr,#mikTim5Cnt]
	str r1,[mikptr,#timer5+CURRENT]
	ldr r1,[mikptr,#systemCycleCount]
	str r1,[mikptr,#nextTimerEvent]
	bx lr
;@----------------------------------------------------------------------------
miTim6CntW:					;@ Timer 6 Count (0xFD1A)
;@----------------------------------------------------------------------------
	and r1,r1,#0xFF
	strb r1,[mikptr,#mikTim6Cnt]
	str r1,[mikptr,#timer6+CURRENT]
	ldr r1,[mikptr,#systemCycleCount]
	str r1,[mikptr,#nextTimerEvent]
	bx lr
;@----------------------------------------------------------------------------
miTim7CntW:					;@ Timer 7 Count (0xFD1E)
;@----------------------------------------------------------------------------
	and r1,r1,#0xFF
	strb r1,[mikptr,#mikTim7Cnt]
	str r1,[mikptr,#timer7+CURRENT]
	ldr r1,[mikptr,#systemCycleCount]
	str r1,[mikptr,#nextTimerEvent]
	bx lr

;@----------------------------------------------------------------------------
miIntRstW:					;@ Interrupt Reset (0xFD80)
;@----------------------------------------------------------------------------
	ldrb r0,[mikptr,#timerStatusFlags]
	bic r0,r0,r1
	strb r0,[mikptr,#timerStatusFlags]
	b cpuSetIrqPin
;@----------------------------------------------------------------------------
miIntSetW:					;@ Interrupt Set (0xFD81)
;@----------------------------------------------------------------------------
	ldrb r0,[mikptr,#timerStatusFlags]
	orr r0,r0,r1
	strb r0,[mikptr,#timerStatusFlags]
	b cpuSetIrqPin

;@----------------------------------------------------------------------------
miPaletteGW:				;@ Green Palette (0xFDAX)
;@----------------------------------------------------------------------------
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
	and r0,r0,#0xF
	add r2,mikptr,#mikPaletteBR
	strb r1,[r2,r0]
	add r2,mikptr,#mikPalette
	strb r1,[r2,r0,lsl#2]
	bx lr

;@----------------------------------------------------------------------------
miRefW:						;@ 0x2001, Last scan line.
;@----------------------------------------------------------------------------
	strb r1,[mikptr,#mikLCDVSize]
	cmp r1,#0x9E
	movmi r1,#0x9E
	cmp r1,#0xC8
	movpl r1,#0xC8
	add r1,r1,#1
	str r1,lineStateLastLine
	mov r0,r1
	b setScreenRefresh


;@----------------------------------------------------------------------------
miRunTimer0:
	.type	miRunTimer0 STT_FUNC
;@----------------------------------------------------------------------------
	mov r0,#0
	ldr mikptr,=mikey_0
	ldr r2,[mikptr,#mikTim0Bkup]
	movs r1,r2,lsl#21
	bxcc lr						;@ CtlA Count Enabled?
	stmfd sp!,{r4-r8,lr}
	bic r2,r2,#0x0B000000		;@ CtlB clear borrow in/out, last clock
	// Ordinary clocked mode as opposed to linked mode
	// 16MHz clock downto 1us == cyclecount >> 4
	//divide = (4 + (CtlA & CLOCK_SEL));
	mov r1,r1,lsr#29			;@ CtlA Clock Select
	add r1,r1,#4
	ldr r5,[mikptr,#systemCycleCount]
	ldr r6,[mikptr,#timer0+CURRENT]
	ldr r7,[mikptr,#timer0+LAST_COUNT]
	//decval = (gSystemCycleCount - LAST_COUNT) >> divide;
	sub r4,r5,r7
	movs r4,r4,lsr r1
	beq tim0NoCount				;@ decval?
	mov r2,r2,ror#24
	orr r2,r2,#2				;@ CtlB Borrow in, because we count
	add r7,r7,r4,lsl r1
	sub r2,r2,r4,lsl#24
	subs r6,r6,r4
	orreq r2,r2,#4				;@ CtlB Last clock
	bpl tim0NoIrq
	orr r2,r2,#1				;@ CtlB Borrow out
	//tst r2,#0x00100000		;@ CtlA & ENABLE_RELOAD
	and r4,r2,#0xFF00
	add r4,r4,#0x0100
	add r2,r2,r4,lsl#16
	add r6,r6,r4,lsr#8
//	biceq r2,r2,#0xFF000000		;@ No reload, clear count.
//	orreq r2,r2,#8				;@ CtlB Timer done
//	moveq r6,#0
	tst r2,#0x800000			;@ CtlA Interrupt Enable?
	ldrbne r0,[mikptr,#timerStatusFlags]
	orrne r0,r0,#1<<0
	strbne r0,[mikptr,#timerStatusFlags]
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
	addne r5,r5,#1
	addeq r6,r6,#1
	addeq r5,r5,r6,lsl r1
	//if (tmp < gNextTimerEvent) {
	//	gNextTimerEvent = tmp;
	//}
	ldr r1,[mikptr,#nextTimerEvent]
	cmp r5,r1
	strcc r5,[mikptr,#nextTimerEvent]
	ldmfd sp!,{r4-r8,lr}
	bx lr

;@----------------------------------------------------------------------------
miRunTimer2:
	.type	miRunTimer2 STT_FUNC
;@----------------------------------------------------------------------------
	mov r0,#0
	ldr mikptr,=mikey_0
	ldr r2,[mikptr,#mikTim2Bkup]
	movs r1,r2,lsl#21
	bxcc lr						;@ CtlA Count Enabled?
	stmfd sp!,{r4-r8,lr}
	bic r2,r2,#0x0B000000		;@ CtlB clear borrow in/out, last clock
	mov r1,r1,lsr#29			;@ CtlA Clock Select
	// 16MHz clock downto 1us == cyclecount >> 4
	//divide = (4 + (CtlA & CLOCK_SEL));
	cmp r1,#7					;@ Link mode?
	moveq r1,#0
	ldrbeq r4,[mikptr,#mikTim0CtlB]
	and r4,r4,#1
	addne r1,r1,#4
	ldr r5,[mikptr,#systemCycleCount]
	ldr r6,[mikptr,#timer2+CURRENT]
	ldr r7,[mikptr,#timer2+LAST_COUNT]
	//decval = (gSystemCycleCount - LAST_COUNT) >> divide;
	subne r4,r5,r7
	movs r4,r4,lsr r1
	beq tim2NoCount				;@ decval?
	mov r2,r2,ror#24
	orr r2,r2,#2				;@ CtlB Borrow in, because we count
	add r7,r7,r4,lsl r1
	sub r2,r2,r4,lsl#24
	subs r6,r6,r4
	orreq r2,r2,#4				;@ CtlB Last clock
	bpl tim2NoIrq
	orr r2,r2,#1				;@ CtlB Borrow out
	//tst r2,#0x00100000		;@ CtlA & ENABLE_RELOAD
	and r4,r2,#0xFF00
	add r4,r4,#0x0100
	add r2,r2,r4,lsl#16
	add r6,r6,r4,lsr#8
//	biceq r2,r2,#0xFF000000		;@ No reload, clear count.
//	orreq r2,r2,#8				;@ CtlB Timer done
//	moveq r6,#0
	tst r2,#0x800000			;@ CtlA Interrupt Enable?
	ldrbne r0,[mikptr,#timerStatusFlags]
	orrne r0,r0,#1<<2
	strbne r0,[mikptr,#timerStatusFlags]
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
	ldmfd sp!,{r4-r8,lr}
	bx lr

;@----------------------------------------------------------------------------
miRunTimer1:
	.type	miRunTimer1 STT_FUNC
;@----------------------------------------------------------------------------
	mov r0,#0
	ldr mikptr,=mikey_0
	ldr r2,[mikptr,#mikTim1Bkup]
	movs r1,r2,lsl#21
	bxcc lr						;@ CtlA Count Enabled?
	tst r2,#0x08000000			;@ CtlB Timer done?
	bxne lr
	mov r1,r1,lsr#29			;@ CtlA Clock Select
	cmp r1,#7					;@ Link mode?
	bxeq lr
	stmfd sp!,{r4-r8,lr}
	bic r2,r2,#0x0B000000		;@ CtlB clear borrow in/out, last clock
	// Ordinary clocked mode as opposed to linked mode
	// 16MHz clock downto 1us == cyclecount >> 4
	//divide = (4 + (CtlA & CLOCK_SEL));
	add r1,r1,#4
	ldr r5,[mikptr,#systemCycleCount]
	ldr r6,[mikptr,#timer1+CURRENT]
	ldr r7,[mikptr,#timer1+LAST_COUNT]
	//decval = (gSystemCycleCount - LAST_COUNT) >> divide;
	sub r4,r5,r7
	movs r4,r4,lsr r1
	beq tim1NoCount				;@ decval?
	mov r2,r2,ror#24
	orr r2,r2,#2				;@ CtlB Borrow in, because we count
	add r7,r7,r4,lsl r1
	sub r2,r2,r4,lsl#24
	subs r6,r6,r4
	orreq r2,r2,#4				;@ CtlB Last clock
	bpl tim1NoIrq
	orr r2,r2,#1				;@ CtlB Borrow out
	tst r2,#0x00100000			;@ CtlA & ENABLE_RELOAD
	andne r4,r2,#0xFF00
	addne r4,r4,#0x0100
	addne r2,r2,r4,lsl#16
	addne r6,r6,r4,lsr#8
	biceq r2,r2,#0xFF000000		;@ No reload, clear count.
	orreq r2,r2,#8				;@ CtlB Timer done
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
	addne r5,r5,#1
	addeq r6,r6,#1
	addeq r5,r5,r6,lsl r1
	//if (tmp < gNextTimerEvent) {
	//	gNextTimerEvent = tmp;
	//}
	ldr r1,[mikptr,#nextTimerEvent]
	cmp r5,r1
	strcc r5,[mikptr,#nextTimerEvent]
	ldmfd sp!,{r4-r8,lr}
	bx lr

;@----------------------------------------------------------------------------
miRunTimer3:
	.type	miRunTimer3 STT_FUNC
;@----------------------------------------------------------------------------
	mov r0,#0
	ldr mikptr,=mikey_0
	ldr r2,[mikptr,#mikTim3Bkup]
	movs r1,r2,lsl#21
	bxcc lr						;@ CtlA Count Enabled?
	tst r2,#0x08000000			;@ CtlB Timer done?
	bxne lr
	stmfd sp!,{r4-r8,lr}
	bic r2,r2,#0x0B000000		;@ CtlB clear borrow in/out, last clock
	mov r1,r1,lsr#29			;@ CtlA Clock Select
	// Ordinary clocked mode as opposed to linked mode
	// 16MHz clock downto 1us == cyclecount >> 4
	//divide = (4 + (CtlA & CLOCK_SEL));
	cmp r1,#7					;@ Link mode?
	moveq r1,#0
	ldrbeq r4,[mikptr,#mikTim1CtlB]
	and r4,r4,#1
	addne r1,r1,#4
	ldr r5,[mikptr,#systemCycleCount]
	ldr r6,[mikptr,#timer3+CURRENT]
	ldr r7,[mikptr,#timer3+LAST_COUNT]
	//decval = (gSystemCycleCount - LAST_COUNT) >> divide;
	subne r4,r5,r7
	movs r4,r4,lsr r1
	beq tim3NoCount				;@ decval?
	mov r2,r2,ror#24
	orr r2,r2,#2				;@ CtlB Borrow in, because we count
	add r7,r7,r4,lsl r1
	sub r2,r2,r4,lsl#24
	subs r6,r6,r4
	orreq r2,r2,#4				;@ CtlB Last clock
	bpl tim3NoIrq
	orr r2,r2,#1				;@ CtlB Borrow out
	tst r2,#0x00100000			;@ CtlA & ENABLE_RELOAD
	andne r4,r2,#0xFF00
	addne r4,r4,#0x0100
	addne r2,r2,r4,lsl#16
	addne r6,r6,r4,lsr#8
	biceq r2,r2,#0xFF000000		;@ No reload, clear count.
	orreq r2,r2,#8				;@ CtlB Timer done
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
	addne r5,r5,#1
	addeq r6,r6,#1
	addeq r5,r5,r6,lsl r1
	//if (tmp < gNextTimerEvent) {
	//	gNextTimerEvent = tmp;
	//}
	ldr r1,[mikptr,#nextTimerEvent]
	cmp r5,r1
	strcc r5,[mikptr,#nextTimerEvent]
tim3Exit:
	ldmfd sp!,{r4-r8,lr}
	bx lr

;@----------------------------------------------------------------------------
miRunTimer5:
	.type	miRunTimer5 STT_FUNC
;@----------------------------------------------------------------------------
	mov r0,#0
	ldr mikptr,=mikey_0
	ldr r2,[mikptr,#mikTim5Bkup]
	movs r1,r2,lsl#21
	bxcc lr						;@ CtlA Count Enabled?
	tst r2,#0x08000000			;@ CtlB Timer done?
	bxne lr
	stmfd sp!,{r4-r8,lr}
	bic r2,r2,#0x0B000000		;@ CtlB clear borrow in/out, last clock
	mov r1,r1,lsr#29			;@ CtlA Clock Select
	// Ordinary clocked mode as opposed to linked mode
	// 16MHz clock downto 1us == cyclecount >> 4
	//divide = (4 + (CtlA & CLOCK_SEL));
	cmp r1,#7					;@ Link mode?
	moveq r1,#0
	ldrbeq r4,[mikptr,#mikTim3CtlB]
	and r4,r4,#1
	addne r1,r1,#4
	ldr r5,[mikptr,#systemCycleCount]
	ldr r6,[mikptr,#timer5+CURRENT]
	ldr r7,[mikptr,#timer5+LAST_COUNT]
	//decval = (gSystemCycleCount - LAST_COUNT) >> divide;
	subne r4,r5,r7
	movs r4,r4,lsr r1
	beq tim5NoCount				;@ decval?
	mov r2,r2,ror#24
	orr r2,r2,#2				;@ CtlB Borrow in, because we count
	add r7,r7,r4,lsl r1
	sub r2,r2,r4,lsl#24
	subs r6,r6,r4
	orreq r2,r2,#4				;@ CtlB Last clock
	bpl tim5NoIrq
	orr r2,r2,#1				;@ CtlB Borrow out
	tst r2,#0x00100000			;@ CtlA & ENABLE_RELOAD
	andne r4,r2,#0xFF00
	addne r4,r4,#0x0100
	addne r2,r2,r4,lsl#16
	addne r6,r6,r4,lsr#8
	biceq r2,r2,#0xFF000000		;@ No reload, clear count.
	orreq r2,r2,#8				;@ CtlB Timer done
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
	addne r5,r5,#1
	addeq r6,r6,#1
	addeq r5,r5,r6,lsl r1
	//if (tmp < gNextTimerEvent) {
	//	gNextTimerEvent = tmp;
	//}
	ldr r1,[mikptr,#nextTimerEvent]
	cmp r5,r1
	strcc r5,[mikptr,#nextTimerEvent]
tim5Exit:
	ldmfd sp!,{r4-r8,lr}
	bx lr

;@----------------------------------------------------------------------------
miRunTimer7:
	.type	miRunTimer7 STT_FUNC
;@----------------------------------------------------------------------------
	mov r0,#0
	ldr mikptr,=mikey_0
	ldr r2,[mikptr,#mikTim7Bkup]
	movs r1,r2,lsl#21
	bxcc lr						;@ CtlA Count Enabled?
	tst r2,#0x08000000			;@ CtlB Timer done?
	bxne lr
	stmfd sp!,{r4-r8,lr}
	bic r2,r2,#0x0B000000		;@ CtlB clear borrow in/out, last clock
	mov r1,r1,lsr#29			;@ CtlA Clock Select
	// Ordinary clocked mode as opposed to linked mode
	// 16MHz clock downto 1us == cyclecount >> 4
	//divide = (4 + (CtlA & CLOCK_SEL));
	cmp r1,#7					;@ Link mode?
	moveq r1,#0
	ldrbeq r4,[mikptr,#mikTim5CtlB]
	and r4,r4,#1
	addne r1,r1,#4
	ldr r5,[mikptr,#systemCycleCount]
	ldr r6,[mikptr,#timer7+CURRENT]
	ldr r7,[mikptr,#timer7+LAST_COUNT]
	//decval = (gSystemCycleCount - LAST_COUNT) >> divide;
	subne r4,r5,r7
	movs r4,r4,lsr r1
	beq tim7NoCount				;@ decval?
	mov r2,r2,ror#24
	orr r2,r2,#2				;@ CtlB Borrow in, because we count
	add r7,r7,r4,lsl r1
	sub r2,r2,r4,lsl#24
	subs r6,r6,r4
	orreq r2,r2,#4				;@ CtlB Last clock
	bpl tim7NoIrq
	orr r2,r2,#1				;@ CtlB Borrow out
	tst r2,#0x00100000			;@ CtlA & ENABLE_RELOAD
	andne r4,r2,#0xFF00
	addne r4,r4,#0x0100
	addne r2,r2,r4,lsl#16
	addne r6,r6,r4,lsr#8
	biceq r2,r2,#0xFF000000		;@ No reload, clear count.
	orreq r2,r2,#8				;@ CtlB Timer done
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
	addne r5,r5,#1
	addeq r6,r6,#1
	addeq r5,r5,r6,lsl r1
	//if (tmp < gNextTimerEvent) {
	//	gNextTimerEvent = tmp;
	//}
	ldr r1,[mikptr,#nextTimerEvent]
	cmp r5,r1
	strcc r5,[mikptr,#nextTimerEvent]
tim7Exit:
	ldmfd sp!,{r4-r8,lr}
	bx lr

;@----------------------------------------------------------------------------
miRunTimer6:
	.type	miRunTimer6 STT_FUNC
;@----------------------------------------------------------------------------
	mov r0,#0
	ldr mikptr,=mikey_0
	ldr r2,[mikptr,#mikTim6Bkup]
	movs r1,r2,lsl#21
	bxcc lr						;@ CtlA Count Enabled?
	tst r2,#0x08000000			;@ CtlB Timer done?
	bxne lr
	mov r1,r1,lsr#29			;@ CtlA Clock Select
	cmp r1,#7					;@ Link mode?
	bxeq lr
	stmfd sp!,{r4-r8,lr}
	bic r2,r2,#0x0B000000		;@ CtlB clear borrow in/out, last clock
	// Ordinary clocked mode as opposed to linked mode
	// 16MHz clock downto 1us == cyclecount >> 4
	//divide = (4 + (CtlA & CLOCK_SEL));
	add r1,r1,#4
	ldr r5,[mikptr,#systemCycleCount]
	ldr r6,[mikptr,#timer6+CURRENT]
	ldr r7,[mikptr,#timer6+LAST_COUNT]
	//decval = (gSystemCycleCount - LAST_COUNT) >> divide;
	sub r4,r5,r7
	movs r4,r4,lsr r1
	beq tim6NoCount				;@ decval?
	mov r2,r2,ror#24
	orr r2,r2,#2				;@ CtlB Borrow in, because we count
	add r7,r7,r4,lsl r1
	sub r2,r2,r4,lsl#24
	subs r6,r6,r4
	orreq r2,r2,#4				;@ CtlB Last clock
	bpl tim6NoIrq
	orr r2,r2,#1				;@ CtlB Borrow out
	tst r2,#0x00100000			;@ CtlA & ENABLE_RELOAD
	andne r4,r2,#0xFF00
	addne r4,r4,#0x0100
	addne r2,r2,r4,lsl#16
	addne r6,r6,r4,lsr#8
	biceq r2,r2,#0xFF000000		;@ No reload, clear count.
	orreq r2,r2,#8				;@ CtlB Timer done
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
	addne r5,r5,#1
	addeq r6,r6,#1
	addeq r5,r5,r6,lsl r1
	//if (tmp < gNextTimerEvent) {
	//	gNextTimerEvent = tmp;
	//}
	ldr r1,[mikptr,#nextTimerEvent]
	cmp r5,r1
	strcc r5,[mikptr,#nextTimerEvent]
	ldmfd sp!,{r4-r8,lr}
	bx lr

;@----------------------------------------------------------------------------
newFrame:					;@ Called before line 0
;@----------------------------------------------------------------------------
	bx lr
;@----------------------------------------------------------------------------
endFrame:
;@----------------------------------------------------------------------------
	stmfd sp!,{lr}
	bl gfxEndFrame

	ldmfd sp!,{pc}

;@----------------------------------------------------------------------------
frameEndHook:
	adr r2,lineStateTable
	ldr r1,[r2],#4
	mov r0,#-1
	stmia mikptr,{r0-r2}		;@ Reset scanline, nextChange & lineState
	bx lr

;@----------------------------------------------------------------------------
lineStateTable:
	.long 0, newFrame			;@ zeroLine
	.long 102, endFrame			;@ After last visible scanline
lineStateLastLine:
	.long 105, frameEndHook		;@ totalScanlines
;@----------------------------------------------------------------------------
#ifdef GBA
	.section .iwram, "ax", %progbits	;@ For the GBA
	.align 2
#endif
;@----------------------------------------------------------------------------
redoScanline:
;@----------------------------------------------------------------------------
	ldr r2,[mikptr,#lineState]
	ldmia r2!,{r0,r1}
	stmib mikptr,{r1,r2}		;@ Write nextLineChange & lineState
	stmfd sp!,{lr}
	mov lr,pc
	bx r0
	ldmfd sp!,{lr}
;@----------------------------------------------------------------------------
miDoScanline:
;@----------------------------------------------------------------------------
	ldmia mikptr,{r0,r1}		;@ Read scanLine & nextLineChange
	add r0,r0,#1
	cmp r0,r1
	bpl redoScanline
	str r0,[mikptr,#scanline]
;@----------------------------------------------------------------------------
checkScanlineIRQ:
;@----------------------------------------------------------------------------
	stmfd sp!,{lr}


	ldr r0,[mikptr,#scanline]
	subs r0,r0,#159				;@ Return from emulation loop on this scanline
	movne r0,#1
	ldmfd sp!,{pc}

;@----------------------------------------------------------------------------
#ifdef GBA
	.section .sbss				;@ For the GBA
#else
	.section .bss
#endif
	.align 2
CHR_DECODE:
	.space 0x200

#endif // #ifdef __arm__
