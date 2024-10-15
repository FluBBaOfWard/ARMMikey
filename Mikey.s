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

	b miRegistersReset

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
	b svRefW

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
	cmp r2,#0xD0
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
	.long miRegR				;@ 0xFD40 ATTEN_A
	.long miRegR				;@ 0xFD41 ATTEN_B
	.long miRegR				;@ 0xFD42 ATTEN_C
	.long miRegR				;@ 0xFD43 ATTEN_D
	.long miRegR				;@ 0xFD44 MPAN
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

	.long miRegR				;@ 0xFD80 INTRST
	.long miRegR				;@ 0xFD81 INTSET
	.long miUnmappedR			;@ 0xFD82
	.long miUnmappedR			;@ 0xFD83
	.long miRegR				;@ 0xFD84 MAGRDY0
	.long miRegR				;@ 0xFD85 MAGRDY1
	.long miRegR				;@ 0xFD86 AUDIN
	.long miRegR				;@ 0xFD87 SYSCTL1
	.long miRegR				;@ 0xFD88 MIKEYHREV
	.long miRegR				;@ 0xFD89 MIKEYSREV
	.long miRegR				;@ 0xFD8A IODIR
	.long miRegR				;@ 0xFD8B IODAT
	.long miRegR				;@ 0xFD8C SERCTL
	.long miRegR				;@ 0xFD8D SERDAT
	.long miUnmappedR			;@ 0xFD8E
	.long miUnmappedR			;@ 0xFD8F

	.long miRegR				;@ 0xFD90 SDONEACK
	.long miRegR				;@ 0xFD91 CPUSLEEP
	.long miRegR				;@ 0xFD92 DISPCTL
	.long miRegR				;@ 0xFD93 PBKUP
	.long miRegR				;@ 0xFD94 DISPADR/DISPADRL
	.long miRegR				;@ 0xFD95 DISPADRH
	.long miUnmappedR			;@ 0xFD96
	.long miUnmappedR			;@ 0xFD97
	.long miUnmappedR			;@ 0xFD98
	.long miUnmappedR			;@ 0xFD99
	.long miUnmappedR			;@ 0xFD9A
	.long miUnmappedR			;@ 0xFD9B
	.long miRegR				;@ 0xFD9C Mtest0
	.long miRegR				;@ 0xFD9D Mtest1
	.long miRegR				;@ 0xFD9E Mtest2
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
	mov r0,#0x00
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
mikeyWrite:					;@ I/O write
;@----------------------------------------------------------------------------
	sub r2,r0,#0xFD00
	cmp r2,#0xD0
	ldrmi pc,[pc,r2,lsl#2]
	b miUnmappedW
io_write_tbl:

	.long miRegW				;@ 0xFD00 TIM0BKUP
	.long miRegW				;@ 0xFD01 TIM0CTLA
	.long miRegW				;@ 0xFD02 TIM0CNT
	.long miRegW				;@ 0xFD03 TIM0CTLB
	.long miRegW				;@ 0xFD04 TIM1BKUP
	.long miRegW				;@ 0xFD05 TIM1CTLA
	.long miRegW				;@ 0xFD06 TIM1CNT
	.long miRegW				;@ 0xFD07 TIM1CTLB
	.long miRegW				;@ 0xFD08 TIM2BKUP
	.long miRegW				;@ 0xFD09 TIM2CTLA
	.long miRegW				;@ 0xFD0A TIM2CNT
	.long miRegW				;@ 0xFD0B TIM2CTLB
	.long miRegW				;@ 0xFD0C TIM3BKUP
	.long miRegW				;@ 0xFD0D TIM3CTLA
	.long miRegW				;@ 0xFD0E TIM3CNT
	.long miRegW				;@ 0xFD0F TIM3CTLB

	.long miRegW				;@ 0xFD10 TIM4BKUP
	.long miRegW				;@ 0xFD11 TIM4CTLA
	.long miRegW				;@ 0xFD12 TIM4CNT
	.long miRegW				;@ 0xFD13 TIM4CTLB
	.long miRegW				;@ 0xFD14 TIM5BKUP
	.long miRegW				;@ 0xFD15 TIM5CTLA
	.long miRegW				;@ 0xFD16 TIM5CNT
	.long miRegW				;@ 0xFD17 TIM5CTLB
	.long miRegW				;@ 0xFD18 TIM6BKUP
	.long miRegW				;@ 0xFD19 TIM6CTLA
	.long miRegW				;@ 0xFD1A TIM6CNT
	.long miRegW				;@ 0xFD1B TIM6CTLB
	.long miRegW				;@ 0xFD1C TIM7BKUP
	.long miRegW				;@ 0xFD1D TIM7CTLA
	.long miRegW				;@ 0xFD1E TIM7CNT
	.long miRegW				;@ 0xFD1F TIM7CTLB

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
	.long miRegW				;@ 0xFD40 ATTEN_A
	.long miRegW				;@ 0xFD41 ATTEN_B
	.long miRegW				;@ 0xFD42 ATTEN_C
	.long miRegW				;@ 0xFD43 ATTEN_D
	.long miRegW				;@ 0xFD44 MPAN
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

	.long miRegW				;@ 0xFD80 INTRST
	.long miRegW				;@ 0xFD81 INTSET
	.long miUnmappedW			;@ 0xFD82
	.long miUnmappedW			;@ 0xFD83
	.long miRegW				;@ 0xFD84 MAGRDY0
	.long miRegW				;@ 0xFD85 MAGRDY1
	.long miRegW				;@ 0xFD86 AUDIN
	.long miRegW				;@ 0xFD87 SYSCTL1
	.long miRegW				;@ 0xFD88 MIKEYHREV
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
	.long miRegW				;@ 0xFD93 PBKUP
	.long miRegW				;@ 0xFD94 DISPADR/DISPADRL
	.long miRegW				;@ 0xFD95 DISPADRH
	.long miUnmappedW			;@ 0xFD96
	.long miUnmappedW			;@ 0xFD97
	.long miUnmappedW			;@ 0xFD98
	.long miUnmappedW			;@ 0xFD99
	.long miUnmappedW			;@ 0xFD9A
	.long miUnmappedW			;@ 0xFD9B
	.long miRegW				;@ 0xFD9C Mtest0
	.long miRegW				;@ 0xFD9D Mtest1
	.long miRegW				;@ 0xFD9E Mtest2
	.long miUnmappedW			;@ 0xFD9F

	.long miRegW				;@ 0xFDA0 GREEN0
	.long miRegW				;@ 0xFDA1 GREEN1
	.long miRegW				;@ 0xFDA2 GREEN2
	.long miRegW				;@ 0xFDA3 GREEN3
	.long miRegW				;@ 0xFDA4 GREEN4
	.long miRegW				;@ 0xFDA5 GREEN5
	.long miRegW				;@ 0xFDA6 GREEN6
	.long miRegW				;@ 0xFDA7 GREEN7
	.long miRegW				;@ 0xFDA8 GREEN8
	.long miRegW				;@ 0xFDA9 GREEN9
	.long miRegW				;@ 0xFDAA GREENA
	.long miRegW				;@ 0xFDAB GREENB
	.long miRegW				;@ 0xFDAC GREENC
	.long miRegW				;@ 0xFDAD GREEND
	.long miRegW				;@ 0xFDAE GREENE
	.long miRegW				;@ 0xFDAF GREENF

	.long miRegW				;@ 0xFDB0 BLUERED0
	.long miRegW				;@ 0xFDB1 BLUERED1
	.long miRegW				;@ 0xFDB2 BLUERED2
	.long miRegW				;@ 0xFDB3 BLUERED3
	.long miRegW				;@ 0xFDB4 BLUERED4
	.long miRegW				;@ 0xFDB5 BLUERED5
	.long miRegW				;@ 0xFDB6 BLUERED6
	.long miRegW				;@ 0xFDB7 BLUERED7
	.long miRegW				;@ 0xFDB8 BLUERED8
	.long miRegW				;@ 0xFDB9 BLUERED9
	.long miRegW				;@ 0xFDBA BLUEREDA
	.long miRegW				;@ 0xFDBB BLUEREDB
	.long miRegW				;@ 0xFDBC BLUEREDC
	.long miRegW				;@ 0xFDBD BLUEREDD
	.long miRegW				;@ 0xFDBE BLUEREDE
	.long miRegW				;@ 0xFDBF BLUEREDF

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
	sub r0,r0,#0x2000
	b _debugIOUnmappedW
;@----------------------------------------------------------------------------
miRegW:
	and r0,r0,#0xFF
	add r2,mikptr,#mikRegs
	strb r1,[r2,r0]
	bx lr

;@----------------------------------------------------------------------------
svRefW:						;@ 0x2001, Last scan line.
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
