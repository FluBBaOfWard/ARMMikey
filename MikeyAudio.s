//
//  MikeyAudio.s
//  Atari Lynx Mikey emulation for ARM32.
//
//  Created by Fredrik Ahlström on 2024-10-14.
//  Copyright © 2024 Fredrik Ahlström. All rights reserved.
//

#ifdef __arm__
#include "ARMMikey.i"

	.global miAudioReset
	.global miAudioMixer

	.syntax unified
	.arm

#ifdef GBA
	.section .ewram, "ax", %progbits	;@ For the GBA
#else
	.section .text						;@ For anything else
#endif
	.align 2

#define PSG_DIVIDE 16
#define PSG_ADDITION 0x00020000*PSG_DIVIDE
#define PSG_NOISE_ADD 0x00020000*PSG_DIVIDE
#define PSG_NOISE_FEED 0x6000
#define PSG_NOISE_FEED2 0x60

;@----------------------------------------------------------------------------
miAudioReset:				;@ mikptr=r12=pointer to struct
;@----------------------------------------------------------------------------
	mov r0,#0x00000800
//	str r0,[mikptr,#mikCh1Counter]
//	str r0,[mikptr,#mikCh2Counter]
//	str r0,[mikptr,#mikCh3Counter]
//	str r0,[mikptr,#mikCh4Counter]
	mov r0,#0x4000
//	str r0,[mikptr,#mikCh4LFSR]
	mov r0,#PSG_NOISE_FEED
//	str r0,[mikptr,#mikCh4Feedback]
//	ldr r0,=romSpacePtr
//	ldr r0,[r0]
//	str r0,[mikptr,#mikCh3Address]
	bx lr

;@----------------------------------------------------------------------------
miAudioMixer:				;@ r0=len, r1=dest, r12=suzptr
;@----------------------------------------------------------------------------
	stmfd sp!,{r4-r11,lr}
;@--------------------------
	ldr lr,=vol4_L

//	ldrb r2,[mikptr,#mikCh1Ctrl]
	ands r9,r2,#0x40				;@ Ch 1 on?
//	ldrbeq r3,[mikptr,#mikCh1Len]
	cmpeq r3,#0
	andne r9,r2,#0xF
	ands r4,r2,#0x30				;@ Duty cycle
	moveq r10,#0xE0000000
	cmp r4,#0x20
	movmi r10,#0xC0000000
	moveq r10,#0x80000000
	movhi r10,#0x40000000

//	ldrb r2,[mikptr,#mikCh2Ctrl]
	tst r2,#0x40					;@ Ch 2 on?
//	ldrbeq r3,[mikptr,#mikCh2Len]
	cmpeq r3,#0
	and r3,r2,#0xF
	orrne r9,r9,r3,lsl#16
	ands r4,r2,#0x30				;@ Duty cycle
	orreq r10,r10,#0x7
	cmp r4,#0x20
	orrmi r10,r10,#0x6
	orreq r10,r10,#0x4
	orrhi r10,r10,#0x2

//	ldrb r3,[mikptr,#mikCh3Trigg]
	ands r2,r3,#0x80
	movne r2,#0xF
//	ldrb r3,[mikptr,#mikCh3Ctrl]
	ands r11,r3,#4					;@ Ch 3 right?
	movne r11,r2,lsl#16
	tst r3,#8						;@ Ch 3 left?
	orrne r11,r11,r2

//	ldrb r3,[mikptr,#mikCh4Ctrl]
	ands r2,r3,#2					;@ Ch 4 on?
//	ldrbeq r2,[mikptr,#mikCh4Len]
	cmpeq r2,#0
//	ldrbne r2,[mikptr,#mikCh4FreqVol]
	and r2,r2,#0xF
	ands r4,r3,#4					;@ Ch 4 right?
	movne r4,r2
	tst r3,#8						;@ Ch 4 left?
	moveq r2,#0
	strb r2,[lr,#vol4_L-vol4_L]
	strb r4,[lr,#vol4_R-vol4_L]

//	add r2,mikptr,#mikCh1Counter
	ldmia r2,{r3-r8}

	b pcmMix
pcmMixReturn:
//	add r0,mikptr,#mikCh1Counter	;@ Counters
	stmia r0,{r3-r8}

	ldmfd sp!,{r4-r11,pc}
;@----------------------------------------------------------------------------

#ifdef NDS
	.section .itcm						;@ For the NDS ARM9
#elif GBA
	.section .iwram, "ax", %progbits	;@ For the GBA
#endif
	.align 2
;@----------------------------------------------------------------------------
;@ r0  = Length
;@ r1  = Destination
;@ r2  = Mixer register
;@ r3  = Channel 1 Wave R
;@ r4  = Channel 2 Wave L
;@ r5  = Channel 3 Sample
;@ r6  = Channel 4 Noise
;@ r7  = Channel 4 LFSR
;@ r8  = Channel 3 Sample Address
;@ r9  = Ch1 & Ch2 Volume
;@ r10 = Ch1 & Ch2 Duty cycle
;@ r11 = Ch3 Volume
;@----------------------------------------------------------------------------
pcmMix:				;@ r0=len, r1=dest, r12=suzptr
// IIIVCCCCCCCCCCC000001FFFFFFFFFFF
// I=sampleindex, V=overflow, C=counter, F=frequency
;@----------------------------------------------------------------------------
	mov r0,r0,lsl#2
mixLoop:
	mov r2,#0x80000000
innerMixLoop:
	add r3,r3,#PSG_ADDITION		;@ Ch1
	movs lr,r3,lsr#29
	addcs r3,r3,r3,lsl#17
	cmp lr,r10,lsr#29
	addcs r2,r2,r9,lsl#24

	add r4,r4,#PSG_ADDITION		;@ Ch2
	movs lr,r4,lsr#29
	addcs r4,r4,r4,lsl#17
	cmp r4,r10,lsl#29
	addcs r2,r2,r9,lsr#8

	adds r5,r5,r5,lsl#16		;@ Ch3
	addcs r8,r8,#1
	ldrb lr,[r8]
	movmi lr,lr,lsl#4
	and lr,lr,#0xF0
	mla r2,lr,r11,r2

	adds r6,r6,#PSG_NOISE_ADD	;@ Ch4
	addcs r6,r6,r6,lsl#16
	movscs r7,r7,lsr#1
//	ldrcs lr,[mikptr,#mikCh4Feedback]
	eorcs r7,r7,lr
	tst r7,#0x00000001
vol4_L:
	addne r2,r2,#0xFF00			;@ Volume left
vol4_R:
	addne r2,r2,#0xFF000000		;@ Volume right

	sub r0,r0,#1
	tst r0,#3
	bne innerMixLoop
#ifdef GBA
	eor r2,#0x80000000
	add r2,r2,r2,lsr#16
	mov r2,r2,lsr#9
	cmp r0,#0
	strbpl r2,[r1],#1
#else
	eor r2,#0x00008000
	cmp r0,#0
	strpl r2,[r1],#4
#endif
	bhi mixLoop				;@ ?? cycles according to No$gba

	b pcmMixReturn
;@----------------------------------------------------------------------------


#endif // #ifdef __arm__
