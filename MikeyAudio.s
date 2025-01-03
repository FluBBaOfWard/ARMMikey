//
//  MikeyAudio.s
//  Atari Lynx Mikey (Sound) emulation for ARM32.
//
//  Created by Fredrik Ahlström on 2024-10-14.
//  Copyright © 2024-2025 Fredrik Ahlström. All rights reserved.
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

;@----------------------------------------------------------------------------
miAudioReset:				;@ mikptr=r10=pointer to struct
;@----------------------------------------------------------------------------
	mov r0,#0
	bx lr

;@----------------------------------------------------------------------------
miAudioMixer:				;@ r0=len, r1=dest, r10=mikptr
;@----------------------------------------------------------------------------
	stmfd sp!,{r4-r11,lr}
;@--------------------------
	ldrb r3,[mikptr,#mikAud0TBack]
	ldrb r4,[mikptr,#mikAud0Count]
	orr r3,r3,r4,lsl#24
	add r3,r3,#0x01
	ldr r4,[mikptr,#audio0+WAVESHAPER]
	mov r4,r4,ror#16
	mov r5,#0x01000000
	ldrb r8,[mikptr,#mikAud0Ctl]
	and r6,r8,#7				;@ CLOCK_SEL
	mov r5,r5,lsr r6
	ldrb r6,[mikptr,#mikAud0OutVal]
	mov r6,r6,lsl#24
	mov r6,r6,asr#24
	ldrb r7,[mikptr,#mikAud0Vol]

	b pcmMix
pcmMixReturn:
	mov r3,r3,lsr#24
	strb r3,[mikptr,#mikAud0Count]
	mov r4,r4,ror#16
	str r4,[mikptr,#audio0+WAVESHAPER]
	strb r6,[mikptr,#mikAud0OutVal]

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
;@ r11 = Ch3 Volume
;@----------------------------------------------------------------------------
pcmMix:				;@ r0=len, r1=dest, r10=mikptr
// IIIVCCCCCCCCCCC000001FFFFFFFFFFF
// I=sampleindex, V=overflow, C=counter, F=frequency
;@----------------------------------------------------------------------------
	mov r0,r0,lsl#5
mixLoop:
	mov r2,#0x00000000
innerMixLoop:
	tst r8,#0x08				;@ Enabled?
	beq noClock
	subs r3,r3,r5				;@ Ch1
	bcs noClock
	add r3,r3,r3,lsl#24
	and r9,r4,r4,lsl#16
	eor r9,r9,r9,lsl#8
	eor r9,r9,r9,lsl#4
	eor r9,r9,r9,lsl#2
	eors r9,r9,r9,lsl#1
	mov r9,r4,lsr#16
	add r4,r4,r9,lsl#16
	orrpl r4,r4,#0x00010000
	movpl r9,r7
	rsbmi r9,r7,#0
	tst r8,#0x20				;@ Integrate?
	moveq r6,r9
	beq noIntegrate
	add r6,r6,r9
	cmp r6,#127
	movpl r6,#127
	cmp r6,#-128
	movmi r6,#-128
noIntegrate:
noClock:
	sub r0,r0,#1
	tst r0,#0x1F
	bne innerMixLoop

	add r2,r2,r6,lsl#24
	eor r2,#0x00008000
	cmp r0,#0
	strpl r2,[r1],#4
	bhi mixLoop				;@ ?? cycles according to No$gba

	b pcmMixReturn
;@----------------------------------------------------------------------------


#endif // #ifdef __arm__
