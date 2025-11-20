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
;@ Channel 0
;@--------------------------
	ldrb r3,[mikptr,#mikAud0TBack]
	ldrb r4,[mikptr,#mikAud0Count]
	orr r3,r3,r4,lsl#24
	ldr r4,[mikptr,#audio0+WAVESHAPER]
	mov r5,#0x01
	ldrb r8,[mikptr,#mikAud0Ctl]
	and r6,r8,#7				;@ CLOCK_SEL
	mov r5,r5,lsl r6
	ldrb r6,[mikptr,#mikAud0OutVal]
	mov r6,r6,lsl#24
	mov r6,r6,asr#24
	ldrb r7,[mikptr,#mikAud0Vol]

	add r12,r1,#0
	bl pcmMix
	mov r3,r3,lsr#24
	strb r3,[mikptr,#mikAud0Count]
	str r4,[mikptr,#audio0+WAVESHAPER]
	strb r6,[mikptr,#mikAud0OutVal]
;@--------------------------
;@ Channel 1
;@--------------------------
	ldrb r3,[mikptr,#mikAud1TBack]
	ldrb r4,[mikptr,#mikAud1Count]
	orr r3,r3,r4,lsl#24
	ldr r4,[mikptr,#audio1+WAVESHAPER]
	mov r5,#0x01
	ldrb r8,[mikptr,#mikAud1Ctl]
	and r6,r8,#7				;@ CLOCK_SEL
	mov r5,r5,lsl r6
	ldrb r6,[mikptr,#mikAud1OutVal]
	mov r6,r6,lsl#24
	mov r6,r6,asr#24
	ldrb r7,[mikptr,#mikAud1Vol]

	add r12,r1,#1
	bl pcmMix
	mov r3,r3,lsr#24
	strb r3,[mikptr,#mikAud1Count]
	str r4,[mikptr,#audio1+WAVESHAPER]
	strb r6,[mikptr,#mikAud1OutVal]
;@--------------------------
;@ Channel 2
;@--------------------------
	ldrb r3,[mikptr,#mikAud2TBack]
	ldrb r4,[mikptr,#mikAud2Count]
	orr r3,r3,r4,lsl#24
	ldr r4,[mikptr,#audio2+WAVESHAPER]
	mov r5,#0x01
	ldrb r8,[mikptr,#mikAud2Ctl]
	and r6,r8,#7				;@ CLOCK_SEL
	mov r5,r5,lsl r6
	ldrb r6,[mikptr,#mikAud2OutVal]
	mov r6,r6,lsl#24
	mov r6,r6,asr#24
	ldrb r7,[mikptr,#mikAud2Vol]

	add r12,r1,#2
	bl pcmMix
	mov r3,r3,lsr#24
	strb r3,[mikptr,#mikAud2Count]
	str r4,[mikptr,#audio2+WAVESHAPER]
	strb r6,[mikptr,#mikAud2OutVal]
;@--------------------------
;@ Channel 3
;@--------------------------
	ldrb r3,[mikptr,#mikAud3TBack]
	ldrb r4,[mikptr,#mikAud3Count]
	orr r3,r3,r4,lsl#24
	ldr r4,[mikptr,#audio3+WAVESHAPER]
	mov r5,#0x01
	ldrb r8,[mikptr,#mikAud3Ctl]
	and r6,r8,#7				;@ CLOCK_SEL
	mov r5,r5,lsl r6
	ldrb r6,[mikptr,#mikAud3OutVal]
	mov r6,r6,lsl#24
	mov r6,r6,asr#24
	ldrb r7,[mikptr,#mikAud3Vol]

	add r12,r1,#3
	bl pcmMix
	mov r3,r3,lsr#24
	strb r3,[mikptr,#mikAud3Count]
	str r4,[mikptr,#audio3+WAVESHAPER]
	strb r6,[mikptr,#mikAud3OutVal]
;@--------------------------
mixLoop2:
	ldr r2,[r1]
	mov r3,r2,asr#24			;@ Ch0
	mov r2,r2,lsl#8
	add r3,r3,r2,asr#24			;@ Ch1
	mov r2,r2,lsl#8
	add r3,r3,r2,asr#24			;@ Ch2
	mov r2,r2,lsl#8
	add r3,r3,r2,asr#24			;@ Ch3
	mov r3,r3,lsl#22
	orr r3,r3,r3,lsr#16
	str r3,[r1],#4
	subs r0,r0,#1
	bne mixLoop2

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
;@ r2  = Scrap
;@ r3  = Timer Counter plus backup
;@ r4  = LFSR
;@ r6  = Output
;@ r7  = Volume
;@ r8  = Settings
;@----------------------------------------------------------------------------
pcmMix:				;@ r0=len, r1=dest, r10=mikptr
// IIIVCCCCCCCCCCC000001FFFFFFFFFFF
// I=sampleindex, V=overflow, C=counter, F=frequency
;@----------------------------------------------------------------------------
	mov r11,r0
	tst r8,#0x08				;@ Enabled?
	cmpne r7,#0					;@ Volume 0?
	beq silenceMix
	add r3,r3,#0x01
mixLoop:
innerMixLoop:
	subs r3,r3,#0x00800000
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
	sub r11,r11,r5,lsl#26
	tst r11,#0xFC000000
	bne innerMixLoop

	subs r11,r11,#1
	strbpl r6,[r12],#4
	bhi mixLoop				;@ ?? cycles according to No$gba

	bx lr
;@----------------------------------------------------------------------------
silenceMix:
silenceLoop:
	subs r11,r11,#1
	strbpl r6,[r12],#4
	bhi silenceLoop
	bx lr


#endif // #ifdef __arm__
