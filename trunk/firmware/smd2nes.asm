;-------------------------------------------------------------------------------
; Sega Mega Drive (SMD) gamepad adapter for NES v1.2
; by m.d./XDS, 2010 (maddevmail@gmail.com)
;
; Fuse bit settings:
;
;	CKSEL0     = P (0) (internal RC oscillator @ 8 MHz)
;	CKSEL1     = P (0)
;	CKSEL2     = . (1)
;	CKSEL3     = P (0)
;	SUT0       = P (0) (14 CK + 65 ms startup time)
;	SUT1       = . (1)
;	CKOUT      = . (1)
;	CKDIV8     = . (1)

;	RSTDISBL   = . (1)
;	BODLEVEL0  = P (0) (brown-out detection threshold is 4.3 V)
;	BODLEVEL1  = P (0)
;	BODLEVEL2  = . (1)
;	WDTON      = P (0) (watchdog timer is always on)
;	SPIEN      = P (0)
;	EESAVE     = . (1)
;	DWEN       = . (1)
;
; This circuit uses an on-board RC oscillator to clock MCU. For more predictable
; operation it is recommended to tune it accurately to 8 MHz via setting OSCCAL
; register value in the initialization section


.include "tn2313def.inc"

; "Turbo" buttons period in 1/60 sec
.equ TURBO_T	= 12

; PORTD - NES interface
.equ LATCH		= PD2
.equ CLK		= PD3
.equ DO			= PD0

; PORTB - Mega Drive gamepad port
.equ SEL	= PB3
.equ D0		= PB7
.equ D1		= PB4
.equ D2		= PB2
.equ D3		= PB1
.equ D4		= PB5
.equ D5		= PB0

; Mega Drive gamepad states
.def state0		= r1	;SELECT = 0
.equ MD0_UP		= D0
.equ MD0_DOWN	= D1
.equ MD0_A		= D4
.equ MD0_START	= D5

.def state1		= r2	;SELECT = 1
.equ MD1_UP		= D0
.equ MD1_DOWN	= D1
.equ MD1_LEFT	= D2
.equ MD1_RIGHT	= D3
.equ MD1_B		= D4
.equ MD1_C		= D5

.def state3		= r3	;6-button mode (after 3rd SELECT pulse)
.equ MD3_Z		= D0
.equ MD3_Y		= D1
.equ MD3_X		= D2
.equ MD3_MODE	= D3

.def prev3		= r4

; virtual NES gamepad state
.def nes		= r5
.equ NES_A		= 0
.equ NES_B		= 1
.equ NES_SEL	= 2
.equ NES_START	= 3
.equ NES_UP		= 4
.equ NES_DOWN	= 5
.equ NES_LEFT	= 6
.equ NES_RIGHT	= 7

.def tmp		= r16
.def tmpi		= r21
.def nes_tmp	= r17
.def turbo		= r18
.def turbo_cnt_a	= r19
.def turbo_cnt_b	= r20

;-------------------------------------------------------------------------------

.cseg

.macro sel_low
	cbi PORTB,SEL
	nop
	nop
	nop
	nop
	nop
	nop
.endm

.macro sel_high
	sbi PORTB,SEL
	nop
	nop
	nop
	nop
	nop
	nop
.endm

.macro sel_pulse
	sel_low
	sel_high
.endm

.org 0
	rjmp reset

;-------------------------------------------------------------------------------
; NES signal handlers

; NES CLK pulse riding edge - shift virtual controller latch
.org INT0addr
	rjmp shift

; NES LATCH pulse - latch virtual controller state
.org INT1addr
	mov nes,nes_tmp

shift:
	mov tmpi,nes
	ori tmpi,~(1<<DO)
	out PORTD,tmpi
	lsr nes
	reti

;-------------------------------------------------------------------------------
; Mega Drive gamepad polling routine (120 timer per second)

.org OC0Aaddr
	wdr
	sei

; acquire state of standard 3-button controller
	in state1,PINB
; 1st SELECT pulse
	sel_low
	in state0,PINB
	sel_high
; 2nd pulse
	sel_pulse
; 3rd
	sel_pulse
; acquire additional data from 6-button controller
	in state3,PINB
; 4th
	sel_low
	sel_high

; calculate NES latch contents
	ser nes_tmp
; UP
	sbrs state1,MD1_UP
	cbr nes_tmp,1<<NES_UP
; LEFT
	sbrs state1,MD1_LEFT
	cbr nes_tmp,1<<NES_LEFT
; RIGHT
	sbrs state1,MD1_RIGHT
	cbr nes_tmp,1<<NES_RIGHT
; DOWN
	sbrs state1,MD1_DOWN
	cbr nes_tmp,1<<NES_DOWN
; SMD B
	sbrs state1,MD1_B
	cbr nes_tmp,1<<NES_A

; SMD A
	sbrs state0,MD0_A
	cbr nes_tmp,1<<NES_B
; SMD START
	sbrs state0,MD0_START
	cbr nes_tmp,1<<NES_START

; SMD MODE
	sbrs state3,MD3_MODE
	cbr nes_tmp,1<<NES_SEL

; emulate 'turbo' A
	dec turbo_cnt_a
	brne check_turbo_b
	ldi turbo_cnt_a,TURBO_T
	ldi tmp,1<<NES_A
	eor turbo,tmp

; emulate 'turbo' B
check_turbo_b:
	dec turbo_cnt_b
	brne check_xy
	ldi turbo_cnt_b,TURBO_T
	ldi tmp,1<<NES_B
	eor turbo,tmp	

check_xy:
; SMD X
	mov tmp,state3
	com tmp
	and tmp,prev3

	sbrs tmp,MD3_X
	rjmp check_y
; X has been pressed - start 'turbo' B sequence
	cbr turbo,1<<NES_B

check_y:
	sbrs tmp,MD3_Y
	rjmp save_prev3
; Y has been pressed - start 'turbo' A sequence
	cbr turbo,1<<NES_A

save_prev3:
	mov prev3,state3

	sbrs state3,MD3_X
	rjmp check_y_up
; X is released - disable 'turbo' B sequence
	ldi turbo_cnt_b,TURBO_T
	sbr turbo,1<<NES_B

check_y_up:
	sbrs state3,MD3_Y
	rjmp apply_turbo
; Y is released - disable 'turbo' A sequence
	ldi turbo_cnt_a,TURBO_T
	sbr turbo,1<<NES_A

apply_turbo:
; apply 'turbo' controls
	and nes_tmp,turbo
	reti

;-------------------------------------------------------------------------------
; Device initialization

reset:

; roll-back prevention
	cli
	in tmp,MCUSR
	clr r0
	out MCUSR,tmp
	andi tmp,0xF
halt:
	breq halt

; tune RC oscillator
	ldi tmp,0x5A
	out OSCCAL,tmp

; initialize stack pointer
	ldi tmp,RAMEND
	out SPL,tmp

; enable SLEEP instruction
; INT0 (CLK) on INT1 (LATCH) on rising edge
	ldi tmp,1<<SE|1<<ISC01|1<<ISC00|1<<ISC11|1<<ISC10
	out MCUCR,tmp

; reduce power consumption
	sbi ACSR,ACD
	ser tmp
	out PORTA,tmp
	out PORTB,tmp
	out PORTD,tmp

; configure I/O
	ldi tmp,1<<SEL
	out DDRB,tmp
	sbi DDRD,DO

; configure NES signal handlers
	ldi tmp,1<<INT0|1<<INT1
	out GIMSK,tmp

; configure MD gamepad polling timer
	ldi tmp,1<<WGM01
	out TCCR0A,tmp
	ldi tmp,64
	out OCR0A,tmp
	ldi tmp,1<<CS02|1<<CS00
	out TCCR0B,tmp
	ldi tmp,1<<OCIE0A
	out TIMSK,tmp

	ser nes_tmp
	mov nes,nes_tmp
	ser turbo

; idle loop
	sei
idle:
	sleep
	rjmp idle

author:
	.db "m.d./XDS, 2010"
