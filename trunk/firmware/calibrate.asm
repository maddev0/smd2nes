.include "tn2313def.inc"

.equ TEST_PIN = PB0

.def tmp = r16

.cseg
.org 0
	ldi tmp,ee_osccal
	out EEAR,tmp
	sbi EECR,EERE
	in tmp,EEDR
	cpi tmp,0xFF
	brne set_osccal
	ldi tmp,0x5B
set_osccal:
	out OSCCAL,tmp

	sbi DDRB,TEST_PIN
loop:
	sbi PORTB,TEST_PIN
	wdr
	nop
	cbi PORTB,TEST_PIN
	rjmp loop

.eseg
ee_osccal:
	.db 0x5A
