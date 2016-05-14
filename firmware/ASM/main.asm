;**********************************************************************
;                      ___             ___                            *
;                     | __|_ _  ___ _ |_ _|__ _                       *
;                     | _|| ' \/ -_) '_| |/ _` |                      *
;                     |___|_||_\___|_||___\__, |                      *
;                                         |___/                       *
;                                                                     *
;   This program implements the ignition controller for the GURGEL    *
;   Enertron Two-Cylinder boxer engine. It is based on the patent     *
;   issued by the Amaral Gurgel, now expired.                         *
;                                                                     *
;**********************************************************************
;                                                                     *
;    Filename:	   main.asm                                           *
;    Version 0.1   May 2nd 2016                                       *
;    - Basic release                                                  *
;    Version 0.7   May 13th 2016                                      *
;    - First Candidate Version                                        *
;                                                                     *
;    Author:  Daniel Jose Viana                                       *
;    Company:  http://danjovic.blogspot.com                           *
;              http://hackaday.io/danjovic                            *
;                                                                     *
;**********************************************************************
;              This code is licensed under GPL V2.0                   *
;**********************************************************************
;         ASCII titles by http://patorjk.com/software/taag/           *
;**********************************************************************

;  ___ ___ ___ _ ___ ___ ______ ___
; | _ \_ _/ __/ |_  ) __/ /__  | __|
; |  _/| | (__| |/ /| _/ _ \/ /|__ \
; |_| |___\___|_/___|_|\___/_/ |___/
;
	list	 p=12f675	; list directive to define processor
	#include <P12F675.inc>	; processor specific variable definitions
	__CONFIG    _CP_OFF & _CPD_OFF & _BODEN_ON & _MCLRE_OFF & _PWRTE_ON & _WDT_OFF & _INTRC_OSC_NOCLKOUT

; Pins from GPIO
_P0    EQU 0 ;
_P1    EQU 1 ;
_P2    EQU 2 ;
_P3    EQU 3 ;
_P4    EQU 4 ;
_P5    EQU 5 ;

_COUT  EQU 2 ;
_VACUUM EQU 3;
_COIL  EQU 5;


;**********************************************************************
;  ___       __ _      _ _   _
; |   \ ___ / _(_)_ _ (_) |_(_)___ _ _  ___
; | |) / -_)  _| | ' \| |  _| / _ \ ' \(_-<
; |___/\___|_| |_|_||_|_|\__|_\___/_||_/__/
;
;**********************************************************************

; Macros
#define  _bank0  bcf STATUS,RP0
#define  _bank1  bsf STATUS,RP0

#ifndef GO_NOT_DONE  ; compatibility with gpasm
#define GO_NOT_DONE 1
#endif 

#define F_CLOCK_MHz      .4              ; internal clock, 4MHz
#define F_CPU_MHz   F_CLOCK_MHz/.4       ; 1 cpu cycle at each 1us (1MHz)
#define T0_presc .16                     ; Prescaler for Timer 0
#define T0_tick_us  .16 ;  T0_presc/F_CPU_MHz ;

#define T_RB_us   .2000
#define T_Ac_us   .1000
#define T_Av_us   .1500

#define Cnt_RB T_RB_us/T0_tick_us  ;  2ms Time for coil to discharge
#define Cnt_Ac T_Ac_us/T0_tick_us  ;  1ms basic advance time
#define Cnt_Av T_Av_us/T0_tick_us  ;1,5ms extra advance time due to engine load

#define T_256C_us .4096; 256*T0_tick_us  ; Time for 256 counts
#define Cnt_Timeout  .800000/T_256C_us


;**********************************************************************
; __   __        _      _    _
; \ \ / /_ _ _ _(_)__ _| |__| |___ ___
;  \ V / _` | '_| / _` | '_ \ / -_|_-<
;   \_/\__,_|_| |_\__,_|_.__/_\___/__/
;
;**********************************************************************

 cblock 0x20

Conta
Timeout_Counter ; 8 bit timeout counter

tmrH      ; 16 bit counting timer
tmrL

Temp        ; temporary storage

w_temp	     ; variable used for context saving
status_temp  ; variable used for context saving
 endc


;**********************************************************************
;  ___ _            _
; / __| |_ __ _ _ _| |_ _  _ _ __
; \__ \  _/ _` | '_|  _| || | '_ \
; |___/\__\__,_|_|  \__|\_,_| .__/
;                           |_|
;**********************************************************************

 
	ORG		0x000			; processor reset vector
   	goto		main			; go to beginning of program


	ORG		0x004			; interrupt vector location
	movwf		w_temp			; save off current W register contents
	movf		STATUS,w		; move status register into W register
	movwf		status_temp		; save off contents of STATUS register

; isr code can go here or be located as a call subroutine elsewhere

	movf		status_temp,w		; retrieve copy of STATUS register
	movwf		STATUS			; restore pre-isr STATUS register contents
	swapf		w_temp,f
	swapf		w_temp,w		; restore pre-isr W register contents
	retfie					; return from interrupt



;**********************************************************************
;  __  __      _        ___             _   _
; |  \/  |__ _(_)_ _   | __|  _ _ _  __| |_(_)___ _ _
; | |\/| / _` | | ' \  | _| || | ' \/ _|  _| / _ \ ' \
; |_|  |_\__,_|_|_||_| |_| \_,_|_||_\__|\__|_\___/_||_|
;
;**********************************************************************
main
	; Configure Hardware
	movlw b'00000001'

	movwf CMCON         ; analog comparator with cout
	movlw b'00001101'   ; ADC right justified, Vref=Vcc, CH3 selected
	movwf ADCON0



	clrwdt



	_bank1; --543210 
	movlw b'11011011' ; GPIO 2 and 5 as outputs
	movwf TRISIO

	movlw b'00011000' ; fosc/8, CH3 as ADC input 
	movwf ANSEL
 
	movlw b'00000011' ; Select TMR0 prescale 1:16, internal clock source
	movwf OPTION_REG

	call 3FFh ;Get the cal value
	movwf OSCCAL ;Calibrate

	; initialize variables
	_bank0


;test
;	bcf GPIO,_COIL  ; turn off coil
;	clrf TMR0
;	bcf INTCON,T0IF  ; clear pending overflow
;	bsf PIR1,CMIF    ; clear any pending interrupt
;	movlw .256-Cnt_Timeout ;  _Timeout_      set timeout for 800ms like the RC timer
;	movwf Timeout_Counter   ;
;wait1
;	btfss INTCON,T0IF     ; timer 1 overflowed (65ms)
;	goto wait1 ; no, keep sampling input
;	bcf INTCON,T0IF     ; increment timeout timer
;	incfsz Timeout_Counter,f ; overflowed ?
;	goto wait1  ; no, keep sampling input
;
;	bsf GPIO,_COIL  ; turn off coil
;	clrf TMR0
;	bcf INTCON,T0IF  ; clear pending overflow
;	bsf PIR1,CMIF    ; clear any pending interrupt
;	movlw .256-Cnt_Timeout ;  _Timeout_      set timeout for 800ms like the RC timer
;	movwf Timeout_Counter   ;
;wait2
;	btfss INTCON,T0IF     ; timer 1 overflowed (65ms)
;	goto wait2  ; no, keep sampling input
;	bcf INTCON,T0IF     ; increment timeout timer
;	incfsz Timeout_Counter,f ; overflowed ?
;	goto wait2  ; no, keep sampling input
;	goto test           ; yes, we timed out, restart everything 






	; start the show
restart
	bcf GPIO,_COIL  ; turn off coil
next_edge
	clrf TMR0
	bcf INTCON,T0IF  ; clear pending overflow
	bsf PIR1,CMIF    ; clear any pending interrupt
	movlw .256-Cnt_Timeout ;  _Timeout_      set timeout for 800ms like the RC timer
	movwf Timeout_Counter   ;
;


wait_rising_edge
	movf PIR1,w ; bit 3 = CMIF
	andlw (1<<CMIF) ; isolate bit CMIF (3)
	movwf Temp
	movf GPIO,w
	andlw (1<<_COUT) ; isolate bit COUT (2)
	iorwf Temp,w
	xorlw b'00001100' ;
	btfsc STATUS,Z    ; rising edge ?
	goto wait_advance ; yes, go on
                      ; no, test for timeout
	btfss INTCON,T0IF     ; timer 1 overflowed (65ms)
	goto wait_rising_edge ; no, keep sampling input
	bcf INTCON,T0IF     ; increment timeout timer
	incfsz Timeout_Counter,f ; overflowed ?
	goto wait_rising_edge  ; no, keep sampling input
	goto restart           ; yes, we timed out, restart everything

	

wait_advance
	; Now wait for Advance Time
	movlw .256-Cnt_Ac       ; base time
	btfss GPIO,_VACUUM
	movlw .256-(Cnt_Ac+Cnt_Av) ; Total advance (tbase + tvacuum)
	clrf TMR0				; reset timer
	bcf INTCON,T0IF         ; clear any pending flag
	movwf TMR0				; now will count up to total advance time

waiting_advance_end
	btfsc INTCON,T0IF		 ; wait for overflow after total advance time
	goto increment            ; if time is over then increment counter while signal is high
	btfss GPIO,_COUT         ; otherwise check if signal drop earlier than advance time
	goto dropcoil            ; if so, drop coil and generate spart immediately
	goto waiting_advance_end ; if not keep waiting
	
	; now check if signal have dropped. according to the circuit
	; the low level is what matters and not the falling edge
	; 
;	clrf tmrH
;	clrf tmrL
;waitdrop
increment                  ; 10 ticks for each interaction (100KHz) , overflow at 0,65s (655360us)
	btfss GPIO,_COUT   ; 1 test if pin low
	goto decrement     ; 1 yes, start decrement phase
	; increment phase
	incf tmrH,f        ; 1  increment timer
	incfsz tmrL,f      ; 1  10 instructions = 10us
	decf tmrH,f        ; 1  timeout on 65535ms.

	movf tmrH,w        ;1  ;testforzero
	iorwf tmrL,w       ;1
	btfss STATUS,Z     ;1 skip if zero
	goto increment     ;2 test again for fallen _COUT

	goto restart       ;On overflow drop off coil and wait for the next pulse.
	                   ;this is an improvement over the original circuit where a signal
	                   ;stuck at a high level would cause the coil to be constantly turned on


decrement                  ; 10 ticks for each interaction  (10us)
	movf tmrH,w        ;1    test for zero 
	iorwf tmrL,w       ;1
	btfsc STATUS,Z     ;1 skip if not zero
	goto dropcoil	   ;1
					   
	; now decrement counter	
	movf    tmrL,f  ; 1
	btfsc STATUS,Z  ; 1
	decf   tmrH,f   ; 1
	decf   tmrL,f   ; 1
	goto decrement  ; 2

dropcoil
	bcf GPIO,_COIL ; turn off coil
	clrf tmrH      ; clear up/down counter
	clrf tmrL

	movlw .256-Cnt_RB        ; Time for coil to rest
	clrf TMR0		; reset timer
	bcf INTCON,T0IF         ; clear any pending flag
	movwf TMR0	        ; now will count up to total advance time
	

read_adc                             ; Use this delay time to read the ADC
	bsf ADCON0, GO_NOT_DONE      ; start ADC conversion
	btfsc ADCON0, GO_NOT_DONE    ; wait for end of ADC conversion
	goto $-1
	movf ADRESH,w
	movwf Conta                  ; save value for computing delay

wait_base_Trb
	btfss INTCON,T0IF		; wait for Coil base Rest time (Trb)
	goto $-1


        ; Now delay variable time between 0 to 6 ms, according to "Conta"

        incf Conta,f    ; adjust interval to start from 1
wait_extra_Trb
        decf Conta,f    ; 1 1 decrement counter
        btfsc STATUS,Z  ; 1 2 reach zero?
        goto raise_coil ; 1 3 yes, turn on coil and wait for next cycle
        nop             ; 1 4 no, decrement counter
        goto $+1        ; 2 6
        goto $+1        ; 2 8
        goto $+1        ; 2 10
        goto $+1        ; 2 12
        goto $+1        ; 2 14
        goto $+1        ; 2 16
        goto $+1        ; 2 18
        goto $+1        ; 2 20
        goto $+1        ; 2 22
        goto wait_extra_Trb ;  2 24  32us per iteraction, maximum time
                            ;        is 256*32us = 8192us = 8.192ms

raise_coil
	bsf GPIO,_COIL  ; turn coil on again
	goto next_edge  ;  and go wait for next rising edge from sensor


; test routine
;rep
;   bsf ADCON0, GO_NOT_DONE
;   btfsc ADCON0, GO_NOT_DONE
;   goto $-1
;	movf ADRESH,w
;	movwf Conta
;	bsf GPIO,5
;rep2
;	nop 
;	nop

;	decfsz Conta,f
;	goto rep2
;	bcf GPIO,5

;	nop;
;	nop
;	nop
;	nop

;	goto rep
;;




;
	END
