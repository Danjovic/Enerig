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
;    Version 0.8   May 16th 2016                                      *
;    - Added Coil charging time control. Microcontroller changed      *
;    to PIC12F683 due to the use of CCP module                        *
;    Version 0.82  Jun 14th 2016                                      *
;    - Added engine speed low and high limits. Added legacy/variable  *
;      dwell time control                                             *
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

;    ___ ___ ___ _ ___  __  __  ___ ____
;   | _ \_ _/ __/ |_  )/ _|/ / ( _ )__ /
;   |  _/| | (__| |/ /|  _/ _ \/ _ \|_ \
;   |_| |___\___|_/___|_| \___/\___/___/
;
	list	 p=12f683	; list directive to define processor
	#include <P12F683.inc>	; processor specific variable definitions

	__CONFIG    _CP_OFF & _CPD_OFF & _BOD_ON & _MCLRE_OFF & _PWRTE_ON & _WDT_OFF & _INTRC_OSC_NOCLKOUT

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

#define f_valid_rpm 0  ; flag for activating coil 1=activate coil
#define f_dwell_mode 1 ; flag for dwell control mode 1=dwell control
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

Flags    ;

TcyH     ; Tcycle (time for a whole turn)
TcyL

TcoilL   ; time needed for the coil to charge -> 3 to 7ms
TcoilH

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

isr_code
        ; test for compare interrupt
        btfsc   PIR1,CCP1IF                     ; Timer 1 Compare?
        goto isr_ccp                            ; yes, goto ccp1 interrupt
        btfsc   PIR1,TMR1IF                     ; Timer 1 overflow
        goto isr_timer1                         ; yes, do timer1 interrupt



isr_end
	movf		status_temp,w		; retrieve copy of STATUS register
	movwf		STATUS			; restore pre-isr STATUS register contents
	swapf		w_temp,f
	swapf		w_temp,w		; restore pre-isr W register contents
	retfie					; return from interrupt


isr_ccp                                         ; Timer 1 Compare turns the coil on
	   btfsc Flags,f_valid_rpm					; is RPM in valid range?
       bsf GPIO,_COIL                           ; yes, Turn coil on
       bcf PIR1,CCP1IF                          ; clear interrupt flag
       goto isr_end                             ; then exit

isr_timer1                                      ; Timer 1 overflows on engine stop
       bcf GPIO,_COIL                           ; Turn coil off
       bcf  T1CON,TMR1ON                        ; stop timer 1
       clrf TMR1H                               ; reset Timer1
       clrf TMR1L
       bcf PIR1,TMR1IF                          ; clear interrupt flag
       goto isr_end                             ; then exit



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
	movwf CMCON0         ; analog comparator with cout

	movlw b'00001101'   ; ADC right justified, Vref=Vcc, CH3 selected
	movwf ADCON0
	
	movlw b'00110000'  ; Timer 1 off, internal clock, prescaler 1:8 (8us/count)
	movwf T1CON
	
	movlw b'00001010' ; Compare mode, generate software interrupt on match
	movwf CCP1CON     ; (CCP1IF bit is set, CCP1 pin is unaffected)

	_bank1;
	movlw b'11011011' ; GPIO 2 and 5 as outputs
	movwf TRISIO

	movlw b'00011000' ; fosc/8, CH3 as ADC input
	movwf ANSEL

	movlw b'00000011' ; Select TMR0 prescale 1:16, internal clock source
	movwf OPTION_REG

    movlw b'00100001' ; CCP1 and TIMER1 Overflow interrupts selected
    movwf PIE1

	;call 3FFh ;Get the cal value
	;movwf OSCCAL ;Calibrate

	; initialize variables
	_bank0
	clrf TMR1H
	clrf TMR1L
	movlw .255
	movwf CCPR1L
	movwf CCPR1H

	movlw 0x77
	movwf TcoilL
	movlw .1
	movwf TcoilH

    ; enable global interrupts
    bcf PIR1,CCP1IF
    bcf PIR1,TMR1IF
    bsf INTCON,GIE
    bsf INTCON,PEIE




;**********************************************************************
;             _        _               
;  _ __  __ _(_)_ _   | |___  ___ _ __ 
; | '  \/ _` | | ' \  | / _ \/ _ \ '_ \
; |_|_|_\__,_|_|_||_| |_\___/\___/ .__/
;                                |_|   
;**********************************************************************
; start the show
restart
	bcf GPIO,_COIL  ; turn off coil
next_edge
	clrf TMR0
	bcf INTCON,T0IF  ; clear pending overflow
	bsf PIR1,CMIF    ; clear any pending interrupt
	movlw .256-Cnt_Timeout ;  _Timeout_      set timeout for 800ms like the RC timer
	movwf Timeout_Counter   ;


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
	movwf TMR0		; now will count up to total advance time

waiting_advance_end
	btfsc INTCON,T0IF	 ; wait for overflow after total advance time
	goto increment           ; if time is over then increment counter while signal is high
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


decrement              ;     10 ticks for each interaction  (10us)
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

    bcf  T1CON,TMR1ON ; stop timer 1

    movf TMR1L,w; Get Tcycle
    movwf TcyL
    movf TMR1H,w 
    movwf TcyH



    ; is it valid     TMR1 = $0500 -> 5859RPM
    ; engine speed ?  TMR1 = $F000 ->  122RPM

	movf TcyH,w
	sublw .4 ; invalid from 0000-04ff 
	btfsc STATUS,C
	goto invalid_RPM

	movf TcyH,w
	sublw 0xef ; invalid from f000-ffff 
	btfss STATUS,C
	goto invalid_RPM
	goto valid_RPM    ; valid from 0500-efff

 
invalid_RPM
		bcf Flags,f_valid_rpm
        clrf TMR1H
        clrf TMR1L
        bsf  T1CON,TMR1ON ; restart Timer 1 and try again
        goto restart

valid_RPM
		bsf Flags,f_valid_rpm ; set flag to enable ccp irq to turn on the coil

		; now check coil charging mode
		btfsc Flags,f_dwell_mode   ; 
		goto variable_dwell_mode

legacy_dwell_mode  ; in legacy mode, use charge start time with 
		movlw .250 ; a fixed value of 2ms (8us*250, TRB)
		movwf CCPR1L
		clrf CCPR1H       
		goto restart_timer_1


variable_dwell_mode
        ; calculate  charge start time (Tcharge = Tcycle - Tcoil)
        movf    TcoilL,W
        subwf   TcyL,f
        movf    TcoilH,W
        btfss   STATUS,C
        incfsz  TcoilH,W
        subwf   TcyH,f      ; at this point Tcycle holds value of Tcharge


        movf TcyH,w         ;then program timer compare to Tcharge
        movwf CCPR1H
        movf TcyL,w
        movwf CCPR1L 

restart_timer_1
        clrf TMR1H          ; Reset and restart Timer 1
        clrf TMR1L
        bsf  T1CON,TMR1ON


        
read_adc                         ; now sample potentiometer;
	bsf ADCON0, GO_NOT_DONE      ; start ADC conversion
	btfsc ADCON0, GO_NOT_DONE    ; wait for end of ADC conversion
	goto $-1
	movf ADRESH,w                ; W holds pot value

	movwf TcoilL                   ; save value

	bsf Flags,f_dwell_mode ; preset flag
	sublw .31					 ; test if voltage < 1/8 of full scale
	btfsc STATUS,C
	bcf Flags,f_dwell_mode ; reset flag for values from below 32



;    movwf TcoilL
	; use pot value to calculate Tcoil. value is multiplied by 2 and then added to 2ms
	; so in the end the Coil charging time is in between 2 to 6 ms

        clrf TcoilH      ;  clear TcoilH
        rlf TcoilL,f     ;  TcoilL<<=1
        btfsc STATUS,C   ;  Carry (Bit 7 was set)?
        incf TcoilH,f    ;  yes, increment TcoilH
                         ; Here Tcoil(H:L) = (0000.0007.6543.210-) which correspond to 0..$1FE
                         ; or 0..510 counts of 8us or 0..4080us (0..4.08ms)

        ; Now add 2ms to the result (8us * 250 = 2000). 
        movlw .250        ; 
        addwf TcoilL,f   ; add to TcoilL
        btfsc STATUS,C   ; overflow?
        incf TcoilH,f    ; yes, add one to TcoilH
        ;incf TcoilH,f    ; add 1 to Tcoilh - high($177)
        
        ; Here Tcoil holds a value between 250 and 760 or  3000 to 7080 which corresponds to 2..6ms


 	goto next_edge  ;  Finally go wait for next rising edge from sensor

;

	END