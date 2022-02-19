;******************************************************************************
; 1S_StorageCharger V1.0 - one cell LiPo storage charger/charger
; for PIC10F322 microcontrollers and LTC4054 or compatible charging chips
;
; Copyright (C)2022, Cybermike
; USE AT YOUR OWN RISK!!! LIPOS ARE DANGEROUS!  YOU MAY BURN YOUR HOUSE DOWN!
; THIS MAY RUIN YOUR BATTERIES!  I DISCLAIM ALL LIABILITY FOR ANYTHING
; BAD THAT HAPPENS!
;
; For non-commercial use only!
;
; This code is kind of crappy.  Sorry 'bout that.
;
; Add these options to your MPLABX linker options:
;       -Wl,-PresetVect=0x0000
;
;  Input Pins
;   RA1 - Short to ground to set storage charge mode.
;         Open (pulled to high) to set charge only mode
;  Output Pins
;   RA2 - Enable discharge load through MOSFET gate
;   RA0 - Enable charger through MOSFET gate
;
;* clock is 8MHz internal.                                           *
;******************************************************************************
    
#ifdef	__10F322

; CONFIG
  CONFIG  FOSC = INTOSC         ; Oscillator Selection bits (INTOSC oscillator: CLKIN function disabled)
  CONFIG  BOREN = OFF           ; Brown-out Reset Enable (Brown-out Reset disabled)
  CONFIG  WDTE = SWDTEN         ; Watchdog Timer Enable (WDT controlled by the SWDTEN bit in the WDTCON register)
  CONFIG  PWRTE = OFF           ; Power-up Timer Enable bit (PWRT disabled)
;  CONFIG  MCLRE = OFF           ; MCLR Pin Function Select bit (MCLR pin function is digital input, MCLR internally tied to VDD)
  CONFIG  MCLRE = ON            ; MCLR Pin Function Select bit (MCLR pin function is enabled)
  CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
  CONFIG  LVP = ON              ; Low-Voltage Programming Enable (LVP Enabled)

  CONFIG  LPBOR = OFF           ; Brown-out Reset Selection bits (BOR disabled)
  CONFIG  BORV = LO             ; Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
  CONFIG  WRT = OFF             ; Flash Memory Self-Write Protection (Write protection off)   
#endif

#include <xc.inc>

PSECT	udata

hard_lvc_counter:	ds  1	; Hard LVC detection counter to filter out noise and add hysterisis
soft_lvc_counter:	ds  1	; Soft LVC detection counter to filter out line noise and add hysterisis
shutoff_counter:	ds  1	; Number of soft LVC detections
current_voltage:	ds  1	; Current battery voltage from A/D
previous_voltage:	ds  1	; Previous voltage, used in charger mode
charge_threshold:	ds  1	; Threshold voltage to enable the charger
initial_voltage:	ds  1	; For reading initial voltage
initial_voltage_hi:	ds  1
db1:	    ds	1
db2:	    ds	1
flags:	    ds	1
w_temp:	    ds	1
status_temp:	ds	1

; Semaphore indicating charge mode jumper was inserted or removed
#define	SOFT_RESET_FLAG	flags,1

; RA2 (output) is set to 1 to enable the discharge load
#define	LOAD_ENABLE	LATA2

; RA1 (input) is shorted to ground to enable the discharger
#define	MODE_PIN	1
#define	CHARGING_MODE	RA1

; RA0 (output) is set to 1 to by the software enable charging
#define	CHARGE_ENABLE	LATA0
 
;
; Constants to set the LVC cutoff voltage.
; NOTE:  HIGHER NUMBERS are LOWER VOLTAGE!!!!
;
; 0x4A 3.625
; 0x47 3.75
; 0x44 3.875
; 0x42 4.0
; 0x40 4.125
    
; 0x45 (69) - 3.8V
; 0x4a (74) - 3.675V
; 0x4d (77) - 3.55V
; 0x53 (83) - 3.3V
; 0x5a (90) - 3.175V
#define	HARD_DISCHARGEDOWN_CUTOFF		0x46	; Below this voltage, enable the charger when in discharge-down mode
//#define	MAX_DISCHARGEDOWN_CYCLES		0xFF	; Maximum # of loop cycles below HI_LVC_CUTOFF before we quit
#define	MAX_DISCHARGEDOWN_CYCLES    0x7F
//#define	HARD_CHARGEUP_CUTOFF			0x43	; Below this voltage, enable the charger when in charge-up mode
#define	HARD_CHARGEUP_CUTOFF		0x44
// good for larger batteries #define	MAX_CHARGEUP_CYCLES		0x50	; Maximum # of loop cycles below HI_LVC_CUTOFF before we quit
#define	MAX_CHARGEUP_CYCLES	0x7F
//#define	LOW_LVC_CUTOFF		0x46	; Below this voltage, turn off the discharge load
#define	LOW_LVC_CUTOFF		0x46
#define	UNPLUGGED		0x4d	; When charging, if below this voltage, battery is unplugged
									; or turn on the charger
#define	HI_LVC_CUTOFF		0x45	; Above this voltage, turn on the discharge load
//#define	HI_LVC_CUTOFF	    0x44
#define	SOFT_LVC_COUNT		10		; Number of successive detections to disable discharge load
#define	HARD_LVC_COUNT		2		; Number of successive detections to enable charger

; Set DEBUG to test in MPLABX simulator with non-functional FVR and A/D converter
;#define	DEBUG	1

INIT_FVR    MACRO
	    LOCAL   wait_for_fvr
;		
; Setup the Fixed voltage reference for low-voltage cutoff detection.
;
	    movlw	10000001B	; Enable FVR and set reference to 1.024V
	    movwf	FVRCON
wait_for_fvr:
	    clrwdt
#ifdef	DEBUG
; MPLAB simulator doesn't simulate the FVR correctly, so this needs to be commented out
; when running in the simulator
#else
	    ;btfss	FVRCON,FVRRDY	; Wait for FVR to stablize
	    btfss	FVRRDY		; Wait for FVR to stablize
	    goto	wait_for_fvr
#endif

;
; Setup A/D converters
;

	    movlw	10111101B		; Clk = Fosc/16, Src = FVR
	    movwf	ADCON
	    ENDM
	    
READ_VOLTAGE	MACRO
		LOCAL	lvc_detect_loop
;
; Trigger the A/D converter to read the current battery voltage by measuring
; the on-board fixed voltage reference (FVR) which has been configured for 1.024 volts.
; The A/D value is ratiometric to Vdd (the battery voltage).  As the battery voltage drops,
; the FVR A/D value increases.
;
		bsf	GO_nDONE
lvc_detect_loop:
		clrwdt
		btfsc	GO_nDONE
		goto	lvc_detect_loop
#ifdef	DEBUG
		movlw	0x46	; test code because the MPLAB simulator doesn't mimic the FVR correctly
		movwf	current_voltage		; Set breakpoint and load db1 with whatever ADRES value you want to test with
#else
		movf	ADRES,w
		movwf	current_voltage
#endif
		ENDM
		
;******************************************************************************
;
; Processor reset vector.  Link option:
; -Wl,-PresetVect=0x0000									
PSECT	resetVect,class=code,delta=2		; processor reset vector
resetVec:
		goto    initialise      ; jump to initialise

;
; Interrupt vector for IOC interrupt on charge mode jumper pin
;
; Link option: -Wl, -PintVect=0x004
PSECT	intVect,class=code,delta=2
		movwf	w_temp
		swapf	STATUS,w
		movwf	status_temp
		bcf	IOCIF
		clrf	IOCAF
		bsf	SOFT_RESET_FLAG
		swapf	status_temp,w
		movwf	STATUS
		swapf	w_temp,f
		swapf	w_temp,w
		retfie
		
PSECT	code

;
; Start of main program
;
initialise:
		clrf	ANSELA
		clrf	LATA
		clrf	PORTA
; Setup WDT to periodically wake during charging
		movlw	00011010B	; set WDT to 8 seconds and turn it off
		;movlw	00011100B	; set WDT to 16 seconds and turn it off
		;movlw	00011110B	; set WDT to 32 seconds and turn it off
		movwf	WDTCON

		clrf	current_voltage
		clrf	previous_voltage
		clrf	flags
		clrf	initial_voltage
		clrf	initial_voltage_hi
		movlw	HARD_DISCHARGEDOWN_CUTOFF
		movwf	charge_threshold
 		clrwdt
		clrf	INTCON		; Disable all interrupts	
		
; Setup the OPTION register
		clrwdt
		movlw	10000011B	; Enable WPU, assign prescaler to TMR0 and set 1:16
		movwf	OPTION_REG

		INIT_FVR
		movlw	00000010B
		movwf	TRISA			; Set all bits as output except RA1 mode selector
		movwf	WPUA
		bcf	nWPUEN	; Turn on weak pull ups

; Initialize variables
		movlw	SOFT_LVC_COUNT
		movwf	soft_lvc_counter
		movlw	HARD_LVC_COUNT
		movwf	hard_lvc_counter
		movlw	MAX_DISCHARGEDOWN_CYCLES
		movwf	shutoff_counter
		clrwdt
; Initialize interrupts
		clrf	INTCON
		clrf	IOCAP
		clrf	IOCAN
		bsf	IOCAP,MODE_PIN
		bsf	IOCAN,MODE_PIN
		bsf	IOCIE
		bsf	GIE

; Ensure charger and discharger are off
		bcf	CHARGE_ENABLE
		bcf	LOAD_ENABLE

; Read mode pin. If 1 (jumper is off), then we are in charge-only mode.
; Otherwise, we are in storage mode.
#ifndef	DEBUG
		btfss	CHARGING_MODE
#endif
		goto	storage_initialize

; In charge only mode, we can just shut down and go to sleep.
; The charger chip will control everything.
; The WDT is enabled to wake
; up and reset after 8 seconds.  That way, if the battery is unplugged, the
; charger will shutdown after the WDT caused reset. 

charger_mode:
		clrf	PORTA			; Turn off all outputs
		clrf	LATA
		bsf	CHARGE_ENABLE
		;clrf	current_voltage
		movlw	0xfe
		movwf	previous_voltage
		movwf	current_voltage
charger_main_loop:
		clrf	FVRCON			; Turn off FVR to conserve power
		clrf	ADCON			; Turn off AD to conserve power
		bsf	CHARGE_ENABLE
		bsf	SWDTEN	    ; Turn on WDT to awke up after 8 seconds
charge_sleep:
		movf	current_voltage,w
		movwf	previous_voltage
		sleep
; See if the charge mode jumper has triggered an IOC interrupt.  If so, reset the CPU
		btfsc	SOFT_RESET_FLAG
		goto	initialise
		
; Read the voltage.  If below the previous voltage, assume the battery has been unplugged.
; Note that a lower voltage is a HIGHER value from ADRES!

		INIT_FVR
		READ_VOLTAGE
; See if current voltage is below the previous voltage.  Increment the previous
; voltage counter as a bit of hysterisis to avoid false triggering.
		incf	previous_voltage,w	; Reduce previous_voltage by 1 tick
		subwf	current_voltage,w	; Is it below?
		btfsc	ZERO			; If the same...
		goto	charger_main_loop	; ...go back to sleep
		btfss	CARRY			; If not below..., go back to sleep
		goto	charger_main_loop

; Assume the battery has been unplugged - turn off charger and reset
		bcf	CHARGE_ENABLE		; Charger off
		call	short_delay		; Let it stablize
		goto	initialise		; and reset

;
;Read the current voltage a four times and average it to make
; a current charge state determination and decide whether to charge up or discharge down
storage_initialize:
		call	delay_500
		READ_VOLTAGE
		addwf	initial_voltage,f
		call	short_delay
		READ_VOLTAGE
		addwf	initial_voltage,f
		btfsc	CARRY
		incf	initial_voltage_hi
		call	short_delay
		READ_VOLTAGE
		addwf	initial_voltage,f
		btfsc	CARRY
		incf	initial_voltage_hi
		call	short_delay
		READ_VOLTAGE
		addwf	initial_voltage,f
		btfsc	CARRY
		incf	initial_voltage_hi
; Now average the four values by shifting right twice
		bcf	CARRY
		rrf	initial_voltage_hi,f
		rrf	initial_voltage,f
		bcf	CARRY
		rrf	initial_voltage_hi,f
		rrf	initial_voltage,f
		movlw	HARD_DISCHARGEDOWN_CUTOFF	; Is it below the point where we need to charge up?
		subwf	initial_voltage,w
		btfss	CARRY			; If so, we are in charge-up mode
		goto	discharge_down_mode
charge_up_mode:
		movlw	HARD_CHARGEUP_CUTOFF
		movwf	charge_threshold
		movlw	MAX_CHARGEUP_CYCLES
		movwf	shutoff_counter
		goto	storage_main_loop
discharge_down_mode:
		movlw	HARD_DISCHARGEDOWN_CUTOFF
		movwf	charge_threshold
		movlw	MAX_DISCHARGEDOWN_CYCLES
		movwf	shutoff_counter
		goto	storage_main_loop

storage_main_loop:
; If there is an IOC interrupt for the charge mode jumper, reset the CPU
		btfsc	SOFT_RESET_FLAG
		goto	initialise
;
; Next, read the current battery voltage and test for low-voltage cutoff. The trick for this is to
; measure the on-board fixed voltage reference (FVR) which has been configured for 1.024 volts.
; The A/D value is ratiometric to Vdd (the battery voltage).  As the battery voltage drops,
; the FVR A/D value increases.
;
		READ_VOLTAGE
test_lvc:
		movf	charge_threshold,w		; Is it below the charging threshold?
		subwf	current_voltage,w
		btfss	CARRY				; If so, we may need to enable the charger
		goto	not_below_charge_threshold	; If not, continue voltage tests
		bcf	LOAD_ENABLE			; Turn off discharge load
		btfsc	CHARGE_ENABLE			; If currently charging
		goto	reset_hard_lvc_counter		; Go right to the end of the loop
		decfsz	hard_lvc_counter,f		; See if below the cutoff for HARD_LVC_COUNTER cycles
		goto	no_charge_yet			; If not, don't turn on the charger yet
enable_charger:
		bsf	CHARGE_ENABLE
		goto	reset_hard_lvc_counter
no_charge_yet:
;Keep track of how many cycles we have been below the charging threshold.
;When the shutoff cycle counter reaches 0, we've run enough, so go to sleep
		decfsz	shutoff_counter,f
		goto	reset_soft_lvc_counter
		goto	go_to_sleep		; We are done - go to sleep!

not_below_charge_threshold:
; Now test to see if the battery voltage is above or below the high storage charge threshold
		movlw	HI_LVC_CUTOFF
		subwf	current_voltage,w			; See if the ADC result < HI_LVC_CUTOFF
		btfss	CARRY
		goto	enable_load		; If not, enable the discharge load and reset the counter
;Keep track of how many cycles we have been below HI_LVC_CUTOFF.  When the
;shutoff cycle counter reaches 0, we've discharged enough, so go to sleep
		decfsz	shutoff_counter,f
		goto	not_done_yet
		goto	go_to_sleep		; We are done - go to sleep!

not_done_yet:
;Now see if it is below th LOW_LVC_CUTOFF point.  If it has been for SOFT_LVC_COUNT cycles,
;then disable the discharge load
		movlw	LOW_LVC_CUTOFF
		subwf	current_voltage,w			; Is it below the LOW_LVC_CUTOFF point?
		btfss	CARRY
		goto	lvc_check_done	; If not, we are in between hysterisis points, so do nothing
		decfsz	soft_lvc_counter,f		; Otherwise, decrement the counter
		goto	lvc_check_done			; If not 0, don't disable the load
disable_load:
		bcf	CHARGE_ENABLE
		bcf	LOAD_ENABLE				; Counter is 0, so we've had SOFT_LVC_COUNT successive
		goto	reset_soft_lvc_counter	; detections, therefore disable the discharge load
enable_load:
		btfss	CHARGE_ENABLE		; If not currently charging
		goto	turn_on_load		; turn on the load
		bcf	CHARGE_ENABLE		; Otheriwse, turn off the charger
		goto	reset_hard_lvc_counter	; And let the load come on next loop cycle
		;call	delay_500
		;call	delay_500
turn_on_load:
		bsf	LOAD_ENABLE		; Turn on load
reset_hard_lvc_counter:
		movlw	HARD_LVC_COUNT
		movwf	hard_lvc_counter

; If the current voltage is greater than the soft LVC cutoff voltage, reset the soft_lvc_counter.
; This counter ensures that we have several successive detections to make sure that spurious noise
; doesn't trigger the LVC
reset_soft_lvc_counter:
		movlw	SOFT_LVC_COUNT
		movwf	soft_lvc_counter
lvc_check_done:
		clrwdt
		call	delay_500		; Wait for one second before restarting the loop
		call	delay_500
		;call	delay_500
		;call	delay_500
		btfss	CHARGE_ENABLE		; If not charging
		goto	storage_main_loop	; end the loop
		call	delay_500		; Otherwise, wait for 2 more seconds
		call	delay_500
		call	delay_500
		call	delay_500
		;call	delay_500
		;call	delay_500
		goto	storage_main_loop

go_to_sleep:
		clrf	LATA			; Turn off all outputs
		clrf	PORTA
		clrf	FVRCON			; Turn off FVR to conserve power
		clrf	ADCON			; Turn off AD to conserve power
		bcf	SWDTEN
		sleep
		goto	initialise

;******************************************************************************
;* various delays (in milliseconds)                                           *
;******************************************************************************

delay_500:
        movlw   244             ;500ms delay
        movwf   db1             ;set outer loop count
        goto    time_delay      ;get going!
short_delay:
	movlw	5
	movwf	db1
time_delay:                      ;outer loop start
        clrf	db2
delay_l:                         ;inner loop start
        clrwdt                     ;waste 8us in loop
        nop                     ;
        nop                     ;
        nop                     ;
        nop                     ;
        nop                     ;
        nop                     ;
        nop                     ;
        nop                     ;
        nop                     ;
        nop                     ;
        nop                     ;
        decfsz   db2,f          ;256 x 16 cycle inner loop
        goto     delay_l        ;do more inner loops
        decfsz   db1,f          ;n cycle outer loop
        goto     time_delay     ;do more outer loops
        retlw    0              ;exit when all done

IRPC	char,---ConditionerV1.0---
    db	'char'
ENDM
    
        end	resetVec
