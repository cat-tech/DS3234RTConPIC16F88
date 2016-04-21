;====================================================================
;	NAME		: DS3234SPIonPIC.asm	
;							
;	DATE		: June 2007
;	AUTHOR		: John
;	DESCRIPTION	: Assembly software for PIC16F88 to display time
;			  on a 4x20 character HD44780-based
;			  LCD display, with 4-bits data transfer via CMOS 4094, HG-2006
;			  
;			Copyright under Gnu GPL, apart from LCD I/O, which is widely used 
;			where LCD is driven using 4094 shift register, openUART, and BinBCD (Microchip AN). 
;			
;			This has been written for the the Sparkfun 'breakout' board  
;			 	with the Maxim DS3234 Real Time Clock/calendar chip. 
;			This is a stripped out version with code only for the RTC and LCD. 
; 			Time is formatted and output to both the first line of the LCD and the serial port. 
; 			No provision is made for setting the time, other than a routine to set predefined 
; 				values.  
;			
;		It has been verified on a PIC16F88, but should compile for any of that ilk 
;			without alteration. 
;			
;			The hardware SPI port on the PIC16F88 appears to be buggy, although
;		 		probably works fine for most SPI use. Note that RB5 can't be used 
; 				for output if hardware SPI is being used. 
;			The DS3234 may be grabbing a part SPI port transfer. In any case I  
;				had much trouble with the counter chain resetting, which should
;				only happen on a full write to the seconds register (register 0). 
;			Proper operation is for /CS deassert/reassert to reset the SPI port transfer. 
;		
;	24/2/2016
;	transferred to 16f88 to allow use of ds3234 SPI clock
;
;	12/4/2016   software SPI complete and appears to be working
;	    still an issue with clock resetting on startup? 
;	Routine written to set clock (RTC_out_def). 
;
;	13/04/2016  Removed use of W_COPY in Write. 
;	    TODO:  check BB_TD 0x13:0 flag at startup, also control 0x0E:7 OSF
;		to make sure clock is running. If not, set default values.
;
;	14/04/2016
;	    RTC_in has been changed to read all 20 bytes. This simplifies the 
;	    code, and also brings in all the control info. This will allow  
;	    easy checking for initial condition on the RTC. 
;
;	    Yes, I could have changed the serial to use the SSP in the '88
;	    but this was originally written for an 'F84. 
;
;	    This is a list of routine entry points, these are the only ones 
;		that should be called from elsewhere in the program, 
;		as distinct from local calls within these routines.
;		
; 		MAIN_PROGRAM
; 		disp_time
; 		main_2
; 		INIT
; 		LCD_INIT
; 		WAIT_200ms
; 		WAIT_1ms
; 		WAIT_5us
; 		WAIT_4us
; 		WriteUART
; 		BinBCD
; 		DISPLAY_TIME_DATE
; 		DigitWrite
; 		Write
; 		LCD_CLEAR_SCREEN
; 		LCD_START_LINE_1
; 		LCD_START_LINE_2
; 		LCD_START_LINE_3
; 		LCD_START_LINE_4
; 		SerialNL
; 		RTC_in
; 		RTC_out_def
; 		;
;
;====================================================================


;******************************************************************************
;   This file is a basic code template for code generation                    *
;   on the  PIC16F88. This file contains the basic code building              *
;   blocks to build upon.                                                     *
;******************************************************************************
;                                                                             *
;    Files Required: P16F88.INC                                               *
;                                                                             *
;******************************************************************************

;------------------------------------------------------------------------------
; PROCESSOR DECLARATION
;------------------------------------------------------------------------------

     LIST      p=16F88              ; list directive to define processor
     #INCLUDE <P16F88.INC>          ; processor specific variable definitions

;------------------------------------------------------------------------------
;
; CONFIGURATION WORD SETUP
;
; The 'CONFIG' directive is used to embed the configuration word within the
; .asm file. The lables following the directive are located in the respective
; .inc file.  See the data sheet for additional information on configuration
; word settings.
;
;------------------------------------------------------------------------------

     __CONFIG    _CONFIG1, _CP_OFF & _CCP1_RB0 & _DEBUG_OFF & _WRT_PROTECT_OFF & _CPD_OFF & _LVP_OFF & _BODEN_ON & _MCLR_ON & _PWRTE_ON & _WDT_OFF & _XT_OSC
     __CONFIG    _CONFIG2, _IESO_OFF & _FCMEN_OFF

;------------------------------------------------------------------------------
;
; VARIABLE DEFINITIONS
;
; Available Data Memory divided into Bank 0 through Bank 3.  Each Bank contains
; Special Function Registers and General Purpose Registers at the locations
; below:
;
;           SFR           GPR               SHARED GPR's
; Bank 0    0x00-0x1F     0x20-0x6F         0x70-0x7F
; Bank 1    0x80-0x9F     0xA0-0xEF         0xF0-0xFF
; Bank 2    0x100-0x10F   0x110-0x16F       0x170-0x17F
; Bank 3    0x180-0x18F   0x190-0x1EF       0x1F0-0x1FF
;
;------------------------------------------------------------------------------


W_TEMP         EQU        0x7D  ; w register for context saving (ACCESS)
STATUS_TEMP    EQU        0x7E  ; status used for context saving (ACCESS)
PCLATH_TEMP    EQU        0x7F  ; variable used for context saving

;------------------------------------------------------------------------------
; EEPROM INITIALIZATION
;
; The 16F88 has 256 bytes of non-volatile EEPROM, starting at address 0x2100
;
;------------------------------------------------------------------------------

DATAEE    ORG  0x2100
    DE    "20160416"  ; latest update on code
			; It is here so I can see it in the programmer!



;	SETTINGS
	ERRORLEVEL -302 	;suppress message 302 (proper bank)
;	            ; list directive to define processor
;	        ; processor specific variable definitions
	


;**************************************************************************************
;
;		serial port		out (Tx) port B,6		;pin 12		white/green
;						in (Rx)  port B,7		;pin 13	(not used)
;
;		display port	CLOCK_LCD	PORTA,0		;pin 17		orange
;						DATA_LCD	PORTA,1		;pin 18		blue
;						STROBE_LCD	PORTA,2		;pin 1		white
;
;
;	DS3234 port	Real Time Clock		SPI interface: 4 wire
;				;			Pic pin		Sparkfun DS board pin
;		MOSI	Port B,2		 8			2	out
;		MISO	Port B,1		 7			3	in
;		SCK	 	Port B,4		10			4	out
;		;//	SS	Port B,5		11			1	out // can't use this pin if hardware SPI active
;		SS	 	Port B,3		 9			1	out
					;						6	vcc
					;						7	gnd
;


SWTXD           equ     PORTB           ; Transmit pin port and pin
SWTXDpin        equ     6					;pin 12
TRIS_SWTXD      equ     TRISB           ; Transmit pin tris and pin
SWRXD           equ     PORTB           ; Receive pin port and pin
SWRXDpin        equ     7
TRIS_SWRXD      equ     TRISB           ; Receive pin tris and pin

Serial			equ	1			; serial out active for 1
LCD				equ	1			; LCD output active for 1

;	DEFINITION PORTA (A0-A4)
	#define	IN_OUT_PORTA
	#define	CLOCK_LCD	PORTA,0		;pin 17
	#define	DATA_LCD	PORTA,1		;pin 18
	#define	STROBE_LCD	PORTA,2		;pin 1
;	LED	3			pin 2

;	 PORTB (B0-B7) pin use
;	SPI		1, 2, 4, 5
;	serial port		6, 7
;	CCP		0

; Defines
;	defines for DS3432 clock using SPI
SPI		equ		PORTB
MOSI	equ		2
MISO	equ		1
SCK		equ		4
SS		equ		3


;	LCD_BYTE
	#define	RS			LCD_BYTE,0	; register select in 
							; LCD control / data
	#define	LCD_1			LCD_BYTE,1	;for free use
	#define LCD_2			LCD_BYTE,2	;for free use
	#define	BACKLIGHT		LCD_BYTE,3

;	BITS1
	#define	Dtemp			BITS1,3
	#define LCD_EN			BITS1,4		;enable output to LCD
	#define SER_EN			BITS1,5		;enable output to serial port


	;   SPI values 			hardware SPI not good for DS3234, causes reset of counter chain. 
	;#define	    SSPCON_val	0x32	    ; mode 3	fosc/64, master
	;#define	    SSPSTAT_val	0xC0
	;#define	    SSPCON_val	0x22	    ; mode 1
	;#define	    SSPSTAT_val	0x00

RAM		equ	H'20' 		;start of RAM
;	RAM
	cblock	RAM
			COUNTER_1				;
			COUNTER_2				; time delays
			COUNTER_3				;
			BITC					; counter for LCD serialiser out to 4094


		; 
			BITS1		
			BYTE			; used for LCD I/O 
			CRC_CHECK
			LCD_BYTE
			MEM_1
			spare1
			spare2
			spare3
			W_COPY
			W_COPY2
			DISP_NIB
			W_COPYa
		; 30

			TEMP2						
			spare14
			BITS2
			MSD		; BCD conversion, tens
			LSD		;
			W_temp		; used within subroutines only - may be reused 
			BITn
			spare15
			spare4
			spare5	
			spare6
			spare7
			spare8
			spare9
			spare10
			spare11
			;	40

			spare12
			spare13
			BIT_COUNT
			uartdata
			BitCount
			delay1
	time_print	
	Byte_counter	; block transfer counter
	Bit_counter		; counter for each bit in serial transfer
	RA_copy			; used to set output bits for port A, (maybe not..) 
	RB_copy			; as previous, for port B
	IO_temp			; temporary holding register for IO routines, available to be used 
					; in other routines
	Clock_Data_Reg		; start of clock data for block transfer DS3234, seconds
	CDR0			; as previous, seconds
	CDR1			; minutes
	CDR2			; hours
	;	50
	
	CDR3			; day of the week		// day of the month
	CDR4			; day of the month		// month
	CDR5			; month					// day of the week
	CDR6			; year
	CDR7				; 	alarms
	CDR8				; 	alarms
	CDR9				; 	alarms
	CDRA				; 	alarms
	CDRB				; 	alarms
	CDRC				; 	alarms
	CDRD				; 	alarms
	CDRE			; control	//control register (write protect), always keep contents $FF
	CDRF			; control / status
	CDR10			; crystal ageing (ignore)
	CDR11			; temperature : sttt oooo
	CDR12			; temperature fraction : dd00 0000
	;	60
	
	CDR13			; BB_TF	    battery backed temperature conversion flag

		;	SPI registers
			RXDATA
			TXDATA
			PBcopy
			
	endc



DEBUG 	equ 	0 ;1


;***************************************************************************************
;	end defines
;***************************************************************************************

;====================================================================
;------------------------------------------------------------------------------
; RESET VECTOR 			This is the the default Microchip routine. 
;------------------------------------------------------------------------------

RESET     ORG     0x0000            ; processor reset vector
          PAGESEL MAIN_PROGRAM		;START
          GOTO    MAIN_PROGRAM		;START             ; go to beginning of program

;------------------------------------------------------------------------------
; INTERRUPT SERVICE ROUTINE
;------------------------------------------------------------------------------

ISR       ORG     0x0004            ; interrupt vector location

;         Context saving for ISR
          MOVWF   W_TEMP            ; save off current W register contents
          MOVF    STATUS,W          ; move status register into W register
          MOVWF   STATUS_TEMP       ; save off contents of STATUS register
          MOVF    PCLATH,W          ; move pclath register into W register
          MOVWF   PCLATH_TEMP       ; save off contents of PCLATH register

;------------------------------------------------------------------------------
; USER INTERRUPT SERVICE ROUTINE GOES HERE
;------------------------------------------------------------------------------

;   Restore context before returning from interrupt
    MOVF    PCLATH_TEMP,W     ; retrieve copy of PCLATH register
    MOVWF   PCLATH            ; restore pre-isr PCLATH register contents
    MOVF    STATUS_TEMP,W     ; retrieve copy of STATUS register
    MOVWF   STATUS            ; restore pre-isr STATUS register contents
    SWAPF   W_TEMP,F
    SWAPF   W_TEMP,W          ; restore pre-isr W register contents
    RETFIE                    ; return from interrupt

 	db	0, "S", 0, "P", 0, "I", 0, " ", 0, "t", 0, "e", 0, "s", 0, "t"

;------------------------------------------------------------------------------
; MAIN PROGRAM
;------------------------------------------------------------------------------
		
MAIN_PROGRAM
	nop
	call	INIT
	call	RTC_in 
	call 	SerialNL		; <CR> <LF> on serial port
	
	MOVLW	5 
	CALL	WAIT_200ms		; wait for Logomatic on serial port
	
	;call	RTC_out_def	    ; setup RTC, this saves the predefined 
							; time and control values into the RTC
	call	LCD_START_LINE_1	; 

	CALL 	DISPLAY_TIME_DATE   ; 
	MOVLW 	" "
	CALL 	Write
	call 	SerialNL		; <CR> <LF> on serial port
	goto	main_3			; 

disp_time					; this is the main wait loop
							
			; display the time, then check if it is 5 or 0 
			; minutes. If it is, continue on.
	nop
	call	RTC_in			; DS3234

	call	LCD_START_LINE_1	; time and date on line 1
	BSF		LCD_EN		;constantly updating, so only to LCD
	BCF		SER_EN
	CALL 	DISPLAY_TIME_DATE

	MOVLW 	" "
	CALL 	Write		    ; LCD_WRITE_CHARACTER

							; wait for 5 minute point before
							; further code
	movf 	CDR0,F			; secs == 0?
	btfss 	STATUS,Z		; 		
	goto 	disp_time
	movf 	CDR1,W
	andlw 	0x0F 			; units minutes == 0?
	btfsc 	STATUS,Z		; 	
	goto  	main_3		    ; minutes == 0, do what we really want..
	movf 	CDR1,W
	andlw 	0x0F 			; units minutes == 5?
	sublw 	0x05
	btfss 	STATUS,Z	    ; minutes == 5, do what we really want..
	;goto  	main_3
	goto 	disp_time	; neither 0 nor 5 minutes
main_3				; ******************************************
				; ******************************************
	BCF		LCD_EN
	BSF		SER_EN
	CALL 	DISPLAY_TIME_DATE   ; send time and date to serial. 
	BSF	 	LCD_EN	    ; re-enable the LCD, becuse we want all info to both
	
main_2
	
			;  code to be run every 5 minutes goes in here...............

	call	RTC_in				; update time from DS3234
	call	LCD_START_LINE_1
	BSF		LCD_EN		; update time on LCD only
	BCF		SER_EN		
	CALL 	DISPLAY_TIME_DATE
	BSF		SER_EN		; re-enable serial out for ...
	
	MOVLW 	" "
	CALL 	LCD_WRITE_CHARACTER
	MOVLW	5
	CALL	WAIT_200ms		; Wait 1 second
	CALL	SerialNL
	
	goto 	disp_time	; 	display time until minutes 
						; divisible by 5  	 

; ********************************************************

INIT

;	BANKSEL		PORTA ; select bank of PORTA
	bcf		STATUS,RP0
	bcf		STATUS,RP1
	CLRF	PORTA ; Initialize PORTA by
				; clearing output data latches
	movlw	B'01000100'	; SWTXD & SPI SS high
	movwf	PORTB 		; This is very important! Do this prior to setting TRIS.
;	BANKSEL		ANSEL ; Select Bank of ANSEL
	bsf		STATUS,RP0		; bank 1
	MOVLW	0x00 ; Configure all pins
	MOVWF	ANSEL ; as digital inputs

					; TRIS bits are '1' for input
	movlw   B'00000000'		;define all I/O ports RA0-RA4 outputs
	movwf	TRISA
	movlw   B'11111111'		;define all I/O ports RB0-RB7 in, RB5 in
	movwf	TRISB
	bcf		SPI,MOSI			;}
	bcf		SPI,SCK				;}3 outputs for SPI
	bcf		SPI,SS				;}
	bsf		SPI,MISO			; SPI input
	bcf		TRIS_SWTXD,SWTXDpin		; serial port TX
	bsf		TRIS_SWRXD,SWRXDpin		; serial port RX
	bcf		PORTA,3				; blink test port, output

	bcf		STATUS,RP0		; back to bank 0



; UARTCODE        CODE
;********************************************************************
;*      Function Name:  OpenUART                                    *
;*      Return Value:   void                                        *
;*      Parameters:     void                                        *
;*      Description:    This routine configures the I/O pins for    *
;*                      software UART.                              *
;********************************************************************
    bsf     SWTXD,SWTXDpin  ; Make TXD high
	bsf		SPI,SS			; set DS3234 chip select high: deselected
	bcf		SPI,SCK			; clock low, idle

;clear all output ports RB0-RB7
	bcf		CLOCK_LCD	    ;reset CLOCK		RA0
;		LCD on port A
;		SPI and serial on port B


; SPI initialisation
	CLRF 	STATUS ; Bank 0
	BSF 	STATUS, RP0 ; Bank 1
	BCF 	PIE1, SSPIE ; Disable SSP interrupt
	BCF 	PIE2, SSPIE ; Disable SSP interrupt
	BCF 	STATUS, RP0 ; Bank 0
	clrf	INTCON		; DISABLE INTERRUPTS

	CALL 	LCD_INIT		;initialize LCD-module	
					; enable outputs both ways by default
	bsf		LCD_EN			; print to LCD
	bsf		SER_EN			; print to serial

	return
	
;**********************************************************
;==========================================================
LCD_INIT
	if DEBUG==1
	    return;
	endif
	
	movlw	D'40'			;LCD-module needs 40 ms after power rise
	call	WAIT_1ms
	bcf		STROBE_LCD		;STROBE low
	bcf		CLOCK_LCD		;CLOCK low
	clrf	LCD_BYTE		;RS='0', BACKLIGHT is off ; RS is register select
	movlw	B'00100000'		;set interface to 4 bits
	call	LCD_WRITE_BYTE
	movlw	D'5'			;LCD-module needs 5 ms for processing
	call	WAIT_1ms
	movlw	B'00101000'		;4-bits interface, 2 lines, 5x7 matrix
	call	LCD_WRITE_BYTE
	movlw	D'5'			;LCD-module needs 5 ms for processing
	call	WAIT_1ms
	movlw	B'00001100'		;display on, cursor off
	call	LCD_WRITE_BYTE
	movlw	B'00000110'		;cursor moves automatically to next position
	call	LCD_WRITE_BYTE

	call	LCD_CLEAR_SCREEN
	return

;====================================================================
WAIT_200ms
;	Wait as many times 200ms as the number in the workspace presents

					; uses COUNTER_3
					; calls WAIT_1ms
	movwf	COUNTER_3
wait_200ms_1
	movlw	D'200'
	call	WAIT_1ms
	decfsz	COUNTER_3,F
	goto	wait_200ms_1
	return
;==================
WAIT_1ms
;	Wait as many times 1ms as the number in the workspace presents

					; uses COUNTER_2
					; calls WAIT_5us
	movwf	COUNTER_2
wait_1ms_1
	movlw	D'200'
	call	WAIT_5us
	decfsz	COUNTER_2,F
	goto	wait_1ms_1
	return
;==================
WAIT_5us
;	Wait as many times 5us as the number in the workspace presents
;	minimum 2x5us, call included
					; uses COUNTER_1
	addlw	D'255'
	movwf	COUNTER_1
wait_5us_1
	nop
	nop
	decfsz	COUNTER_1,F
	goto	wait_5us_1
	return
;==================
WAIT_4us
;	Wait 4 us (call included)
	return

;*********************************************************

; UARTCODE        CODE
;************************************************************************
;*      Function Name:  WriteUART                                       *
;*      Return Value:   void                                            *
;*      Parameters:     data: byte to send out the UART                 *
;*      Description:    This routine sends a byte out the TXD pin.      *
;*      Special Note:   The user must provide a routine:                * 
;*                      DelayTXBit():                                   *
;*                        DelayTXBit should have:                       *
;*                              8 Tcy for overhead                      *
;*                              2 Tcy call to DelayTXBit                *
;*                              ? Tcy                                   *
;*                              2 Tcy for return from DelayTXBit        *
;*                      = (((2*OSC_FREQ)/(4*BAUDRATE))+1)/2  Tcy        *
;*				  4 MHz & 9600 baud    ==> 104.67 Tcy                	*
;*                                                                      *
;*                                                                      *
;************************************************************************

WriteUART
    movwf   uartdata				; save output character

WriteUART_2
    movlw   0x09                    ; Set bit counter for 9 bits
    movwf   BitCount
    bcf     STATUS,C        	; Send start bit (carry = 0)
    ; banksel SWTXD
    goto    SendStart
SendBit
    ;banksel uartdata
	bcf 	STATUS,C
    rrf 	uartdata,F              ; Rotate next bit into carry
    nop
    nop
    nop
    nop
SendStart
	btfsc 	STATUS,C                ; Set or clear TXD pin according
	goto 	check0
	nop
 	nop
	nop
	bcf     SWTXD,SWTXDpin  	; to what is in the carry
check0
	btfss   STATUS,C
	goto 	check1
	bsf     SWTXD,SWTXDpin
	nop
	nop
	nop
check1
	call    DelayTXBitUART      ; Delay for 1 bit time
	;banksel BitCount
	decfsz  BitCount,F          ; Count only 9 bits
	goto    SendBit
	;banksel SWTXD
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
 	bsf 	SWTXD,SWTXDpin  	; Stop bit is high
 	call 	DelayTXBitUART      ; Delay for stop bit
 	movlw 	1
	call 	WAIT_1ms			; wait approx 1 char space 
	return

DelayTXBitUART

	movlw	D'27' ; this gives 9616 baud 
			;0x1F ;1 ; 		; was D'30'	now d'34'
			;delay time for 9600 baud, 4 MHz clock
	movwf	delay1

delayTXdelay1
	decfsz	delay1,f
	goto 	delayTXdelay1
	nop
	nop
	return

;*********************************************************

;
;       This routine converts the 8 bit binary number in the W Register
; to a 2 digit BCD number.
;       The least significant digit is returned in location LSD and
; the most significant digit is returned in location MSD.
;
;   Performance :
;               Program Memory  :       10
;               Clock Cycles    :       81 (worst case when W = 63 Hex )
;                                          ( i.e max Decimal number 99 )
;
;       Program:          BIN8BCD.ASM 	Microchip AN 526 g
;       Revision Date:   
;                         1-13-97      Compatibility with MPASMWIN 1.40
;     Modified for constant spacing. i.e. always 2 characters	
BinBCD
	clrf    MSD
	movwf   LSD
gtenth  
	movlw   D'10'
	subwf   LSD,W
	BTFSS   STATUS,C
	goto    over
	movwf   LSD
	incf    MSD, F
	goto    gtenth
	
over    
	movf	MSD,W
	btfsc	STATUS,Z		; if tens is not zero
	goto	write_temp_6

write_temp_5
	addlw	"0"
	call	Write		; display tens
	goto 	write_temp_9
	
write_temp_6	; this should only be entered if a 
		; digit is not being printed in tens place
	movlw	" "		;space to replace tens
	call	Write		;

write_temp_9
	movf	LSD,W
	addlw	"0"
	call	Write		; display units
	return

;*********************************************************

DISPLAY_TIME_DATE	; outputs formatted time and  
			; date to  LCD &/or serial port 
			; LCD_EN and SER_EN flag bits in 
			; must be set before entry, to indicate which 
			; outputs to use. 
	MOVLW 	3
	MOVWF 	time_print
	MOVLW 	CDR2
	MOVWF 	FSR
	GOTO 	time_in
time_loop
	MOVLW 	":"
	CALL 	Write
time_in
	SWAPF 	INDF,W
	ANDLW 	0x07
	CALL 	DigitWrite
	MOVF 	INDF,W
	ANDLW 	0x0F
	CALL 	DigitWrite
	DECF 	FSR,F
	DECFSZ 	time_print,F
	GOTO 	time_loop	
	
	MOVLW 	" "
	CALL 	Write

display_date 		; this is not called, just for documentation
	MOVLW 	2
	MOVWF 	time_print
	MOVLW 	CDR4
	MOVWF 	FSR
	
date_loop
date_in	
	SWAPF 	INDF,W
	ANDLW 	0x0F
	CALL 	DigitWrite
	MOVF 	INDF,W
	ANDLW 	0x0F
	CALL 	DigitWrite
	MOVLW 	"/"
	CALL 	Write
	INCF 	FSR,F
	DECFSZ 	time_print,F
	GOTO 	date_loop	
	
	MOVLW 	"2"
	CALL 	Write
	MOVLW 	"0"
	CALL 	Write
	SWAPF 	INDF,W
	ANDLW 	0x0F
	CALL 	DigitWrite
	MOVF 	INDF,W
	ANDLW 	0x0F
	CALL 	DigitWrite
	MOVLW 	" "
	CALL 	Write
	return
	
			; This pair of routines sends characters to serial or the LCD, 
			; depending on the SER_EN and LCD_EN flags.   
DigitWrite	    ; This prints a single decimal digit. 
		    ; No checking is done that the number is within 
		    ; range of 0 to 9. Not masked for single digit. 
	addlw 	D'48' 			; adjust BCD to ASCII 0x30
Write
	movwf 	W_COPY2			; 
	BTFSC 	LCD_EN
	CALL 	LCD_WRITE_CHARACTER
	MOVF 	W_COPY2,W
	BTFSC 	SER_EN
	call 	WriteUART  		; then output on serial port
	return

;====================================================================
LCD_CLEAR_SCREEN			;clears screen, cursor on upper left position
	bcf 	RS
	movlw	B'00000001'
	call	LCD_WRITE_BYTE
	movlw	D'2'			;LCD-module needs 2 ms for processing
	call	WAIT_1ms
	return

LCD_START_LINE_1			;put cursor on 1st position of 1st line
	bcf 	RS
	movlw	B'10000000'
	call	LCD_WRITE_BYTE
	return

LCD_START_LINE_2			;put cursor on 1st position of 2nd line
	bcf 	RS
	movlw	B'11000000'
	call	LCD_WRITE_BYTE
	return

LCD_START_LINE_3			;put cursor on 1st position of 3rd line
	bcf 	RS
	movlw	B'10010000'
	call	LCD_WRITE_BYTE
	return

LCD_START_LINE_4			;put cursor on 1st position of 4th line
	bcf 	RS
	movlw	B'11010000'
	call	LCD_WRITE_BYTE
	return

LCD_WRITE_CHARACTER			;display character from workspace
	bsf	RS
	call	LCD_WRITE_BYTE
	return

LCD_WRITE_BYTE				;writes 8 bits from workspace with given RS/BL
					; RS - register select - switches between control
					; and display characters
	movwf	MEM_1			;keep 2nd half of databyte stored
	call	LCD_WRITE_HALF		;send 1st half of databyte (bit 7-4)
	swapf	MEM_1,W			;get back 2nd half of databyte
	call	LCD_WRITE_HALF		;send 2nd half of databyte (bit 3-0)
	return

LCD_WRITE_HALF
	andlw	B'11110000'		;select only the first 4 bits
	movwf	BYTE			;store them in BYTE
	movf	LCD_BYTE,W		;0	20160315		;get LCD_BYTE for the right 4 bits
	iorwf	BYTE,F			;merge and restore in BYTE
	movlw	D'8'			;write D7-D0 to 4094 ...
	movwf	BITC			;... and keep bit-counter in BITC
lcd_write_half_1
	bcf		DATA_LCD	;make DATA-line '0' ...
	btfsc	BYTE,7		;... but if leftmost bit='1' ...
	bsf 	DATA_LCD	;... then make DATA-line '1'
	bsf 	CLOCK_LCD	;give a clock-pulse ...
	bcf 	CLOCK_LCD	;... and make CLOCK '0' again
	rlf		BYTE,1		;move bits in BYTE 1 position to the left
	decfsz	BITC,F		;repeat until BIT=0
	goto	lcd_write_half_1
	bsf 	STROBE_LCD	;make STROBE '1' to strobe data into 4094 ...
	nop
	bcf 	STROBE_LCD	;... and make STROBE '0' again
	movlw	D'2'		;LCD-module needs 8x5=40 us processing time
	call	WAIT_5us
	return
;********************************************************

SerialNL
	movlw 	0x0D	;"/r"			; carraige return
	call 	WriteUART
	movlw 	0x0A	;"/n"			; line feed
	call 	WriteUART
	return

;===========================================================================
;
;	DS3234 clock interface
;	SPI interface: MOSI, MISO, SCLK, SS
;
;	defines for DS3432 clock using TWI
;													Sparkfun
;								;		Pic pin		RTC board pin
;#define		MOSI	PORTB,0x02	;B2		 8				2
;#define		MISO	PORTB,0x01	;B1		 7				3
;#define		SCK		PORTB,0x04	;B4		10				4
;#define		SS		PORTB,0x05	;B5		11				1
;														5	int / sq wave
;								;						6	vcc
;								;						7	gnd

;		CKP	0 clock polarity normally low
;		DS3234 will sample on falling edge, output change on rising edge
;		TRISB	xx00 x01x
;		PORTB	xx10 xxxx
;		PIR1				peripheral interrupt
;		PIE1				peripheral interrupt enable


; DS3234 registers:
;	00	seconds
;	01	minutes
;	02	hours
;	03	day
;	04	'date' day of month
;	05	month + century
;	06	year
;	//07 ~ 0D		alarm registers
;	0E	Control										default: 1C
;	0F	control / status: b7 osc stop flag, b2 busy		def: C8, set to 00
;	11	Temperature MSB						sign, 7 bit integer
;	12	Temperature LSB						2 bit fraction : 0.25 deg increment


;	retrieve time/date by feeding out address, then taking all registers in.
;		Register address will automatically increment.
;			{chip select set (low)
;			{address out	} these 2 use the same I/O routine
;			{data in/out	}
;			{chip select clear (high)

;	chip select
set_SPI_CS
	BCF 	SPI,MOSI	; try making sure data line is low before
				; there is any activity on SPI bus
	BCF 	STATUS, RP1 ; ensure Bank 0
	BCF 	STATUS, RP0 ;
	BCF 	SPI,SS		; active low
	return


;	address out not needed as separate routine - load TXDATA with address,
;		then let SPI routine send it out

	
		    ; loads RTC data into Clock_Data_Reg set
		    ; i.e. copies data from RTC to CDR0 to CDR13
			; No provision is made for RAM access, although this could be 
			; 	easily written. 
RTC_in

	call	set_SPI_CS		; set chip select active
	CLRF	TXDATA			; start with reg 0, this sets the address
	MOVLW	0x14		; 19  bytes + address, 19 (0x13) times through the exit decrementer
	MOVWF	Byte_counter
	MOVLW	Clock_Data_Reg
	MOVWF	FSR		; set up for indexed store of bytes.
	incf 	FSR,F
	CALL	SPI_out			; address has been set, send
								; ignore any return
CBI_LOOP
	CALL	SPI_out			; 
	MOVF	RXDATA,W			; pick up incoming data
	MOVWF	INDF		; save the data to @ FSR
	INCF	FSR,F		; FSR to the next location
	call	WAIT_4us
	DECFSZ	Byte_counter,F
	GOTO	CBI_LOOP	; not yet to 0 
	call	des_SPI_CS		; tell the RTC that we are going to jump to another address

	RETURN


;	chip deselect
des_SPI_CS
	BCF 	STATUS, RP1 ; ensure Bank 0
	BCF 	STATUS, RP0 ;
	BSF 	SPI,SS		; inactive high
				; CS is now off, but I want the data line (MOSI) low
				; so output 0x00
	BCF 	SPI,MOSI
	return


		; loads RTC with default data set
		; i.e. copies data into RTC 
		; data in BCD format
RTC_out_def

	call	set_SPI_CS		; set chip select active
	MOVLW	0x80		; address 0, write
	MOVWF	TXDATA
	CALL	SPI_out			; address has been set, send
	call	WAIT_4us	    ; waits are inserted to assist analysis with LA 

	MOVLW	0x01		; data 0: seconds
	MOVWF	TXDATA
	CALL	SPI_out			; 
	call	WAIT_4us	    ; wait 4 us before re-asserting CS

	MOVLW	0x45		; data 1 : minutes
	MOVWF	TXDATA
	CALL	SPI_out			; 
	call	WAIT_4us	    ; wait 
	MOVLW	0x01		; data 2 : hours
	MOVWF	TXDATA
	CALL	SPI_out			; 
	call	WAIT_4us	    ; wait 
	MOVLW	0x06		; data 3 : day of week 1~7
	MOVWF	TXDATA
	CALL	SPI_out			; 
	call	WAIT_4us	    ; wait 
	MOVLW	0x15		; data 4 : date in month
	MOVWF	TXDATA
	CALL	SPI_out			; 
	call	WAIT_4us	    ; wait 
	MOVLW	0x04		; data 5 : month
	MOVWF	TXDATA
	CALL	SPI_out			; 
	call	WAIT_4us	    ; wait 
	MOVLW	0x16		; data 6 : year
	MOVWF	TXDATA
	CALL	SPI_out			; 
	call	des_SPI_CS		; tell the RTC that we are going to jump to another address

	movlw	10
	call	WAIT_5us	    ; wait 50 us before re-asserting CS
	
	call	set_SPI_CS		; set chip select active
	MOVLW	0x8E		; address 8E : control
	MOVWF	TXDATA
	CALL	SPI_out			; 
	call	WAIT_4us	    ; wait 
	MOVWF	TXDATA
	MOVLW	0x04	    ; 1C		; data 0E : control
	MOVWF	TXDATA
	CALL	SPI_out			; 
	call	WAIT_4us	    ; wait 
	MOVLW	0x30	    ;00		; data 0F : control / status
			    ; lowest conversion rate on battery
	MOVWF	TXDATA
	CALL	SPI_out			; 
	call	des_SPI_CS		; 
	
	call	set_SPI_CS		; set chip select active
	MOVLW	0x13		; address 13: BB_TD
	MOVWF	TXDATA		; battery backed temp conversion
	CALL	SPI_out			; 
	call	WAIT_4us	    ; wait 
	MOVWF	TXDATA
	MOVLW	0x00	    ; set BB_TD to 0
	MOVWF	TXDATA
	CALL	SPI_out			; 
	call	WAIT_4us	    ; wait 
	call	des_SPI_CS		; 
	
	RETURN

;************************************************************************
;*      Function Name:  SPI_out                                         *
;*      Return Value:   RXDATA                                          *
;*      Parameters:     TXDATA: byte to send out on SPI                 *
;*      Description:    data out on MOSI, in on MISO, clock on SCK      *
;*                                                                      *
;************************************************************************
			; SPI has input and output at the same time, so this is both. 
			; implemented as SPI mode 1
			; Software SPI used as both Microchip and Maxim DS3234 seem 
			; 	to be buggy implementations on SPI. 
SPI_out
	; TXDATA  RXDATA
	if DEBUG==1
	    return;
	endif
	       			; 

    movlw   0x08     ; Set bit counter for 8 bits
    movwf   BitCount
	
SPInextBit
	bcf 	STATUS,C
    rlf 	TXDATA,F    ; Rotate next bit into carry
						; MSB first
    btfsc 	STATUS,C    ; Set or clear MOSI pin according
    goto 	SPIcheck0
    bcf  	SPI,MOSI  	; to what is in the carry
    goto 	SPIcheck1
SPIcheck0
    bsf 	SPI,MOSI
SPIcheck1
			; this is synchronous, timing is not critical as long 
			; as we respect clock polarity
			; data is set on first clock edge, and latched on second
	bsf		SPI,SCK		; set the clock
	movlw	2
	call	WAIT_5us
				;   MISO bit in
	rlf 	RXDATA,F	; carry bit goes into RXDATA,0
	bsf 	RXDATA,0
	btfss	SPI,MISO	; input bit
	bcf 	RXDATA,0

	bcf 	SPI,SCK		; clear clock bit
	movlw	2
	call	WAIT_5us
	
    decfsz 	BitCount,F          ; Count only 8 bits
    goto 	SPInextBit
    nop
    bcf  	SPI,MOSI 	; 
	return

	end
