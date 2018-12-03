;****************************************************************
; 			Defintition of all port of e-puck			 		*
;			Version 1.0 november 2005							*
;			Michael Bonani, 									*
;																*
;****************************************************************

;.IFNDEF _EPUCK_PORTS
;.def 	_EPUCK_PORTS

.include "p30F6014A.inc"

;/*********************GENERAL SETUP************************/

.equiv FOSC, 7372800     ; 7.3728Mhz crystal in XTL mode 
.equiv PLL, 8       	; 8x PLL
 

.equiv FCY,     ((FOSC*PLL)/(4))	; Instruction cycle frequency 
.equiv MILLISEC , (FCY/1000)		; 1mSec delay constant
.equiv MICROSEC,  (FCY/1000000)		; 1uSec delay constant
.equiv NANOSEC,   (FCY/1000000000)		; 1nSec delay constant

.equiv	TCY_PIC,		(1000000000/FCY)		;time instruction cycle in [ns]
.equiv	INTERRUPT_DELAY,	(10*TCY_PIC);delay to start an interrupt in [ns] (observe with p30f6014) 

.equiv TRUE,	1
.equiv FALSE,	0


;/********************** OUTPUTS ***************************/
.equiv OUTPUT_PIN, 0 
;/*LEDS*/
;/*First in front of robot than turning clokwise*/
.equiv	LEDPORT, LATA
.equiv LED0, LATA6
.equiv LED1, LATA7 
.equiv LED2, LATA9
.equiv LED3, LATA12
.equiv LED4, LATA10
.equiv LED5, LATA13
.equiv LED6, LATA14
.equiv LED7, LATA15

.macro ONLED num
	bset LEDPORT,#\num
.endm
.macro OFFLED num
	bclr LEDPORT,#\num
.endm

.equiv	LEDPORT2, LATC
.equiv FRONT_LED, LATC1
.equiv BODY_LED, LATC2


;/*IR*/
.equiv PULSE_PORT1, LATF
.equiv PULSE_IR0, LATF7		; PULSE IR 0 and 4
.equiv PULSE_IR1, LATF8		; PULSE IR 1 and 5
.equiv PULSE_PORT2, LATg
.equiv PULSE_IR2, LATG0		; PULSE IR 2 and 6
.equiv PULSE_IR3, LATG1		; PULSE IR 3 and 7


;/*First in front left of robot than turning clokwise*/
.equiv IR0, 8  ; ir proximity sensor 0 on AD channel 8
.equiv IR1, 9  ; ir proximity sensor 1 on AD channel 9
.equiv IR2, 10  ; ir proximity sensor 2 on AD channel 10
.equiv IR3, 11  ; ir proximity sensor 3 on AD channel 11
.equiv IR4, 12  ; ir proximity sensor 4 on AD channel 12
.equiv IR5, 13  ; ir proximity sensor 5 on AD channel 13
.equiv IR6, 14  ; ir proximity sensor 6 on AD channel 14
.equiv IR7, 15  ; ir proximity sensor 7 on AD channel 15


;/*analog*/
.equiv MIC1, 2  ; microphone 1 on AD channel 2
.equiv MIC2, 3  ; microphone 2 on AD channel 3
.equiv MIC3, 4  ; microphone 3 on AD channel 4

.equiv ACCX, 5  ; X Axis of accelerometer on AD channel 5
.equiv ACCY, 6  ; Y Axis of accelerometer on AD channel 6
.equiv ACCZ, 7  ; Z Axis of accelerometer on AD channel 7

;/*basic audio*/
.equiv audio_on, LATF0


;/*motors*/
.equiv MOTOR_PORT, LATD
.equiv MOTOR1_PHA, LATD0
.equiv MOTOR1_PHB, LATD1
.equiv MOTOR1_PHC, LATD2
.equiv MOTOR1_PHD, LATD3
.equiv MOTOR2_PHA, LATD4
.equiv MOTOR2_PHB, LATD5
.equiv MOTOR2_PHC, LATD6
.equiv MOTOR2_PHD, LATD7



;/*camera*/
.equiv CAM_RESET, LATC13

;/* I2C */
.equiv SIO_D,	LATG3
.equiv SIO_C,	LATG2


;/********************** INPUTS **************************/
.equiv INPUT_PIN, 1

;/*low battery signal active low when Vbatt<3.4V*/
.equiv BATT_LOW, RF1


;/* selector on normal extension*/
.equiv SELECTOR_PORT, PORTG
.equiv SELECTOR0, RG6
.equiv SELECTOR1, RG7
.equiv SELECTOR2, RG8
.equiv SELECTOR3, RG9


;/*IR TV receiver on normal extension*/
.equiv REMOTE, RF6


;/*CAMERA*/
;/*data higher 8 bits of port D*/
.equiv CAM_DATA, PORTD;

.equiv CAM_Y0, RD8
.equiv CAM_Y1, RD9
.equiv CAM_Y2, RD10
.equiv CAM_Y3, RD11
.equiv CAM_Y4, RD12
.equiv CAM_Y5, RD13
.equiv CAM_Y6, RD14
.equiv CAM_Y7, RD15


;/*clock interupt*/
.equiv CAM_PWDN, RC2
.equiv CAM_VSYNC, RC4
.equiv CAM_HREF, RC3
.equiv CAM_PCLK, RC14


; .ENDIF
