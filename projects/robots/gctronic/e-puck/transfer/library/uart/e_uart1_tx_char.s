
/***************************************************************************************************************

Title:		uart1_tx_char.s

Author:		Inspired Microchip library
			Davis Daidi�
			Michael Bonani
History:
	11/07/05	Start day
	16/09/05	Adaptation uart2->uart1
	22/12/05	Optimisation and chand name of function

****************************************************************************************************************/
;to be used with uart_txrx_char.h 

.include "p30F6014A.inc"


.global U1TXPtr
.global U1TXLength
.global U1TXOk

.section  .data, near

U1TXPtr:  .hword 0x0000
U1TXLength: .hword 0x0000
U1TXOk:		.hword 0x0000

.global         __U1TXInterrupt

.section        .text
__U1TXInterrupt:
		bclr    IFS0, #U1TXIF           ;Clear the interrupt flag
        push.d  w0                      ;save context - w0,w1, w2
        push    w2
        

		cp0		U1TXOk
		bra		z, end_transmission

        cp0     U1TXPtr                 ;Check if the pointer is valid (not zero)
        bra     z, exit2_U1TXInt        ;Else exit

		cp0		U1TXLength
		bra		z,exit2_U1TXInt

        mov     U1TXPtr, w1
        mov     #4, w2                  ;Initialize w2=4 as a decrementing counter
		
		; make sure the buffer is empty

load_next_char:

		btsc	U1STA,#9				; make sure the buffer is not full
		bra 	load_next_char

        mov		U1TXLength, w0			;load lenght of buffer
		cp0     w0                    	;unless end of buffer is encountered
        bra     z, loaded_all_char      
		dec		w0,w0
		mov 	WREG,U1TXLength			;rest of lenght

		mov.b   [w1++], w0              ;Load 4 characters into UART TX queue
        mov.b   WREG, U1TXREG
        dec     w2, w2
        bra     nz, load_next_char
        bra     exit1_U1TXInt           ;Exit ISR after loading 4 chars

loaded_all_char:
        clr     U1TXPtr                 ;Clear pointer if all char send
        bra     end_transmission

exit1_U1TXInt:
        mov     w1, U1TXPtr             ;Save new value of pointer
end_transmission:
 		pop     w2                      ;Restore context - w0, w1, w2
        pop.d   w0
        retfie                          ;Return from Interrupt

exit2_U1TXInt:
        mov		#0,w0
		mov		w0,U1TXOk
		pop     w2                      ;Restore context - w0, w1, w2
        pop.d   w0
        retfie                          ;Return from Interrupt


.global _e_send_uart1_char

; in: w0 pointer char hon buffer
; in: w1 lengh of buffer

_e_send_uart1_char:
wait_l:	cp0		U1TXOk			; check if pointer is under trsnfer
		bra     nz, wait_l
		bclr    IEC0, #U1TXIE  ;disable interupt
		dec		w1,w1
		mov		w1, U1TXLength   ; move lengh of buffer in U2TXLength	
		mov.b	[w0++],w1
		mov 	w1,U1TXREG
		mov     w0, U1TXPtr
		mov		#1,w1
		mov		w1,U1TXOk
		bset    IEC0, #U1TXIE	;enable interupt	
        return

.global _e_uart1_sending

; in: w0 pointer char hon buffer
; in: w1 lengh of buffer

_e_uart1_sending:
		mov U1TXOk,w0
        return


.end                                    ;EOF



        
