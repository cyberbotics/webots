/***************************************************************************************************************

Title:		UART1_RX_CHAR.s

Author:		Inspired Microchip library
			Bonani Michael
			Stephane Magnenat

History:
	21/12/05	Start day
	12/03/07	added rx buffer

****************************************************************************************************************/
;to be used with uart_txrx_char.h

.include "e_epuck_ports.inc"


.equiv	BAUDRATE, 115000 

.extern _U1RXRcvCnt
.extern _U1RXReadCnt

.section .text

.global _e_init_uart1
_e_init_uart1:
		; Init uart 1 at 11500bps, 8 data bits, 1 stop bit, Enable ISR for RX and TX
		; Initialize and Enable UART1 for Tx and Rx
		clr     U1MODE                              ; Set SFRs to a known state
		clr     U1STA
		bclr    U1STA, #URXISEL1
		bclr    U1STA, #URXISEL0
	
		bclr    IFS0, #U1RXIF
		bset    IEC0, #U1RXIE                       ; Enable Rx ISR processing
		bset    U1MODE, #UARTEN                     ; Enable UART
		mov     #(((FCY/BAUDRATE)/16)-1), W0        ; Initialize Baud rate
		mov     w0, U1BRG                           ; to 115.2 Kbaud
		
		bclr    IFS0, #U1TXIF                       ; Enable Txmit ISR processing
		bset    IEC0, #U1TXIE

		; set a higher priority on UART interrupt ( so an ISR can use uart )
		; uart is slow, don't abuse it in interrupt
		bclr.b INTCON1+1,#7
		mov IPC2, w0
		mov #0xF00F, w1
		and w1,w0,w0
		mov #0x0550, w1
		add w1,w0,w0
		mov w0,IPC2
		
		bset    U1STA, #UTXISEL
		bset    U1STA, #UTXEN                       ; Enable Transmission
		
		; Reception counters to 0
		clr _U1RXRcvCnt
		clr _U1RXReadCnt
		return


.end


