/**********************************************************************
* © 2005 Microchip Technology Inc.
*
* FileName:        DataEEPROM.s
* Dependencies:    Header (*.inc/.h) files if applicable, see below
* Processor:       dsPIC30Fxxxx
* Compiler:        MPLAB® C30 v1.32.00 or higher
* IDE:             MPLAB® IDE v7.21 or later
* Dev. Board Used: dsPICDEM 1.1 Development Board
* Hardware Dependencies: None
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Inc. (Microchip) licenses this software to you
* solely for use with Microchip dsPIC® digital signal controller
* products. The software is owned by Microchip and is protected under
* applicable copyright laws.  All rights reserved.
*
* SOFTWARE IS PROVIDED AS IS.  MICROCHIP EXPRESSLY DISCLAIMS ANY
* WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
* PARTICULAR PURPOSE, OR NON-INFRINGEMENT. IN NO EVENT SHALL MICROCHIP
* BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL
* DAMAGES, LOST PROFITS OR LOST DATA, HARM TO YOUR EQUIPMENT, COST OF
* PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
* BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
* ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER SIMILAR COSTS.
*
* REVISION HISTORY:
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Author         Date      Comments on this revision
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EB/HV          11/02/05  First release of source file
*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*
* ADDITIONAL NOTES:
**********************************************************************/

.include "p30Fxxxx.inc"

.equ    EE_WORD_ERASE_CODE, 0x4044
.equ    EE_WORD_WRITE_CODE, 0x4004
.equ    EE_ROW_ERASE_CODE, 0x4045
.equ    EE_ROW_WRITE_CODE, 0x4005
.equ    EE_ALL_ERASE_CODE, 0x4046
.equ    CONFIG_WORD_WRITE_CODE, 0x4006

.global _ReadEE
.global _EraseEE
.global _WriteEE

.section .text
/* DATA EEPROM Read Routines */
_ReadEE:
        push    TBLPAG
        mov     w0, TBLPAG
        cp      w3, #1
        bra     z, L0
        cp      w3, #16
        bra     z, L0
        mov     #-1, w0
        bra     L1
L0:     tblrdl  [w1++],[w2++]
        dec     w3, w3
        bra     nz, L0
L1:     pop     TBLPAG
        return

/* DATA EEPROM Erase Routines */
_EraseEE:
        push.d  w4
        bclr    SR, #Z
        mov     #EE_WORD_ERASE_CODE, W4
        cp      w2, #1
        bra     z, L2
        mov     #EE_ROW_ERASE_CODE, W4
        cp      w2, #16
        bra     z, L2
        mov     #EE_ALL_ERASE_CODE, W4
        mov     #0xFFFF, w5
        cp      w2, w5
        bra     z, L2
        mov     #-1, w0
        pop.d   w4
        return
L2:
        push    TBLPAG
        mov     W0, NVMADRU
        mov     W1, NVMADR
        mov     W4, NVMCON
        push    SR
        mov     #0xE0, W0
        ior     SR
        mov     #0x55, W0
        mov     W0, NVMKEY
        mov     #0xAA, W0
        mov     W0, NVMKEY
        bset    NVMCON, #WR
        nop
        nop
L3:     btsc    NVMCON, #WR
        bra     L3
        clr     w0
        pop     SR
L4:     pop     TBLPAG
        pop.d   w4
        return


/* DATA EEPROM Write Routines */
_WriteEE:
        push    w4
        bclr    SR, #Z
        mov     #EE_WORD_WRITE_CODE, W4
        cp      w3, #1
        bra     z, L5
        mov     #EE_ROW_WRITE_CODE, W4
        cp      w3, #16
        bra     z, L5
        pop     w4
        mov     #-1, w0
        return

L5:     push    TBLPAG
        mov     W1, TBLPAG
        push    W2
L6:     tblwtl  [W0++],[W2++]
        dec     w3, w3
        bra     nz, L6

        mov     W1, NVMADRU
        pop     W2
        mov     W2, NVMADR
        mov     W4, NVMCON
        push    SR
        mov     #0xE0, W0
        ior     SR
        mov     #0x55, W0
        mov     W0, NVMKEY
        mov     #0xAA, W0
        mov     W0, NVMKEY
        bset    NVMCON, #WR
        nop
        nop
L7:     btsc    NVMCON, #WR
        bra     L7
        clr     w0
        pop     SR
        pop     TBLPAG
        pop     w4
        return


.end
