/**********************************************************************
* © 2005 Microchip Technology Inc.
*
* FileName:        DataEEPROM.h
* Dependencies:    Header (.h) files if applicable, see below
* Processor:       dsPIC30Fxxxx
* Compiler:        MPLAB® C30 v1.32.00 or higher
* IDE:             MPLAB® IDE v7.20.01 or later
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
* Author                 Date      Comments on this revision
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Erminio Bonizonni/HV   11/02/05  First release of source file
*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*
* ADDITIONAL NOTES:
*
**********************************************************************/

#define WORD    1
#define ROW     16
#define ALL_EEPROM      0xFFFF
#define ERROREE -1

/*
 * ReadEErow prototype:
 * Parameters Definition:
 * Page:        is the 8 most significant bits of the source address in EEPROM
 * Offset:      is 16 least significant bits of the source address in EEPROM
 * DataOut:     is the 16-bit address of the destination RAM location or array
 * Size:        is the number of words to read from EEPROM and is a value of 1 or 16
 * Return Value:
 * Function returns ERROREE (or -1) if Size is invalid
 */
extern int ReadEE(int Page, int Offset, int* DataOut, int Size);

/*
 * EraseEErow prototype:
 * Parameters Definition:
 * Page:        is the 8 most significant bits of the address in EEPROM to be erased
 * Offset:      is 16 least significant bits of the address in EEPROM to be erased
 * Size:        is the number of words to read from EEPROM and is a value of 1, 16 or
 *              0xFFFF (for erasing ALL EEPROM memory)
 * Return Value:
 * Function returns ERROREE (or -1) if Size is invalid
 */
extern int EraseEE(int Page, int Offset, int Size);

/*
 * WriteEErow prototype:
 * Parameters Definition:
 * Page:        is the 8 most significant bits of the destination address in EEPROM
 * Offset:      is 16 least significant bits of the destination address in EEPROM
 * DataIn:      is the 16-bit address of the source RAM location or array
 * Size:        is the number of words to read from EEPROM and is a value of 1 or 16
 * Return Value:
 * Function returns ERROREE (or -1) if Size is invalid
 */
extern int WriteEE(int* DataIn, int Page, int Offset, int Size);
