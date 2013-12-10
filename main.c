/*
 * File:   main.c
 * Author: tylerjw
 *
 * Created on December 7, 2013, 2:55 PM
 */

/******************************************************************************
 * Software License Agreement
 *
 * Copyright � 2011 Microchip Technology Inc.  All rights reserved.
 * Microchip licenses to you the right to use, modify, copy and distribute
 * Software only when embedded on a Microchip microcontroller or digital
 * signal controller, which is integrated into your product or third party
 * product (pursuant to the sublicense terms in the accompanying license
 * agreement).
 *
 * You should refer to the license agreement accompanying this Software
 * for additional information regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED ìAS ISî WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY
 * OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
 * PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,
 * BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT
 * DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
 * INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
 * COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY
 * CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
 * OR OTHER SIMILAR COSTS.
 *
 *****************************************************************************/

// port_B bit 2 - pin 23 --> 1k Ohm R -> - LED + -> +3.2V rail

#include <p32xxxx.h>
#include <plib.h>
#include <stdbool.h>

#define SYS_CLK     80000000L

#define CE  BIT_3
#define OE  BIT_2
#define WE  BIT_1
#define A16 BIT_4

int main(void);
void delay(volatile unsigned int count);
void mem_init();
bool mem_test();
bool mem_word_test();
void mem_write_word(unsigned int, unsigned int);
unsigned int mem_read_word(unsigned int);
void output_clock_sig();

int main(void) {
    SYSTEMConfigPerformance(SYS_CLK);

    PORTSetPinsDigitalOut(IOPORT_E, BIT_4); // led
    mem_init();
    if(mem_word_test())
        mPORTEWrite(BIT_4);
    else
        mPORTEWrite(0);

    while (1);
}

void output_clock_sig()
{
    TRISG &= ~(BIT_6);
    REFOCON = (BIT_17 | BIT_15 | BIT_12);
    PPSUnLock;
    PPSOutput(3,RPG6,REFCLKO);
    PPSLock;
}

void delay(volatile unsigned int count)
{
    while(--count);
}

void mem_init()
{
    PORTSetPinsDigitalOut(IOPORT_B, 0xFFFF); // address buss
    PORTSetPinsDigitalOut(IOPORT_C, 0b00011110); // controll pins & A16
    mPORTCWrite(0);
}

bool mem_test()
{
    unsigned int test_value;

    // write to memory
    PORTSetPinsDigitalOut(IOPORT_D, 0xFFFF); // mem i/o pins
    mPORTCSetBits(OE | WE); // OE and WE high
    mPORTBWrite(0x0000); // set address to memory location zero
    mPORTDWrite(0xAAAA); // data to send
    mPORTCClearBits(WE); // clear WE
    mPORTCSetBits(WE); // pull WE back up

    // read from memory
    PORTSetPinsDigitalIn(IOPORT_D, 0xFFFF); // mem i/o pins
    mPORTCClearBits(OE);
    test_value = mPORTDRead();

    if(test_value == 0xAAAA)
        return true;
    else
        return false;
}

bool mem_word_test()
{
    unsigned int value = 0xAAAA;
    unsigned int addr = 0x1234;
    unsigned int test_var;

    mem_write_word(addr, value);
    test_var = mem_read_word(addr);

    if(test_var == value)
        return true;
    else
        return false;
}

void mem_write_word(unsigned int addr, unsigned int output)
{
    // write to memory
    PORTSetPinsDigitalOut(IOPORT_D, 0xFFFF); // mem i/o pins
    mPORTCSetBits(OE | WE); // OE and WE high
    mPORTBWrite(addr); // set address to memory
    mPORTDWrite(output); // data to send
    mPORTCClearBits(WE); // clear WE
    mPORTCSetBits(WE); // pull WE back up
}

unsigned int mem_read_word(unsigned int addr)
{
    // read from memory
    PORTSetPinsDigitalIn(IOPORT_D, 0xFFFF); // mem i/o pins
    mPORTBWrite(addr); // set address to memory
    mPORTCWrite(WE); // sets WE and clears OE
    return mPORTDRead(); // read the data and return it
}
