/***************************************************************************
 *                                                                         *
 *  This program is free software; you can redistribute it and/or modify   *
 *  it under the terms of the GNU General Public License as published by   *
 *  the Free Software Foundation; either version 2 of the License, or      *
 *  (at your option) any later version.                                    *
 *                                                                         *
 *  This program is distributed in the hope that it will be useful,        *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *  GNU General Public License for more details.                           *
 *                                                                         *
 *  You should have received a copy of the GNU General Public License      *
 *  along with this program; if not, write to the Free Software            *
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111 USA    *
 *                                                                         *
 ***************************************************************************
 *                                                                         *
 *               (c) Copyright, 2017 Elijah Brown                          *
 *                                                                         *
 ***************************************************************************
 *                                                                         *
 * Filename:    main.c                                                     *
 *                                                                         *
 ***************************************************************************/

#define _XTAL_FREQ 32768

#pragma config FOSC = LP   // Low-power crystal
#pragma config WDTE = OFF  // Watchdog Timer Disabled
#pragma config PWRTE = ON  // Power-up Timer Enabled
#pragma config MCLRE = OFF // MCLR pin function disabled
#pragma config CP = OFF    // Program memory protection off
#pragma config CPD = OFF   // Data memory protection off
#pragma config BOREN = OFF // Brown-out Reset disabled
#pragma config IESO = OFF  // Int/Ext Switchover off
#pragma config FCMEN = OFF // Fail-Safe Clock Monitor off

#include <xc.h>
#include <stdint.h>

/*
 * Types
 */
typedef enum
{
    HOURONE = 0,
    HOURTWO = 1,
    MINONE = 2,
    MINTWO = 3,
    OFF0 = 4,
    OFF1 = 5,
    CALSIGN = 6,
    CALONE = 7,
    CALTWO = 8,
    CALTHREE = 9
} TDisplayState;

typedef enum
{
    RUNNING = 0,
    SETHOUR = 1,
    SETMIN = 2,
    SETCAL = 3
} TRunState;

// Function Prototypes
void init(void);
void SetCal(void);
void SetClock(void);
uint8_t eepromRead(uint8_t addr);
void eepromWrite(uint8_t addr, uint8_t data);

/*
 * Globals
 */
volatile uint8_t tick = 0;
volatile uint8_t gSeconds = 0;
volatile uint8_t gButtonTime = 0;
volatile uint8_t gButtonEvent = 0;
volatile int8_t xtalCalPpm = 0;
volatile int32_t xtalCalAccum = 0;
uint8_t gTwelveHrMode = 1;
uint8_t gMinutes = 0;
uint8_t gHours = 12;
TDisplayState gDispState = HOURONE;
TRunState gClockState = RUNNING;

// This line defines the EEPROM values to set at programming.
// eeprom layout:
// Addr (byte)   Purpose
// 0             hours
// 1             mins
// 2             crystal offset in ppm
__EEPROM_DATA(12, 0, 0, 0, 0, 0, 0, 0);

/*
 * Initialize PIC peripherals
 */
void init(void)
{
    // port directions: 1=input, 0=output
    TRISA = 0b00000100;
    TRISB = 0b00100000;
    TRISC = 0b00000000;

    // set all outputs low
    PORTB = 0x00;
    PORTC = 0x00;

    // Disable ADC inputs (all pins are digital)
    ANSEL = 0b00000000;
    ANSELH = 0b00000000;
    // Clear the timers
    TMR1H = 0xFF;
    TMR1L = 0x58;
    TMR0 = 0x00;

    // Timer 0 setup
    T0CS = 0; // internal clock source

    // Timer 1 setup
    T1CON = 0b00110001;

    // Timer 2 setup
    T2CON = 0b01111101;
    PR2 = 0x7F;

    // GIE, TMR0, PEIE Interrupts
    INTCON = 0b11100000;
    // TMR1 and TMR2 Interrupt enable
    PIE1 = 0b00000011;

    // Enable weak pull-up for RA2
    WPUA = 0b00000100;

    // Disable comparators
    CM1CON0 = 0x00;
    CM2CON0 = 0x00;
    T0CS = 0x0;
}

/*
 * Sets PORTB and PORTC to display the specified number on the IN-18 Nixie
 * Assumes open collector drivers (active high)
 */
void DisplayNum(unsigned char num)
{
    switch (num)
    {
    case 0:
        PORTB = 0x80;
        PORTC = 0x00;
        break;
    case 1:
        PORTB = 0x00;
        PORTC = 0x01;
        break;
    case 2:
        PORTB = 0x00;
        PORTC = 0x02;
        break;
    case 3:
        PORTB = 0x00;
        PORTC = 0x04;
        break;
    case 4:
        PORTB = 0x00;
        PORTC = 0x08;
        break;
    case 5:
        PORTB = 0x00;
        PORTC = 0x10;
        break;
    case 6:
        PORTB = 0x40;
        PORTC = 0x00;
        break;
    case 7:
        PORTB = 0x10;
        PORTC = 0x00;
        break;
    case 8:
        PORTB = 0x00;
        PORTC = 0x40;
        break;
    case 9:
        PORTB = 0x00;
        PORTC = 0x80;
        break;
    }
}

/*
 * Called when the SET button is held down
 */
void SetClock()
{
    while (gClockState != RUNNING)
    {
        switch (gClockState)
        {
        case SETHOUR:
            if (tick)
            {
                if (++gDispState > MINONE)
                    gDispState = 0;
                PORTB = 0x00;
                PORTC = 0x00;
                __delay_ms(100);
                tick = 0;
            }

            if (gButtonEvent == 2)
            {
                gButtonEvent = 0;
                gClockState = SETMIN;
                gDispState = MINONE;
                break;
            }
            if (gButtonEvent == 1)
            {
                gButtonEvent = 0;
                if (gTwelveHrMode)
                {
                    if (++gHours > 12)
                    {
                        gHours = 1;
                    }
                }
                else
                {
                    if (++gHours > 23)
                    {
                        gHours = 0;
                    }
                }
            }
            switch (gDispState)
            {
            case HOURONE:
                DisplayNum(gHours / 10);
                break;
            case HOURTWO:
                DisplayNum(gHours % 10);
                break;
            case MINONE:
                PORTB = 0x00;
                PORTC = 0x00;
                break;
            }
            break;

        case SETMIN:
            if (tick)
            {
                if (++gDispState > OFF0)
                {
                    gDispState = MINONE;
                }
                PORTB = 0x00;
                PORTC = 0x00;
                __delay_ms(100);
                tick = 0;
            }

            if (gButtonEvent == 2)
            {
                gButtonEvent = 0;
                gClockState = RUNNING;
                gSeconds = 0;

                // save new time
                eepromWrite(0, gHours);
                eepromWrite(1, gMinutes);
            }
            if (gButtonEvent == 1)
            {
                gButtonEvent = 0;
                if (++gMinutes > 59)
                {
                    gMinutes = 0;
                }
            }
            switch (gDispState)
            {
            case MINONE:
                DisplayNum(gMinutes / 10);
                break;
            case MINTWO:
                DisplayNum(gMinutes % 10);
                break;
            case OFF0:
                PORTB = 0x00;
                PORTC = 0x00;
                break;
            }
            break;
        }
    }
}

void SetCal()
{
    while (gClockState != RUNNING)
    {
        switch (gClockState)
        {
        case SETCAL:
            if (tick)
            {
                if (++gDispState > CALTHREE)
                    gDispState = CALSIGN;
                PORTB = 0x00;
                PORTC = 0x00;
                __delay_ms(100);
                tick = 0;
            }

            if (gButtonEvent == 2)
            {
                eepromWrite(2, (uint8_t)xtalCalPpm);
                gButtonEvent = 0;
                gClockState = RUNNING;
                break;
            }
            if (gButtonEvent == 1)
            {
                gButtonEvent = 0;
                xtalCalPpm++;
            }
            uint8_t neg = 0;
            int16_t ppm = xtalCalPpm;
            if (ppm < 0)
            {
                neg = 1;
                ppm = -ppm;
            }
            switch (gDispState)
            {
            case CALSIGN:
                if (neg)
                {
                    DisplayNum(9);
                }
                else
                {
                    // skip this for positive case
                    gDispState = CALONE;
                }
                break;
            case CALONE:
                // drop the leading zero
                if (!(ppm / 100))
                {
                    gDispState = CALTWO;
                    break;
                }
                DisplayNum(ppm / 100);
                break;
            case CALTWO:
                DisplayNum((ppm / 10) % 10);
                break;
            case CALTHREE:
                DisplayNum(ppm % 10);
                break;
            }
            break;
        }
    }
}

/*
 * main
 */
void main(void)
{
    init();

    // set time to last known values
    gHours = eepromRead(0);
    gMinutes = eepromRead(1);
    xtalCalPpm = (int8_t)eepromRead(2);

    while (1)
    {
        if (tick)
        {
            // state machine updates on the 1 Hz tick
            if (++gDispState > OFF1)
                gDispState = 0;
            PORTB = 0x00;
            PORTC = 0x00;
            __delay_ms(100);

            if (gSeconds > 0 && gSeconds < 59)
            {
                INTCONbits.GIE = 0; // Disable interrupts
                // correct for drift
                if (xtalCalAccum >= 1000000L)
                {
                    // running fast, drop a second
                    gSeconds--;
                    xtalCalAccum -= 1000000L;
                }
                else if (xtalCalAccum <= -1000000L)
                {
                    // running slow, add a second
                    gSeconds++;
                    xtalCalAccum += 1000000L;
                }
                INTCONbits.GIE = 1; // re-enable interrupts
            }
            tick = 0;
        }

        if (gSeconds > 59)
        {
            gMinutes++;
            gSeconds = 0;

            if ((gMinutes % 5) == 0)
            {
                // save time to eeprom. Only every 5 to avoid wearing it out...
                eepromWrite(0, gHours);
                eepromWrite(1, gMinutes);
            }
        }
        if (gMinutes > 59)
        {
            gHours++;
            gMinutes = 0;
        }
        if (gTwelveHrMode)
        {
            if (gHours > 12)
            {
                gHours = 1;
            }
        }
        else
        {
            if (gHours > 23)
            {
                gHours = 0;
            }
        }

        switch (gDispState)
        {
        case HOURONE:
            // drop the leading zero
            if (!(gHours / 10))
            {
                gDispState = HOURTWO;
                break;
            }
            DisplayNum(gHours / 10);
            break;
        case HOURTWO:
            DisplayNum(gHours % 10);
            break;
        case MINONE:
            DisplayNum(gMinutes / 10);
            break;
        case MINTWO:
            DisplayNum(gMinutes % 10);
            break;
        case OFF0:
            PORTB = 0x00;
            PORTC = 0x00;
            break;
        case OFF1:
            PORTB = 0x00;
            PORTC = 0x00;
            break;
        }

        // Holding the button down puts the clock into set mode
        if (gButtonEvent == 2)
        {
            gButtonEvent = 0;
            gClockState = SETHOUR;
            gDispState = HOURONE;
            SetClock();
        }

        // Holding the button down >7s puts the clock into cal mode
        if (gButtonEvent == 3)
        {
            gButtonEvent = 0;
            gClockState = SETCAL;
            gDispState = CALSIGN;
            SetCal();
        }
    }
}

/*
 * Interrupt service routine, called whenever any interrupt occurs
 */
void __interrupt() isr(void)
{
    // Timer 0 interrupt at 32 hz
    if (INTCON & 0b00000100)
    {
        // Clear interrupt flag
        T0IF = 0;
        if (!RA2)
        {
            // button is pressed
            if (gButtonTime < 255)
            {
                gButtonTime++;
            }
            else
            {
                PORTB = 0x00;
                PORTC = 0x00;
            }
        }
        else
        {
            if (gButtonTime)
            {
                if (gButtonTime >= 234)
                {
                    gButtonEvent = 3;
                }
                else if (gButtonTime > 32 && gButtonTime < 234)
                {
                    gButtonEvent = 2;
                }
                else
                {
                    gButtonEvent = 1;
                }
            }
            gButtonTime = 0;
        }
    }

    // Timer 1 interrupt every 0.2s
    if (PIR1 & 0b00000001)
    {
        // Clear interrupt flag
        PIR1 &= 0b11111110;
        TMR1H = 0xFF;
        TMR1L = 0x58;
    }

    // Timer 2 interrupt at exactly 1Hz
    if (PIR1 & 0x02)
    {
        if (gClockState == RUNNING)
        {
            gSeconds++;
            xtalCalAccum += xtalCalPpm;
        }
        tick = 1;
        TMR2IF = 0;
    }
}

/*
 * Read a byte from EEPROM.
 */
uint8_t eepromRead(uint8_t addr)
{
    EEADR = addr;
    EECON1bits.EEPGD = 0; // EEPROM, not Flash
    EECON1bits.RD = 1;
    return EEDAT;
}

/*
 * Write a byte to EEPROM
 */
void eepromWrite(uint8_t addr, uint8_t data)
{
    EEADR = addr;
    EEDAT = data;
    EECON1bits.EEPGD = 0;
    EECON1bits.WREN = 1;

    INTCONbits.GIE = 0; // Disable interrupts
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;
    INTCONbits.GIE = 1; // Re-enable interrupts

    while (EECON1bits.WR)
        ;
    EECON1bits.WREN = 0;
}
