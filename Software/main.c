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


#include <htc.h>
#include <pic.h>
#define _XTAL_FREQ 32768

__CONFIG(LP & WDTDIS & PWRTEN & MCLRDIS & UNPROTECT & UNPROTECT & BORDIS & IESODIS & FCMDIS);

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
    OFF1 = 5
} TDisplayState;

typedef enum
{
    RUNNING = 0,
    SETHOUR = 1,
    SETMIN = 2
} TRunState;


/*
 * EEPROM
 */
__eeprom unsigned char gMinutesStore;
__eeprom unsigned char gHoursStore = 12;
__eeprom unsigned char gTwelveHrMode = 1;

/*
 * Globals
 */
volatile unsigned char tick = 0;
volatile unsigned char gSeconds = 0;
unsigned char gMinutes = 0;
unsigned char gHours = 12;
volatile unsigned char gButtonTime = 0;
volatile unsigned char gButtonEvent = 0;
TDisplayState gDispState = HOURONE;
TRunState gClockState = RUNNING;


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
                    if (++gDispState > 2)
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
                    if (++gDispState > 4)
                    {
                        gDispState = 2;
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
                    gMinutesStore = gMinutes;
                    gHoursStore = gHours;
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

/*
 * main
 */
void main(void)
{
    init();

    // set time to last known values
    gHours = gHoursStore;
    gMinutes = gMinutesStore;

    while (1)
    {
        if (tick)
        {
            // state machine updates on the 1 Hz tick
            if (++gDispState > 5)
                gDispState = 0;
            PORTB = 0x00;
            PORTC = 0x00;
            __delay_ms(100);
            tick = 0;
        }

        if (gSeconds > 59)
        {
            gMinutes++;
            gSeconds = 0;

            // save time to EEPROM
            gMinutesStore = gMinutes;
            gHoursStore = gHours;
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
    }
}

/*
 * Interrupt service routine, called whenever any interrupt occurs
 */
interrupt isr(void)
{
    // Timer 0 interrupt every 1/128s
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
                if (gButtonTime > 70)
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
        }
        tick = 1;
        TMR2IF = 0;
    }
}