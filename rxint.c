//************************************************************************
//
// MODULES USED
//
//************************************************************************
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "decoder.h"
#include "rxint.h"
#include "fifo.h"

//************************************************************************
//
// GLOBAL VARIABLES
//
//************************************************************************
uint8_t rxstate;
uint16_t bitcount;
//************************************************************************
//
// EXPORTED FUNCTIONS
//
//************************************************************************

//
// The IntGPIOE function is executed on every rising edge of the
// Received clock

//SIGNAL(SIG_INTERRUPT0)
ISR(INT0_vect)
{
    //static local variables retain their value when the function is exited
    uint8_t dat;
    static uint8_t lastdat;     //removing storage class specifier: static should break functionality
    static uint8_t dbyte;
    static uint16_t shiftreg;
    static uint16_t newbit;
    static uint16_t crcreg;

    // Read the state of the data line
    //dat = GPIOPinRead(GPIO_PORTE_BASE, RxData); 
    dat = PINB & (1<<RxData);

    // NRZI Decode latest bit and shift into register
    shiftreg <<= 1;
    if (dat == lastdat)
        // Data the same = 1
        shiftreg |= 0x0001;
    else
        // Data changed = 0;
        shiftreg &= 0xfffe;

    // Save the last state of the data line
    lastdat = dat;

    // Execute state-machine
    switch (rxstate)
    {
        // Set RxDCaqc and PLLacq after an initial delay
        // Go to the RX_PREAMBLE state
        case RX_INIT:
            if (!(--bitcount))
            {
                //GPIOPinWrite(GPIO_PORTB_BASE, RxDCacq | PLLacq, RxDCacq | PLLacq);
                PORTC |= (1<<RxDCacq)|(1<<PLLacq);
                rxstate = RX_PREAMBLE;
                shiftreg = 0l;
            }
            break;

        // Look for 16 bits of preamble
        case RX_PREAMBLE:
            if (shiftreg == 0x5555)
            {
                bitcount = 0x00;
                rxstate = RX_START;
                // Set PLLacq low
                //GPIOPinWrite(GPIO_PORTB_BASE, PLLacq, 0);
                PORTC &= ~(1<<PLLacq);
            }
            break;

        // Look for start flag
        case RX_START:
            if ((shiftreg & 0x00ff) == 0x007e)
            {
                bitcount = 0;
                rxstate = RX_PRELOAD;
                //GPIOPinWrite(GPIO_PORTC_BASE, USER_LED, USER_LED);
                PORTC |= (1<<USER_LED);             
            }
            else if (bitcount >= 24)
            {
                // Start flag not found - go back and look for
                // preamble
                rxstate = RX_PREAMBLE;
                //GPIOPinWrite(GPIO_PORTB_BASE, PLLacq, PLLacq);
                PORTC |= (1<<PLLacq);
            }
            break;
    
        // Load an initial 8 bits into the shift register.
        // This means that when we see the end flag we have just
        // passed the last CRC bit through the CRC calculation
        case RX_PRELOAD:
            if (++bitcount == 8)
            {
                // Go to the receiving state
                bitcount = 0;
                crcreg = 0xffff;
                rxstate = RX_DATA;
                FIFO_Reset_Packet();
            }
            break;

        // Receive data
        case RX_DATA:

            // Remove stuffing bits
            // An extra zero is inserted after five 1's by the transmitter
            if ((shiftreg & 0x3f00) != 0x3e00)
            {
                // It's not a stuffing bit

                // Increment the bit count
                ++bitcount;
                // Extract the new data bit
                newbit = (shiftreg >> 8) & 0x0001;
                // Shift new bit into a byte
                dbyte = (dbyte >> 1) | ((shiftreg >> 1) & 0x0080);

                // If 8 bits received but into FIFO
                if (!(bitcount & 0x07))
                    FIFO_Put(dbyte);
                    

                // Pass new bit through CRC calculation
                if ((crcreg ^ newbit) & 0x0001)
                    // Xor with the CRC polynomial (X^16 + X^12 + X^5 + 1)
                    crcreg = (crcreg >> 1) ^ 0x8408;
                else
                {
                    // There appears to be a bug in the Keil compiler - right shift
                    // doesn't work unless something else is done on the same line.
                    // Here I xor shifted result with 1 and then xor again to correct
                    // this back
                    crcreg = (crcreg >> 1);// ^ 0x0001;
                    //crcreg ^= 0x0001;
                }

            }

            // Have we got an end flag ?
            if ((shiftreg & 0x00ff) == 0x007e)
            {
                // Reset state machine to look for preamble
                rxstate = RX_PREAMBLE;
                // Set PLLacq back to 1
                PORTC |= (1<<PLLacq);
                // Turn off LED
                PORTC &= ~(1<<USER_LED);
                // Check for good CRC
                // This should give a result of 0xF0B8
                if (crcreg == 0xf0b8)
                {
                    // Good packet received
                    // ... Release into FIFO
                    FIFO_Write_Packet();
                    //printf("%04X\n",crcreg);
                    //printf("\n---%d\n",bitcount);
                    PORTC ^= _BV(CRC_LED);  
                }
            }
            break;

        default:
            break;

    }
    // Clear the GPIO interrupt.
    GIFR |= (1<<INTF0);         // cleared automatically, alternatively can clear by writing logical 1 to it.
}// IntGPIOE


//************************************************************************
//
// EOF
//
//************************************************************************

