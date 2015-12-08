/****************************************************
TARGET:         ATMEGA8
CLOCK:          8 MHz
Peripherals:    CMX589 GMSK demodulator
Author:         Alex Skafidas
Date:           16 Oct 2009-20.32

Notes:
for delay.h you need defined F_CPU before calling header file OR 
set optimization switch -O1 (or more) and -DF_CPU=mhz switch

            (RESET)         PC6 1  28 PC5 (ADC5/SCL)    ------>CRC_LED
            (RXD)           PD0 2  27 PC4 (ADC4/SDA)    ------>USER_LED
            (TXD)           PD1 3  26 PC3 (ADC3)        ------>
RxCLK ----> (INT0)          PD2 4  25 PC2 (ADC2)        ------>PLLacq
            (INT1)          PD3 5  24 PC1 (ADC1)        ------>RxHOLDN
            (XCK/T0)        PD4 6  23 PC0 (ADC0)        ------>RxDCacq
            VCC                 7  22 GND
            GND                 8  21 AREF
            (XTAL1/TOSC1)   PB6 9  20 AVCC
            (XTAL2/TOSC2)   PB7 10 19 PB5 (SCK)
            (T1)            PD5 11 18 PB4 (MISO)
            (AIN0)          PD6 12 17 PB3 (MOSI/OC2)
            (AIN1)          PD7 13 16 PB2 (SS/OC1B)
RxData ---> (ICP1)          PB0 14 15 PB1 (OC1A)
*****************************************************/

#ifndef     F_CPU
#define     F_CPU           8000000UL
#endif

// Define baud rate
#define USART_BAUD 38400
#define USART_UBBR_VALUE ((F_CPU/(USART_BAUD<<4))-1)
//#define BIT(x) (1<<x)
//#include <inttypes.h>
#include <stdio.h>
#include <avr/io.h>
#include <string.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "decoder.h"
#include "rxint.h"
#include "fifo.h"
#include "nmea.h"

//************************************************************************
// LOCAL VARIABLES
//************************************************************************

static uint8_t rx_pkt[128];
static uint8_t pkt_size;

//*****************************************************************************
// Main AIS Decoder Program
//*****************************************************************************
void Port_Init(void);
void USART_Init(void);

static int put_char(char c, FILE *stream);
static FILE mystdout = FDEV_SETUP_STREAM(put_char, NULL,_FDEV_SETUP_WRITE);

int main(void)
{
    SREG &= ~_BV(SREG_I);               // Disable all interrupts

    Port_Init();
    stdout = &mystdout;     //set the output stream

    GICR = 0x00;                // disable external INTs, clear before changing ISC10,11
    MCUCR |= _BV(ISC01) | _BV(ISC00);           // interrupt on INT0 pin rising edge        
    GICR  |= _BV(INT0); // enable external INTs
    USART_Init();
    // Configure clock and data as inputs, interrupt on rising edge of clock

    // Set up the LED output
    PORTC &= ~_BV(USER_LED);

    // Initialise the CMX589A control pins
    // RxHOLDN high
    PORTC |=  _BV(RxHOLDN);
    // RxDCacq and PLLacq initially low
    PORTC &= ~(_BV(RxDCacq) | _BV(PLLacq) );

    // Initialise receive interrupt into initial state
    rxstate = RX_INIT;
    // wait for a few bit times before raising RxDCacq and PLLacq
    bitcount = 1024;

    // Initialise the FIFO for received packets
    FIFO_Init();
    
    GIFR |= _BV(INTF0);
    

    // Output banner message to UART
    puts("UAIS");   
    SREG |= _BV(SREG_I);        // Global Interrupt enable

    // Sit in a tight loop, waiting for packets to be received
    while(1)
    {
        pkt_size = FIFO_Get_Packet(rx_pkt, 128);    //SIZE OF BUFFER

        // A good packet must be bigger than the 2-byte CRC !
        if (pkt_size > 2 )
        {
            // Send NMEA message - discarding the CRC
            NMEA_Send(rx_pkt, pkt_size-2);
        }
    }
}// main

void Port_Init(void)
{
    PORTB = 0x00; DDRB = 0xEE;  //8     PB0 input for RxData
    PORTC = 0x00; DDRC = 0xEF;  //7     all outputs
    PORTD = 0x00; DDRD = 0xFB;  //8     PD2 input for INT0
}

void USART_Init(void)
{
    // Set baud rate
    UBRRH = (uint8_t)(USART_UBBR_VALUE>>8);
    UBRRL = (uint8_t)USART_UBBR_VALUE;
    //Set data frame format: asynchronous mode,no parity, 1 stop bit, 8 bit size
    UCSRC=(1<<URSEL)|(0<<UMSEL)|(0<<UPM1)|(0<<UPM0)|(0<<USBS)|(0<<UCSZ2)|(1<<UCSZ1)|(1<<UCSZ0); 
    //Enable Transmitter and Interrupt on receive complete
    UCSRB=(1<<TXEN);
}

static int put_char(char c, FILE *stream)
{
    loop_until_bit_is_set(UCSRA,UDRE); //wait for UDR to be clear
    UDR = c;
    return 0;
}
//************************************************************************
//
// EOF
//
//************************************************************************
