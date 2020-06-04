// Useful advice! http://www.microchip.com/forums/m790211.aspx
// FCY must be defined before the includes or things go wrong.
// For some odd reason, the config tool does not set this based
// on the MCU + Oscillator selected
// by stancecoke
#define SYS_FREQ        7370000L
#define FCY             SYS_FREQ/4

//*****************************************************************************
// Device Config; generate using Window > PIC Memory Views > Configuration bits
//*****************************************************************************

// FOSC
#pragma config FOSFPR = FRC             // Oscillator (Internal Fast RC (No change to Primary Osc Mode bits))
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_ON             // Watchdog Timer (Enabled)

// FBORPOR
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON          // PBOR Enable (Enabled)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
//#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

//*****************************************************************************
// Device Config /end (paste config between these two)
//*****************************************************************************

#include <xc.h>
#include <libpic30.h>
#include <uart.h>
#include <stdio.h>
/* Received data is stored in array Buf */
char Buf[80];
char * Receivedddata = Buf;
/* This is UART1 transmit ISR */
void __attribute__((__interrupt__, no_auto_psv)) _U2TXInterrupt(void)
//void _ISR __attribute__((no_auto_psv)) _SPI1Interrupt(void)
{
 IFS1bits.U2TXIF = 0;
}
/* This is UART1 receive ISR */
void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void)
{
 IFS1bits.U2RXIF = 0;
/* Read the receive buffer till atleast one or more character can be
read */
 while( DataRdyUART2())
 {
 ( *( Receivedddata)++) = ReadUART2();
 }
} 

int main(void) {
    _TRISD8 = 0; // set D3 to output
    _LATD8 = 1;
    
    /* Data to be transmitted using UART communication module */
char Txdata[30];
/* Holds the value of baud register */
unsigned int baudvalue;
/* Holds the value of uart config reg */
unsigned int U2MODEvalue;
/* Holds the information regarding uart
TX & RX interrupt modes */
unsigned int U2STAvalue;
/* Turn off UART1module */
 CloseUART2();
/* Configure uart1 receive and transmit interrupt */
 ConfigIntUART2(UART_RX_INT_EN & UART_RX_INT_PR6 &
 UART_TX_INT_DIS & UART_TX_INT_PR2);
/* Configure UART1 module to transmit 8 bit data with one stopbit.
Also Enable loopback mode */
 baudvalue = 5;
 U2MODEvalue = UART_EN & UART_IDLE_CON &
 UART_DIS_WAKE & UART_EN_LOOPBACK &
 UART_EN_ABAUD & UART_NO_PAR_8BIT &
 UART_1STOPBIT;
 U2STAvalue = UART_INT_TX_BUF_EMPTY &
 UART_TX_PIN_NORMAL &
 UART_TX_ENABLE & UART_INT_RX_3_4_FUL &
 UART_ADR_DETECT_DIS &
 UART_RX_OVERRUN_CLEAR;
 OpenUART2(U2MODEvalue, U2STAvalue, baudvalue);
    //welcome message
    sprintf(Txdata, "BionX OSF v0.0\r\n");
    putsUART2 ((unsigned int *)Txdata);
    /* Wait for transmission to complete */
    while(BusyUART2());
    

/* Read all the data remaining in receive buffer which are unread 
 while(DataRdyUART1())
 {
 (*( Receivedddata)++) = ReadUART1() ;
 }*/

 

    while(1)
    {
        _LATD8 = 0;
        __delay_ms(1000);
        _LATD8 = 1;
        __delay_ms(1000);

     /* Load transmit buffer and transmit the same till null character is
    encountered */
    sprintf(Txdata, "%b\r\n", PORTD);

    putsUART2 ((unsigned int *)Txdata);
    /* Wait for transmission to complete */
    while(BusyUART2());
    }
 /* Turn off UART module */
    CloseUART2();
    return 0;
}