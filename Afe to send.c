// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL with LCD/Keyboard Interface
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red Backlight LED:
//   PB5 drives an NPN transistor that powers the red LED
// Green Backlight LED:
//   PE5 drives an NPN transistor that powers the green LED
// Blue Backlight LED:
//   PE4 drives an NPN transistor that powers the blue LED
// ST7565R Graphics LCD Display Interface:
//   MOSI (SSI2Tx) on PB7
//   MISO (SSI2Rx) is not used by the LCD display but the pin is used for GPIO for A0
//   SCLK (SSI2Clk) on PB4
//   A0 connected to PB6
//   ~CS connected to PB1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include <math.h>

#define RED_BL_LED   (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 5*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define RST (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4)))
#define VDD  (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4)))

#define GREEN_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4))) // on-board blue LED

#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4))) // RF Sync

#define CS_NOT       (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 1*4)))
#define A0           (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4)))

// Set pixel arguments
#define CLEAR  0
#define SET    1
#define INVERT 2


//Location
#define SPEED_SOUND 0.034; // check actual value(cm per us)

#define slotDelay 100000;

double xn1, xn2, yn1, yn2, zn1, zn2;
double x1, x2, x3, y_1, y2, y3, z1, z2, z3,y1;
double r1, r2, r3;
double dt1, dt2, dt3;

char dispstr[100];



//AFE
char str[81];
int8_t offset[10];
char type[10];
float time = 0;
bool timeUpdate = false;
uint16_t raw1;
uint8_t slotNumber;

uint32_t timerCapture;

uint8_t slotNumberList[100];
uint16_t slotCounter=0;

uint8_t count;
bool firstTime = true;
bool counterStart = false;
//AFE

bool burst1Received = false;
bool burst2Received = false;
bool burst3Received = false;
uint8_t burstNumber;


//Location

// RF

uint8_t seq_count=0;
bool sync_detected = false;
uint8_t data_len;
uint32_t sum;
char buffer[256];
char data[256];
static int byte_count = 0;
char in;

uint8_t k=0;
bool data_r = false;
char sync_seq[5]={0x41,0x42,0x43,0x44,0x45};

// RF

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
        putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}


////////////////////////////////////////////////////////////////////

uint8_t  pixelMap[1024];
uint16_t txtIndex = 0;

// 96 character 5x7 bitmaps based on ISO-646 (BCT IRV extensions)
const uint8_t charGen[100][5] = {
    // Codes 32-127
    // Space ! " % $ % & ' ( ) * + , - . /
    {0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x4F, 0x00, 0x00},
    {0x00, 0x07, 0x00, 0x07, 0x00},
    {0x14, 0x7F, 0x14, 0x7F, 0x14},
    {0x24, 0x2A, 0x7F, 0x2A, 0x12},
    {0x23, 0x13, 0x08, 0x64, 0x62},
    {0x36, 0x49, 0x55, 0x22, 0x40},
    {0x00, 0x05, 0x03, 0x00, 0x00},
    {0x00, 0x1C, 0x22, 0x41, 0x00},
    {0x00, 0x41, 0x22, 0x1C, 0x00},
    {0x14, 0x08, 0x3E, 0x08, 0x14},
    {0x08, 0x08, 0x3E, 0x08, 0x08},
    {0x00, 0x50, 0x30, 0x00, 0x00},
    {0x08, 0x08, 0x08, 0x08, 0x08},
    {0x00, 0x60, 0x60, 0x00, 0x00},
    {0x20, 0x10, 0x08, 0x04, 0x02},
    // 0-9
    {0x3E, 0x51, 0x49, 0x45, 0x3E},
    {0x00, 0x42, 0x7F, 0x40, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46},
    {0x21, 0x41, 0x45, 0x4B, 0x31},
    {0x18, 0x14, 0x12, 0x7F, 0x10},
    {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3C, 0x4A, 0x49, 0x49, 0x30},
    {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36},
    {0x06, 0x49, 0x49, 0x29, 0x1E},
    // : ; < = > ? @
    {0x00, 0x36, 0x36, 0x00, 0x00},
    {0x00, 0x56, 0x36, 0x00, 0x00},
    {0x08, 0x14, 0x22, 0x41, 0x00},
    {0x14, 0x14, 0x14, 0x14, 0x14},
    {0x00, 0x41, 0x22, 0x14, 0x08},
    {0x02, 0x01, 0x51, 0x09, 0x3E},
    {0x32, 0x49, 0x79, 0x41, 0x3E},
    // A-Z
    {0x7E, 0x11, 0x11, 0x11, 0x7E},
    {0x7F, 0x49, 0x49, 0x49, 0x36},
    {0x3E, 0x41, 0x41, 0x41, 0x22},
    {0x7F, 0x41, 0x41, 0x22, 0x1C},
    {0x7F, 0x49, 0x49, 0x49, 0x41},
    {0x7F, 0x09, 0x09, 0x09, 0x01},
    {0x3E, 0x41, 0x49, 0x49, 0x3A},
    {0x7F, 0x08, 0x08, 0x08, 0x7F},
    {0x00, 0x41, 0x7F, 0x41, 0x00},
    {0x20, 0x40, 0x41, 0x3F, 0x01},
    {0x7F, 0x08, 0x14, 0x22, 0x41},
    {0x7F, 0x40, 0x40, 0x40, 0x40},
    {0x7F, 0x02, 0x0C, 0x02, 0x7F},
    {0x7F, 0x04, 0x08, 0x10, 0x7F},
    {0x3E, 0x41, 0x41, 0x41, 0x3E},
    {0x7F, 0x09, 0x09, 0x09, 0x06},
    {0x3E, 0x41, 0x51, 0x21, 0x5E},
    {0x7F, 0x09, 0x19, 0x29, 0x46},
    {0x46, 0x49, 0x49, 0x49, 0x31},
    {0x01, 0x01, 0x7F, 0x01, 0x01},
    {0x3F, 0x40, 0x40, 0x40, 0x3F},
    {0x1F, 0x20, 0x40, 0x20, 0x1F},
    {0x3F, 0x40, 0x70, 0x40, 0x3F},
    {0x63, 0x14, 0x08, 0x14, 0x63},
    {0x07, 0x08, 0x70, 0x08, 0x07},
    {0x61, 0x51, 0x49, 0x45, 0x43},
    // [ \ ] ^ _ `
    {0x00, 0x7F, 0x41, 0x41, 0x00},
    {0x02, 0x04, 0x08, 0x10, 0x20},
    {0x00, 0x41, 0x41, 0x7F, 0x00},
    {0x04, 0x02, 0x01, 0x02, 0x04},
    {0x40, 0x40, 0x40, 0x40, 0x40},
    {0x00, 0x01, 0x02, 0x04, 0x00},
    // a-z
    {0x20, 0x54, 0x54, 0x54, 0x78},
    {0x7F, 0x44, 0x44, 0x44, 0x38},
    {0x38, 0x44, 0x44, 0x44, 0x20},
    {0x38, 0x44, 0x44, 0x48, 0x7F},
    {0x38, 0x54, 0x54, 0x54, 0x18},
    {0x08, 0x7E, 0x09, 0x01, 0x02},
    {0x0C, 0x52, 0x52, 0x52, 0x3E},
    {0x7F, 0x08, 0x04, 0x04, 0x78},
    {0x00, 0x44, 0x7D, 0x40, 0x00},
    {0x20, 0x40, 0x44, 0x3D, 0x00},
    {0x7F, 0x10, 0x28, 0x44, 0x00},
    {0x00, 0x41, 0x7F, 0x40, 0x00},
    {0x7C, 0x04, 0x18, 0x04, 0x78},
    {0x7C, 0x08, 0x04, 0x04, 0x78},
    {0x38, 0x44, 0x44, 0x44, 0x38},
    {0x7C, 0x14, 0x14, 0x14, 0x08},
    {0x08, 0x14, 0x14, 0x18, 0x7C},
    {0x7C, 0x08, 0x04, 0x04, 0x08},
    {0x48, 0x54, 0x54, 0x54, 0x20},
    {0x04, 0x3F, 0x44, 0x40, 0x20},
    {0x3C, 0x40, 0x40, 0x20, 0x7C},
    {0x1C, 0x20, 0x40, 0x20, 0x1C},
    {0x3C, 0x40, 0x20, 0x40, 0x3C},
    {0x44, 0x28, 0x10, 0x28, 0x44},
    {0x0C, 0x50, 0x50, 0x50, 0x3C},
    {0x44, 0x64, 0x54, 0x4C, 0x44},
    // { | } ~ cc
    {0x00, 0x08, 0x36, 0x41, 0x00},
    {0x00, 0x00, 0x7F, 0x00, 0x00},
    {0x00, 0x41, 0x36, 0x08, 0x00},
    {0x0C, 0x04, 0x1C, 0x10, 0x18},
    {0x00, 0x00, 0x00, 0x00, 0x00},
    // Custom assignments beyond ISO646
    // Codes 128+: right arrow, left arrow, degree sign
    {0x08, 0x08, 0x2A, 0x1C, 0x08},
    {0x08, 0x1C, 0x2A, 0x08, 0x08},
    {0x07, 0x05, 0x07, 0x00, 0x00},
};

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port B and E peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOF;


    //PF3 GREEN LED
         GPIO_PORTF_DIR_R |= 0x08;  // bit 3 is output
         GPIO_PORTF_DR2R_R |= 0x08; // set drive strength to 2mA
         GPIO_PORTF_DEN_R |= 0x08;  // enable

    //PF2 BLUE LED
        GPIO_PORTF_DIR_R |= 0x04;  // bit 2 is output
        GPIO_PORTF_DR2R_R |= 0x04; // set drive strength to 2mA
        GPIO_PORTF_DEN_R |= 0x04;  // enable


    // Configure three backlight LEDs
    GPIO_PORTB_DIR_R |= 0x20;  // make bit5 an output
    GPIO_PORTB_DR2R_R |= 0x20; // set drive strength to 2mA
    GPIO_PORTB_DEN_R |= 0x20;  // enable bit5 for digital
    GPIO_PORTE_DIR_R |= 0x30;  // make bits 4 and 5 outputs
    GPIO_PORTE_DR2R_R |= 0x30; // set drive strength to 2mA
    GPIO_PORTE_DEN_R |= 0x30;  // enable bits 4 and 5 for digital

    // Configure A0 and ~CS for graphics LCD
    GPIO_PORTB_DIR_R |= 0x42;  // make bits 1 and 6 outputs
    GPIO_PORTB_DR2R_R |= 0x42; // set drive strength to 2mA
    GPIO_PORTB_DEN_R |= 0x42;  // enable bits 1 and 6 for digital

    // Configure SSI2 pins for SPI configuration
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;           // turn-on SSI2 clocking
    GPIO_PORTB_DIR_R |= 0x90;                        // make bits 4 and 7 outputs
    GPIO_PORTB_DR2R_R |= 0x90;                       // set drive strength to 2mA
    GPIO_PORTB_AFSEL_R |= 0x90;                      // select alternative functions for MOSI, SCLK pins
    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB4_SSI2CLK; // map alt fns to SSI2
    GPIO_PORTB_DEN_R |= 0x90;                        // enable digital operation on TX, CLK pins
    GPIO_PORTB_PUR_R |= 0x10;                        // must be enabled when SPO=1

    // Configure the SSI2 as a SPI master, mode 3, 8bit operation, 1 MHz bit rate
    SSI2_CR1_R &= ~SSI_CR1_SSE;                      // turn off SSI2 to allow re-configuration
    SSI2_CR1_R = 0;                                  // select master mode
    SSI2_CC_R = 0;                                   // select system clock as the clock source
    SSI2_CPSR_R = 40;                                // set bit rate to 1 MHz (if SR=0 in CR0)
    SSI2_CR0_R = SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_8; // set SR=0, mode 3 (SPH=1, SPO=1), 8-bit
    SSI2_CR1_R |= SSI_CR1_SSE;

    // Configure UART0 pins
        SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
        GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
        GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
        GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

        // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
        UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
        UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
        UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
        UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
        UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
        UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

//// RF

        //PA2 ORANGE LED
            GPIO_PORTA_DIR_R |= 0x04;  // bit 2 is output
            GPIO_PORTA_DR2R_R |= 0x04; // set drive strength to 2mA
            GPIO_PORTA_DEN_R |= 0x04;  // enable

        // UART1
            SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1; // turn-on UART0, leave other uarts in same status
            GPIO_PORTC_DEN_R |= 0x30;                         // default, added for clarity
            GPIO_PORTC_AFSEL_R |= 0x30;                       // default, added for clarity
            GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX;

            //uart1
            UART1_CTL_R = 0;                 // turn-off UART0 to allow safe programming
            UART1_CC_R = UART_CC_CS_SYSCLK;                 // use system clock (40 MHz)
            UART1_IBRD_R = 2083; // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
            UART1_FBRD_R = 21;                               // round(fract(r)*64)=45
            UART1_LCRH_R = UART_LCRH_WLEN_8; // configure for 8N1 w/o FIFO
            UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
            UART1_IM_R = UART_IM_RXIM;                       // turn-on RX interrupt
            NVIC_EN0_R |= 1 << (INT_UART1-16);               // turn-on interrupt 21 (UART0)


            // WT5CCP0(PD6)
                GPIO_PORTD_AFSEL_R |= 0x40;                      // select alternative functions for FREQ_IN pin
                GPIO_PORTD_PCTL_R &= ~GPIO_PCTL_PD6_M;           // map alt fns to FREQ_IN
                GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD6_WT5CCP0;
                GPIO_PORTD_DEN_R |= 0x40;                        // enable bit 6 for digital input

                NVIC_ST_RELOAD_R =11999999; //3319999;//300ms
              //

}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
                                                // Approx clocks per us
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             B    WMS_LOOP0");       // 1*3
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// Blocking function that writes data to the SPI bus and waits for the data to complete transmission
void sendGraphicsLcdCommand(uint8_t command)
{
    CS_NOT = 0;                        // assert chip select
    __asm (" NOP");                    // allow line to settle
    __asm (" NOP");
    __asm (" NOP");
    __asm (" NOP");
    A0 = 0;                            // clear A0 for commands
    SSI2_DR_R = command;               // write command
    while (SSI2_SR_R & SSI_SR_BSY);
    CS_NOT = 1;                        // de-assert chip select
}

// Blocking function that writes data to the SPI bus and waits for the data to complete transmission
void sendGraphicsLcdData(uint8_t data)
{
    CS_NOT = 0;                        // assert chip select
    __asm (" NOP");                    // allow line to settle
    __asm (" NOP");
    __asm (" NOP");
    __asm (" NOP");
    A0 = 1;                            // set A0 for data
    SSI2_DR_R = data;                  // write data
    while (SSI2_SR_R & SSI_SR_BSY);    // wait for transmission to stop
    CS_NOT = 1;                        // de-assert chip select
}

void setGraphicsLcdPage(uint8_t page)
{
  sendGraphicsLcdCommand(0xB0 | page);
}

void setGraphicsLcdColumn(uint8_t x)
{
  sendGraphicsLcdCommand(0x10 | ((x >> 4) & 0x0F));
  sendGraphicsLcdCommand(0x00 | (x & 0x0F));
}

void refreshGraphicsLcd()
{
    uint8_t x, page;
    uint16_t i = 0;
    for (page = 0; page < 8; page ++)
    {
        setGraphicsLcdPage(page);
        setGraphicsLcdColumn(0);
        for (x = 0; x < 128; x++)
            sendGraphicsLcdData(pixelMap[i++]);
    }
}

void clearGraphicsLcd()
{
    uint16_t i;
    // clear data memory pixel map
    for (i = 0; i < 1024; i++)
        pixelMap[i] = 0;
    // copy to display
    refreshGraphicsLcd();
}

void initGraphicsLcd()
{
    sendGraphicsLcdCommand(0x40); // set start line to 0
    sendGraphicsLcdCommand(0xA1); // reverse horizontal order
    sendGraphicsLcdCommand(0xC0); // normal vertical order
    sendGraphicsLcdCommand(0xA6); // normal pixel polarity
    sendGraphicsLcdCommand(0xA3); // set led bias to 1/9 (should be A2)
    sendGraphicsLcdCommand(0x2F); // turn on voltage booster and regulator
    sendGraphicsLcdCommand(0xF8); // set internal volt booster to 4x Vdd
    sendGraphicsLcdCommand(0x00);
    sendGraphicsLcdCommand(0x27); // set contrast
    sendGraphicsLcdCommand(0x81); // set LCD drive voltage
    sendGraphicsLcdCommand(0x04);
    sendGraphicsLcdCommand(0xAC); // no flashing indicator
    sendGraphicsLcdCommand(0x00);
    clearGraphicsLcd();           // clear display
    sendGraphicsLcdCommand(0xAF); // display on
}

void drawGraphicsLcdPixel(uint8_t x, uint8_t y, uint8_t op)
{
    uint8_t data, mask, page;
    uint16_t index;

    // determine pixel map entry
    page = y >> 3;

    // determine pixel map index
    index = page << 7 | x;

    // generate mask
    mask = 1 << (y & 7);

    // read pixel map
    data = pixelMap[index];

    // apply operator (0 = clear, 1 = set, 2 = xor)
    switch(op)
    {
        case 0: data &= ~mask; break;
        case 1: data |= mask; break;
        case 2: data ^= mask; break;
    }

    // write to pixel map
    pixelMap[index] = data;

    // write to display
    setGraphicsLcdPage(page);
    setGraphicsLcdColumn(x);
    sendGraphicsLcdData(data);
}

void drawGraphicsLcdRectangle(uint8_t xul, uint8_t yul, uint8_t dx, uint8_t dy, uint8_t op)
{
    uint8_t page, page_start, page_stop;
    uint8_t bit_index, bit_start, bit_stop;
    uint8_t mask, data;
    uint16_t index;
    uint8_t x;

    // determine pages for rectangle
    page_start = yul >> 3;
    page_stop = (yul + dy - 1) >> 3;

    // draw in pages from top to bottom within extent
    for (page = page_start; page <= page_stop; page++)
    {
        // calculate mask for this page
        if (page > page_start)
            bit_start = 0;
        else
            bit_start = yul & 7;
        if (page < page_stop)
            bit_stop = 7;
        else
            bit_stop = (yul + dy - 1) & 7;
        mask = 0;
        for (bit_index = bit_start; bit_index <= bit_stop; bit_index++)
            mask |= 1 << bit_index;

        // write page
        setGraphicsLcdPage(page);
        setGraphicsLcdColumn(xul);
        index = (page << 7) | xul;
        for (x = 0; x < dx; x++)
        {
            // read pixel map
            data = pixelMap[index];
            // apply operator (0 = clear, 1 = set, 2 = xor)
            switch(op)
            {
                case 0: data &= ~mask; break;
                case 1: data |= mask; break;
                case 2: data ^= mask; break;
            }
            // write to pixel map
            pixelMap[index++] = data;
            // write to display
            sendGraphicsLcdData(data);
        }
    }
}

void setGraphicsLcdTextPosition(uint8_t x, uint8_t page)
{
    txtIndex = (page << 7) + x;
    setGraphicsLcdPage(page);
    setGraphicsLcdColumn(x);
}

void putcGraphicsLcd(char c)
{
    uint8_t i, val;
    uint8_t uc;
    // convert to unsigned to access characters > 127
    uc = (uint8_t) c;
    for (i = 0; i < 5; i++)
    {
        val = charGen[uc-' '][i];
        pixelMap[txtIndex++] = val;
        sendGraphicsLcdData(val);
    }
    pixelMap[txtIndex++] = 0;
    sendGraphicsLcdData(0);
}

void putsGraphicsLcd(char str[])
{
    uint8_t i = 0;
    while (str[i] != 0)
        putcGraphicsLcd(str[i++]);
}


void writeLcdCoordinates(double x, double y, double z)
{
    char xStr[20];
    char yStr[20];
    char zStr[20];

    sprintf(xStr, "x: %0.3f cm", x);
    sprintf(yStr, "y: %0.3f cm", y);
    sprintf(zStr, "z: %0.3f cm", z);

    clearGraphicsLcd();

    setGraphicsLcdTextPosition(0, 1);

    putsGraphicsLcd(xStr);
    setGraphicsLcdTextPosition(0, 3);
    putsGraphicsLcd(yStr);
    setGraphicsLcdTextPosition(0, 5);
    putsGraphicsLcd(zStr);

   // setGraphicsLcdTextPosition(80, 1);
    //putsGraphicsLcd("slot");


   // sprintf(zStr, "%d", slotNumber);
    //setGraphicsLcdTextPosition(80, 3);
      //  putsGraphicsLcd(zStr);

}

//////////////////////////////////////////////////////////////////
//Location Code
//////////////////////////////////////////////////////////////////

void getX(double r1, double r2, double r3, double x1, double y_1, double z1)
{

    xn1 = (r1*r1 - r2*r2 + x2*x2)/ (2 * x2);
}

void getY(double r1, double r2, double r3, double x1, double y_1, double z1)
{
    if(burstNumber == 2)
    {

        yn1 = ((r1 + r2 + x2) * (r1 + r2 - x2) * (r1 - r2 + x2) * (r2 - r1 + x2));
        yn1 = pow(yn1,0.5)/ (2*x2);
        yn1 = abs(yn1);
    }
    else
    {
        yn1 = ( pow(r1,2)*x2 - pow(r1,2)*x3 + pow(r2,2)*x3 - pow(r3,2)*x2 - pow(x2,2)*x3 + x2*pow(x3,2) + x2*pow(y3,2) )/ (2*x2*y3);
    }

}

void getZ(double r1, double r2, double r3, double x1, double y_1, double z1)
{
    double a;

    a = -pow(r1,4)*pow(x2,2) + 2*pow(r1,4)*x2*x3 - pow(r1,4)*pow(x3,2) - pow(r1,4)*pow(y3,2);
    a += -2*pow(r1,2)*pow(r2,2)*x2*x3 + 2*pow(r1,2)*pow(r2,2)*pow(x3,2) + 2*pow(r1,2)*pow(r2,2)*pow(y3,2);
    a += 2*pow(r1,2)*pow(r3,2)*pow(x2,2) - 2*pow(r1,2)*pow(r3,2)*x2*x3 + 2*pow(r1,2)*pow(x2,3)*x3;
    a += -4*pow(r1,2)*pow(x2,2)*pow(x3,2) + 2*pow(r1,2)*x2*pow(x3,3) + 2*pow(r1,2)*x2*x3*pow(y3,2);
    a += -pow(r2,4)*pow(x3,2) - pow(r2,4)*pow(y3,2) + 2*pow(r2,2)*pow(r3,2)*x2*x3 + 2*pow(r2,2)*pow(x2,2)*pow(x3,2);
    a += 2*pow(r2,2)*pow(x2,2)*pow(y3,2) - 2*pow(r2,2)*x2*pow(x3,3) - 2*pow(r2,2)*x2*x3*pow(y3,2);
    a += -pow(r3,4)*pow(x2,2) - 2*pow(r3,2)*pow(x2,3)*x3 + 2*pow(r3,2)*pow(x2,2)*pow(x3,2);
    a += 2*pow(r3,2)*pow(x2,2)*pow(y3,2) - pow(x2,4)*pow(x3,2) - pow(x2,4)*pow(y3,2) + 2*pow(x2,3)*pow(x3,3);
    a += 2*pow(x2,3)*x3*pow(y3,2) - pow(x2,2)*pow(x3,4) - 2*pow(x2,2)*pow(x3,2)*pow(y3,2) - pow(x2,2)*pow(y3,4);

    zn1 = -pow(a,0.5)/(2*x2*y3);
    zn2 = -zn1;
    zn1 = abs(zn1);
}

double getR(double dt)
{
    double r;

    r = dt * SPEED_SOUND;

    return r;
}

void getLocation(void)
{
    //convert dt to seconds;

     BLUE_LED^=1;
    dt1 -= slotDelay;
    dt2 -= slotDelay;
    dt3 -= slotDelay;

    r1 = getR(dt1);


    r2 = getR(dt2);
    r3 = getR(dt3);
    if((r1 + r2 + r3 ) > 1000)
    {
        setGraphicsLcdTextPosition(0, 3);
            putsGraphicsLcd("N/A");
    }
    else
    {

            //r1 = 1732;
            //r2 = 1732;
            //r3 = 1732;

            //x1 = 0; x2 = 1260; x3 = 1260;
            //y_1 = 0; y2 = 0;    y3 = 607;
            //z1 = 0, z2 = 0;    z3 = 0;
            x2=287;
            y3=399;
            x3=287;

            getX(r1, r2, r3, x1, y_1, z1);
            getY(r1, r2, r3, x1, y_1, z1);
            if(burstNumber == 3)
                getZ(r1, r2, r3, x1, y_1, z1);
            else
                zn1 = 0;


            //printf("xn1 = %f, yn1 = %f\n\n, zn1 = %f, zn2 = %f\n\n", xn1, yn1, zn1, zn2);

          /*  sprintf(dispstr, "%f", xn1);
            putsUart0(dispstr);
            sprintf(dispstr, "%f", yn1);
            putsUart0(dispstr);
            sprintf(dispstr, "%f", zn1);
            putsUart0(dispstr); */

            writeLcdCoordinates(xn1,yn1,zn1);
    }

}

//////////////////////////////////////////////////////////////////
//RF Code
//////////////////////////////////////////////////////////////////

uint16_t getChecksum()
{
    uint16_t result;
    // this is based on rfc1071
    while ((sum >> 16) > 0)
        sum = (sum & 0xFFFF) + (sum >> 16);
    result = sum & 0xFFFF;
    return ~result;
}

void Uart1Isr()
{
    char in;
    static uint8_t i=0;
    in = UART1_DR_R;
    //GREEN_LED ^= 1;

    if(sync_detected==true)
    {
        buffer[k] = in;
        //data_pos = k;
        k++;
        if(k == buffer[0]+1)
        {
            data_r = true;
        }
    }

    if (in == sync_seq[i] && sync_detected == false )
    {
        seq_count++;
        i++;
        if(seq_count == 5)
        {
            ORANGE_LED ^= 1;
            sync_detected = true;
            seq_count=0;
            i=0;
            k=0;
        }
    }
    else
    {
        seq_count=0;
        i=0;
    }
}

void sum_data(void* data, uint16_t size_in_bytes)
{
    uint8_t* pData = (uint8_t*)data;
    uint16_t i;
    uint8_t phase = 1;

    uint16_t data_temp;
    for (i = 0; i < size_in_bytes; i++)
    {
        if (phase)
        {
            data_temp = *pData;
            sum += data_temp << 8;
        }
        else
            sum += *pData;
        phase = 1 - phase;
        pData++;
    }
}




////////////////////////////////////////////////////////////////////////

//AFEsystick,wt5
void systickIsr()
{
    //putsUart0("inside systick\r\n");
    if(counterStart)
    {
        //RED_LED ^= 1;
        if(slotNumber==2)
        {
            UART1_IM_R |= UART_IM_RXIM;
          //  GREEN_LED = 1;
            // turn-on RX interrupt
            WTIMER5_IMR_R &= ~TIMER_IMR_CAEIM; //turn off interrupt
            //count++;
            counterStart = false;

            if(burstNumber>=2 )//&& burst3Received)
                getLocation();

            dt1 = dt2 = dt3 = 0;
            burst1Received = burst2Received =  burst3Received = false;
            //slotNumberList[slotCounter] = slotNumber;
              //          slotCounter++;
                //        if(slotCounter==99)
                  //      {
                    //        slotCounter = 0;
                            //putcUart0(slotNumber+'0');

                       // }

            slotNumber=0;
            burstNumber = 0;
        }
        else
        {
            WTIMER5_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
            WTIMER5_IMR_R |= TIMER_IMR_CAEIM;                 // turn-on interrupts
            WTIMER5_TAV_R = 0;
               //         slotNumberList[slotCounter] = slotNumber;

            //slotCounter++;
            //sprintf(dispstr, "%d", slotNumber);
            // setGraphicsLcdTextPosition(80, 3);
             //putsGraphicsLcd(dispstr);
           // putcUart0(slotNumber+'0');
            slotNumber += 1;
           // if(slotCounter==99)
            //{
              //  slotCounter = 0;
                //putsUart0("Its 99");
           // }


        }

    }

}

void setTimerMode()
{
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;     // turn-on timer
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER5_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    WTIMER5_CTL_R = TIMER_CTL_TAEVENT_POS;           // measure time from positive edge to positive edge
    WTIMER5_IMR_R &= ~TIMER_IMR_CAEIM;                 // turn-off interrupts
    WTIMER5_TAV_R = 0;                               // zero counter for first period
    //WTIMER5_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN3_R |= 1 << (INT_WTIMER5A-16-96);         // turn-on interrupt 120 (WTIMER5A)
    //putsUart0("Inside Timer set\r\n");
}

void WideTimer5Isr()
{
    time = WTIMER5_TAR_R;                        // read systick timer value
    time -= timerCapture;
    WTIMER5_TAR_R = 0;                           // zero counter for next edge

    time /= 40;                                  // scale to us units
    if(slotNumber == 0 && time>100000 && time < 140000)
    {

        dt1 = time;
        burst1Received = true;
        burstNumber++;
    }
    else if(slotNumber == 1 && time>100000 && time < 140000)
    {

        dt2 = time;
        burst2Received = true;
        burstNumber++;
    }
    else if(slotNumber == 2 && time>100000 && time < 140000)
    {
        dt3 = time;
        burst3Received = true;
        burstNumber++;
    }

    timeUpdate = true;                           // set update flag
    //BLUE_LED ^= 1;                              // status
    //putsUart0("Interrupt triggered\n");
    WTIMER5_ICR_R = TIMER_ICR_CAECINT;               // clear interrupt flag
    WTIMER5_IMR_R &= ~TIMER_IMR_CAEIM;                 // turn-off interrupts
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off counter
}

void decode_coordinates()
{
    x1 = (((data[0] & 0xFF) << 8) + (data[1] & 0xFF));

    y1 = (((data[2] & 0xFF) << 8) + (data[3] & 0xFF));

    x2 = (((data[4] & 0xFF) << 8) + (data[5] & 0xFF));

    y2 = (((data[6] & 0xFF) << 8) + (data[7] & 0xFF));

    x3 = (((data[8] & 0xFF) << 8) + (data[9] & 0xFF));

    y3 = (((data[10] & 0xFF) << 8) + (data[11] & 0xFF));


}


//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    // Initialize hardware
    initHw();
    //GREEN_LED=1;
    setTimerMode();
    // Turn-on all LEDs to create white backlight
    //RED_BL_LED = 1;
    RST = 1;
    VDD = 1;

    // Initialize graphics LCD
    initGraphicsLcd();
 //   setTimerMode();
    // Draw X in left half of screen
    /*uint8_t i;
    for (i = 0; i < 64; i++)
        drawGraphicsLcdPixel(i, i, SET);
    for (i = 0; i < 64; i++)
        drawGraphicsLcdPixel(63-i, i, INVERT); */

    // Draw text on screen
    //setGraphicsLcdTextPosition(0, 32);


    while(1)
       {
            if(sync_detected==true ) //&& data_r == true)
            {
                UART1_IM_R &= ~UART_IM_RXIM;
                GREEN_LED = 0;
                counterStart = true;
                WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;
                NVIC_ST_CTRL_R = 0x00000007;//enable systick
                NVIC_ST_CURRENT_R = NVIC_ST_RELOAD_R;
                WTIMER5_IMR_R |= TIMER_IMR_CAEIM;                 // turn-on interrupts
                WTIMER5_TAV_R = 0;
                WTIMER5_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
                timerCapture = WTIMER5_TAV_R;
            }
            sync_detected = false;
            k=0;
            data_r = false;
       }
}


