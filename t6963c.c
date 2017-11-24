// Wy�wietlacz graficzny ze sterownikiem Toshiba T6963C
// (c) Rados�aw Kwiecie�, 2007
// Kompilator : arm-elf-gcc
//
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.h"
#include "driverlib/pwm.h"
#include <stdarg.h>
#include "t6963c.h"
#include "graphic.h"

//#define LCD_DEBUG

//#define LCD_DEBUG
//#define LCD_DEBUG_inByte    1

uint32_t g_ui32SysClock;
static const char * const g_pcHex = "0123456789abcdef";
volatile uint8_t g_bFeedWatchdog;

__inline void outByte(unsigned char byte)
{
    //AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, (0xFF << T6963_D0));
    //AT91F_PIO_ForceOutput(AT91C_BASE_PIOA, (unsigned int) (byte << T6963_D0));
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, 0xFF);
    //SysCtlDelay(g_ui32SysClock/1000000);
    ROM_GPIOPinWrite(GPIO_PORTD_BASE, 0xFF, byte & 0xFF);
}

__inline unsigned char inByte(void)
{
    //AT91F_PIO_CfgInput(AT91C_BASE_PIOA, (0xFF << T6963_D0));
    //return ((AT91F_PIO_GetInput(AT91C_BASE_PIOA) >> T6963_D0) & 0xFF);
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, 0xFF);
    GPIOPadConfigSet(GPIO_PORTD_BASE, 0xFF,
    GPIO_STRENGTH_12MA,
                     GPIO_PIN_TYPE_STD_WPU);
    GPIOPinWrite(GPIO_PORTD_BASE, 0xFF, 0xFF);
#ifdef LCD_DEBUG_inByte
    SysCtlDelay(g_ui32SysClock/10000);
    UARTprintf("Get input: %d\n", ROM_GPIOPinRead(GPIO_PORTD_BASE, 0xFF)&0xFF);
#endif
    return ROM_GPIOPinRead(GPIO_PORTD_BASE, 0xFF)&0xFF;
}

void delay(void)
{
    //volatile int i;
    //for (i = 0; i < 4; i++)
    //    ;
    SysCtlDelay(g_ui32SysClock/1000000);
}

// funkcja odczytuj�ca bajt statusu wy�wietlacza
int GLCD_Chceck_Status(void)
{
    int tmp;

    //AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, T6963_CE | T6963_RD);
    ROM_GPIOPinWrite(GPIO_PORTK_BASE, T6963_CE | T6963_RD, 0x00);
    delay();

    tmp = inByte();

    ROM_GPIOPinWrite(GPIO_PORTK_BASE, T6963_CE | T6963_RD, T6963_CE | T6963_RD);

    //AT91F_PIO_SetOutput(AT91C_BASE_PIOA, T6963_CE | T6963_RD);
    //UARTprintf("Check Status: %x\n", tmp);
    return tmp;
}

// funkcja zapisu rozkazu do sterownika
//void GLCD_Write_Command(int command)
//{
//    while (!(GLCD_Chceck_Status() & 0x03))
//        ;
//
//    outByte(command);
//
//    ROM_GPIOPinWrite(GPIO_PORTK_BASE, T6963_CE | T6963_WR, 0x00);
//    delay();
//    ROM_GPIOPinWrite(GPIO_PORTK_BASE, T6963_CE | T6963_WR, 0xFF);
//#ifdef LCD_DEBUG
//    UARTprintf("Command Written: %x\n", command);
//#endif
//
//}
// funkcja zapisu rozkazu do sterownika
uint8_t GLCD_Write_Command(int command)
{
    uint32_t timeout=g_ui32SysClock/1000;
    while (!(GLCD_Chceck_Status() & 0x03))
    {
        if(--timeout == 0)
        {
            UARTprintf("[ERROR] GLCD_Write_Command TIMEOUT\n");
            return 1;
        }
    }
    outByte(command);

    ROM_GPIOPinWrite(GPIO_PORTK_BASE, T6963_CE | T6963_WR, 0x00);
    delay();
    ROM_GPIOPinWrite(GPIO_PORTK_BASE, T6963_CE | T6963_WR, 0xFF);
#ifdef LCD_DEBUG_LV1
    UARTprintf("Command Written: %x\n", command);
#endif
    return 0;
}

//void GLCD_Write_Data(int data)
//{
//    while (!(GLCD_Chceck_Status() & 0x03))
//        ;
//
//    outByte(data);
//
//    //AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, T6963_CE | T6963_WR | T6963_CD);
//    ROM_GPIOPinWrite(GPIO_PORTK_BASE, T6963_CE | T6963_WR | T6963_CD, 0x00);
//
//    delay();
//    ROM_GPIOPinWrite(GPIO_PORTK_BASE, T6963_CE | T6963_WR | T6963_CD, T6963_CE | T6963_WR | T6963_CD);
//#ifdef LCD_DEBUG
//    UARTprintf("Data Written: %x\n", data);
//#endif
//}
uint8_t GLCD_Write_Data(int data)
{
    uint32_t timeout=g_ui32SysClock/1000;
    while (!(GLCD_Chceck_Status() & 0x03))
    {
        if(--timeout == 0)
        {
            UARTprintf("[ERROR] GLCD_Write_Data TIMEOUT\n");
            return 1;
        }
    }

    outByte(data);

    //AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, T6963_CE | T6963_WR | T6963_CD);
    ROM_GPIOPinWrite(GPIO_PORTK_BASE, T6963_CE | T6963_WR | T6963_CD, 0x00);

    delay();
    ROM_GPIOPinWrite(GPIO_PORTK_BASE, T6963_CE | T6963_WR | T6963_CD, T6963_CE | T6963_WR | T6963_CD);
#ifdef LCD_DEBUG_LV1
    UARTprintf("Data Written: %x\n", data);
#endif
    return 0;
}
//int GLCD_Read_Data(void)
//{
//    int tmp;
//    while (!(GLCD_Chceck_Status() & 0x03))
//        ;
//
//    ROM_GPIOPinWrite(GPIO_PORTK_BASE, T6963_CE | T6963_RD | T6963_CD, 0x00);
//
//    delay();
//    tmp = inByte();
//    ROM_GPIOPinWrite(GPIO_PORTK_BASE, T6963_CE | T6963_RD | T6963_CD, T6963_CE | T6963_RD | T6963_CD);
//    UARTprintf("Read Data: %d\n", tmp);
//    //AT91F_PIO_SetOutput(AT91C_BASE_PIOA, T6963_CE | T6963_RD | T6963_CD);
//    return tmp;
//}
int GLCD_Read_Data(void)
{
    int tmp;
    uint32_t timeout=g_ui32SysClock/10;
    while (!(GLCD_Chceck_Status() & 0x03))
    {
        if(--timeout == 0)
        {
            return 0;
        }
    }

    ROM_GPIOPinWrite(GPIO_PORTK_BASE, T6963_CE | T6963_RD | T6963_CD, 0x00);

    delay();
    tmp = inByte();
    ROM_GPIOPinWrite(GPIO_PORTK_BASE, T6963_CE | T6963_RD | T6963_CD, T6963_CE | T6963_RD | T6963_CD);
    UARTprintf("Read Data: %d\n", tmp);
    //AT91F_PIO_SetOutput(AT91C_BASE_PIOA, T6963_CE | T6963_RD | T6963_CD);
    return tmp;
}

void GLCD_Clear_Text(void)
{
    int i;
    GLCD_Write_Data(GLCD_TEXT_HOME);
    GLCD_Write_Data(GLCD_TEXT_HOME >> 8);
    GLCD_Write_Command(T6963_SET_ADDRESS_POINTER);
    GLCD_Text_GoTo(0,0);

    for (i = 0; i < GLCD_TEXT_SIZE; i++)
    //for (i = 0; i < 240*64; i++)
    {
        GLCD_Write_Data(0);
        GLCD_Write_Command(T6963_DATA_WRITE_AND_INCREMENT);
    }
}

void GLCD_Clear_CG(void)
{
    int i;
    GLCD_Write_Data(GLCD_EXTERNAL_CG_HOME & 0xFF);
    GLCD_Write_Data(GLCD_EXTERNAL_CG_HOME >> 8);
    GLCD_Write_Command(T6963_SET_ADDRESS_POINTER);

    for (i = 0; i < 256 * 8; i++)
    {
        GLCD_Write_Data(0);
        GLCD_Write_Command(T6963_DATA_WRITE_AND_INCREMENT);
    }
}

void GLCD_Clear_Graphic(void)
{
    int i;
    GLCD_Write_Data(GLCD_GRAPHIC_HOME & 0xFF);
    GLCD_Write_Data(GLCD_GRAPHIC_HOME >> 8);
    GLCD_Write_Command(T6963_SET_ADDRESS_POINTER);

    for (i = 0; i < GLCD_GRAPHIC_SIZE; i++)
    {
        GLCD_Write_Data(0x00);
        GLCD_Write_Command(T6963_DATA_WRITE_AND_INCREMENT);
    }
}

void GLCD_Write_Char(char ch)
{
    GLCD_Write_Data(ch - 32);
    GLCD_Write_Command(T6963_DATA_WRITE_AND_INCREMENT);
}

void GLCD_Write_String(char * str)
{
    while (*str)
    {
        GLCD_Write_Char(*str++);
    }
}

void GLCD_Text_GoTo(int x, int y)
{
#ifdef LCD_DEBUG
    UARTprintf("In goto\n");
#endif
    int address;

    address = GLCD_TEXT_HOME + x + (GLCD_TEXT_AREA * y);

    GLCD_Write_Data(address);
    GLCD_Write_Data(address >> 8);
    GLCD_Write_Command(T6963_SET_ADDRESS_POINTER);
#ifdef LCD_DEBUG
    UARTprintf("Out goto\n");
#endif
}

void GLCD_Define_Character(int charCode, int * defChar)
{
    int address;
    int i;

    address = GLCD_EXTERNAL_CG_HOME + (8 * charCode);

    GLCD_Write_Data(address);
    GLCD_Write_Data(address >> 8);
    GLCD_Write_Command(T6963_SET_ADDRESS_POINTER);

    for (i = 0; i < 8; i++)
    {
        GLCD_Write_Data(*(defChar + i));
        GLCD_Write_Command(T6963_DATA_WRITE_AND_INCREMENT);
    }
}

void GLCD_Initalize(void)
{
    UARTprintf("Initializing GLCD\n");
    g_bFeedWatchdog = 0;
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, 0xFF);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, T6963_WR | T6963_RD | T6963_CE | T6963_CD | T6963_FS | T6963_RESET);
    ROM_GPIOPinWrite(GPIO_PORTK_BASE, T6963_WR | T6963_RD | T6963_CE | T6963_CD | T6963_FS | T6963_RESET, 0x00);
    SysCtlDelay(g_ui32SysClock/2);
    ROM_GPIOPinWrite(GPIO_PORTK_BASE, T6963_RESET, T6963_RESET);
    SysCtlDelay(g_ui32SysClock/2);

    while (1)
    {
        volatile int i;
        // //Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
        UARTprintf("Try to reset GLCD\n");
        ROM_GPIOPinWrite(GPIO_PORTK_BASE, T6963_RESET, 0x00);
        SysCtlDelay(g_ui32SysClock);
        ROM_GPIOPinWrite(GPIO_PORTK_BASE, T6963_RESET, T6963_RESET);


    #if (GLCD_FONT_WIDTH == 8)
        ROM_GPIOPinWrite(GPIO_PORTK_BASE, T6963_FS, 0x00);
    #endif

        if(GLCD_Write_Data(GLCD_GRAPHIC_HOME & 0xFF) == 1)
            continue;
        if(GLCD_Write_Data(GLCD_GRAPHIC_HOME >> 8) == 1)
            continue;
        if(GLCD_Write_Command(T6963_SET_GRAPHIC_HOME_ADDRESS) == 1)
            continue;

        if(GLCD_Write_Data(GLCD_GRAPHIC_AREA) == 1)
            continue;
        if(GLCD_Write_Data(0x00) == 1)
            continue;
        if(GLCD_Write_Command(T6963_SET_GRAPHIC_AREA) == 1)
            continue;

        if(GLCD_Write_Data(GLCD_TEXT_HOME) == 1)
            continue;

        if(GLCD_Write_Data(GLCD_TEXT_HOME >> 8) == 1)
            continue;

        if(GLCD_Write_Command(T6963_SET_TEXT_HOME_ADDRESS) == 1)
            continue;

        if(GLCD_Write_Data(GLCD_TEXT_AREA) == 1)
            continue;

        if(GLCD_Write_Data(0x00) == 1)
            continue;

        if(GLCD_Write_Command(T6963_SET_TEXT_AREA) == 1)
            continue;

        if(GLCD_Write_Data(GLCD_OFFSET_REGISTER) == 1)
            continue;

        if(GLCD_Write_Data(0x00) == 1)
            continue;

        if(GLCD_Write_Command(T6963_SET_OFFSET_REGISTER) == 1)
            continue;

        if(GLCD_Write_Data(0) == 1)
            continue;

        if(GLCD_Write_Data(0) == 1)
            continue;

        if(GLCD_Write_Command(T6963_SET_ADDRESS_POINTER) == 1)
            continue;

        if(GLCD_Write_Command(
                T6963_DISPLAY_MODE | T6963_GRAPHIC_DISPLAY_ON
                        | T6963_TEXT_DISPLAY_ON | T6963_CURSOR_DISPLAY_ON) == 1)
            continue;

        if(GLCD_Write_Command(T6963_MODE_SET) == 1)
            continue;
        //GLCD_Clear_Graphic();
        GLCD_Clear_Text();
        GLCD_Clear_CG();
        GLCD_Clear_Graphic();
        GLCD_Text_GoTo(0,0);
        GLCD_Write_String("  Initializing System.... ");
        SysCtlDelay(g_ui32SysClock/10);
        GLCD_Text_GoTo(0,1);
        GLCD_Write_String("     > Initialized Motor.... ");
        SysCtlDelay(g_ui32SysClock/10);

        GLCD_Text_GoTo(0,2);
        GLCD_Write_String("     > Initialized Zigbee.... ");
        SysCtlDelay(g_ui32SysClock/10);

        GLCD_Text_GoTo(0,3);
        GLCD_Write_String("     > Initialized RFID.... ");
        SysCtlDelay(g_ui32SysClock/10);

        GLCD_Text_GoTo(0,4);
        GLCD_Write_String("     > Initialized Modbus.... ");
        SysCtlDelay(g_ui32SysClock/10);

        GLCD_Clear_Text();
        GLCD_Clear_CG();
        GLCD_Clear_Graphic();
        GLCD_Text_GoTo(0,0);
        //g_bFeedWatchdog = true;
//        GLCD_Text_GoTo(0,2);
//        GLCD_Write_String("- Status   \t:OK");
//
//
//        GLCD_Text_GoTo(0,3);
//        GLCD_Write_String("- Connected\t:1 - OK: 1, NG: 0");
//
//        GLCD_Text_GoTo(0,7);
//        GLCD_Write_String(" B1 - Reset  |   B2 - Option");
    //    UARTprintf("LCD Was Initialized\n");
     //   GLCDPrintfNormal(0, 0, " System Initializing....");
        return;
    }

}

void GLCD_SetPixel(int x, int y, int color)
{
    int tmp;
    int address;

    address = GLCD_GRAPHIC_HOME + (x / GLCD_FONT_WIDTH)
            + (GLCD_GRAPHIC_AREA * y);

    GLCD_Write_Data(address & 0xFF);
    GLCD_Write_Data(address >> 8);
    GLCD_Write_Command(T6963_SET_ADDRESS_POINTER);

    GLCD_Write_Command(T6963_DATA_READ_AND_NONVARIABLE);
    tmp = GLCD_Read_Data();

    if (color)
        tmp |= (1 << (GLCD_FONT_WIDTH - 1 - (x % GLCD_FONT_WIDTH)));
    else
        tmp &= ~(1 << (GLCD_FONT_WIDTH - 1 - (x % GLCD_FONT_WIDTH)));

    GLCD_Write_Data(tmp);
    GLCD_Write_Command(T6963_DATA_WRITE_AND_INCREMENT);
}

void GLCDPrintf(uint8_t *str, float number)
{
#ifdef LCD_DEBUG
    UARTprintf("In Print\n");
#endif
    char buffer[25];
    snprintf(buffer, 25, "%s %.2f\n", str, number);
    GLCD_Write_String(buffer);
#ifdef LCD_DEBUG
    UARTprintf("Out Print\n");
#endif
}

void GLCDvPrintfNormal(const char *pcString, va_list vaArgP)
{
    uint32_t ui32Idx, ui32Value, ui32Pos, ui32Count, ui32Base, ui32Neg;
    char *pcStr, pcBuf[16], cFill;

    //
    // Check the arguments.
    //
    ASSERT(pcString != 0);

    //
    // Loop while there are more characters in the string.
    //
    while(*pcString)
    {
        //
        // Find the first non-% character, or the end of the string.
        //
        for(ui32Idx = 0;
            (pcString[ui32Idx] != '%') && (pcString[ui32Idx] != '\0');
            ui32Idx++)
        {
        }

        //
        // Write this portion of the string.
        //
        GLCDWriteStringIndex(pcString, ui32Idx);

        //
        // Skip the portion of the string that was written.
        //
        pcString += ui32Idx;

        //
        // See if the next character is a %.
        //
        if(*pcString == '%')
        {
            //
            // Skip the %.
            //
            pcString++;

            //
            // Set the digit count to zero, and the fill character to space
            // (in other words, to the defaults).
            //
            ui32Count = 0;
            cFill = ' ';

            //
            // It may be necessary to get back here to process more characters.
            // Goto's aren't pretty, but effective.  I feel extremely dirty for
            // using not one but two of the beasts.
            //
again:

            //
            // Determine how to handle the next character.
            //
            switch(*pcString++)
            {
                //
                // Handle the digit characters.
                //
                case '0':
                case '1':
                case '2':
                case '3':
                case '4':
                case '5':
                case '6':
                case '7':
                case '8':
                case '9':
                {
                    //
                    // If this is a zero, and it is the first digit, then the
                    // fill character is a zero instead of a space.
                    //
                    if((pcString[-1] == '0') && (ui32Count == 0))
                    {
                        cFill = '0';
                    }

                    //
                    // Update the digit count.
                    //
                    ui32Count *= 10;
                    ui32Count += pcString[-1] - '0';

                    //
                    // Get the next character.
                    //
                    goto again;
                }

                //
                // Handle the %c command.
                //
                case 'c':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ui32Value = va_arg(vaArgP, uint32_t);

                    //
                    // Print out the character.
                    //
                    GLCDWriteStringIndex((char *)&ui32Value, 1);

                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle the %d and %i commands.
                //
                case 'd':
                case 'i':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ui32Value = va_arg(vaArgP, uint32_t);

                    //
                    // Reset the buffer position.
                    //
                    ui32Pos = 0;

                    //
                    // If the value is negative, make it positive and indicate
                    // that a minus sign is needed.
                    //
                    if((int32_t)ui32Value < 0)
                    {
                        //
                        // Make the value positive.
                        //
                        ui32Value = -(int32_t)ui32Value;

                        //
                        // Indicate that the value is negative.
                        //
                        ui32Neg = 1;
                    }
                    else
                    {
                        //
                        // Indicate that the value is positive so that a minus
                        // sign isn't inserted.
                        //
                        ui32Neg = 0;
                    }

                    //
                    // Set the base to 10.
                    //
                    ui32Base = 10;

                    //
                    // Convert the value to ASCII.
                    //
                    goto convert;
                }

                //
                // Handle the %s command.
                //
                case 's':
                {
                    //
                    // Get the string pointer from the varargs.
                    //
                    pcStr = va_arg(vaArgP, char *);

                    //
                    // Determine the length of the string.
                    //
                    for(ui32Idx = 0; pcStr[ui32Idx] != '\0'; ui32Idx++)
                    {
                    }

                    //
                    // Write the string.
                    //
                    GLCDWriteStringIndex(pcStr, ui32Idx);

                    //
                    // Write any required padding spaces
                    //
                    if(ui32Count > ui32Idx)
                    {
                        ui32Count -= ui32Idx;
                        while(ui32Count--)
                        {
                            GLCDWriteStringIndex(" ", 1);
                        }
                    }

                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle the %u command.
                //
                case 'u':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ui32Value = va_arg(vaArgP, uint32_t);

                    //
                    // Reset the buffer position.
                    //
                    ui32Pos = 0;

                    //
                    // Set the base to 10.
                    //
                    ui32Base = 10;

                    //
                    // Indicate that the value is positive so that a minus sign
                    // isn't inserted.
                    //
                    ui32Neg = 0;

                    //
                    // Convert the value to ASCII.
                    //
                    goto convert;
                }

                //
                // Handle the %x and %X commands.  Note that they are treated
                // identically; in other words, %X will use lower case letters
                // for a-f instead of the upper case letters it should use.  We
                // also alias %p to %x.
                //
                case 'x':
                case 'X':
                case 'p':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ui32Value = va_arg(vaArgP, uint32_t);

                    //
                    // Reset the buffer position.
                    //
                    ui32Pos = 0;

                    //
                    // Set the base to 16.
                    //
                    ui32Base = 16;

                    //
                    // Indicate that the value is positive so that a minus sign
                    // isn't inserted.
                    //
                    ui32Neg = 0;

                    //
                    // Determine the number of digits in the string version of
                    // the value.
                    //
convert:
                    for(ui32Idx = 1;
                        (((ui32Idx * ui32Base) <= ui32Value) &&
                         (((ui32Idx * ui32Base) / ui32Base) == ui32Idx));
                        ui32Idx *= ui32Base, ui32Count--)
                    {
                    }

                    //
                    // If the value is negative, reduce the count of padding
                    // characters needed.
                    //
                    if(ui32Neg)
                    {
                        ui32Count--;
                    }

                    //
                    // If the value is negative and the value is padded with
                    // zeros, then place the minus sign before the padding.
                    //
                    if(ui32Neg && (cFill == '0'))
                    {
                        //
                        // Place the minus sign in the output buffer.
                        //
                        pcBuf[ui32Pos++] = '-';

                        //
                        // The minus sign has been placed, so turn off the
                        // negative flag.
                        //
                        ui32Neg = 0;
                    }

                    //
                    // Provide additional padding at the beginning of the
                    // string conversion if needed.
                    //
                    if((ui32Count > 1) && (ui32Count < 16))
                    {
                        for(ui32Count--; ui32Count; ui32Count--)
                        {
                            pcBuf[ui32Pos++] = cFill;
                        }
                    }

                    //
                    // If the value is negative, then place the minus sign
                    // before the number.
                    //
                    if(ui32Neg)
                    {
                        //
                        // Place the minus sign in the output buffer.
                        //
                        pcBuf[ui32Pos++] = '-';
                    }

                    //
                    // Convert the value into a string.
                    //
                    for(; ui32Idx; ui32Idx /= ui32Base)
                    {
                        pcBuf[ui32Pos++] =
                            g_pcHex[(ui32Value / ui32Idx) % ui32Base];
                    }

                    //
                    // Write the string.
                    //
                    GLCDWriteStringIndex(pcBuf, ui32Pos);

                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle the %% command.
                //
                case '%':
                {
                    //
                    // Simply write a single %.
                    //
                    GLCDWriteStringIndex(pcString - 1, 1);

                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle all other commands.
                //
                default:
                {
                    //
                    // Indicate an error.
                    //
                    GLCDWriteStringIndex("ERROR", 5);

                    //
                    // This command has been handled.
                    //
                    break;
                }
            }
        }
    }
}
void GLCDWriteStringIndex(const char * str, uint32_t ui32Len)
{
    unsigned int uIdx;

    for(uIdx = 0; uIdx < ui32Len; uIdx++)
    {
        //
        // Send the character to the UART output.
        //
        GLCD_Write_Char(str[uIdx]);
    }
}
void GLCDPrintfNormal(uint8_t col, uint8_t row, const char *pcString, ...)
{
    GLCD_Text_GoTo(col, row);
    va_list vaArgP;

    //
    // Start the varargs processing.
    //
    va_start(vaArgP, pcString);

    GLCDvPrintfNormal(pcString, vaArgP);

    //
    // We're finished with the varargs now.
    //
    va_end(vaArgP);
}
