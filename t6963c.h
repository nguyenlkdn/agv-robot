// port danych
#define T6963_D0 		0

// port sygna��w steruj�cych

// sygna�y sterujace

// LCD2CTLPORT GPIO_PORTP_BASE

#define T6963_WR			(1 << 0)    // PK0
#define T6963_RD			(1 << 1)    // PK1
#define T6963_CE			(1 << 2)    // PK2
#define T6963_CD			(1 << 3)    // PK3
#define T6963_FS			(1 << 6)    // PK6
#define T6963_RESET         (1 << 7)    // PK7

// LCD2DATAPORT GPIO_PORTL_BASE

// parametry wy�wietlacza
#define GLCD_NUMBER_OF_LINES				64
#define GLCD_PIXELS_PER_LINE				240
#define GLCD_FONT_WIDTH						6

#define GLCD_GRAPHIC_AREA					(GLCD_PIXELS_PER_LINE / GLCD_FONT_WIDTH)
#define GLCD_TEXT_AREA						(GLCD_PIXELS_PER_LINE / GLCD_FONT_WIDTH)
#define GLCD_GRAPHIC_SIZE					(GLCD_GRAPHIC_AREA * GLCD_NUMBER_OF_LINES)
#define GLCD_TEXT_SIZE						(GLCD_TEXT_AREA * (GLCD_NUMBER_OF_LINES/8))

#define GLCD_TEXT_HOME						0
#define GLCD_GRAPHIC_HOME					(GLCD_TEXT_HOME + GLCD_TEXT_SIZE)
#define GLCD_OFFSET_REGISTER				2
#define GLCD_EXTERNAL_CG_HOME				(GLCD_OFFSET_REGISTER << 11)

#define T6963_SET_CURSOR_POINTER			0x21
#define T6963_SET_OFFSET_REGISTER			0x22
#define T6963_SET_ADDRESS_POINTER			0x24

#define T6963_SET_TEXT_HOME_ADDRESS			0x40
#define T6963_SET_TEXT_AREA					0x41
#define T6963_SET_GRAPHIC_HOME_ADDRESS		0x42
#define T6963_SET_GRAPHIC_AREA				0x43

#define T6963_MODE_SET						0x80

#define T6963_DISPLAY_MODE					0x90
#define T6963_CURSOR_BLINK_ON			0x01
#define T6963_CURSOR_DISPLAY_ON			0x02
#define T6963_TEXT_DISPLAY_ON			0x04
#define T6963_GRAPHIC_DISPLAY_ON		0x08

//#define T6963_
//#define T6963_

#define T6963_CURSOR_PATTERN_SELECT			0xA0
#define T6963_CURSOR_1_LINE				0x00
#define T6963_CURSOR_2_LINE				0x01
#define T6963_CURSOR_3_LINE				0x02
#define T6963_CURSOR_4_LINE				0x03
#define T6963_CURSOR_5_LINE				0x04
#define T6963_CURSOR_6_LINE				0x05
#define T6963_CURSOR_7_LINE				0x06
#define T6963_CURSOR_8_LINE				0x07

#define T6963_SET_DATA_AUTO_WRITE			0xB0
#define T6963_SET_DATA_AUTO_READ			0xB1
#define T6963_AUTO_RESET					0xB2

#define T6963_DATA_WRITE_AND_INCREMENT		0xC0
#define T6963_DATA_READ_AND_INCREMENT		0xC1
#define T6963_DATA_WRITE_AND_DECREMENT		0xC2
#define T6963_DATA_READ_AND_DECREMENT		0xC3
#define T6963_DATA_WRITE_AND_NONVARIALBE	0xC4
#define T6963_DATA_READ_AND_NONVARIABLE		0xC5

#define T6963_SCREEN_PEEK					0xE0
#define T6963_SCREEN_COPY					0xE8

void GLCD_Initalize_Interface(void);
int GLCD_Chceck_Status(void);
uint8_t GLCD_Write_Command(int command);
uint8_t GLCD_Write_Data(int data);
int GLCD_Read_Data(void);
void GLCD_Clear_Text(void);
void GLCD_Clear_CG(void);
void GLCD_Clear_Graphic(void);
void GLCD_Write_Char(char ch);
void GLCD_Write_String(char * str);
//void GLCD_Write_StringPgm(prog_char * str);
void GLCD_Text_GoTo(int x, int y);
void GLCD_Define_Character(int charCode, int * defChar);
void GLCD_Initalize(void);
void GLCD_SetPixel(int x, int y, int color);
void GLCDPrintf(uint8_t *str, float number);
void GLCDvPrintfNormal(const char *pcString, va_list vaArgP);
void GLCDPrintfNormal(uint8_t col, uint8_t row, const char *pcString, ...);
void GLCDWriteStringIndex(const char * str, uint32_t ui32Len);

