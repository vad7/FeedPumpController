/*
 * LCD HD44780 Character 2x16 I2C supported
 *
 * Created: 19.02.2013 13:48:43
 *  Author: Vadim Kulakov (vad7@yahoo.com)
 */ 
#include <util/delay.h>
#include <avr/pgmspace.h>

//#define LCDCH_I2C_INTERFACE // LCD connected to I2C interface, if omitted - direct connection

#ifdef LCDCH_I2C_INTERFACE
#define LCDCH_I2C_Addr			0x4E	// 0x27 (7-bit)
#define LCDCH_DATA				0x01	// RS pin
//#define LCDCH_READ  			0x02	// RW pin, if omitted - write only mode with delays
#define LCDCH_E					0x04
#define LCDCH_LED				0x08	// if omitted - lighting is not used
#define LCDCH_DATASHIFT			4		// Starting pin for data 4..7
#define LCDCH_DATAMASK			(0x0F<<LCDCH_DATASHIFT)

uint8_t LCDCH_NotResponding		= 0;
#else

// Direct connected
// DATA PORT: D4..D7
#define LCDCH_DATAPORT			PORTB	// Data wire (4bit)
#define LCDCH_DATAPORTDDR		DDRB	// Direction
#define LCDCH_DATAPORTIN		PINB	// Read
#define LCDCH_DATASHIFT			2		// Starting pin for data 4..7
#define LCDCH_DATAMASK			(0x0F<<LCDCH_DATASHIFT)
// CONTROL PORT: RS,RW,E
#define LCDCH_CTRLPORT			PORTB
#define LCDCH_CTRLPORTDDR		DDRB	// Direction
#define LCDCH_DATA				(1<<PORTB6)	// RS pin, Selects registers (in sequence!)
#define LCDCH_E					(1<<PORTB7)	// Starts data read/write (in sequence!)
//#define LCDCH_READ			(1<<PORTDx)	// RW pin, Selects read or write (in sequence!), if omitted then don't used - always write (RW pin connect to ground!)
#endif

#define LCDCH_TIMESTROBE		1  // us
#define LCDCH_COMMAND			0
#define LCDCH_FLAGBUSY			0x80
#define LCDCH_2LINES			0b1000
#define LCDCH_SECOND_FONT		0b0100
#define LCDCH_RUS_FONT			0b0010

#define LCDCH__ClearDisplay		1
#define LCDCH__ReturnHome		2
#define LCDCH__EntryMode		4   
#define LCDCH__EntryModeShift	1	// Shift
#define LCDCH__EntryModeInc		2	// Increment by 1
#define LCDCH__Display			8
#define LCDCH__DisplayOn		4
#define LCDCH__DisplayCursorOn	2
#define LCDCH__DisplayCursorBlink 1
#define LCDCH__Shift			16  // shift display or cursor
#define LCDCH__ShiftDisplay		8   
#define LCDCH__ShiftCursor		0   
#define LCDCH__ShiftRight		4   
#define LCDCH__ShiftLeft		0   
#define LCDCH__SetCharDataPos	64	 // Low 0-5bits are address
#define LCDCH__SetCursorPos		128  // Low 0-6bits are address

void Delay50us(uint8_t ms)
{
	while(ms-- > 0) _delay_us(50);
}

#ifdef LCDCH_I2C_INTERFACE
#ifdef LCDCH_LED
uint8_t LCDCH_Light	= LCDCH_LED; // On
void LCDCH_I2C_Write(uint8_t data)
{
	I2C_Write(data | LCDCH_Light);
}
#else
#define LCDCH_I2C_Write(data) I2C_Write(data);
#endif
#endif

void LCDCH_Write4b(uint8_t data) // if 0x8X - Command
{
#ifdef LCDCH_I2C_INTERFACE
	if(I2C_Start(LCDCH_I2C_Addr + I2C_WRITE)) {
		LCDCH_NotResponding = 1;
		return;
	}
	LCDCH_I2C_Write(data); // Strobe up
	LCDCH_I2C_Write(data | LCDCH_E); // Strobe up
	LCDCH_I2C_Write(data); // Strobe down
	I2C_Stop();
#else
	LCDCH_DATAPORTDDR |= LCDCH_DATAMASK; // OUT
#ifdef LCDCH_READ
	LCDCH_CTRLPORT &= ~LCDCH_READ; // Write mode
#endif
	LCDCH_CTRLPORT = (LCDCH_CTRLPORT & ~(LCDCH_DATA | LCDCH_E)) | (data & LCDCH_DATA); // Set RS pin
	LCDCH_DATAPORT = (LCDCH_DATAPORT & ~LCDCH_DATAMASK) | (data & LCDCH_DATAMASK);
	LCDCH_CTRLPORT |= LCDCH_E; // Strobe up
	_delay_us(LCDCH_TIMESTROBE);
	LCDCH_CTRLPORT &= ~LCDCH_E; // Strobe down
#endif
}

#ifdef LCDCH_READ
#ifdef LCDCH_I2C_INTERFACE
uint8_t LCDCH_Read4b(uint8_t data) // LCDCH_DATA flag for data read otherwise command, hi nibble: +LCDCH_READ
{
	I2C_Start(LCDCH_I2C_Addr + I2C_WRITE);
	LCDCH_I2C_Write(data | LCDCH_DATAMASK | LCDCH_READ);
	LCDCH_I2C_Write(data | LCDCH_DATAMASK | LCDCH_READ | LCDCH_E); // Strobe up
	I2C_Stop();
	I2C_Start(LCDCH_I2C_Addr + I2C_READ);
	uint8_t readed = I2C_Read(I2C_NOACK) & LCDCH_DATAMASK;
	I2C_Stop();
	I2C_Start(LCDCH_I2C_Addr + I2C_WRITE);
	LCDCH_I2C_Write(data | LCDCH_READ);
	if((data & LCDCH_READ) == 0) LCDCH_I2C_Write(0); // Strobe down
	I2C_Stop();
	return readed;
}
#else
uint8_t LCDCH_Read4b(const uint8_t data) // Read data flag
{
	LCDCH_DATAPORTDDR &= ~LCDCH_DATAMASK; // IN
	LCDCH_CTRLPORT = (LCDCH_CTRLPORT & ~(LCDCH_DATA | LCDCH_E)) | LCDCH_READ | data; // Set RW, RS pin
	LCDCH_CTRLPORT |= LCDCH_E; // Strobe up
	_delay_us(LCDCH_TIMESTROBE);
	uint8_t readed = LCDCH_DATAPORTIN & LCDCH_DATAMASK;
	LCDCH_CTRLPORT &= ~LCDCH_E; // Strobe down
	if((data & LCDCH_READ) == 0) LCDCH_CTRLPORT &= ~LCDCH_READ;
	return readed;
}
#endif

uint8_t LCDCH_ReadByte(const uint8_t what) // command or data
{
	return (LCDCH_Read4b(what | LCDCH_READ) << (4 - LCDCH_DATASHIFT)) | (LCDCH_Read4b(what) >> LCDCH_DATASHIFT);
}
#endif

uint8_t LCDCH_WaitWhileBusy(void)
{
#ifdef LCDCH_READ
	uint8_t i = 0;
	while(LCDCH_ReadByte(LCDCH_COMMAND) & LCDCH_FLAGBUSY)
	{
		Delay50us(1);
		if((--i) == 0) return 1;
	}
	return 0;
#else
	Delay50us(1);
	return 0;
#endif
}

void LCDCH_WriteByte(const uint8_t data)
{
#ifndef DEBUGMODE
	if(LCDCH_WaitWhileBusy() == 0) {
		LCDCH_Write4b(((data & 0xF0) >> (4 - LCDCH_DATASHIFT)) | LCDCH_DATA);
		LCDCH_Write4b(((data & 0x0F) << LCDCH_DATASHIFT) | LCDCH_DATA);
	}
#endif
}

uint8_t LCDCH_WriteCommand(const uint8_t data)
{
#ifndef DEBUGMODE
	if(LCDCH_WaitWhileBusy()) return 1;
	LCDCH_Write4b((data & 0xF0) >> (4 - LCDCH_DATASHIFT));
	LCDCH_Write4b((data & 0x0F) << LCDCH_DATASHIFT);
#endif
	return 0;
}

void LCDCH_SetCursor(const uint8_t row, const uint8_t col)  // y = 1..4, x = 1..20
{
#ifndef DEBUGMODE
	//static int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	LCDCH_WaitWhileBusy();
	LCDCH_WriteCommand(LCDCH__SetCursorPos | ((row > 2 ? 0x14 : 0) + ((row-1) & 0x01) * 0x40 + col-1));
#endif
}

void LCDCH_LoadCharacterPGM(const uint8_t code, const uint8_t *chardata, uint8_t num_chars)
{
	LCDCH_WaitWhileBusy();
	LCDCH_WriteCommand(LCDCH__SetCharDataPos | (code * 8));
	for (register uint8_t i = 0; i < num_chars * 8; i++){
		LCDCH_WaitWhileBusy();
		LCDCH_WriteByte(pgm_read_byte(&chardata[i]));
	}
}

void LCDCH_ClearDisplay(void)
{
#ifndef DEBUGMODE
	LCDCH_WaitWhileBusy();
	LCDCH_WriteCommand(LCDCH__ClearDisplay);
	#ifndef LCDCH_RW
	Delay50us(33);
	#endif
#endif
}

uint8_t LCDCH_Init(const uint8_t setup)  // Return 0 if Ok, setup: 0b1*** - 2 line, 0b*1** - second font table
{
#ifndef DEBUGMODE
#ifdef LCDCH_I2C_INTERFACE
	LCDCH_NotResponding = 0;
	I2C_Init();
#else
	#ifdef LCDCH_READ
	LCDCH_CTRLPORTDDR |= LCDCH_DATA | LCDCH_E | LCDCH_READ; // Out
	#else
	LCDCH_CTRLPORTDDR |= LCDCH_DATA | LCDCH_E; // Out
	#endif
#endif
	Delay50us(210); Delay50us(210); // >21ms
	wdt_reset();
	LCDCH_Write4b(0b0011 << LCDCH_DATASHIFT);
#ifdef LCDCH_I2C_INTERFACE	
	if(LCDCH_NotResponding) return 2;
#endif
	Delay50us(82); // >4.1 ms
	LCDCH_Write4b(0b0011 << LCDCH_DATASHIFT);
	Delay50us(2); // 100us
	LCDCH_Write4b(0b0011 << LCDCH_DATASHIFT);
	Delay50us(2); // 100us
	LCDCH_Write4b(0b0010 << LCDCH_DATASHIFT); // 4 bit
	Delay50us(2); // 100us
	LCDCH_Write4b(0b0010 << LCDCH_DATASHIFT); // 4 bit
	LCDCH_Write4b((setup & 0x0F) << LCDCH_DATASHIFT);
	LCDCH_ClearDisplay();
	wdt_reset();
	if(LCDCH_WriteCommand(LCDCH__EntryMode | LCDCH__EntryModeInc)) return 1;
	if(LCDCH_WriteCommand(LCDCH__Display | LCDCH__DisplayOn)) return 1;
#endif
	return 0;
}

void LCDCH_WriteString(const char *str) // Print string from PGM at cursor pos
{
#ifndef DEBUGMODE
	char c;
	while ((c = *str++)) LCDCH_WriteByte(c);
#endif
}

void LCDCH_WriteStringPGM(const char *str) // Print string from PGM at cursor pos
{
#ifndef DEBUGMODE
	char c;
	while ((c = pgm_read_byte(str++))) LCDCH_WriteByte(c);
#endif
}

void LCDCH_WriteStringEEPROM(const char *str) // Print string from PGM at cursor pos
{
	char c;
	while ((c = eeprom_read_byte((uint8_t *)str++))) LCDCH_WriteByte(c);
}

char buffer[21]; // must be insufficient space!
// Write to buffer, number, decimal point position, number of character for formating (negative spacing -> leading '0'), 
//  if leading = '0' - negative number is not supported
char FormatNumberDelimiter = '.';
void FormatNumber(uint16_t num, const int8_t dec, int8_t spacing)
{
#ifndef DEBUGMODE
	char c = 0;
	uint8_t sign = 0;
	if (num < 0) {
		num = -num;
		sign = 1;
	}
	int8_t i = 0;
	do { // in reverse order
		buffer[i++] = (c = '0') + num % 10;
		if(i == dec) buffer[i++] = (c = FormatNumberDelimiter);
	} while ((num /= 10) > 0);
	while((i < dec + 1) || (dec && i == dec + 1)) {
		buffer[i++] = '0';
		if(i == dec) buffer[i++] = (c = FormatNumberDelimiter);
	}
	if(spacing < 0) {
		spacing = -spacing;
		c = '0';
	} else {
		c = ' ';
		if (sign) buffer[i++] = '-';
	}
	while(i < spacing) { 
		buffer[i++] = c;
	}
	buffer[i] = '\0';
	for (uint8_t j = i - 1, i = 0; i<j; i++, j--) { // reversing
		c = buffer[i];
		buffer[i] = buffer[j];
		buffer[j] = c;
	}
	//i = 0;
	//str[i++] = '0' + num / 10;
	//if(dec > 0) str[i++] = '.';
	//str[i++] = '0' + num % 10;
	//if(dec < 0) str[i++] = '.';
	//str[i] = '\0';
#else
	utoa(num, buffer, 10);
#endif
}

void RemoveNonNumericChars(char *buf) {
	uint8_t c, i = 0;
	while((c = buf[i])) {
		if((c >= '0' && c <= '9') || c == '-') {
			i++;
			continue;
		}
		uint8_t j = i;
		do {
			c = buf[j + 1];
			buf[j++] = c;
		} while(c);
	}
}

static const char ascii_HEX[] PROGMEM = { "0123456789ABCDEF" };

void FormatNumberHEX(const uint8_t num) {
		
	buffer[0] = pgm_read_byte(&ascii_HEX[(num & 0xF0) >> 4]);
	buffer[1] = pgm_read_byte(&ascii_HEX[num & 0x0F]);
	buffer[2] = 0;
}

