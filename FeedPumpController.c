/*
 * TempAlarm.c
 *
 * Created: 17.08.2015 16:18:58
 *  Author: Vadim Kulakov, vad7@yahoo.com
 *
 * ATtiny4313
 FUSES: BODLEVEL0,BODLEVEL01 (BOD: 4.3V), EESAVE, RSTDISBL(if Buzzer used), CKSEL3, CKSEL1, CKSEL0
 LOW = 0x64 
 HI = 0x99
 Ext = 0xFF
 */ 
//#define SETUPMODE 0				// 1 - For setup only firmware (OSCAL, Flow sensor)
//#define DEBUGMODE 1				// For debug in the Proteus
//#define DEBUGMODELCD 1			// For debug in the Proteus LCD test
#define CALIBRATE_MODE_WATER		// make setup item: 3 - Calibrate flow sensor
//#define CALIBRATE_MODE_OSCCAL		// make setup item: 4 - OSCCAL, p14 generating 62500Hz out
#define SWITCH_OUT3_EVERYDAY		// Discharge water - Turn on OUT3 (PORTD3) every day for 30 sec if there was no water consumption 
#define USE_BUZZER					// Buzzer when alarm on RESET pin
#define F_CPU 8000000UL 

#include <string.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/atomic.h>

#include "LCDHD44780.h"
#include "I2C.h"
#include "rtc.h"

#define MAX_DISCHARGE			200 // Liters, stop discharging water if exceeded

#define KEY_SET_PRESSING		!(PINB & (1<<PORTB1))
#define KEY_NEXT_PRESSING		!(PINB & (1<<PORTB0))
#define KEYS_INIT				PORTB = (1<<PORTB1) | (1<<PORTB0) // Pull-up

#define PORTSW					PORTD
#define PUMP					(1<<PORTD5)
#define PUMP_ON					PORTSW |= PUMP
#define PUMP_OFF				PORTSW &= ~PUMP
#define PUMP_ACTIVE				(PORTSW & PUMP)

#define OUT2					(1<<PORTD4)
#define OUT2_ON					PORTSW |= OUT2
#define OUT2_OFF				PORTSW &= ~OUT2
#define OUT2_ACTIVE				(PORTSW & OUT2)

#define OUT3					(1<<PORTD3)
#define OUT3_ON					PORTSW |= OUT3  // Discharge water
#define OUT3_OFF				PORTSW &= ~OUT3
#define OUT3_ACTIVE				(PORTSW & OUT3)

#define REGEN					(1<<PORTD0)
#define REGEN_START_ON			PORTSW |= REGEN
#define REGEN_START_OFF			PORTSW &= ~REGEN
#define REGEN_START_ACTIVE		(PORTSW & REGEN)

#define REGEN_IN				(1<<PORTD1)
#define REGEN_ACTIVE			!(PIND & REGEN_IN)

#define WASHING					(1<<PORTD6)
#define WASHING_ACTIVE			!(PIND & WASHING)

#ifdef USE_BUZZER
#define BUZZER					(1<<PORTA2)
#define BUZZER_ON				PORTA |= BUZZER
#define BUZZER_OFF				PORTA &= ~BUZZER
#define BUZZER_SWITCH			PORTA ^= BUZZER
#define BUZZER_INIT				DDRA |= BUZZER // Out
#else
#define BUZZER_ON
#define BUZZER_OFF
#define BUZZER_INIT
#endif 

#define EXTERNAL_INIT			DDRD = PUMP | OUT2 | OUT3 | REGEN; PORTD = WASHING | REGEN_IN // Out, pull-up

#define MAX_MENU_ITEM	14
#define MENU_ITEM_EDIT	6
#define SETUPCURSOR		8

uint8_t SetupPos = 0, SetupItem = 0, SetupSel;
uint8_t Timer0_1sec				= 0;
uint8_t Timer1SecTick			= 0;
volatile uint8_t TimerCnt		= 7;
uint16_t FlowCnt				= 0;
int16_t MinPumpOnTime			= 0;
register uint8_t DisplayNum	asm("10");	//= 0;
register uint8_t NeedSaveBytes asm("17"); //= 0;
uint8_t BadBattery				= 0;
uint8_t AlarmChar				= ' ';
#define UseOut2AsRegenValve		1 // 0 - Out2 active when consuming water, 1 - Regeneration valve (turn on while regeneration proceeded)

struct _used_water {
	uint16_t FlowImpulses;
	int16_t PumpOnTime;				// ms (0.001 sec)
	uint16_t UsedSinceLastRegen;	// Liters
	uint16_t UsedToday;				// Liters
	uint8_t NeedRegen;				// 0 - not, 1 - wait a regen hour, 2 - regen in process
	uint16_t RegCnt;
	uint16_t DaysFromLastRegen;
	uint16_t UsedYesterday;			// Liters
	uint8_t WeekDay; // 1-7 active wday
	uint16_t UsedAverageDay;		// Liters
	uint16_t UsedLastRegen;			// Liters
	uint16_t UsedTotal;				// m3 (*1000L)
	uint16_t UsedDischarge;			// Liters
	uint16_t UsedTotalBelow1000;	// L < 1000
} __attribute__ ((packed));
struct _used_water UsedWater;

struct _EEPROM {
	uint8_t _OSCCAL;
	uint16_t PulsesPerLiter;		// flow sensor sensitivity
	uint16_t MaxFlow;				// max impulses in sec / 10. = "(max flow in m3/h) / 3.6 * PulsesPerLiter / 10"
	uint16_t UsedBeforeRegen;		// liters (*0.001 m3)
	uint16_t MinPumpOnTime;			// *1ms (*0.001 sec)
	uint16_t RegenHour;
	uint16_t MinDischarge;			// Alarm value, Liters
	uint16_t MinRegen;				// Alarm value, Liters
	uint16_t DischargeTime;			// sec < 59 !!!
	uint16_t MaxDischarge;
	struct _used_water UsedWater;	// for every day backup
} __attribute__ ((packed));
struct _EEPROM EEMEM EEPROM;

#define EDIT_CHAR_SAVE 'o' // 0x01[<-']
#define EDIT_CHAR_CANCEL 'x'
char PEditChars[] = { '0','0','0','0','0', ' ', EDIT_CHAR_SAVE, ' ', EDIT_CHAR_CANCEL, 0 };
static const char scroll_used_label[] PROGMEM = { 'A', 'R', 'T', 'D' };

#ifndef DEBUGMODELCD
ISR(TIMER1_COMPA_vect) // 1/10 sec
{
	if(++Timer0_1sec == 10) { // 1 sec
		Timer0_1sec = 0;
		Timer1SecTick = 1;
		uint8_t n = TimerCnt;
		if(n) TimerCnt = --n;
		if(AlarmChar != ' ') BUZZER_SWITCH; else BUZZER_OFF;
	}
	if(PUMP_ACTIVE) {
		UsedWater.PumpOnTime -= 100; // 100ms
		if(UsedWater.PumpOnTime <= 0) {
			PUMP_OFF;
		}
		NeedSaveBytes = sizeof(UsedWater.FlowImpulses) + sizeof(UsedWater.PumpOnTime);
	} else if(UsedWater.PumpOnTime >= MinPumpOnTime && !WASHING_ACTIVE) {
		PUMP_ON;
	}
}
#endif
ISR(INT0_vect)
{
	if(++FlowCnt == 0) FlowCnt = 65535;
}

// void Delay10ms(uint8_t ms)
// {
// 	while(ms-- > 0) {
// 		_delay_ms(10);
// 		wdt_reset();
// 	}
// }

void WaitKeysRelease(void) // all keys released or pressed
{
	Delay50us(255);
	while(1) {
		wdt_reset();
		if(KEY_SET_PRESSING != KEY_NEXT_PRESSING); else break;
	} 
	Delay50us(255);
	Delay50us(255);
}

uint8_t WaitAKey(void)
{
	uint8_t i = 0;
	WaitKeysRelease();
	do {
		//sleep_cpu();
		wdt_reset();
		if(KEY_SET_PRESSING) {
			i = 1;
			break;
		} else if(KEY_NEXT_PRESSING) break; // any pressed
	} while(TimerCnt != 0);
	WaitKeysRelease();
	return i;
}

void ShowTime(void)
{
	FormatNumber(rtc.hour, 0, -2);
	LCDCH_WriteString(buffer);
	LCDCH_WriteByte(':');
	FormatNumber(rtc.min, 0, -2);
	LCDCH_WriteString(buffer);
	LCDCH_WriteByte(':');
	FormatNumber(rtc.sec, 0, -2);
	LCDCH_WriteString(buffer);
}

int main(void)
{
	CLKPR = (1<<CLKPCE); CLKPR = 0; // Clock prescaler division factor: 1
	EXTERNAL_INIT;
	KEYS_INIT;
	#ifdef USE_BUZZER
	BUZZER_INIT;
	#endif
	MCUCR = (1<<SE) | (1<<ISC00); // Idle sleep enable, INT0 any change int
	GIMSK = (1<<INT0); // Flow Sensor
	#ifndef DEBUGMODELCD
	uint8_t n = eeprom_read_byte(&EEPROM._OSCCAL);
	if(n == 0xFF) // EEPROM empty
	{
		//for(uint16_t i = 0; i < sizeof(EEPROM); i++) eeprom_update_byte((uint8_t *)&EEPROM + i, 0);
		eeprom_update_byte(&EEPROM._OSCCAL, OSCCAL);
		//eeprom_update_word(&EEPROM.PulsesPerLiter, 400);
		//eeprom_update_word((uint16_t *)&EEPROM.MinPumpOnTime, 1000);
		//eeprom_update_word(&EEPROM.MaxFlow, 60);
		//eeprom_update_word(&EEPROM.UseOut2AsRegenValve, 1);
		//eeprom_update_word(&EEPROM.UsedBeforeRegen, 7000);
		//eeprom_update_word(&EEPROM.RegenHour, 04);
		#ifdef DEBUGMODE
		I2C_Write_Block(I2C_RTC_ADDRESS, 8, sizeof(UsedWater), (uint8_t *)&UsedWater);
		#endif
	} else {
		OSCCAL = n;
	}
	MinPumpOnTime = eeprom_read_word((uint16_t *)&EEPROM.MinPumpOnTime);
	//UseOut2AsRegenValve = eeprom_read_byte((uint8_t *)&EEPROM.UseOut2AsRegenValve);
	/* Start 0.1 sec system timer (TC2.OC) */
	OCR1A = F_CPU / (10 * 256) - 1; //  F_CPU / (FREQ * Prescaller) - 1
	TIMSK = (1<<OCIE1A); // Output Compare A Match Interrupt
	TCCR1B = (1<<WGM12) | (1<<CS12) | (0<<CS11) | (0<<CS10); // // CTC mode, /256
	//
	GIFR = (1<<INTF0); // Clear flow sensor flag
	WDTCR = (1<<WDCE) | (1<<WDE); WDTCR = (1<<WDE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);	//  Watchdog Reset 2s
	#endif
	sei();
 	I2C_Init();
	#ifndef DEBUGMODE
	LCDCH_Init(LCDCH_2LINES);
	#ifndef DEBUGMODELCD
	//LCDCH_SetCursor(1,1);
 	LCDCH_WriteStringPGM(PSTR("Feed Pump Ctrl"));
 	LCDCH_SetCursor(2,1);
 	LCDCH_WriteStringPGM(PSTR("by Vadim Kulakov"));
 	LCDCH_SetCursor(3,1);
 	LCDCH_WriteStringPGM(PSTR("vad7@yahoo.com"));
 	LCDCH_SetCursor(4,1);
 	LCDCH_WriteStringPGM(PSTR("(c) 2016"));
	#endif
	#endif
	#ifndef DEBUGMODELCD
 	I2C_Read_Block(I2C_RTC_ADDRESS, 8, sizeof(UsedWater), (uint8_t *)&UsedWater); // restore struct from NVRAM
	RTC_GetTime();
	if(rtc.sec > 60) { // battery down or first power up
		eeprom_read_block(&UsedWater, &EEPROM.UsedWater, sizeof(UsedWater));
		RTC_SetTime();
		BadBattery = 1;
		if(UsedWater.WeekDay > 7) goto xResetData;
	}
	#endif
	while(1) {
		__asm__ volatile ("" ::: "memory"); // Need memory barrier
		wdt_reset();
		sleep_cpu();
		#ifndef DEBUGMODELCD
		if(Timer1SecTick) { // 1 sec passed
			Timer1SecTick = 0;
			typeof(FlowCnt) flow;
			// Calc flow in 0.0001 m3
			ATOMIC_BLOCK(ATOMIC_FORCEON) { 
				flow = FlowCnt;
				FlowCnt = 0;
			}
			if(UseOut2AsRegenValve) {
				if(REGEN_ACTIVE) {
					OUT2_ON;
				} else {
					OUT2_OFF;
				}
			}
			uint16_t ppl = eeprom_read_word(&EEPROM.PulsesPerLiter);
//				// if ppl is not divisible by 100 without remainder: 
//  			uint16_t tmp = ppl / 10;
//  			if(flow > 180) {
//  				tmp = flow * 36 / tmp * 10;
//  			} else {
//  				tmp = flow * 360 / tmp;
//  			}
			uint16_t tmp = flow * 36 / (ppl / 100); // ppl: 100,200,300,400,500,etc
			FormatNumber(tmp, 3, 6); // m3/h, if ppl = 100/200/300/400/etc: flow * 36 / (ppl / 100)
			if(flow) {
				if(!UseOut2AsRegenValve) OUT2_ON;
				UsedWater.FlowImpulses += flow;
				if(!WASHING_ACTIVE) {
					uint16_t tmp_mf = eeprom_read_word(&EEPROM.MaxFlow);
					tmp = flow * 100 / tmp_mf; // ms
					// if next line omitted Max = 655 impulses per sec (655/400*3.6 = 5.895 m3/h)
					//if(flow > 655) tmp += (uint16_t) 65535 / tmp_mf;	// Max = 1310 impulses per sec
					if(tmp > 1000) tmp = 1000;							// limit pump on time
					ATOMIC_BLOCK(ATOMIC_FORCEON) UsedWater.PumpOnTime += tmp;
					//asm("LDI R11, 4");
					NeedSaveBytes = sizeof(UsedWater.FlowImpulses) + sizeof(UsedWater.PumpOnTime);
				}
				// inc Liters
xCalcFlowImp:	if(UsedWater.FlowImpulses >= ppl) {
					UsedWater.FlowImpulses -= ppl;
					if(REGEN_ACTIVE || WASHING_ACTIVE) {
						UsedWater.UsedLastRegen++;
					} else {
						UsedWater.UsedToday++;
						UsedWater.UsedSinceLastRegen++;
						if(++UsedWater.UsedTotalBelow1000 >= 1000) {
							UsedWater.UsedTotalBelow1000 = 0;
							UsedWater.UsedTotal++;
						}
						if(UsedWater.NeedRegen == 0 && UsedWater.UsedSinceLastRegen > eeprom_read_word(&EEPROM.UsedBeforeRegen)) {
							UsedWater.NeedRegen = 1;
						}
						NeedSaveBytes = sizeof(UsedWater.FlowImpulses) + sizeof(UsedWater.PumpOnTime) + sizeof(UsedWater.UsedSinceLastRegen) + sizeof(UsedWater.UsedToday) + sizeof(UsedWater.NeedRegen);
					}
					goto xCalcFlowImp;
				}
			} else {
				if(!UseOut2AsRegenValve) OUT2_OFF;
			}
			RTC_GetTime();
			if(WASHING_ACTIVE || REGEN_ACTIVE) {
				REGEN_START_OFF;
				if(UsedWater.NeedRegen != 2) {
					UsedWater.NeedRegen = 2;
					UsedWater.UsedLastRegen = 0;
					PUMP_OFF;
					goto xUpdateNVRAM_All;
				}
			} else {
				if(UsedWater.NeedRegen == 2) {
					UsedWater.RegCnt++;
					UsedWater.UsedSinceLastRegen = 0;
					UsedWater.DaysFromLastRegen = 0;
					if(UsedWater.UsedLastRegen < eeprom_read_word(&EEPROM.MinRegen)) {
						AlarmChar = '#';
					}
					UsedWater.NeedRegen = 0;
					goto xUpdateNVRAM_All;
				} else if(UsedWater.NeedRegen && eeprom_read_byte((uint8_t *)&EEPROM.RegenHour) == rtc.hour) {
					REGEN_START_ON;
				}
			}
			if(OUT3_ACTIVE) {
				if(rtc.sec >= eeprom_read_byte((uint8_t *)&EEPROM.DischargeTime) || UsedWater.UsedToday > eeprom_read_word(&EEPROM.MaxDischarge)) {
					OUT3_OFF;
					UsedWater.UsedDischarge = UsedWater.UsedToday;
					if(UsedWater.UsedDischarge < eeprom_read_word(&EEPROM.MinDischarge)) {
						AlarmChar = 0xFF; // '?'
					}
					UsedWater.UsedToday = 0;
					goto xUpdateNVRAM_All;
				}
			}
			if(rtc.wday != UsedWater.WeekDay) { // Next day
				UsedWater.UsedYesterday = UsedWater.UsedToday;
				if(UsedWater.UsedToday > 10) { // liters, not empty
					UsedWater.UsedAverageDay = (UsedWater.UsedAverageDay + UsedWater.UsedToday) / 2;
				#ifdef SWITCH_OUT3_EVERYDAY
				} else {
					UsedWater.UsedDischarge = 0;
					OUT3_ON;
				#endif
				}
				UsedWater.UsedToday = 0;
				UsedWater.DaysFromLastRegen++;
				UsedWater.WeekDay = rtc.wday;
				if(rtc.wday == 7) {
xSaveToEEPROM:		eeprom_update_block(&EEPROM.UsedWater, &UsedWater, sizeof(UsedWater));
				}
xUpdateNVRAM_All: 
				NeedSaveBytes = sizeof(UsedWater);
			}
			if(NeedSaveBytes) {
				I2C_Write_Block(I2C_RTC_ADDRESS, 8, NeedSaveBytes, (uint8_t *)&UsedWater);
				NeedSaveBytes = 0;
			}
			if(TimerCnt <= 1) { 
				#ifndef DEBUGMODE
				if(SetupPos) {
					if(SetupItem == 4) { // Save OSCCAL value
						TCCR0A = 0;
						TCCR0B = 0; // stop timer						
						eeprom_update_byte(&EEPROM._OSCCAL, OSCCAL);
					}
					goto xSetupExit; // exit setup by timeout
				}
				// Display:
				// 12345678901234567890
				//  0.000 m3/h*>< 0.000
				// Regs: 0 Days:  4
				// Day: 0.000 Yd: 0.000 
				// 1.00:00:00  A: 0.000 
	 			if((rtc.sec & 3) == 0) { // next info
xNextDisplay:		if(++DisplayNum >= 4) DisplayNum = 0;
	 			}
				if(TimerCnt == 1) LCDCH_ClearDisplay();
	 			LCDCH_SetCursor(1,1);
	 			LCDCH_WriteString(buffer); // Print previous filled value  FormatNumber!!!
	 			LCDCH_WriteStringPGM(PSTR(" m3/h"));
				LCDCH_WriteByte(REGEN_ACTIVE ? 0x7E : (UsedWater.NeedRegen == 1 ? '*' : ' '));
				LCDCH_WriteByte(0x7E); // '->'
				LCDCH_WriteByte(WASHING_ACTIVE ? 0x7F : ' '); // <-
	 			FormatNumber(REGEN_ACTIVE || WASHING_ACTIVE ? UsedWater.UsedLastRegen : UsedWater.UsedSinceLastRegen, 3, 6);
	 			LCDCH_WriteString(buffer);
	 			LCDCH_SetCursor(2,1);
	 			LCDCH_WriteStringPGM(PSTR("Regs: "));
	 			FormatNumber(UsedWater.RegCnt, 0, 0);
	 			LCDCH_WriteString(buffer);
	 			LCDCH_WriteStringPGM(PSTR(" Days:"));
	 			FormatNumber(UsedWater.DaysFromLastRegen, 0, 3);
	 			LCDCH_WriteString(buffer);
	 			LCDCH_SetCursor(3,1);
	 			LCDCH_WriteStringPGM(PSTR("Day:"));
	 			FormatNumber(UsedWater.UsedToday, 3, 6);
	 			LCDCH_WriteString(buffer);
	 			LCDCH_WriteStringPGM(PSTR(" Yd:"));
	 			FormatNumber(UsedWater.UsedYesterday, 3, 6);
	 			LCDCH_WriteString(buffer);
	 			LCDCH_SetCursor(4,1);
	 			LCDCH_WriteByte('0' + UsedWater.WeekDay);
	 			LCDCH_WriteByte('.');
	 			ShowTime();
				LCDCH_WriteByte(BadBattery ? '!' : ' ');
				LCDCH_WriteByte(AlarmChar); // ' ' or 'alarm char'
				LCDCH_WriteByte(pgm_read_byte(&scroll_used_label[DisplayNum])); //DisplayWeekDay <= 6 ? '1' + DisplayWeekDay : 0x7E);
		 		LCDCH_WriteByte(':');
		 		FormatNumber(((uint16_t *)&UsedWater.UsedAverageDay)[DisplayNum], DisplayNum == 2 ? 0 : 3, 6);
		 		LCDCH_WriteString(buffer);
				#endif
			}
		}
		#endif //		*/

		#ifndef DEBUGMODE
		if(KEY_SET_PRESSING) {
			uint16_t *epos = &EEPROM.PulsesPerLiter + (SetupItem - MENU_ITEM_EDIT); // Start menu item for edit EEPROM
			if(SetupPos == 1) {	// Menu item	
				SetupPos = 2;
				LCDCH_SetCursor(2,1);
				switch(SetupItem) {
				case 1: // Turn On or Off pump & out2
					cli();
					uint8_t pos = 1;
					while(1) {
						LCDCH_SetCursor(2,1);
						LCDCH_WriteByte(PUMP_ACTIVE ? 'P' : 'p');	// 1
						LCDCH_WriteByte(OUT2_ACTIVE ? 'R' : 'r');	// 2
						LCDCH_WriteByte(OUT3_ACTIVE ? 'D' : 'd');	// 3
						LCDCH_WriteByte('x');						// 4
						LCDCH_SetCursor(2, pos);
						if(WaitAKey()) { // SET pressed
							if(pos == 4) break; // exit
							PORTSW ^= pos == 1 ? PUMP : pos == 2 ? OUT2 : OUT3;
						} else {
							pos = 1 + (pos & 3); //if(++pos > 4) pos = 1;
						}
					}
					sei();
					break;
				case 2: // Set time
					goto xShowSetupMenuItem_1;
				case 3: // Calibrate flow sensor
#ifdef CALIBRATE_MODE_WATER
					TimerCnt = 240;
					FlowCnt = 0;
					do {
						__asm__ volatile ("" ::: "memory"); // Need memory barrier
						sleep_cpu();
						wdt_reset();
						if(KEY_NEXT_PRESSING) break;
						LCDCH_SetCursor(2,1);
						LCDCH_WriteStringPGM(PSTR("Pulses: "));
						typeof(FlowCnt) f;
						ATOMIC_BLOCK(ATOMIC_FORCEON) f = FlowCnt;
						FormatNumber(f, 0, 0);
						LCDCH_WriteString(buffer);
					} while(TimerCnt);
					LCDCH_WriteByte(0x7F); // '<-'
					TimerCnt = 255;
					WaitAKey();
#endif					
					break;
				case 4:	// Edit OSCCAL
#ifdef CALIBRATE_MODE_OSCCAL
					LCDCH_WriteStringPGM(PSTR("p14\x7E""62500"));
					TCCR0A = (1<<COM0A0) | (1<<WGM01) | (1<<WGM00);
					TCCR0B = (1<<WGM02) | (0<<CS02) | (1<<CS01) | (1<<CS00); //  // fclk/2 = 62500, Fast PWM, Toggle, clk/64
					goto xSetTimerCnt_WaitKeysRelease;
#else			
					break;					
#endif
				case 5: // Clear memory
					#ifndef DEBUGMODELCD
					LCDCH_WriteStringPGM(PSTR("RESET?"));
					TimerCnt = 15;
					WaitKeysRelease();
					if(TimerCnt == 0) {
xResetData:				memset(&UsedWater, 0, sizeof(UsedWater));
						goto xUpdateNVRAM_All;
					}
					goto xSaveToEEPROM;
					#endif
				default: {// 6..11 Set EEPROM vars. 00000 o x
					FormatNumber(eeprom_read_word(epos), 0, -5);
					memcpy(PEditChars, buffer, 5);
					LCDCH_WriteString(PEditChars);
					SetupSel = 9;
					goto xSetCursor;
				}
				}
xSetupExit:		
				SetupPos = 0;
				SetupItem = 0;
				TimerCnt = 0;
				LCDCH_WriteCommand(LCDCH__Display | LCDCH__DisplayOn);
				LCDCH_ClearDisplay();
				goto xWaitKeysRelease;
			} else if(SetupItem == 2) { // SetupItem = 2 (time)
				if(SetupSel == SETUPCURSOR) { // day of week
					if(++rtc.wday > 7) rtc.wday = 1;
				} else if(SetupSel == SETUPCURSOR + 3) {
					if(++rtc.hour == 24) rtc.hour = 0;
				} else {
					if(++rtc.min == 60) rtc.min = 0;
				}
				#ifndef DEBUGMODELCD
				RTC_SetTime();
				#endif
				goto xShowSetupMenuItem_1;
#ifdef CALIBRATE_MODE_OSCCAL
			} else if(SetupItem == 4) {
				OSCCAL--;
#endif
			} else if(SetupItem) { // Edit EEPROM var
				if(PEditChars[SetupSel-1] == EDIT_CHAR_SAVE) {
					eeprom_update_word(epos, atoi(PEditChars));
					goto xSetupExit;
				} else if(PEditChars[SetupSel-1] == EDIT_CHAR_CANCEL) {
					goto xSetupExit;
				} else { // Change digit
					PEditChars[SetupSel-1]++;
					if(PEditChars[SetupSel-1] > '9' || PEditChars[SetupSel-1] < '0') PEditChars[SetupSel-1] = '0';
					LCDCH_WriteByte(PEditChars[SetupSel-1]);
				}
				goto xSetCursor;
			} else { // Enter setup
				SetupPos = 1;
				SetupItem = 1;
				SetupSel = SETUPCURSOR;
xShowSetupMenu:	LCDCH_ClearDisplay();	// 12345678901234567890
				LCDCH_WriteStringPGM(PSTR("Setup:"));
				FormatNumber(SetupItem, 0, 2);
				LCDCH_WriteString(buffer);
xSetCursor:		LCDCH_WriteCommand(LCDCH__Display | LCDCH__DisplayOn | LCDCH__DisplayCursorOn | LCDCH__DisplayCursorBlink);
				LCDCH_SetCursor(SetupPos, SetupSel);
			}
xSetTimerCnt_WaitKeysRelease:			
			TimerCnt = 30;
xWaitKeysRelease:
			WaitKeysRelease();
		} else if(KEY_NEXT_PRESSING) {
			if(SetupPos == 1) {
				if(++SetupItem > MAX_MENU_ITEM) SetupItem = 1; // Max menu item
				goto xShowSetupMenu;
			} else if(SetupItem == 2) { // SetupItem = 2 (time)
				SetupSel += 3; // 8, 11, 14
				if(SetupSel > 14) SetupSel = SETUPCURSOR;
xShowSetupMenuItem_1:
				LCDCH_SetCursor(2,1);
				LCDCH_WriteStringPGM(PSTR("Time: W"));
				LCDCH_WriteByte('0' + rtc.wday);
				LCDCH_WriteByte(',');
				ShowTime();
				goto xSetCursor;
#ifdef CALIBRATE_MODE_OSCCAL
			} else if(SetupItem == 4) {
				OSCCAL++;
#endif
			} else if(SetupItem) { // Edit EEPROM var
				while(1) {
					if(++SetupSel > 9) SetupSel = 1;
					if(PEditChars[SetupSel-1] == EDIT_CHAR_SAVE || PEditChars[SetupSel-1] == EDIT_CHAR_CANCEL || (PEditChars[SetupSel-1] <= '9' && PEditChars[SetupSel-1] >= '0')) break; // skip non numeric & o/x
				}
				goto xSetCursor;
			} else {
				AlarmChar = ' ';
				WaitKeysRelease();
				TimerCnt = 1;
				#ifndef DEBUGMODELCD
				goto xNextDisplay; // in the "m3/h" field will be displayed last buffer value (history) !!!
				#endif
			}
			goto xSetTimerCnt_WaitKeysRelease;
		}
		#endif
	}
}

/*
На экране:
Строки:
1: текущее потребление в м3/ч -> потребленное после последней регенерации в м3;
   * - будет регенерация, -> - идет регенерация, <- - идет обратная промывка
2: Regs - Количество регенераций; Days - дней после регенерации
3. Day - потребленное за день; Yd - потребленное вчера
4. День недели; Текущее время; Тревога: "!" - села батарейка, 
     "?" - низкий проток; "#" - малый расход на регенерацию;
   Израсходовано в литрах:
   "A:" - среднее за день (если за день 0, то не учитывается), 
   "R:" - за последнюю регенерацию,
   "T:" - итого в кубометрах, 
   "D:" - за последний пролив воды, если за день было < 10л.

Настройка, нажать кнопку SET (выход через 30 сек, кроме п.1,3)
1. Вкл./Выкл. (NEXT - выбор выхода, SET - смена состояния)
   P - Насос, R - OUT2 (во время регенерации), D - OUT3 (пролив)
2. Установка времени (W - номер дня недели 1..7)
3. Счетчик импульсов датчика за 4 минуты, NEXT - выход
4. Подстройка частоты (на выводе 14 (LCD 11) - 62500 Hz) (SET: -, NEXT: +)
5. Сброс счетчиков (нажать на SET более 15 секунд)
6. Установка числа импульсов на литр
7. Максимальный поток = (макс. в m3/ч) / 3.6 * (число импульсов на литр) / 10
   насос включается пропорционально текущему расходу, но не выше макс.
   Чем меньше значение - тем больше дозируется
8. Ресурс в литрах для запуска регенерации в определенный час
9. Минимальное время включения дозирующего насоса в мс (0.001 с)
10. Час регенерации
11. Минимальное кол-во литров во время пролива воды (меньше - авария)
12. Минимальное кол-во литров на регенерацию (меньше - авария)
13. Время пролива воды в сек, не больше 58!
14. Максимальный расход воды на пролив в литрах

*/


/*const uint8_t DefCharacters[] PROGMEM = {	
	0b00100, // Drop
	0b01100,
	0b11110,
	0b11111,
	0b10111,
	0b11111,
	0b01110,
	0,
	0b00001, // Enter (<-')
	0b00101,
	0b01001,
	0b11111,
	0b01000,
	0b00100,
	0b00000,
	0,
	0b01110, // ^3
	0b00010,
	0b00110,
	0b00010,
	0b01110,
	0b00000,
	0b00000,
0 };*/
