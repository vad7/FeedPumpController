typedef struct {
	//uint8_t year;	/* 2000 + 00..99 */
	//uint8_t	month;	/* 1..12 */
	//uint8_t	mday;	/* 1.. 31 */
	uint8_t	wday;	/* 1..7 */
	uint8_t	hour;	/* 0..23 */
	uint8_t	min;	/* 0..59 */
	uint8_t	sec;	/* 0..59, > 59 - init */ 
} RTC;

RTC rtc;

#define RTC_BUFFER_SIZE 4
uint8_t bufferRTC[RTC_BUFFER_SIZE];

uint8_t BCDToInt(uint8_t i)
{
	return (i & 0x0F) + (i >> 4) * 10;
}

// return 0 - ok, 1 - failed
uint8_t RTC_GetTime(void)
{
	if(I2C_Read_Block(I2C_RTC_ADDRESS, 0, RTC_BUFFER_SIZE, bufferRTC)) return 1;

	rtc.sec = BCDToInt(bufferRTC[0]);
	rtc.min = BCDToInt(bufferRTC[1]);
	rtc.hour = BCDToInt(bufferRTC[2]);
	rtc.wday = bufferRTC[3];
	//rtc.mday = (bufferRTC[4] & 0x0F) + (bufferRTC[4] >> 4) * 10;
	//rtc.month = (RTC_buf[5] & 0x0F) + ((RTC_buf[5] >> 4) & 1) * 10;
	//rtc.year = (RTC_buf[6] & 0x0F) + (RTC_buf[6] >> 4) * 10;
	return 0;
}

uint8_t IntToBCD(uint8_t i)
{
	return i / 10 * 16 + i % 10;
}
// return 0 - ok, 1 - failed
uint8_t RTC_SetTime(void)
{
	bufferRTC[0] = 0; //IntToBCD(rtc.sec);
	bufferRTC[1] = IntToBCD(rtc.min);
	bufferRTC[2] = IntToBCD(rtc.hour);
	bufferRTC[3] = IntToBCD(rtc.wday);
	//bufferRTC[4] = IntToBCD(rtc.mday);
	//bufferRTC[5] = IntToBCD(rtc.month);
	//RTC_buf[6] = IntToBCD(rtc.year);
	return I2C_Write_Block(I2C_RTC_ADDRESS, 0, RTC_BUFFER_SIZE, bufferRTC);
}

