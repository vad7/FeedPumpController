/*
 * I2C.c
 *
 * Created: 17.01.2014 14:00:10
 *  Author: Vadim Kulakov, vad7@yahoo.com
 */

//#define IC2_MULTI_MASTER	// If other master(s) available, if omitted - only ONE master on I2C bus

#define I2C_PORT			PORTA
#define I2C_DDR				DDRA
#define I2C_IN				PINA
#define I2C_SCL				(1<<PORTA0)
#define I2C_SDA				(1<<PORTA1)

#define I2C_RTC_ADDRESS		0xD0
//#define I2C_ADDR_EEPROM		0xA0 // OMIT if I2C EEPROM memory is not used

#define I2C_DELAY_US		5 // 4.7us - 100kb, 1.3us - 400kb
#define I2C_DELAY_US_LOW	I2C_DELAY_US - (16 / (F_CPU / 1000000)) // compensate instruction between bits

#define I2C_WRITE			0
#define I2C_READ			1
#define I2C_NOACK			1
#define I2C_ACK				0

void I2C_Delay(void) {
	_delay_us(I2C_DELAY_US);
}

void I2C_DelayLow(void) {
	_delay_us(I2C_DELAY_US_LOW);
}

void I2C_Stop(void)
{
	I2C_DDR |= I2C_SDA; // set LOW
	I2C_Delay();
	I2C_DDR &= ~I2C_SCL; // release
	//while((I2C_IN & I2C_SCL) == 0) ;
	I2C_Delay();
	I2C_DDR &= ~I2C_SDA; // release
	I2C_Delay();
}

void I2C_Init(void)
{
	I2C_DDR &= ~I2C_SCL; // in, release
	I2C_PORT &= ~I2C_SCL; // 0
	I2C_DDR &= ~I2C_SDA; // in, release
	I2C_PORT &= ~I2C_SDA; // 0
#ifndef IC2_MULTI_MASTER
	I2C_Delay();
	// Start
	if((I2C_IN & I2C_SDA) == 0) { // some problem here
		I2C_DDR |= I2C_SDA; // set LOW
		I2C_Delay();
		I2C_DDR |= I2C_SCL; // set low
		I2C_Stop();
	}
	//
#endif
}

// return: 1 - if write failed, 0 - ok
uint8_t I2C_WriteBit(uint8_t bit)
{
	//while(I2C_IN & I2C_SCL) ;
	if(bit)
		I2C_DDR &= ~I2C_SDA; // release
	else
		I2C_DDR |= I2C_SDA; // set low
	I2C_DelayLow();
	I2C_DDR &= ~I2C_SCL; // release
	//while((I2C_IN & I2C_SCL) == 0) ;
	I2C_Delay();
	#ifdef IC2_MULTI_MASTER
		if(bit && (I2C_IN & I2C_SDA) == 0) return 1; // other master active
	#endif
	I2C_DDR |= I2C_SCL; // set low
	return 0;
}

uint8_t I2C_ReadBit(void)
{
	I2C_DDR &= ~I2C_SDA; // release
	I2C_DelayLow();
	I2C_DDR &= ~I2C_SCL; // release
	//while((I2C_IN & I2C_SCL) == 0) ;
	I2C_Delay();
	uint8_t bit = (I2C_IN & I2C_SDA) != 0;
	I2C_DDR |= I2C_SCL; // set low
	return bit;
}

// return: 1 - if write failed, 0 - ok
uint8_t I2C_Write(uint8_t data)
{
	for(uint8_t i = 0; i < 8; i++)
	{
		#ifdef IC2_MULTI_MASTER
			if(I2C_WriteBit(data & 0x80)) return 2;
		#else
			I2C_WriteBit(data & 0x80);
		#endif
		data <<= 1;
	}
	return I2C_ReadBit();
}

uint8_t I2C_Read(uint8_t ack)
{
	uint8_t data = 0;
	for(uint8_t i = 0; i < 8; i++)
	{
		data = (data << 1) | I2C_ReadBit();
	}
	I2C_WriteBit(ack);
	return data;
}

// Return: 1 - failed, 0 - ok, 
uint8_t I2C_Start(uint8_t addr)
{
	for(uint8_t i = 1; i != 0; i++)
	{
		// Restart
		I2C_DDR &= ~I2C_SDA; // release
		I2C_Delay();
		I2C_DDR &= ~I2C_SCL; // release
//		do {
//			I2C_Delay();
//		} while((I2C_IN & I2C_SCL) == 0);
		I2C_Delay();
		if((I2C_IN & I2C_SDA) == 0) {
			continue; // other master active
		}
		I2C_DDR |= I2C_SDA; // set LOW
		I2C_Delay();
		I2C_DDR |= I2C_SCL; // set low
		I2C_Delay();
		if(I2C_Write(addr) == 0) return 0;
		I2C_Stop();
	}
	return 1;
}

// return 0 - ok
#ifndef I2C_ADDR_EEPROM
uint8_t I2C_Read_Block(uint8_t addr, uint8_t pos, uint8_t cnt, uint8_t *buffer)
{
	if(I2C_Start(addr + I2C_WRITE) == 0) {
		if (I2C_Write(pos) == 0) {	// set memory pos
			if(I2C_Start(addr + I2C_READ) == 0) {
				do {
					cnt--;
					*buffer++ = I2C_Read(cnt ? I2C_ACK : I2C_NOACK);
				} while(cnt);
			}
		}
	}
	I2C_Stop();
	return cnt;
}
// return 0 - ok
uint8_t I2C_Write_Block(uint8_t addr, uint8_t pos, uint8_t cnt, uint8_t *buffer)
{
	if(I2C_Start(addr + I2C_WRITE) == 0) {
		if (I2C_Write(pos) == 0) {	// memory pos
			for(; cnt; cnt--) {
				if(I2C_Write(*buffer++)) break;
			}
		}
	}
	I2C_Stop();
	return cnt;
}
#else

// return 0 - ok
uint8_t I2C_Read_Block(uint8_t addr, uint16_t pos, uint8_t cnt, uint8_t *buffer)
{
	if(I2C_Start(addr + I2C_WRITE) == 0) {
		if ((addr != I2C_ADDR_EEPROM || (addr == I2C_ADDR_EEPROM && I2C_Write(pos / 256) == 0)) && I2C_Write(pos & 255) == 0) {
			if(I2C_Start(addr + I2C_READ) == 0) {
				do {
					cnt--;
					*buffer++ = I2C_Read(cnt ? I2C_ACK : I2C_NOACK);
				} while(cnt);
			}
		}
	}
	I2C_Stop();
	return cnt;
}
// return 0 - ok
// EEPROM write 1 byte or page size bytes
uint8_t I2C_Write_Block(uint8_t addr, uint16_t pos, uint8_t cnt, uint8_t *buffer)
{
	if(I2C_Start(addr + I2C_WRITE) == 0) {
		if ((addr != I2C_ADDR_EEPROM || (addr == I2C_ADDR_EEPROM && I2C_Write(pos / 256) == 0)) && I2C_Write(pos & 255) == 0) {
			for(; cnt; cnt--) {
				if(I2C_Write(*buffer++)) break;
			}
		}
	}
	I2C_Stop();
	return cnt;
}
#endif
