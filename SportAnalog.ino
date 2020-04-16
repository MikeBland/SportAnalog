/* ============================================================
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * ============================================================*/

// Author: Mike Blandford

// Link bit IO8
// ID bit IO3
// Sport bit IO2


#define DEBUG		0

#define DEBUG_BIT		0x10
#define DEBUG_PORT	PORTB
#define DEBUG_DDR		DDRB
#define DEBUG_PIN		PINB

#define DEBUGA_BIT		0x08


#define SENSOR_ID		0x1B
#define A1_ID       0xF102
#define A2_ID       0xF103
#define A3_ID				0x0900
#define A4_ID 			0x0910

// A3/A4 return U32 as V/100
// 3.3V = 1024
// ADC*33/8*10/128

#define TEMP2_ID			5

#define SENSOR_CONFIG_ID		0x1C


// For 8MHz CPU
//#define RXCENTRE		19
#define RXINTRA			43
#define RXSTOP			43
#define TXDELAY			41

// Clocks at 8MHz for 1 bit at 57600 baud
#define BITRATE			139
#define RXCENTRE		(69-20)


#define XMIT_START_ADJUSTMENT 3

#define IDLEDELAYTIME	208

// Link for scaling option is IO8
#define LINK_BIT		0x01
#define LINK_PORT		PORTB
#define LINK_DDR		DDRB
#define LINK_PIN		PINB

// ID option bit is IO3
#define ID_BIT		0x08
#define ID_PORT		PORTD
#define ID_DDR		DDRD
#define ID_PIN		PIND

// A3 option bit is IO4
#define A3_BIT		0x10
#define A3_PORT		PORTD
#define A3_DDR		DDRD
#define A3_PIN		PIND

// A4 option bit is IO5
#define A4_BIT		0x20
#define A4_PORT		PORTD
#define A4_DDR		DDRD
#define A4_PIN		PIND

// SPORT_BIT is the one with INT0 available (IO2)
// Unless PINCHANGE is defined

// This is IO2
#define SPORT_BIT		0x04
#define SPORT_PORT	PORTD
#define SPORT_DDR		DDRD
#define SPORT_PIN		PIND


#define FORCE_INDIRECT(ptr) __asm__ __volatile__ ("" : "=e" (ptr) : "0" (ptr))

typedef uint8_t   prog_uint8_t  __attribute__((__progmem__));//,deprecated("prog_uint8_t type is deprecated.")));
#define APM __attribute__(( section(".progmem.data") ))

const prog_uint8_t APM Indices[] = {	0x00, 0xA1, 0x22, 0x83, 0xE4, 0x45,
																			0xC6, 0x67, 0x48, 0xE9, 0x6A, 0xCB,
                                      0xAC, 0x0D, 0x8E, 0x2F, 0xD0, 0x71,
                                      0xF2, 0x53, 0x34, 0x95, 0x16, 0xB7,
																			0x98, 0x39, 0xBA, 0x1B } ;



struct t_anacontrol
{
	uint16_t Analog ;
	uint16_t AnaAve ;
	uint8_t AnaCount ;
} AnalogControl[4]  ;

uint8_t SensorId ;

uint8_t Crc ;
uint8_t Timer12ms ;
uint8_t SendStart ;
uint8_t Idle ;

uint8_t ReceivedAppId[2] ;

uint8_t Send32packet ;
uint8_t ReceivedPacket[8] ;

uint16_t MillisPrecount ;
uint16_t lastTimerValue ;
uint32_t TotalMicros ;
uint32_t TotalMillis ;



void init()
{
//	initSportUart() ;
  
	
	// Timer1
  TCCR1A = 0x00 ;    //Init.
  TCCR1B = 0xC1 ;    // I/p noise cancel, rising edge, Clock/1

	EICRA = 3 ;					// INT0 rising edge generates interrupt

	init_from_eeprom() ;

	ADCSRA = 0x86 ;
	PORTC |= 2 ;		// Pull up on second analog i/p

	DIDR0 |= 0x0D ;	// Disable digital pins 3,2,0

	ID_DDR &= ~ID_BIT ;		// Input
	ID_PORT |= ID_BIT ;		// pullup

	LINK_DDR &= ~LINK_BIT ;	//Input
	LINK_PORT |= LINK_BIT ;	// pullup

	A3_DDR &= ~A3_BIT ;		// Input
	A3_PORT |= A3_BIT ;		// pullup

	A4_DDR &= ~A4_BIT ;		// Input
	A4_PORT |= A4_BIT ;		// pullup


#if DEBUG
	DEBUG_DDR |= DEBUG_BIT | DEBUGA_BIT ;
	DEBUG_PORT &= ~DEBUG_BIT & ~DEBUGA_BIT ;
#endif
	 
}

void setup()
{
	
}

static uint8_t rx_pin_read()
{
	return SPORT_PIN & SPORT_BIT ;
}

inline void setTX()
{
	SPORT_DDR |= SPORT_BIT ;
	SPORT_PORT &= ~SPORT_BIT ;		// low as inverse logic
}

inline void setRX()
{
	SPORT_DDR &= ~SPORT_BIT ;
	SPORT_PORT &= ~SPORT_BIT ;		// low so no pullup
}

void waitCompA()
{
	while ( ( TIFR1 & (1 << OCF1A) ) == 0 )
	{
		// null body
	}
	TIFR1 = (1 << OCF1A) ; 		// Clear flag
}

void swrite(uint8_t b)
{
	uint16_t lBitRate = BITRATE ;
	// Write the start bit
	OCR1A = TCNT1 + lBitRate ;
	TIFR1 = (1 << OCF1A) ; 		// Clear flag
	SPORT_PORT |= SPORT_BIT ;		// high for start bit as inverse logic
	waitCompA() ;
  // Write each of the 8 bits
    for ( uint8_t i = 8 ; i ; i -= 1 )
    {
      if (b & 1) // test bit
				SPORT_PORT &= ~SPORT_BIT ;		// send 1
      else
				SPORT_PORT |= SPORT_BIT ;			// send 0
			OCR1A += lBitRate ;
			waitCompA() ;
			b >>= 1 ;
    }
		SPORT_PORT &= ~SPORT_BIT ;		// restore pin to natural state

	OCR1A += lBitRate ;
	waitCompA() ;
}

static void initSerial()
{
  setRX() ;
}

void sendByte( uint8_t byte )
{
	uint16_t lcrc = Crc ;
	// CRC update
  lcrc += byte; //0-1FF
  lcrc += lcrc >> 8; //0-100
  Crc = lcrc ;

	if ( ( byte == 0x7E ) || ( byte == 0x7D ) )
	{
  	swrite( 0x7D ) ;
		
		byte &= ~0x20 ;
	}
  swrite(byte) ;
}


static void sendCrc()
{
  sendByte(0xFF-Crc) ;
}

static void sendValue( uint8_t type, uint16_t value, uint16_t id )
{
	setTX() ;
  Crc = 0;
  sendByte(type); // DATA_FRAME
  sendByte( (uint8_t)id );
  sendByte( id >> 8 );
  sendByte(value);
  sendByte(value>>8);
  sendByte(0);
  sendByte(0);
  sendCrc();
	setRX() ;
}

static uint16_t convertA3A4( uint16_t value )
{
// ADC*33/8*10/128, to keep inside a 16 bit value
	value *= 33 ;
	value /= 8 ;
	value *= 10 ;
	value /= 128 ;
	return value ;
}


static uint8_t whichId = 0 ;

static void sendData()
{
	uint16_t value ;
	uint16_t id ;
//	TCCR0B = 1 ;		// Clock div 1
	if ( whichId == 0 )
	{
  	value = AnalogControl[0].Analog ;
		id = A2_ID ;
	}
	else if ( whichId == 1 )
	{
  	value = AnalogControl[1].Analog ;
		id = TEMP2_ID ;
		if ( (ID_PIN & ID_BIT) == 0 )
		{
			id = A1_ID ;
		}
	}
	else if ( whichId == 2 )
	{
  	value = AnalogControl[2].Analog ;
		value = convertA3A4( value ) ;
		id = A3_ID ;
	}
	else if ( whichId == 3 )
	{
  	value = AnalogControl[3].Analog ;
		value = convertA3A4( value ) ;
		id = A4_ID ;
	}
	
	if ( SendStart )
	{
	  sendValue( 0x10, value, id ) ;
	}
	else
	{
	  sendValue( 0, 0, 0 ) ;
	}

	// Locate next value to send

	if ( SendStart )
	{
		SendStart -= 1 ;
		whichId += 1 ;			// Go to next item
		if ( whichId == 1 )
		{
			if ( (DIDR0 & 2) == 0 )	// A1 not enabled
			{
				if ( SendStart )
				{
					SendStart -= 1 ;
				}
				whichId = 2 ;
			}
		}
		if ( whichId == 2 )
		{
			if ( A3_PIN & A3_BIT )	// A3 not enabled
			{
				if ( SendStart )
				{
					SendStart -= 1 ;
				}
				whichId = 3 ;
			}
		}
		if ( whichId == 3 )
		{
			if ( A4_PIN & A4_BIT )	// A4 not enabled
			{
				if ( SendStart )
				{
					SendStart -= 1 ;
				}
				whichId = 3 ;
			}
		}
	}
	else
	{
		whichId = 0 ;
	}
}

static void readSensors()
{
	struct t_anacontrol *panalog ;
	uint8_t index = 0 ;
	uint16_t val ;
	uint16_t x ;

	panalog = &AnalogControl[0] ;
	FORCE_INDIRECT( panalog ) ;

  // set the reference to Vcc and the measurement to ADC1
  ADMUX = 0x40 ;

	do
	{
		ADCSRA |= _BV(ADSC); // Start conversion
  	while (bit_is_set(ADCSRA,ADSC)); // measuring

  	val = ADC ; // read the value from the sensor
		if ( index == 1 )
		{
			x = val ;
		}


		if ( ( LINK_PIN & LINK_BIT ) || ( index == 1 ) )
		{
			val += 1 ;
			val >>= 2 ;	
		}
	
		panalog->AnaAve += val ;
		panalog->AnaCount += 1 ;
	
		if ( panalog->AnaCount > 3 )
		{
			if ( index < 2 )
			{
				if ( ( LINK_PIN & LINK_BIT ) || ( index == 1 ) )
				{
					val = panalog->AnaAve >> 2 ;
				}
				else
				{
					// Scale 15K/3.3K to 10K/3.3K
					// Map 744 counts to 256
					// We have 4 samples added so 2976 goes to 256
					// Use 11/128, is about 0.2% out, but MUCH better than the resistor tolerance
		//			val = panalog->AnaAve * 11 ;
					// Shifts and adds are faster and shorter than a multiply
					val = panalog->AnaAve << 2 ;	// *4
					val += panalog->AnaAve ;			// *5
					val <<= 1 ;										// *10
					val += panalog->AnaAve ;			// *11
					val >>= 7 ;		// divide by 128
				}	
				panalog->Analog = (val > 255) ? 255 : val ;
			}
			else
			{
				val = panalog->AnaAve >> 2 ;
				panalog->Analog = val ;
			}
			panalog->AnaCount = 0 ;
			panalog->AnaAve = 0 ;
		}
		panalog += 1 ;
  	ADMUX += 1 ;

	} while ( ++index < 4 ) ;
	// x has full 10 bit input of second ADC
	if ( x < 1000 )
	{
		// We have a real analog input on the second channel
		DIDR0 |= 2 ;	// Disable digital pin
		PORTC &= ~2 ;		// Remove pull up on second analog i/p
	}
}

//
// The receive routine called by the interrupt handler
//
static uint8_t recv()
{
  uint8_t d = 0 ;
  // If RX line is high, then we don't see any start bit
  // so interrupt is probably not for us
#if DEBUG
	DEBUG_PORT &= ~DEBUGA_BIT ;
	DEBUG_PORT |= DEBUG_BIT ;
#endif
//  if (rx_pin_read())
//  {
		
  // Wait approximately 1/2 of a bit width to "centre" the sample
		waitCompA() ;
#if DEBUG
	DEBUG_PORT &= ~DEBUG_BIT ;
	DEBUG_PORT |= DEBUG_BIT ;
#endif
    // Read each of the 8 bits
	  for ( uint8_t i = 0 ; i < 8 ; i += 1 )
  	{
    	d >>= 1 ;
			OCR1A += BITRATE ;
			waitCompA() ;
#if DEBUG
	DEBUG_PORT &= ~DEBUG_BIT ;
#endif
	    if (!rx_pin_read())
			{
    	  d |= 0x80 ;
			}
#if DEBUG
	DEBUG_PORT |= DEBUG_BIT ;
#endif
  	}
	  // skip the stop bit
		OCR1A += BITRATE ;
		waitCompA() ;
//	}
#if DEBUG
	DEBUG_PORT &= ~DEBUG_BIT ;
#endif
	return d ;
}



void wait4msIdle()
{
	// We want 4mS with NO transitions on INT0
	// and INT0 (PD2) is LOW ( = idle high for inverted data)
	TCNT0 = 0 ;
	TCCR0B = 4 ;		// Clock div 256, 32 uS per count
	// 125 counts is 4mS
	EIFR = (1 << INTF0) ;		// CLEAR flag
	do
	{
		if ( ( EIFR & (1 << INTF0) ) || ( SPORT_PIN & SPORT_BIT) )
		{
			EIFR = (1 << INTF0) ;
			TCNT0 = 0 ;			
		}
//		wdt_reset() ;
	} while ( TCNT0 < 126 ) ;
	TCCR0B = 0 ;		// Stop timer
}


uint8_t getBaudrate()
{
	uint8_t rx ;
	TCCR0B = 2 ;		// Clock div 8, 1 uS per count
	EIFR = (1 << INTF0) ;		// CLEAR flag
	for(;;)
	{
		if ( EIFR & (1 << INTF0) )
		{
			rx = TCNT0 ;		// Note timer value
			EIFR = (1 << INTF0) ;
			break ;
		}
//		wdt_reset() ;
	}
	// Got the start of the start bit

	for(;;)
	{
		if ( EIFR & (1 << INTF0) )
		{
			EIFR = (1 << INTF0) ;
			break ;
		}
//		wdt_reset() ;
	}

	EICRA &= ~1 ;		// INT0 interrupt on falling edge
	for(;;)
	{
		if ( EIFR & (1 << INTF0) )
		{
			rx = TCNT0 - rx ;
			EIFR = (1 << INTF0) ;
			break ;
		}
//		wdt_reset() ;
	}
	// rx now the time (in 1uS units) of the 8 bits
	return rx ;
}


int16_t getByte()
{
	uint16_t time ;
	uint8_t rx ;
	
	time = TCNT1 ;
	EIFR = (1 << INTF0) ;		// Clear flag
	while ( TCNT1 - time < 8000 )	// 1mS timeout
	{
		if ( EIFR & (1 << INTF0) )
		{
			OCR1A = TCNT1 + RXCENTRE ;
			TIFR1 = (1 << OCF1A) ; 		// Clear flag
			EIFR = (1 << INTF0) ;		// Clear flag
			rx = recv() ;
			return rx ;
		}
	}
	return -1 ;
}

int16_t getPacket()
{
	uint8_t count ;
	uint8_t stuffing = 0 ;
	int16_t value ;
	uint8_t rxbyte ;
	
	for ( count = 0 ; count < 8 ; count += 1 )
	{
		value = getByte() ;
		if ( value == -1 )
		{
			return -1 ;
		}
		rxbyte = value ;
		if ( ( rxbyte == 0x7E ) || ( rxbyte == 0x7D ) )
		{
			stuffing = 1 ;
		}
		if ( stuffing )
		{
			value = getByte() ;
			if ( value == -1 )
			{
				return -1 ;
			}
			rxbyte = value + 0x20 ;
		}
		stuffing = 0 ;
		ReceivedPacket[count] = rxbyte ;
	}
	return 0 ;
}

static uint8_t checkSportPacket()
{
	uint8_t *packet = ReceivedPacket ;
  uint16_t crc = 0 ;
  for ( uint8_t i = 0 ; i < 8 ; i += 1 )
	{
    crc += packet[i]; //0-1FF
    crc += crc >> 8; //0-100
    crc &= 0x00ff;
  }
  return (crc == 0x00ff) ;
}

void loop()
{
  static uint8_t lastRx = 0 ;
	uint8_t rx ;

	readSensors() ;
  readSensors() ;
  readSensors() ;
  readSensors() ;
	
	wait4msIdle() ;
	getBaudrate() ;		// Skip first frame (X6R problem)
	wait4msIdle() ;
	getBaudrate() ;		// Skip second frame as well
	wait4msIdle() ;
	
	EICRA = 3 ;
	EIFR = (1 << INTF0) ;
	
	for(;;)
	{
		if ( EIFR & (1 << INTF0) )
		{
//			TCCR0B = 0 ;		// stop timer
//			TCNT0 = 0 ;
//			TCCR0B = 1 ;		// Clock div 1
			OCR1A = TCNT1 + RXCENTRE ;
			TIFR1 = (1 << OCF1A) ; 		// Clear flag
			EIFR = (1 << INTF0) ;		// Clear flag
#if DEBUG
			DEBUG_PORT |= DEBUGA_BIT ;
#endif
			rx = recv() ;

		  if (lastRx == 0x7e )
		  {
			 	Timer12ms += 1 ;
			 	if ( Timer12ms > 16 )
			 	{
			 		Timer12ms = 0 ;
			 		SendStart = 4 ;
			 	}
				if (rx == SensorId)
				{
    			lastRx = 0 ;
					// Delay around 400uS
//					TCCR0B = 0 ;		// stop timer
					OCR1A = TCNT1 + 3200 ;
					TIFR1 = (1 << OCF1A) ; 		// Clear flag
//					TCCR0B = 3 ;		// Clock div 64
					waitCompA() ;
					if ( Idle == 0 )
					{
  			  	sendData() ;
					}
						else if ( Send32packet )
						{
							TCCR0B = 1 ;		// Clock div 1
							setTX() ;
  						Crc = 0 ;
  						sendByte(0x32); // DATA_FRAME (0x10) or 0
  						sendByte( ReceivedAppId[0] ) ;
  						sendByte( ReceivedAppId[1] ) ;
  						sendByte(1);
  						sendByte(SensorId & 0x1F);
  						sendByte(0);
  						sendByte(0);
  						sendCrc();
							setRX() ;
							Send32packet = 0 ;
						}
  			  readSensors() ;
				}
				else
				{
					lastRx = 0 ;
//					if ( (rx & 0x1F) == SENSOR_CONFIG_ID)
//					{
						// receive complete packet
						if ( getPacket() != -1 )
						{
							if ( checkSportPacket() )
							{
								if ( ReceivedPacket[0] == 0x21 )
								{
									if ( ReceivedPacket[1] == 0xFF )
									{
										if ( ReceivedPacket[2] == 0xFF )
										{
											Idle = 1 ;
										}
									}
								}
								else if ( ReceivedPacket[0] == 0x20 )
								{
									if ( ReceivedPacket[1] == 0xFF )
									{
										if ( ReceivedPacket[2] == 0xFF )
										{
											Idle = 0 ;
										}
									}
								}
								else if ( ReceivedPacket[2] == 0xF1 )
								{
									if ( ( ReceivedPacket[1] == 0x02 ) || ( ReceivedPacket[1] == 0x03 ) )
									{
										if ( ReceivedPacket[0] == 0x31 )
										{
											if ( ReceivedPacket[3] == 0x01 )
											{
												// New Id is in ReceivedPacket[4]
												uint8_t newId ;
												uint8_t id ;
												newId = ReceivedPacket[4] ;
												for ( uint8_t i = 0 ; i < sizeof(Indices) ; i += 1 )
												{
  												id = pgm_read_byte( &Indices[i] ) ;
													if ( ( id & 0x1F ) == ( newId & 0x1F ) )
													{
														SensorId = id ;
														chk_wrieeprom( 0, id ) ;
														break ;
													}
												}
											}
										}
										else if ( ReceivedPacket[0] == 0x30 )
										{
											if ( ReceivedPacket[3] == 0x01 )
											{
												ReceivedAppId[0] = ReceivedPacket[1] ;
												ReceivedAppId[1] = ReceivedPacket[2] ;
												// Request to read current value
												Send32packet = 1 ;
											}
										}
									}
								}
							}
						}
					}
//					else
//					{
//						lastRx = rx ;
//					}
				}
			}
			else
			{
				lastRx = rx ;
			}
			EIFR = (1 << INTF0) ;
		}
	}
}


uint32_t micros()
{
	uint16_t elapsed ;
	uint8_t millisToAdd ;
	uint8_t oldSREG = SREG ;
	cli() ;
	uint16_t time = TCNT1 ;	// Read timer 1
	SREG = oldSREG ;

	elapsed = time - lastTimerValue ;
	
 #if F_CPU == 20000000L   // 20MHz clock 
   #error Unsupported clock speed
  #elif F_CPU == 16000000L  // 16MHz clock                                                  
   #error Unsupported clock speed
//        elapsed >>= 4 ;
  #elif F_CPU == 8000000L   // 8MHz clock
        elapsed >>= 3 ;
    #else
    #error Unsupported clock speed
  #endif

        //elapsed >>= 4 ;
	
	uint32_t ltime = TotalMicros ;
	ltime += elapsed ;
	cli() ;
	TotalMicros = ltime ;	// Done this way for RPM to work correctly
	lastTimerValue = time ;
	SREG = oldSREG ;	// Still valid from above
	
	elapsed += MillisPrecount;
	millisToAdd = 0 ;
	if ( elapsed  > 3999 )
	{
		millisToAdd = 4 ;
		elapsed -= 4000 ;
	}
	else if ( elapsed  > 2999 )
	{
		millisToAdd = 3 ;		
		elapsed -= 3000 ;
	}
	else if ( elapsed  > 1999 )
	{
		millisToAdd = 2 ;
		elapsed -= 2000 ;
	}
	else if ( elapsed  > 999 )
	{
		millisToAdd = 1 ;
		elapsed -= 1000 ;
	}
	TotalMillis += millisToAdd ;
	MillisPrecount = elapsed ;
	return TotalMicros ;
}

uint32_t millis()
{
	micros() ;
	return TotalMillis ;
}


static void init_from_eeprom()
{
	uint8_t value ;
	uint8_t newValue ;
	uint8_t i ;

//	SensorId = SENSOR_ID ;		// Set default

	value = eeprom_read( 0 ) ;
	if ( value == 0xFF )
	{
		newValue = SENSOR_ID ;		// Set default
	}
	else
	{
		for ( i = 0 ; i < sizeof(Indices) ; i += 1 )
		{
  		newValue = pgm_read_byte( &Indices[i] ) ;
			if ( ( value & 0x1F ) == ( newValue & 0x1F ) )
			{
				break ;
			}
		}
		if ( i >= sizeof(Indices) )
		{
			// Not found
			newValue = SENSOR_ID ;		// Set default
		}
	}
	
	if ( newValue != value )
	{
		chk_wrieeprom( 0, newValue ) ;
	}
	SensorId = newValue ;
}

// Write an 8 bit byte to EEPROM, Address->byte
static void chk_wrieeprom( uint8_t uiAddress, uint8_t ucData )
{
/* Wait for completion of previous write */
	while(EECR & (1<<EEPE))
	{
		// null body
	}
/* Set up address and data registers */
	EEAR = uiAddress;
	EEDR = ucData;
/* Write logical one to EEMWE */
	EECR |= (1<<EEMPE);
/* Start eeprom write by setting EEWE */
	EECR |= (1<<EEPE);
}


// Read an 8 bit byte from EEPROM, Address->byte
static uint8_t eeprom_read( uint8_t address )
{
/* Wait for completion of previous write */
	while(EECR & (1<<EEPE))
	{
		// null body
	}
/* Set up address register */
	EEAR = address ;
/* Start eeprom read by writing EERE */
	EECR |= (1<<EERE) ;
/* Return data from data register */
	return EEDR ;
}

