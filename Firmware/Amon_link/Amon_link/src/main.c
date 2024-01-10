/*
 * main.c
 *
 * Created: 2/21/2023 9:42:00 PM
 *  Author: Tinta T.
 */ 


#include <xc.h>
#include "../inc/main.h"

/* Timer_1 IRQ */
ISR(TIMER1_COMPA_vect)
{
	// Startup init //
	if (InitEND == 0)
	{
		
		if (InitTime >= 30)		// Init time - over
		{	
			InitEND = 1;
			InitTime = 0;
			GPIO_PIN_Write(GPIO_PORT_B,LED_Red,GPIO_HIGH);
		}
		else					// Init time - not over
		{
			InitTime++;
			GPIO_PIN_Toggle(GPIO_PORT_B,LED_Red); // Blink led red
		}
	}
	
	// Pair sequence (triggered after startup or with button) //
	if(PairTrig == 1)
	{			
	
		if (PairTime >= 60 || PairStatus == 1)		// Pair time over
		{	
			PairTrig = 0;
			PairTime = 0;
			
			GPIO_PIN_Write(GPIO_PORT_B,LED_Blue,GPIO_HIGH);	// Blue led off	

			// Change device status on state of connection
			if (PairStatus == CONNECTED)
			{
				DeviceState = STATE_CONNECTED;
				uint8_t DataToSend[15] = {"Connected\r\n\0"};
				USART_DATA_Transmit(&DataToSend[0]);
			}
			else
			{
				DeviceState = STATE_NOT_CONNECTED;
				uint8_t DataToSend[35] = {"Pairing unsuccessful!\r\n\0"};
				USART_DATA_Transmit(&DataToSend[0]);
			}
		}
		else									   // Pair time - not over
		{
			PairTime++;
			GPIO_PIN_Toggle(GPIO_PORT_B,LED_Blue); // Blink blue led
		}
		
	}
	
}


/* UART IRQ Receive */
ISR(USART_RX_vect)
{
	RecievedDataBuffer = UDR0; // incoming data

	// Read data and save in buffer
	if (RecievedDataBuffer == '\0'){
		
		Data_Count = 0;		// reset data counter for saving in array
		USART_RX_DATA_Decode(&UART_DATA_Recieve[0]);
	}
	else
	{
		// Save received data and increase counter
		UART_DATA_Recieve[Data_Count] = RecievedDataBuffer;
		Data_Count++;
	}
}


///****** MAIN BEGIN ******///
int main(void)
{	

	_delay_ms(100); // Start delay for all devices to power on
	
	while(1){
	
		/* State machine */
		switch (DeviceState){
			
			/*
			/ Initialize all devices and I/O - first step (default) after power one
			*/
			case STATE_INIT:
			
				InitError = DeviceInit();	
				
				
				_delay_ms(10000);
			
				// No error in init and after init time
				if (InitError == 0 && InitEND == 1)						
				{						
					uint8_t DataToSend[30] = {"****** Amon Link ******\r\n\0"};
					USART_DATA_Transmit(&DataToSend[0]);
					DeviceState = STATE_CONNECTING;	// Change device state
				}

			break;
		
			/* 
			/ State when device is not connected to drone (after unsuccessful pairing, new connecting sequence can bi triggered by button)
			*/
			case STATE_NOT_CONNECTED:
			
				GPIO_PIN_Write(GPIO_PORT_B,LED_Blue,GPIO_LOW); // Blue led on

				// Dodaj da se s tipko proži pair

				if (PairTrig == 0){
					//PairTrig = 1;
				}
		
			break;
		
			/* 
			/ Connection with drone is established
			*/
			case STATE_CONNECTED:
			
				GPIO_PIN_Write(GPIO_PORT_B,LED_Green,GPIO_LOW);	// Green led on
		
			break;
		
			/* 
			/ Connecting stage (triggered by button or automatically after first initialization)
			*/
			case STATE_CONNECTING:
			
				if (PairTrig == 0){
					PairTrig = 1;
					uint8_t DataToSend[35] = {"Pairing...\r\n\0"};
					USART_DATA_Transmit(&DataToSend[0]);
				}
		
			break;
		
			/* 
			/ Error state, somethings wrong with device on initialization ("cant be used yet")
			*/
			case STATE_ERROR:

				GPIO_PIN_Write(GPIO_PORT_B,LED_Red,GPIO_LOW); // Red led on
		
			break;
		
			/*
			/ Undefined state - catastrophic failure
			*/
			default:	
		
				DeviceState = STATE_ERROR;
		}
	
	}
	
}

///****** END MAIN ******///



/* Initialization function for device */
uint8_t DeviceInit()
{
	uint8_t error = 0;
	
	// GPIO
	DDRB |= (1 << RF_SCK) |(1 << RF_MOSI) | (1 << LED_Green) | (1 << LED_Blue) | (1 << LED_Red) ; // set port B 
	DDRC = 0b00000000; // set port C I/O
	DDRD |= (1 << RF_CSN2) | (1 << RF_CE2) | (1 << RF_CE1) | (1 << RF_CSN1) ; // set port D I/O
	GPIO_PIN_Write(GPIO_PORT_B,LED_Blue,GPIO_HIGH);
	GPIO_PIN_Write(GPIO_PORT_B,LED_Green,GPIO_HIGH);
	
	// IRQ
	IRQ_Timer1_Init();
	USART_Init(USART_PRESC);
	
	// SPI - RF
	SPI_Init();
	
	GPIO_PIN_Write(GPIO_PORT_D,RF_CE1,GPIO_HIGH); // set radio1 as transmitter
	GPIO_PIN_Write(GPIO_PORT_D,RF_CE2,GPIO_LOW);  // set radio2 as receiver
	
	GPIO_PIN_Write(GPIO_PORT_D,RF_CSN1,GPIO_LOW); // CSN high - off
	GPIO_PIN_Write(GPIO_PORT_D,RF_CSN2,GPIO_LOW); // CSN high - off
	//INT0_interrupt_Init();
	//NRF24L01_Inti1();
	//NRF24L01_Inti2();
	
	sei();	// Enable all interrupts
	
	// Error return - NOT IMPLEMENTED
	if(error == 0){
		return 0;
	}else{
		return 1;
	}
}


/* Init for timer */
void IRQ_Timer1_Init()
{
	TCCR1B |= (1 << WGM12 );   // Configure timer 1 for CTC mode
	OCR1A = 25000;             // 10Hz (100mS) at 16MHz, prescaler 64
	TIMSK1 |= (1 << OCIE1A );  // Enable interrupt
	TCCR1B |= ((1 << CS10 ) | (1 << CS11 )); // Start Timer F_CPU/64
}

/* Init for UART */
void USART_Init(unsigned int presc)
{
	UBRR0H = (unsigned char)(presc>>8);
	UBRR0L = (unsigned char)presc;
	UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0) | (1<<TXCIE0);
	UCSR0C = USART_MODE | USART_PARITY_MODE | USART_STOP_BIT | USART_DATA_BIT;
}

/* Interrupt on pin change */
void INT0_interrupt_Init()
{
	// Set detect on falling edge
	EICRA |= (1 << ISC01);
	EICRA &= ~(1 << ISC00);
	
	// enable int
	EIMSK |= (1 << INT0); 
}



// Interrupt for RF RX data
/*
ISR(INT0_vect)
{
	cli();
	
	
	
	sei();
}
*/


