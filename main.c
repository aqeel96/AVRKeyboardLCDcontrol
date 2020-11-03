#define F_CPU 14745600
#include <stdio.h>
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <inttypes.h>

#include "delay.h"
#include "lcd.h"
#include "uart.h"

uint16_t indicator, indicator3, keynumber, motor;
uint16_t wait;
//declaring the variable volatile enables any function in the program to change them
volatile uint16_t timer,second_timer;
volatile uint8_t indicator2, count_0,Pause,reset;
volatile uint16_t seconds, minutes, ten_sec, ten_min;
volatile uint8_t estop = 0;


// PIN DEFINITIONS:
//
// PC0 -- KEYPAD COL1 (left column)
// PC1 -- KEYPAD COL2 (centre column)
// PC2 -- KEYPAD COL3 (right column)
// PC3 -- KEYPAD ROW1 (top row)
// PC4 -- KEYPAD ROW2 (second row)
// PC5 -- KEYPAD ROW3 (third row)
// PB2 -- KEYPAD ROW4 (last row)

// NOTE: We have not connected bottom row
// because PC6 is being used as RESET input (tied high)



void keypad_init()
{
	// set ports to proper input and output
	// enable pull up resistors for inputs
	// Set the columns as ouput
	// Set row pins to input mode
	// Port C bit 0 and 1 and 2 as output (columns)
	DDRC |= (1<<PC2) |(1<<PC1) |(1<<PC0) ;
	DDRC &= ~(1<<PC3); // set PC3 as input
	DDRC &= ~(1<<PC4); // set PC4 as input
	DDRC &= ~(1<<PC5); // set PC5 as input
	DDRB &= ~(1<<PB2); // set PB2 as input


	// turn on the internal resistors for the input pins
	PORTC |= (1<<PC3); // turn on internal pull up resistor for PC3
	PORTC |= (1<<PC4); // turn on internal pull up resistor for PC6
	PORTC |= (1<<PC5); // turn on internal pull up resistor for PC5
	PORTB |= (1<<PB2); // turn on internal pull up resistor for PB2

	// Set column output pins low, so input low if contact made
	PORTC &= ~(1<<PC2 | 1<<PC1 | 1<<PC0 ); // set PC0, PC1 and PC2 low

}
// END keypad_init


uint8_t read_row()
{
	// Reads row pins from Port C
	// and returns them as first 3 bits in an integer
	uint8_t rowvl_C;
	uint8_t rowvl_B;

	rowvl_C = (PINC >> 3);					  // Read value from Pins on PortC,Then shift right 3 to get pins3, 4 and 5 in first 3 bits
	rowvl_C &= (1<<PC2 | 1<<PC1 | 1<<PC0);   // mask out other bits
	rowvl_B = (PINB << 1);					  // Read value from Pins on PortB, Then shift left 1 to put it in 4 bit.
	rowvl_B &= (1<<PB3);                     // mask out other bits
	rowvl_C |= rowvl_B;
	
	return rowvl_C;

}


uint8_t keypressed()
{
	// checks to see if any key has been pressed
	// sets columns to 0, and checks if any row goes low
	// Returns a 1 (true) if a key has been pressed, 0 if not pressed
	// Set column output pins low, so input low if contact made
	uint8_t row_pins;
	uint8_t kp;
	PORTC &= ~(1<<PC2 | 1<<PC1 | 1<<PC0 ); // set PC0, PC1 and PC2 low
	delay_us(10);                          // delay to allow signals to settle
	row_pins = read_row();
	kp = (row_pins != 15);              	// if 15, all high so no key
	
	return kp;
}

char keypad_read(char lastchar)
{
	// read from keypad
	// (assumes key has been pressed)
	// returns keyval 0..9
	// takes in lastchar, and returns that if invalid read from keypad
	uint8_t rowval_1;
	char keych;                        // Initialise to $
	keych = '$';
	
	PORTC |= ( (1<<PC2) | (1<<PC1));   // set  other 2 high
	PORTC &= ~(1<<PC0 );               // set PC0  low - check column 1
	delay_us(10);                      // delay to allow signals to settle

	rowval_1 = read_row();
	
	switch(rowval_1)
	{
		case 14: keych = '1';
		break;
		case 13: keych = '4';
		break;
		case 11: keych = '7';
		break;
		case 7: keych = '*';
		break;
		default: keych = '$';
		break;
	}
	
	if (keych=='$')                        // if still not valid key
	{
		PORTC |= ( (1<<PC2) | (1<<PC0));   // set  other 2 high
		PORTC &= ~(1<<PC1 );               // set PC1 low - check column 2
		delay_us(10);                      // delay to allow signals to settle
		rowval_1 = read_row();

		switch(rowval_1)
		{
			case 14: keych = '2';
			break;
			case 13: keych = '5';
			break;
			case 11: keych = '8';
			break;
			case 7: keych = '0';
			break;
			default: keych = '$';
			break;
		}
	}

	if (keych=='$')          // if still not valid key
	{
		PORTC |= ( (1<<PC1) | (1<<PC0));  // set  other 2 high
		PORTC &= ~(1<<PC2 );      // set PC2 low - check column 3
		delay_us(10);              // delay to allow signals to settle
		rowval_1 = read_row();
		switch(rowval_1)
		{
			case 14: keych = '3';
			break;
			case 13: keych = '6';
			break;
			case 11: keych = '9';
			break;
			case 7: keych = '#';
			break;
			default: keych = '$';
			break;
		}
	}
	
	if (keych != '$')       // if valid character
	{
		lastchar = keych;    // update last char
	}
	return lastchar;
}  // END keypad_read

void init_interrupts()
{
	// To set up the interrupts for the pin - PCINT1
	// Enable the particular pin change interrupt - PC0
	// Enable global interrupts
	
	PCMSK0 = (1<<PC1);                 // Enable PCINT1 interrupt, and clear others
	PCICR &= ~((1<<PC2) | (1<<PC1));  // Set PCIE0 and clear PCIE1 and PCIE2
	PCICR |= 1;
	sei();                             // Enable global interrupts
	
}  // END init_interrupts


void init_switch_port()
{
	// Initialises Port B
	// PB1 set as input for switch
	// PB3 and PB5 set as outputs for "motors"
	DDRB |= (1<<PB5); // Set PB5 as output
	DDRB |= (1<<PB4); // Set PB4 as output
	DDRB |= (1<<PB3); // Set PB3 as output

	DDRB &= ~(1<<PB1); // set PB1 as input
	PORTB |= (1<<PB1); // turn on internal pull up resistor for PB1

	PORTB |= (1<<PB3); // Motor turns off on PB3
	PORTB |= (1<<PB4); // Motor turns off on PB4
	PORTB |= (1<<PB5); // Motor turns off on PB5
	
}  // END init_switch_port








ISR(PCINT0_vect)
{
	// This uses the predefined ISR function to create an ISR
	// The compiler generates the initial and final code
	// (including the RETI instruction)
	// This will result in the following code being run when
	// Interrupt PC0 (Pin change 0) happens
	
	estop = 1;            // Provides the flag for the interrupt
	char star_pressed = 'C'; // Declared the function for the indication of star pressed and calls the keypad_ read() function
					 // Turns all motors off
	
	lcd_clear_and_home();							// Clears LED Display
	lcd_write_string(PSTR("EMERGENCY MODE:ON!!"));	// Displays 'EMERGENCY MODE:ON!!' on the LED
	
	while (estop == 1)
	{
		if (keypressed() == 1)   // Checks is key is pressed
		star_pressed = keypad_read(star_pressed);
		if (star_pressed == '*') // if the * key is pressed reset to list of original programmes
		{
			reset = 1;
			estop = 0;
			lcd_clear_and_home();
			lcd_write_string(PSTR("Choose Program (0-9)"));
			lcd_goto_position(1,0);
			lcd_write_string(PSTR(" 0 is user program"));
			lcd_goto_position(2,0);
			lcd_write_string(PSTR("3,5,*,# invalid keys"));
		}
	}
	PCIFR |= (1<<PC0); // clears PC1 interrupt flag set in meantime

}

void clock() //Create the clock, determines the timer settings on Timer/counter control register A
{

	TCCR0A |= (1<<WGM01);  // Generates the Pulse waves
	TCCR1B |= (0<<WGM12);

	OCR0A = 144;		   // Compares the pulses waves stored
	OCR1A = 14401;

	TIMSK0 |= (1<<OCIE0A); // Masks the timers interrupt
	TIMSK1 |= (1<<OCIE1A);

}

void delay (uint16_t timing)
{
	TCCR0B |= (1<<CS02) | (1<<CS00); // Starts the timer
	timer = timing;
	while (( timer != 0) && (indicator == 1)) //while indicator flag is raise and the timer is running continuously loop the following
	{

		if (keypressed() == 1) // If '1' is pressed make the flag equal 0
		{
			indicator = 0;
			break;
		}
	}
	TCCR0B &= (0<<CS02) | (0<<CS00); // Turn off the timer/counter
}

ISR (TIMER0_COMPA_vect)
{
	timer = timer - 1;
	second_timer = second_timer +1;
	if ( second_timer == 400 )
	{
		indicator2 = 1;
	}
}



ISR (TIMER1_COMPA_vect)
{
	seconds= seconds - 1;
	if(minutes == 0)
	{
		if(seconds == 0)
		{
			count_0 = 1;
			TCCR1B &= (0<<CS12) | (0<<CS10) | (0<<WGM12);
		}
	}
	else if (seconds == 0)
	{
		minutes = minutes-1;
		seconds = 60;
	}
}

//___________________main program__________________


int main() {
	
	 
	  DDRB = 0xFF;
	  PORTB = 0x00;

	  char keychar= 'C';
	  Pause = 0;
	  
	  // start up the LCD
	  lcd_init();
	  lcd_home();

	  
	  // initialise keypad, switch port
	  keypad_init();
	  init_switch_port();
	  clock();
	  
	  // wait a bit then enable interrupts
	  delay_ms(10);
	  init_interrupts();
	  
	  // declare the variables to represent each bit, of our two 3 bit numbers
	  if (keychar == 'C')
	  {
		  lcd_clear_and_home();
		  lcd_write_string(PSTR("Choose Function"));
		  lcd_goto_position(1,0);
		  lcd_write_string(PSTR("1-4"));
	  }
	  
	while(1)
	{
		   if (keypressed())
		   keychar = keypad_read(keychar);
		   
		   
		   // function 1
		    if (keychar == '1')
		    {
				lcd_clear_and_home();
				lcd_write_string(PSTR("Function 1"));
				while(1)
				{
					for(int i=0;i<10;i++)
					{
						PORTB |= (1<<PB0);
						delay_ms(500/i);
						PORTB |= (0<<PB0);
						delay_ms(500/i);
					}
					for(int i=0;i<10;i++)
					{
						PORTB |= (1<<PB0);
						delay_ms(200*i);
						PORTB |= (0<<PB0);
						delay_ms(200*i);
					}
					if (keychar == '#')
					{
						lcd_clear_and_home();
						lcd_write_string(PSTR("Breaking"));
						delay_ms(1000);
						break;
					}
				}
				
				
				
			}
			
			//function 4
			if (keychar == '4')
			{
				lcd_clear_and_home();
				lcd_write_string(PSTR("Function 4"));
				delay_ms(10);
				while(keypressed()!= '#')
				{
				}
				
			}
			
		   //function 2
		    if (keychar == '2')
		    {
			    lcd_clear_and_home();
			    lcd_write_string(PSTR("Function 2"));
				while(keypressed()!= '#')
				{
					if (keypressed())
					{
						char keychar2 = keypad_read(keychar);
						
						if (keychar2 == '1' || keychar2 == '4' || keychar2 == '7')
						{
							lcd_clear_and_home();
							lcd_write_string(PSTR("Column1"));
							for(int i=0;i<5;i++)
							{
								PORTB |= (1<<PB0);
								delay_ms(200);
								PORTB |= (0<<PB0);
								delay_ms(200);
							}
						}
						if (keychar2 == '2' || keychar2 == '5' || keychar2 == '8')
						{
							lcd_clear_and_home();
							lcd_write_string(PSTR("Column2"));
							delay_ms(10);
							for(int i=0;i<8;i++)
							{
								PORTB |= (1<<PB0);
								delay_ms(200);
								PORTB |= (0<<PB0);
								delay_ms(200);
							}
						}
						if (keychar2 == '3' || keychar2 == '6' || keychar2 == '9')
						{
							lcd_clear_and_home();
							lcd_write_string(PSTR("Column3"));
							delay_ms(10);
							for(int i=0;i<10;i++)
							{
								PORTB |= (1<<PB0);
								delay_ms(200);
								PORTB |= (0<<PB0);
								delay_ms(200);
							}
						}
						if (keychar2 == '#')
						{
							break;
						}
					}
				}	    
		    }
		   
		   
		   //function 3
		    if (keychar == '3')
		    {
			    lcd_clear_and_home();
			    lcd_write_string(PSTR("Function 3"));
				while(keypressed()!= '#')
				{
					if (keypressed())
					{
					char keychar3 = keypad_read(keychar);
							
					if (keychar3 == '1')
					{
						lcd_clear_and_home();
						lcd_write_string(PSTR("3Pulse"));
						for(int i=0;i<3;i++)
						{
							PORTB |= (1<<PB0);
							delay_ms(300);
							PORTB |= (0<<PB0);
							delay_ms(300);
						}
					}
					if (keychar3 == '2')
					{
						lcd_clear_and_home();
						lcd_write_string(PSTR("5Pulse"));
						for(int i=0;i<5;i++)
						{
							PORTB |= (1<<PB0);
							delay_ms(300);
							PORTB |= (0<<PB0);
							delay_ms(300);
						}
					}
					if (keychar3 == '3')
					{
						lcd_clear_and_home();
						lcd_write_string(PSTR("7Pulse"));
						for(int i=0;i<7;i++)
						{
							PORTB |= (1<<PB0);
							delay_ms(300);
							PORTB |= (0<<PB0);
							delay_ms(300);
						}
					}
					if (keychar3 == '#')
					{
						break;
					}
					}
					
				}
			    
			    
		    }
		   
		   
	}
	
	}

	