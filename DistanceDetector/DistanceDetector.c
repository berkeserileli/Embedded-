/*
 * ELE362_PROJE.c
 *
 * Created: 19.08.2024 20:39:30
 * Author : eserb
 */ 

#include <avr/io.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <avr/interrupt.h>


//#define A (1 << PORTD7)
//#define B (1 << PORTD6) 
//#define C (1 << PORTD3) 
//#define D (1 << PORTD4) 
//#define E (1 << PORTD5) 
//#define F (1 << PORTB4) 
//#define G (1 << PORTD2)

void displayDigits(int myDigit){
	 switch(myDigit){ 
		 case 0: PORTD = (1<<PORTD7)|(1 << PORTD6)|(1 << PORTD3)|(1 << PORTD4)|(1 << PORTD5); //0 7-7-Segment disp 
		 PORTB = (1<<PORTB4);
		 break; 
		 
		 case 1: PORTD = (1 << PORTD6)|(1 << PORTD3); //1 7-Segment disp
		 PORTB &= ~(1<<PORTB4);
		 break;
		 
		 case 2: PORTD = (1 << PORTD7)|(1 << PORTD6)|(1 << PORTD4)|(1 << PORTD5)|(1 << PORTD2); //2 7-Segment disp 
		 PORTB &= ~(1<<PORTB4);
		 break;
		  
		 case 3: PORTD = (1 << PORTD7)|(1 << PORTD6)|(1 << PORTD3)|(1 << PORTD4)|(1 << PORTD2); //3 7-Segment disp
		 PORTB &= ~(1<<PORTB4);
		 break;
		 
		 case 4:
		 PORTD = (1 << PORTD6)|(1 << PORTD3)|(1 << PORTD2);
		 PORTB = (1<<PORTB4);
		 break;
		  
		 case 5:
		 PORTD = (1 << PORTD7)|(1 << PORTD3)|(1 << PORTD4)|(1 << PORTD2); //5 7-Segment disp
		 PORTB = (1<<PORTB4);
		 break;
		 
		 case 6:
         PORTD = (1 << PORTD7)|(1 << PORTD3)|(1 << PORTD4)|(1 << PORTD5)|(1 << PORTD2); //6 7-Segment disp
		 PORTB = (1<<PORTB4);
		 break;
		 
		 case 7:
		 PORTD = (1 << PORTD7)|(1 << PORTD6)|(1 << PORTD3); //7 7-Segment disp 
		 break;
		  
		 case 8: 
		 PORTD = (1 << PORTD7)|(1 << PORTD6)|(1 << PORTD3)|(1 << PORTD4)|(1 << PORTD5)|(1 << PORTD2); //8 7-Segment disp
		 PORTB = (1<<PORTB4);
		 break;
		 
		 case 9:
		 PORTD = (1 << PORTD7)|(1 << PORTD6)|(1 << PORTD3)|(1 << PORTD4)|(1 << PORTD2); //9 7-Segment disp
		 PORTB = (1<<PORTB4);
		 break;
		  } 
		  }
		  
		  uint8_t pressed = 0;
		  
int main() {
	 uint8_t timeOfFlight; // time of flight
	 uint16_t distance; // distance (cm)
	 uint8_t real_distance;
	 uint8_t digits[2];
	 uint8_t *p;
	 p = digits;
	 DDRC |= (1<<PORTC1); // PC1 OUTPUT (SENSOR TRIGGER)
	 DDRC &= ~(1<<PORTC0); // PC0 INPUT (SENSOR ECHO) 
	 DDRB |= (1<<PORTB2); //PB2 SERVO OUTPUT OC1B SERVO (D10)
	 DDRB &= ~(1<<PINB3); //PB3 BUTON INPUT FOR INTERRUPT (D11)
	 DDRB |= (1<<PORTB4); //PB4 OUTPUT (disp)
	 DDRB |= (1<<PORTD2); //PD2 OUTPUT (disp)
	 DDRC |= (1<<PORTC2); //PC2 LED GREEN OUTPUT
	 DDRC |= (1<<PORTC3); //PC3 LED BLUE OUTPUT
	 DDRC |= (1<<PORTC4); //PC4 LED RED OUTPUT
	 DDRD = 0xFF; //PORTD PULL-UP 
	 PCMSK0 |= (1 << PCINT3);
	 PCICR |= (1 << PCIE0);
	 
	 sei();
	 // USART SETTINGS
	 UCSR0A = 0x00; // RESET UDRE0
	 UBRR0H = 0x00;
	 UBRR0L = 0x67; // BAUD RATE = 9600
	 UCSR0B = (1<<TXEN0); // ENABLE TX
	 UCSR0C = (1<<UCSZ00) | (1<<UCSZ01); // ASENKRON & NO EVEN-ODD & STOP BIT & 8 BIT 
	 
	 //TIMER1 SETTINGS
	 TCCR1B |= (1<<CS11); // PRESCALING TIMER1 N=8
	 TCCR1B |= (1<<WGM13); // PHASE & FREQUENCY C. PWM
	 TCCR1A |= (1<<COM1B1); // NON-INVERTING PWM FOR OC1B 
	 ICR1 = 20000; // 50Hz FOR SERVO		  

while (1) {
	 if(pressed){
		  UCSR0B |= (1 << RXEN0);
		   }
		    while(pressed){
				 PORTC |= (1<<PORTC1); // START TRIGGER
				  _delay_us(10); // WAIT 10us FOR 8 SONIC PULSES
				   PORTC &= ~(1<<PORTC1); // FINISH TRIGGER
				    // TIMER0 Initialization
					 TCCR0B = 0x00; // STOP TIMER0
					 TCNT0 = 0x00; // RESET TIMER0
					 while(~PINC & (1<<PINC0)); // WAIT UNTIL ECHO SET (1)
					 TCCR0B = (1<<CS00)|(1<<CS02);// START TIMER0 (PRESCALING 1024)
					 while(PINC & (1<<PINC0)); // WAIT UNTIL ECHO RESET (0)
					 timeOfFlight = TCNT0; // INSTANT VALUE
					 distance = 11*(uint16_t)timeOfFlight;
					 distance /= 10; // distance = 1.1 x timeOfFlight 
					 // SPLIT DISTANCE INTO DIGITS
					 for(uint8_t i=0; i<2; i++){
						  *p = distance%10;
						   distance /= 10;
						    p++;
							 } 
					// TRANSMITTING DIGITS VIA UART
					 for(uint8_t i=0; i<2; i++){
						  p--;
						  while(~UCSR0A & (1<<UDRE0)); // WAIT UNTIL UDR EMPTY
						  UDR0 = 0x30 + *p; // TRANSMIT DIGITS AS ASCII
						   }
					real_distance = digits[1]*10 + digits[0]; //DISTANCE TRANSMITTED
					
					 //Leds and angles 
					 if(digits[1]==0){
						  displayDigits(digits[1]);
						  PORTC = (1<<PORTC2); //LED GREEN
						  for(uint16_t t=500; t<=500 +(real_distance*25); t+=10){ //limit t = 725(25*9+500)
							   OCR1B = t; // SEFVO POSITION
							    _delay_ms(10); // SERVO SPEED
								 }
						  displayDigits(digits[0]);
						  for(uint16_t t= 500+ (real_distance*25); t>=500; t-=10){
							   OCR1B = t; // SERVO POSITION 
							   _delay_ms(10); // SERVO SPEED
							    }
					 } else if(digits[1]==1){
						  displayDigits(digits[1]); //1 7-Segment disp 
						  PORTC = (1<<PORTC2); //LED GREEN
						  for(uint16_t t=500; t<=325+(real_distance*25); t+=10){//limit t= 800(19*25+325)
							   OCR1B = t; // SERVO POSITION 
							   _delay_ms(10); //SERVO SPEED
							    }
						 displayDigits(digits[0]);
						  for(uint16_t t= 325+(real_distance*25); t>=500; t-=10){
							   OCR1B = t; // SERVO POSITION 
							   _delay_ms(10); // SERVO SPEED
							    }
					} else if(digits[1]==2){
						 displayDigits(digits[1]); //2 7-Segment disp
						  PORTC = (1<<PORTC2); //Led Green
						  for(uint16_t t=500; t<=175+(real_distance*25); t+=10){ // limit t = 900(29*25+175)
							   OCR1B = t; // SERVO POSITION 
							   _delay_ms(10); // SERVO SPEED
							    } 
						displayDigits(digits[0]);
						 for(uint16_t t= 175+(real_distance*25); t>=500; t-=10){
							  OCR1B = t; // SERVO POSITION
							   _delay_ms(10); // SERVO SPEED
							    }
					} else if(digits[1]==3){
						displayDigits(digits[1]); //3 7-Segment disp
						 PORTC = (1<<PORTC2)|(1<<PORTC3); //Led Green and Blue 
						 for(uint16_t t=500; t<=real_distance*25; t+=10){ // limit t = 975 (25*39)
							  OCR1B = t; // SERVO POSITION
							   _delay_ms(10); // SERVO SPEED
							    }
						displayDigits(digits[0]);
						 for(uint16_t t=real_distance*25; t>=500; t-=10){
							  OCR1B = t; // SERVO POSITION
							   _delay_ms(10); // SERVO SPEED
							    }
				   } else if(digits[1]==4){ //4 7-Segment disp
					    displayDigits(digits[1]);
						 PORTC = (1<<PORTC2)|(1<<PORTC3); //Led Green and Blue
						 for(uint16_t t=500; t<=real_distance*25; t+=10){ //limit t = 1225 (49*25)
							  OCR1B = t; // SERVO POSITION 
							  _delay_ms(10); // SERVO SPEED
							   }
						displayDigits(digits[0]);
						 for(uint16_t t=real_distance*25; t>=500; t-=10){
							  OCR1B = t; // SERVO POSITION
							   _delay_ms(10); // SERVO SPEED }
				  } else if(digits[1]==5){
					   displayDigits(digits[1]); //5 7-Segment disp
					    PORTC = (1<<PORTC2)|(1<<PORTC3); //Led Green and Blue
						 for(uint16_t t=500; t<=real_distance*25; t+=10){ //limit t = 1475 (59*25) 
							 OCR1B = t; // SERVO POSITION 
							 _delay_ms(10); // SERVO SPEED
							  } 
						displayDigits(digits[0]);
						 for(uint16_t t=real_distance*25; t>=500; t-=10){
							 OCR1B = t; // SERVO POSITION 
							 _delay_ms(10); // SERVO SPEED
							  }
							  }
				 else{
					  if(digits[1] == 6){ //6 7-Segment disp
						   displayDigits(digits[1]);
						    PORTC = (1<<PORTC2)|(1<<PORTC3)|(1<<PORTC4); //Led Green,Blue and Red 
							}
							 else if(digits[1] == 7){ //7 7-Segment disp
						   displayDigits(digits[1]);
						    PORTC = (1<<PORTC2)|(1<<PORTC3)|(1<<PORTC4);
							}
							else if(digits[1] ==8){ //8 7-Segment disp
						   displayDigits(digits[1]);
						    PORTC = (1<<PORTC2)|(1<<PORTC3)|(1<<PORTC4);
							}
							else if(digits[1] ==9){ //9 7-Segment disp
						   displayDigits(digits[1]);
						    PORTC = (1<<PORTC2)|(1<<PORTC3)|(1<<PORTC4);
							}
							 for(uint16_t t=500; t<=real_distance*25; t+=10){ //limit 1725 (69*25) - 2475(99*25)
								  OCR1B = t; //SERVO POSITION
								   _delay_ms(10); // SERVO SPEED
								    }
									 displayDigits(digits[0]);
							 for(uint16_t t=real_distance*25; t>=500; t-=10){
								  OCR1B = t; // SERVO POSITION 
								  _delay_ms(10); // SERVO SPEED 
								  }
								  } while(~UCSR0A & (1<<UDRE0)); // Wait until UDR empty
								   UDR0 = 0x0A; // TRANSMITTED NEW LINE IN ASCII
								    _delay_ms(3000); // WAIT FOR NEXT PROCESS
						   }
						}
					}
					
					
ISR(PCINT0_vect){
	 if(~PINB & (1 << PINB3)){
		  pressed = !pressed;
		  PORTC &= ~((1<<PORTC2)|(1<<PORTC3)|(1<<PORTC4)); //Turn off LEDs
		  PORTD = (1 << PORTD7)|(1 << PORTD2)|(1 << PORTD5); //F 7-Segment disp
		  PORTB = (1 << PORTB4);
 }
}