// this is the header file that tells the compiler what pins and ports, etc.
// are available on this chip.
#include <avr/io.h>
#include <util/delay.h>

#include <avr/interrupt.h>

// define what pins the LEDs are connected to.
// in reality, PD6 is really just '6'
#define LED PA0

// Some macros that make the code more readable
#define output_low(port,pin) port &= ~(1<<pin)
#define output_high(port,pin) port |= (1<<pin)
#define set_input(portdir,pin) portdir &= ~(1<<pin)
#define set_output(portdir,pin) portdir |= (1<<pin)

uint8_t DUTY;
uint8_t direction = 1;
uint8_t step = 64;

ISR (TIMER0_COMPA_vect)
{
	// Clear LED on compare
	if(DUTY < 0xFC) //only clear when less than 0xFC otherwise we have flickers
		PORTA &= ~(1 << LED);
}

ISR (TIMER0_OVF_vect)
{
	// Set OCR0A here from global variable
	uint8_t d = DUTY;

	OCR0A = d;
		
	// Set LED high to match COM0A1 mode
	// Disable this if we are only using the PWM pin
	
	if(DUTY > 0x04) //only set when less than 0x04 otherwise we have dont go completly off
		PORTA |= (1 << LED);
}

ISR (TIMER1_COMPA_vect)
{
	if(DUTY >= 0xFF)
	{
		direction = 0;
	}
	
	if(DUTY == 0x00)
	{
		direction = 1;
	}
	
	if(direction == 1)
	{
		if(DUTY > 0xFF - step)
		{
			DUTY = 0xFF;
		}
		else
		{
			DUTY += step;
		}
	}
	else
	{
		if(DUTY < 0x0 + step)
		{
			DUTY = 0x0;
		}
		else
		{
			DUTY -= step;
		}
	}
}

void init_timer0_FAST_PWM_OCR0A_TOP_OC0A()
{
  //NOTE: we should only update the OCR0A value in the TOV interrupt routine
	
  // REGISTERS //
  
  // OCR0A - Output Compare Register 0 A - Used to set TOP value for Counter 0
  // TIMSK - Timer Interrupt Mask Register - Used to enable interrupts for counter  
  // TCCR0A - Timer/Counter Control Register 0 A - Used to configure Timer/Counter 0
  // TCCR0B - Timer/Counter Control Register 0 B - Used to configure Timer/Counter 0

  // TCCR0A - COM1A1 - Clear OC0A on Compare Match, set OC0A at TOP
  //				 NOTE: Depends on Waveform Generation Mode
  
  // TCCR0B - WGM03:0 = 3 (TOP = 0xFF) toggle on OCR0A 
  //					Note use 7 if a different frequency is required i.e. modify the TOP value
  // TCCR0B - CS02:0 Timer prescaling 
  //	      NOTE: Must set some bits otherwise the timer has no clock source
  
  TCCR0B |= (1 << CS00); //F_CPU/1
  
  TCCR0A |= (1 << WGM01) | (1 << WGM00); //Enable Fast PWM mode with TOP=0xFF and toggle on OCR0A
  
  TCCR0A |= (1 << COM0A1); //Clear OC0A on Compare Match, set OC0A at TOP

  TCNT0 = 0; //Initialize Counter
  
  OCR0A = DUTY; //Set the duty cycle this will get updated each time the timer overflows in the OVF ISR
  
  TIMSK |= (1 << OCIE0A) | (1 << TOIE0); //Enable Interrupt for Channel A of Timer/Counter 0 for compare and overflow
  
  sei();
}

void init_timer1_CTC_ICR1_TOP_ISR(uint16_t TOP)
{
// REGISTERS //
  
  // OCR1A - Output Compare Register 1 A - Used to set TOP value for Counter 1
  // TIMSK - Timer Interrupt Mask Register - Used to enable interrupts for counter  
  // TCCR1A - Timer/Counter Control Register 1 A - Used to configure Timer/Counter 1
  // TCCR1B - Timer/Counter Control Register 1 B - Used to configure Timer/Counter 1

  // TCCR1A - COM1A0 - Compare Output Mode (Toggle on match)
  //				 NOTE: Depends on Waveform Generation Mode
  
  // TCCR1B - WGM13:0 = 4 (TOP = OCR1A) or 12 (TOP = ICR1) Ref Table 46
  // TCCR1B - CS12:0 Timer prescaling 
  //	      NOTE: Must set some bits otherwise the timer has no clock source
  
  TCCR1B |= (1 << CS12); //F_CPU/1024
  
  TCCR1B |= (1 << WGM12) | (1 << WGM13); //Enable CTC mode with ICR1 as TOP

  TCNT1 = 0; //Initialize Counter
  ICR1 = TOP; 
  
  TIMSK |= (1 << OCIE1A); //Enable Interrupt for Channel A of Timer/Counter 1
  sei(); //Enable Interrupts Globally
}

int main(void) {
    
  // initialize the direction of PORTD #6 to be an output
  set_output(DDRA, LED);
  set_output(DDRB, PB2); //OC0A
  
  DUTY = 0; // 50% duty cycle
  
  uint16_t TOP = 4100; // 0.5 Hz Frequency 
  
  init_timer0_FAST_PWM_OCR0A_TOP_OC0A();
  
  init_timer1_CTC_ICR1_TOP_ISR(TOP);
  
  while (1) 
  {
	  
  }
}