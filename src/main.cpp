#include <avr/io.h>
#include <avr/interrupt.h>
//#include <avr/signal.h>
#include <util/delay.h>
#include "timerISR.h"
#include "serialATmega.h"
#include "scheduler.h"


unsigned char SetBit(unsigned char x, unsigned char k, unsigned char b) {
   return (b ?  (x | (0x01 << k))  :  (x & ~(0x01 << k)) );
              //   Set bit to 1           Set bit to 0
}

unsigned char GetBit(unsigned char x, unsigned char k) {
   return ((x & (0x01 << k)) != 0);
}

void ADC_init() {
  ADMUX = (1<<REFS0);
	ADCSRA|= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
	// ADEN: setting this bit enables analog-to-digital conversion.
	// ADSC: setting this bit starts the first conversion.
	// ADATE: setting this bit enables auto-triggering. Since we are
	//        in Free Running Mode, a new conversion will trigger whenever
	//        the previous conversion completes.
}

unsigned int ADC_read(unsigned char chnl){
  uint8_t low, high;

  ADMUX  = (ADMUX & 0xF8) | (chnl & 7);
  ADCSRA |= 1 << ADSC ;
  while((ADCSRA >> ADSC)&0x01){}

	low  = ADCL;
	high = ADCH;

	return ((high << 8) | low) ;
}

void init_sonar(){
  sei();			/* Enable global interrupt */
	TIMSK1 = (1 << TOIE1);	/* Enable Timer1 overflow interrupts */
	TCCR1A = 0;
  
}

double read_sonar(){
    long count;
    PORTC = SetBit(PORTC,2,1);
    _delay_us(10);
    PORTC = SetBit(PORTC,2,0);

    TCNT1 = 0;	/* Clear Timer counter */
		TCCR1B = 0x41;	/* Capture on rising edge, No prescaler*/
		TIFR1 = 1<<ICF1;	/* Clear ICP flag (Input Capture flag) */
		TIFR1 = 1<<TOV1;	/* Clear Timer Overflow flag */

		/*Calculate width of Echo by Input Capture (ICP) */
		while ((TIFR1 & (1 << ICF1)) == 0);/* Wait for rising edge */
		TCNT1 = 0;	/* Clear Timer counter */
		TCCR1B = 0x01;	/* Capture on falling edge, No prescaler */
		TIFR1 = 1<<ICF1;	/* Clear ICP flag (Input Capture flag) */
		TIFR1 = 1<<TOV1;	/* Clear Timer Overflow flag */
		TimerOverflow = 0;/* Clear Timer overflow count */
		while ((TIFR1 & (1 << ICF1)) == 0);/* Wait for falling edge */
		count = ICR1 + (65535 * TimerOverflow);	/* Take count */
		
		return((double)count / 932.46);
}

 
int nums[16] = {0b1111110, 0b0110000, 0b1101101, 0b1111001, 0b0110011, 0b1011011, 0b1011111, 0b1110000, 0b1111111, 0b1111011, 0b1110111, 0b0011111, 0b1001110, 0b0111101, 0b1001111, 0b1000111 }; 
// a  b  c  d  e  f  g

void outNum(int num){
	PORTD = nums[num] << 1;
  PORTB = SetBit(PORTB, 1 ,nums[num]&0x01);
}

//TODO: create all global varibles needed for tasks to comunicate
int distance = 4 ;


//TODO:create enum and Tick function for each task
//Reminder that they will need different names for each task
//unsigned long int findGCD(unsigned long int a, unsigned long int b) ;
enum displayStates{INITDISP}; //TODO: fill out with rest of states for display task 

int Tick_display1(int state){

  distance = read_sonar() ;
  //char buf[50] ;
 // sprintf(buf, "%s\n", distance) ;
  //serial_println(distance);
  switch(state){

    default:

      break;

  }
  switch(state){
    
    default:
      
      break;

  }

  return state;
}

int Tick_display2(int state){
  outNum(distance) ;
  switch(state){

    default:

      break;

  }
  switch(state){
    
    default:
      
      break;

  }

  return state;
}



int main(void)
{
  DDRD = 0xFF ;
  DDRB = 0xFF ;
  DDRC = 0x00 ;
  //TODO: initialize all your inputs and ouputs
  serial_init(9600);
  ADC_init();//initializes ADC
  init_sonar();//initializes sonar

  //TODO: set all the variables for GCD calcualtions(task periods)
  unsigned long int Tick_display_calc1 = 1000;//display period
  unsigned long int Tick_display_calc2 = 1 ;//display period
  //unsigned long int //task2 period

  unsigned long int tmpGCD = findGCD(Tick_display_calc1, Tick_display_calc2);
  unsigned long int GCD = tmpGCD;

  //TODO: set all variable for how many timer interupts needed before task needs to tick
  //Recalculate GCD periods for scheduler
  unsigned long int Tick_display_period1 = Tick_display_calc1/GCD;
  unsigned long int Tick_display_period2 = Tick_display_calc2/GCD;

  static task task1;
  static task task2 ;

  task *tasks[] = {&task1, &task2};//TODO: fill out the task array with task variables
  const unsigned short numTasks = sizeof(tasks)/sizeof(task*);

  //TODO: initilaize the task variables
  // Task 1
  task1.state = -1;//Task initial state.
  task1.period = Tick_display_period1;//Task Period.
  task1.elapsedTime = Tick_display_period1;//Task current elapsed time.
  task1.TickFct = &Tick_display1;//Function pointer for the tick.

  // Task 2
  task2.state = -1;//Task initial state.
  task2.period = Tick_display_period2;//Task Period.
  task2.elapsedTime = Tick_display_period2;//Task current elapsed time.
  task2.TickFct = &Tick_display2;//Function pointer for the tick.

  TimerSet(GCD);
  TimerOn();
  while (1)
  {
    for ( unsigned int i = 0; i < numTasks; i++ ) {
    // Task is ready to tick
      if ( tasks[i]->elapsedTime == tasks[i]->period ) {
        // Setting next state for task
        tasks[i]->state = tasks[i]->TickFct(tasks[i]->state);
        // Reset the elapsed time for next tick.
        tasks[i]->elapsedTime = 0;
      }
      tasks[i]->elapsedTime += 1;
    }
    
    while(!TimerFlag);
    TimerFlag = 0;
  }
  return 0;
}