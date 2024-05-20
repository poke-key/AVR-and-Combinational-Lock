#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "timerISR.h"
#include "serialATmega.h"

#define DISPLAY_DELAY  500
/*global variable*/
int count = 0;
int delay = 0 ;
int delay_counter;

bool locked_state = true ; // start with locked 
bool update_state = false ;
void outNum(int num) ;


void rotateLock(int direction) {
    int phases[8] = {0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001}; //8 phases of the stepper motor step
    DDRB = 0xFF; PORTB = 0x00; //sets all of port b as outputs even though we are only using pins 2-5(digital pins 10-13)(stepper motor)
    
    int i = 0;
    int j = 1028 ;
    while (j)
    {
        if(direction){//button not pressed
            PORTB = (PORTB & 0x03) | phases[i] << 2;//& first to reset pins 2-5 but not 0-1 then | with phase shifted left 2 to assign the right value to pins 2-5
            i++;//increment to next phase
            if(i>7){ //if all phases are completed, restart
                i = 0;
            }
        }else{
            PORTB = (PORTB & 0x03) | phases[i] << 2;
            i--;
            if(i<0){
                i = 8;
            }
        }
        _delay_ms(1);
        j-- ;
    }


}
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


int nums[16] = {0b1111110, 0b0110000, 0b1101101, 0b1111001, 0b0110011, 0b1011011, 0b1011111, 0b1110000, 0b1111111, 0b1111011, 0b1110111, 0b0011111, 0b1001110, 0b0111101, 0b1001111, 0b1000111 }; 
// a  b  c  d  e  f  g
void outNum(int num){
	PORTD = nums[num] << 1; //assigns bits 1-7 of nums(a-f) to pins 2-7 of port d
  PORTB = SetBit(PORTB, 0 ,nums[num]&0x01); // assigns bit 0 of nums(g) to pin 0 of port b
}

//directions[] and outDir() will be neeeded for ex 2 and 3
int directions[4] = {0b0111110, 0b0111101, 0b0000101, 0b0001110}; //TODO: copmlete the array containg the values needed for the 7 sgements for each of the 4 directions
// a  b  c  d  e  f  g
//TODO: display the direction to the 7-seg display. HINT: will be very similar to outNum()
void outDir(int dir){
  PORTD = directions[dir] << 1; //assigns bits 1-7 of nums(a-f) to pins 2-7 of port d
  PORTB = SetBit(PORTB, 0 ,directions[dir]&0x01); // assigns bit 0 of nums(g) to pin 0 of port b
}

int phases[8] = {0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001}; //8 phases of the stepper motor step

enum states { UP, DOWN, RIGHT, LEFT, INIT, IDLE, REMAIN_UP, REMAIN_DOWN, REMAIN_RIGHT, REMAIN_LEFT, LOCK, CHECK_LOCK, UNLOCK, WRONG_P, UPDATE_P, RESET, TIMER} state; //TODO: finish the enum for the SM
int passcode[4] = {UP, UP, RIGHT, LEFT};
int entered_passcode[4] = {0, 0, 0, 0};
int prev_state = IDLE ;

void update_passcode(int location, int pstate)
{
    passcode[location] = pstate ;
}

void Tick() {
  // State Transistions
  if(state == TIMER) { 
        delay_counter--;
        if (!delay_counter) {
            if(count == 4){
                state = CHECK_LOCK;
            }
            else {
                count++;
                outNum(count);
                state = IDLE;
            }
        }
        return;
  }
  
  switch(state) {
    case INIT:
      outNum(count);
      state = IDLE;
      break;
    case IDLE:
      if((PINC >> 2 & 0x01) == 0) state = UPDATE_P; 
      else if(ADC_read(0) > 950) state = UP;
      else if(ADC_read(0) < 100) state = DOWN;
      else if(ADC_read(1) > 950) state = RIGHT;
      else if(ADC_read(1) < 100) state = LEFT;
      break;
    case UP:
      if(ADC_read(0) <= 950) state = IDLE; 
      else state = REMAIN_UP;
      break;
    case DOWN:
      if(ADC_read(0) >= 100) state = IDLE;
      else state = REMAIN_DOWN; 
      break;
    case RIGHT:
      if(ADC_read(1) <= 950) state = IDLE; 
      else state = REMAIN_RIGHT;
      break;
    case LEFT:
      if(ADC_read(1) >= 100) state = IDLE;
      else state = REMAIN_LEFT;
      break;
    case REMAIN_UP:
      if(ADC_read(0) <= 950) state = IDLE;
      break;
    case REMAIN_DOWN:
      if(ADC_read(0) >= 100) state = IDLE;
      break;
    case REMAIN_RIGHT:
      if(ADC_read(1) <= 950) state = IDLE;
      break;
    case REMAIN_LEFT:
      if(ADC_read(0) >= 100) state = IDLE;
      break;
    case LOCK:
      break;
    case UPDATE_P: 
        {
            if (!locked_state) {
                PORTC = SetBit(PORTC, 4, 1); // light up the LED
                update_state = true ;
            }
            state = IDLE ;
        }
        break ;
    case CHECK_LOCK:
       {
            if (update_state ) {
                state = IDLE ;
                update_state = false;
                PORTC = SetBit(PORTC, 4, 0); // off
                count = 0;
                locked_state = true ;
                outNum(count);
                
            }
            else {
                int bGood = true ;
                //password check
                for(int i = 0; i < 4; i++) {
                    if(passcode[i] != entered_passcode[i]) {
                        count = 0;
                        outNum(count);
                        bGood = false ;
                        state = WRONG_P ;
                        break; 
                    }
                }
                if (bGood ) {
                    if (locked_state )
                        state = UNLOCK; 
                    else 
                        state = LOCK ;
                }
                count = 0;
                outNum(count);
            }
       }
      break ;
    case UNLOCK:
      break;
    case RESET:
      state = IDLE; break;
    default:
      state = INIT;
      break;
  }

  // State Actions
  switch(state) {
    case INIT:
      break;
    case IDLE:
      if (locked_state)
        PORTC = SetBit(PORTC, 5, 1);
      else
        PORTC = SetBit(PORTC, 5, 0);
      break;
    case UP:
    case DOWN:
    case RIGHT:
    case LEFT:
        outDir(state);
        if( update_state ) 
            update_passcode(count, state);
        else 
            entered_passcode[count] = state;
        delay_counter = DISPLAY_DELAY;
        state = TIMER; 
        break;
    case REMAIN_UP:
    case REMAIN_DOWN:
    case REMAIN_RIGHT:
    case REMAIN_LEFT:
      break;
    case LOCK:
      locked_state = true ;
      rotateLock(1);  //clockwise
      state = INIT ;
      break;
    case UNLOCK:
      locked_state = false ;
      rotateLock(0);  //anti-clockwise
    case WRONG_P:
      state = INIT ;
      break;
    case CHECK_LOCK:
      break;
    case RESET:
      outNum(count);
      count = 0;
      break;
    default:
      break;
  }
  if(count == 4) {
    delay_counter = DISPLAY_DELAY;
    state = TIMER;
  }    
}

int main(void)
{
    DDRB = 0xFF; PORTB = 0x00; //sets all of port b as outputs even though we are only using pins 2-5(digital pins 10-13)(stepper motor)
    DDRD = 0xFF; PORTD = 0x00;
    DDRC = 0x30; PORTC = 0xF; //decimal point
    serial_init(9600);
    ADC_init();//initializes the analog to digital converter
    state = INIT;
    TimerSet(1); //period of 1 ms. good period for the stepper mottor
    TimerOn();
    while (1)
    {
        Tick();      // Execute one synchSM tick
        while (!TimerFlag){}
        TimerFlag = 0; //Wait for SM period, Lower flag
    }
    return 0;
}