/*
  lcd.c - Routines for sending data and commands to the LCD shield
*/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>

#include "lcd.h"                // Declarations of the LCD functions
#include "ds18b20.h"
//NOTE THAT I DID NOT USE A SEPERATE ENCODER.H or ENCODER.C FILE AS IT WAS NOT REQUIRED


//HELPER FUNCTIONS
void process_temperature();
void play_note(unsigned short freq);
void variable_delay_us(int delay);
void blink_red_LED();
void stop_blink_red_LED();
void clearScreen();


enum states {COOL, WARM, HOT};
int current_temperature;
unsigned char temperature_ones = 0;
unsigned char temperature_tenths = 0;
unsigned char current_temperature_state = COOL;

volatile unsigned char led_blink_count;
volatile unsigned int remaining_half_cycles;
volatile int rotary_count;
volatile int temperature_threshold;
volatile int current_note;
volatile unsigned char ISR_note_count;
volatile unsigned char temperature_state = 2; // start at 2 for the initialization print
volatile unsigned char new_state, old_state; // used for rotary encoder
volatile unsigned char a, b; // used for rotary encoder
volatile unsigned char initial = 0; // used for rotary encoder
volatile unsigned char changed = 1;  // Flag for state change


#define DEFAULT_TEMPERATURE 80
#define DEFAULT_NOTE 10
#define TEMPERATURE_ADDRESS 1
#define NOTE_ADDRESS 2


/*
  The note_freq array contains the frequencies of the notes from
  C3 to C5 in array locations 0 to 24.  Used calculate the timer delay
  and for the length of the time the note is played.
*/
#define NUM_TONES 25

unsigned int note_freq[NUM_TONES] =
{ 131, 139, 147, 156, 165, 176, 185, 196, 208, 220, 233, 247,
  262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494, 523};

char* note_letter[12] =
{ "C ", "C#", "D ", "D#", "E ", "F ", "F#", "G ", "G#", "A ", "A#", "B " };

int main(){
  //Initialization:
 lcd_init();
 ds_init();

 //check eeprom
 temperature_threshold = eeprom_read_byte((void*) TEMPERATURE_ADDRESS);
 if(temperature_threshold > 100 || temperature_threshold < 60){
   temperature_threshold = DEFAULT_TEMPERATURE;
 }

 current_note = eeprom_read_byte((void*) NOTE_ADDRESS);
 if(current_note == 0xff){
   current_note = DEFAULT_NOTE;
 }

 // initialize TIMER0
 //Clear Timer on Compare
 TCCR0A |= (1<< WGM01);
 //16000000 / 256 / 131 / 2 = 238.5 --> value will always fit --> any frequncy higher will fit
 //Set prescalar to 256  (100 is the code for 256 prescalar)
 TCCR0B |= ( (1<< CS02) );

 //iniitalize TIMER1  blink_red_LED
 //Clear Timer on Compare
 TCCR1B |= (1<< WGM12);
 //Load the MAX count (Prescalar = 64, counting to 25,000, 0.1s expected delay)
 OCR1A = 25000;
 //Set prescalar to 64 and start counter (011 is the code for 64)
 TCCR1B |= ( (1<< CS11) | (1 << CS10) );


//Set up LEDs for output
  DDRD |= ((1<<PD2) | (1<< PD3));

 //Pullup resistors for buttons
 PORTC |= (1 << PC1);
 PORTC |= (1 << PC2);

 //From lab 7 for Rotary Encoder
 DDRC |= (1 << PC5); // Enable output for buzzer
 PORTB |= ( (1 << PB3) | (1 << PB4)); // Pullup resistor for rotator
 PCICR |= (1 << PCIE0); // Set PCICR to enable pin change interrupts on PORTB
 PCMSK0 |= ( (1 << PCINT3) | (1 << PCINT4)); // set PCMSK0 to enable on PORTB

 //Initialize Rotary Threshold to Note
 rotary_count = current_note;

 //Enable Global Interrupts
 sei();

 // Show the splash screen
 clearScreen();
 lcd_stringout("Jaiveer Project");
 _delay_ms(1000);
 clearScreen();


//FROM LAB 7 INITIALIZE
 initial = PINC;
 a=0;
 b=0;
 a |= ( initial & (1 << PC1) );
 b |= ( initial & (1 << PC5) );

 if (!b && !a)
old_state = 0;
 else if (!b && a)
old_state = 1;
 else if (b && !a)
old_state = 2;
 else
old_state = 3;

 new_state = old_state;
 //END LAB 7 INITIALIZE

while(1){
  //Continuosly Check Room Temperature and update temperature state
  process_temperature(); // This function will check temperature and update the screen if it has changed

  //Code to update temperature state
  if(current_temperature_state == COOL){
    if(temperature_state == 2){
      //turn on green light on intialization
      PORTD |= (1 << PD3);
    }
    if(current_temperature > temperature_threshold*16){
      if (current_temperature - temperature_threshold*16 > 2*16){
        current_temperature_state = HOT;
        //turn off green light and turn on red light
        PORTD &= ~(1 << PD3);
        PORTD |= (1 << PD2);
        play_note(note_freq[current_note]);
        changed = 1;
      }
      else{
        current_temperature_state = WARM;
        //turn off green light
        PORTD &= ~(1 << PD3);
        blink_red_LED();
        changed = 1;
      }
    }
  }
  else if(current_temperature_state == WARM){
    if(current_temperature <= temperature_threshold*16){
      current_temperature_state = COOL;
      stop_blink_red_LED();
      //turn on green light
      PORTD |= (1 << PD3);
      changed = 1;
    }
    else if (current_temperature - temperature_threshold*16 > 2*16){
      current_temperature_state = HOT;
      stop_blink_red_LED();
      //Leave red light on
      PORTD |= (1 << PD2);
      play_note(note_freq[current_note]);
      changed = 1;
    }
  }
  else if(current_temperature_state == HOT){
    if(current_temperature <= temperature_threshold*16){
      current_temperature_state = COOL;
      //turn off red and turn on green lights
      PORTD &= ~(1 << PD2);
      PORTD |= (1 << PD3);
      changed = 1;
    }
    else if(current_temperature - temperature_threshold*16 < 2*16){
      current_temperature_state = WARM;
      //turn off red light then blink LED
      PORTD &= ~(1 << PD2);
      blink_red_LED();
      changed = 1;
    }
  }

  //Continuosly Check Button Press
  char button_press = PINC;
  //Alert Note Button
  if(!(button_press & (1 << PC1))){
  //  lcd_stringout("Alert");
    temperature_state = 0;
    rotary_count = current_note;
    lcd_moveto(1,11);
    lcd_stringout(">");
    lcd_moveto(1,5);
    lcd_stringout(" ");
  }
  //Temperature Threshold button
  else if(!(button_press & (1<< PC2))){
  //  lcd_stringout("Temperature");
    temperature_state = 1;
    rotary_count = temperature_threshold;
    lcd_moveto(1,11);
    lcd_stringout(" ");
    lcd_moveto(1,5);
    lcd_stringout("<");
  }

  //Check if State Changed due to Rotary Encoder
  if (changed) { // Did state change?
  changed = 0;        // Reset changed flag
  char line1[17];
  //Update either Temperature Threshold or Alert Note Print
  //temperature_State == 2 is for the first time we print to screen, then it is either 1 (temp) or 0 (alert note button)
  if(temperature_state == 2){
    lcd_moveto(1,11);
    lcd_stringout(">");
    lcd_moveto(1,0);
    snprintf(line1, 5, "%4d", temperature_threshold);
    lcd_stringout(line1);
    lcd_moveto(0,13);
    lcd_stringout(note_letter[current_note%12]);
    lcd_moveto(1,12);
    snprintf(line1, 5, "%4d", (current_note/12 + 3));
    temperature_state = 0;
  }
  else if(temperature_state){
    lcd_moveto(1,0);
    snprintf(line1, 5, "%4d", temperature_threshold);
    eeprom_update_byte((void *) TEMPERATURE_ADDRESS, temperature_threshold);
  }
  else{
    lcd_moveto(0,13);
    lcd_stringout(note_letter[current_note%12]);
    lcd_moveto(1,12);
    snprintf(line1, 5, "%4d", (current_note/12 + 3));
    eeprom_update_byte((void *) NOTE_ADDRESS, current_note);
  }

  lcd_stringout(line1);
  lcd_moveto(0,6);

  //Code to print the current state to screen
  if(current_temperature_state == HOT){
    lcd_stringout("HOT ");
  }
  else if(current_temperature_state == WARM){
    lcd_stringout("WARM");
  }
  else if (current_temperature_state == COOL){
    lcd_stringout("COOL");
  }
}

}
}


void process_temperature(){

  //Create a char array and call ds_convert to get the correct data from ds
  unsigned char data[2];
  ds_convert();
  while(1){
  if(ds_temp(data)){
    break;
  }
}
  // char line1[17];
  // //data[1] &= 7;
  // snprintf(line1, 8, "%d%d", data[1], data[0]);
  // lcd_moveto(1,0);
  // lcd_stringout(line1);

  //Put 2 8-bit values in one int called temperature
  int temperature = 0;

  temperature = (data[1]  << 8) + data[0];

  // temperature |= (data[1]  << 8);
  // temperature |= data[0];

  //Now Temperature contains all the data scaled at 2^4 (16x)
  temperature *= 9;
  temperature /= 5;

  //check if there was a change in temperature, if so, print out new temperature
  //NOTE: We are anding temperature with 15 (1111) to isolate the right-most 4 bits , which are the bits that represent decimal places
  if( (temperature_ones != (temperature >> 4)) || (temperature_tenths != (temperature & 15) ) ){
  temperature_ones = 0;
  temperature_tenths = 0;
  temperature_ones |= (temperature >> 4);
  temperature_tenths |= (temperature & 15);
  temperature_ones += 32;

  char line1[6];
  //data[1] &= 7;
  snprintf(line1, 6, "%3d.%d", temperature_ones, temperature_tenths);
  lcd_moveto(0,0);
  lcd_stringout(line1);
  temperature += 512; //512 is 32*16 which is the scale we need to apply to 32 when addiing
  current_temperature = temperature;

  //Note --> when printing to led we use temperature_ones and temperature_tenths, but internally we use current_temperature to keep track of the value
  //this makes making comparisons in other parts of the code easier
}

}


void clearScreen(){
  lcd_moveto(0,0);
  lcd_stringout("                 ");
  lcd_moveto(1,0);
  lcd_stringout("                 ");
  lcd_moveto(0,0);
}

ISR(PCINT0_vect){
  //Taken from previous lab
  // Read the input bits and determine A and B.
  initial = 0;
  initial |= PINB;
  a = 0;
  b = 0;
  a |= ( initial & (1 << PB3) );
  b |= ( initial & (1 << PB4) );


  if (old_state == 0) {

      // Handle A and B inputs for state 0
      if(a){
        new_state = 1;
        rotary_count++;
      }
      else if(b){
        new_state = 2;
        rotary_count--;
      }
  }
  else if (old_state == 1) {

      // Handle A and B inputs for state 1
      if(b){
        new_state = 3;
        rotary_count++;
      }
      else if(!a){
        new_state = 0;
      rotary_count--;
      }
  }
  else if (old_state == 2) {

      // Handle A and B inputs for state 2
      if(!b){
        new_state = 0;
        rotary_count++;
      }
      else if(a){
        new_state = 3;
        rotary_count--;
      }
  }
  else {   // old_state = 3

      // Handle A and B inputs for state 3
      if(!b){
        new_state = 1;
        rotary_count--;
      }
      else if(!a){
        new_state = 2;
        rotary_count++;
      }
  }

  //Use Rotary count value to update either threshold or current note, depending on current temperature_state
  if(temperature_state){
    if(rotary_count < 60){
      rotary_count = 60;
    }
    if(rotary_count > 100){
      rotary_count = 100;
    }
    temperature_threshold = rotary_count;
  }
  else{
    if(rotary_count < 0){
      rotary_count = 0;
    }
    if(rotary_count > 24){
      rotary_count = 24;
    }
    current_note = rotary_count;
  }
    	// If state changed, update the value of old_state,
    	// and set a flag that the state has changed.
    	if (new_state != old_state) {
    	    changed = 1;
    	    old_state = new_state;
    	}
}

/*
  Play a tone at the frequency specified for one second
*/
void play_note(unsigned short freq)
{
   //16000000 / 256 / 2 = 31250
   //length that each node will need to cound to is 31250 / Frequency
   OCR0A = 16000000 / 256 / 2 / freq;
   //Number of half cycles to run -- frequency of note
   remaining_half_cycles = freq;
    //CODE TO Turn on TIMER:
   TIMSK0 |= (1 << OCIE0A);
}

//Blinks it at 1Hz (set by initialization)
void blink_red_LED(){
  //turn on timer
  TIMSK1 |= (1 << OCIE1A);
}

void stop_blink_red_LED(){
  //turn off timer
  TIMSK1 &= ~(1 << OCIE1A);
  //Make sure red LED is off
  PORTD &= ~(1 << PD2);
}

//Used to play note
ISR(TIMER0_COMPA_vect){
  //Decrement frequncy value
  remaining_half_cycles--;
  //turn off the timer here, let the state machine know we're done
  if(remaining_half_cycles == 0){
    //CODE TO Turn Off TIMER:
   TIMSK0 &= ~(1 << OCIE0A);
  }
  //Otherwise --> toggle PORT bit back and forth
  else{
    //put a one in the PIN Register, that toggles it
    // Toggle PC5 (Buzzer), by setting PINC to 1
      PINC |= (1 << PC5);
  }
}

//Used to blink LED
ISR(TIMER1_COMPA_vect){
  led_blink_count++;
  if(led_blink_count == 10){
    PIND |= (1 << PD2);
    led_blink_count = 0;
  }
}
