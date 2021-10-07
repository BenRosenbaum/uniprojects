/*
 * File:   ELEC3042_45582912_Minor_Project.c
 * Author: benrosenbaum
 *
 * Created on April 2, 2021, 11:51 AM
 */

//only specifications have been completed, not the extension

#include <avr/io.h>
#include <xc.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>


//volatile uint32_t   cur_ms = 0;     // time in milliseconds since system started
//volatile uint32_t   cur_ms_ovf = 0; // overflow of cur_ms
//volatile uint8_t    cur_AtoD = 0;   // current value from A to D converter

//uint8_t addr = 0x27; //address for LCD


int state = 0;
int count =1;
int i;
int counter=1;





void timer_1setup(){
    cli();
    
     TCCR1A = (1 << WGM11);
    //TCCR1A= (1<<COM1A1)|(1<<WGM11)|(1<<WGM10)|(1<<COM1A0); //normal port operation - toggle OC2A on compare match
    //TCCR1B= (1<<WGM13)|(1<<WGM12)|(1<<CS12)|(1<<CS11)|(1<<CS10); //Mode 15 - FAST PWM mode - 8 prescalar 
    TCCR1B = (1 << WGM12)|(1 <<WGM13)|(1<<CS10);
    
     //overflow interrupt enabled
    DDRB|= (1<<PB1); 
    //OCR1A = 0x0FFF; //results in 1953Hz for buzzer
    TIMSK1= (1<<OCIE1A);
    TIFR1= (1<<OCF1A);
    //OCR1B = 0; //50% duty cycle 
    //TCNT1=0;
    OCR1A = 499; //2000Hz
    
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    
    sei(); //enable global interrupt
    
}
 

void timer_0setup(){
    //timer0
    cli(); //clear global interrupt flag
    
    OCR0A = 249/8; //interrupt occurs every 1/2000 second
    TCCR0A|= (1<<WGM01); //timer control register A
    TCCR0B|= (1<<CS02); //prescalar set to 256
    
    TCNT0 = 0; //timer counter
    
    //want interrupt
    TIMSK0 |= (1<<OCIE0A); //interrupt mask register - compare A match interrupt enable
    
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    
    sei(); //enable global interrupt
   
    
    
}

 




void setup() {
    DDRD  = 0b00000010; // allow UART to still be default
    PORTD = 0b11111101; // turn on pullups for all inputs
    DDRB  = 0b11111111; // all outputs, so we can use the default LED on PB5
    PORTB = 0;          // turn off all LEDs
    DDRC  = 0b00000000; // all inputs, including the I2C interface
    PORTC = 0b00111110; // do NOT turn on pullup for analog input
          
}


  //Functions for buttons and motion sensor provided below

int zone1pressed(){
    return ((PINC & _BV(1)) == 0);
}

int zone2pressed(){
    return((PINC & _BV(2))==0);  
}

int aPressed(){
    return((PIND & _BV(7))==0);  
}

int bPressed(){
    return((PIND & _BV(6))==0);
}

int cPressed(){
    return((PIND & _BV(5))==0); 
}

int dPressed(){
    return((PIND & _BV(4))==0);
}

int enterPressed(){
    return((PIND & _BV(2))==0);  
}

int cancelPressed(){
    return((PIND & _BV(3))==0);
}


void zone(){
    //subsystem - can be called whenever
    //trigger system - when sensor is triggered synonymous light will illuminate
    if (zone1pressed()){ //motion sensor 1 triggered
        PORTB=0x08; // zone 1 LED
    }
    if (zone2pressed()){ //motion sensor 2 is triggered
        PORTB=0x10; // zone 2 LED
    }
    else {
       PINB&=0x08; //LED for sensor 1 is off
       PINB&=0x10; //LED for sensor 2 is off
    }
}



void state0(){
    //all clear state
    PORTB=0x00; // all LEDS off and siren off
    if(PC1==1 || PC2==1){
    if (aPressed()){
        state = 1; // move onto next state if condition is met
   }
}
}
    

void state1(){
    //all clear state
    //PORTB=0x00; // all LEDS off and siren off
    
    if (enterPressed()){
        state=2; // move onto next state if condition is met

        
}
}

 

void state2(){
    //trigger state
    
    counter++;
            if(counter%2000==0){
    PORTB^=_BV(PB2); //armed LED flashes 5 times at 1 Hz
    
    count++;
    
    if (count>10){ //come back to;
        count = 1;
        PORTB|= _BV(PB2);
        state=3; // move onto next state if condition is met
   }
}
}





void state3(){
    
    //armed state - armed LED remains illuminated
    if (zone1pressed() || zone2pressed()){ //if motion sensor 1 or 2 is triggered
        state = 4; // move onto next state if condition is met
    }
    if(dPressed()){
        state=6;
    }
    
}



void state4(){
    //5 second delay after system armed
    counter++;
    if(counter%1000==0){
    
    count++;
    if (count > 10){ 
          count =1;  
        state = 5; // move onto next state if condition is met

}
    }
}




 
void state5(){
    //strobe light
    counter++;
    if(counter%1000==0){
    PORTB|=_BV(PB2); //ARMED LED remains on
    PORTB^=_BV(PB0);  //toggle strobe  
    } if(counter%2000==0){
    PORTB^=_BV(PB1); // toggle siren
    }
    if (dPressed()){
        state = 6; // move onto next state if condition is met
    }
}




void state6(){
    
    //disarming the system
    
    if (enterPressed()){
        state = 0; // move onto the first state if condition is met
    }
}



    


ISR(TIMER0_COMPA_vect) {
    
    //Timer called every half second)
    if(state == 0){
        state0();
    }
    if(state == 1){
        state1();
    }
    if(state == 2){
        state2();
    }
    if(state == 3){
        state3();
    }
     if(state == 4){
        state4();
    }
    if(state == 5){
        state5();
    }
    if(state ==6 ){
        state6();
    }
    
}
ISR(TIMER1_OVF_vect){
    PORTB^=_BV(PB1);  

}

int main (void){
    state=0;
    state0();
    setup();
    timer_1setup();
    timer_0setup();
    

    while (1) {
        
        zone();
        
        sei();
        sleep_cpu();
        sleep_disable();
       
}
}
