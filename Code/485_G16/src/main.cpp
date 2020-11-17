//#include <Arduino.h>

#include <avr/io.h>
#include "USART.h"

uint8_t address;

void setup(){

    PORTB |= (1 << PB3 | 1 << PB2 | 1 << PB1 | 1 << PB0);
    PORTC |= (1 << PC2 | 1 << PC1);

    DDRB |= (1 << PB5);
    DDRC |= (1 << PC0);
    DDRD |= (1 << PD1);
 
    address = PINB & (1 << PIN3 | 1 << PIN2 | 1 << PIN1 | 1 << PIN0);
    
    //PORTC |= !address << PC0; 

    initUSART(9600, !address);
}

void loop(){
    if(!address){
        PORTC |= !address << PC0; 
        transmitByte(PINC & (1 << PC2 | 1 << PC1));
        PORTC &= ~(!address << PC0); 
    }   
    else
    {
        uint8_t received = receiveByte();
        switch (address)
        {
        case 1:
            if(received & (1<<PC1)){
                PORTB |= (1<<PB5);
            }
            else
            {
                PORTB &= ~(1<<PB5);
            }
            break;
        case 2:
            if(received & (1<<PC2)){
                PORTB |= (1<<PB5);
            }
            else
            {
                PORTB &= ~(1<<PB5);
            }
        default:
            break;
        }
    }
}

void main(){
    setup();
    while(1) loop();
}