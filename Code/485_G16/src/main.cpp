//#include <Arduino.h>

#include <avr/io.h>
//#include <avr/interrupt.h>
//#include "USART.h"

#define UBRRH_VAL   ((F_CPU / (16 * baud)) - 1) >> 8
#define UBRRL_VAL   ((F_CPU / (16 * baud)) - 1) & 0xff

uint8_t address;

void initUSART(uint32_t baud, uint8_t isTX) {
    UBRR0H = UBRRH_VAL;
    UBRR0L = UBRRL_VAL;

    UCSR0A &= ~(1 << U2X0);

    UCSR0B = (isTX << TXEN0) | (!isTX << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00) | (1 << UPM00) | (1 << UPM01); // - parity
}

void transmitByte(uint8_t data) {
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = data;
}

uint8_t receiveByte(void) {
    loop_until_bit_is_set(UCSR0A, RXC0);
    return UDR0;
}

void setup(){
    PORTB |= (1 << PB3 | 1 << PB2 | 1 << PB1 | 1 << PB0);
    PORTC |= (1 << PC2 | 1 << PC1);

    DDRB |= (1 << PB5);
    DDRC |= (1 << PC0);
    DDRD |= (1 << PD1);
 
    address = PINB & (1 << PIN3 | 1 << PIN2 | 1 << PIN1 | 1 << PIN0);

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
                PORTB &= ~(1<<PB5);
            }
            else
            {
                PORTB |= (1<<PB5);
            }
            break;
        case 2:
            if(received & (1<<PC2)){
                PORTB &= ~(1<<PB5);
            }
            else
            {
                PORTB |= (1<<PB5);
            }
        default:
            break;
        }
    }
}

void masterLoop(){
    PORTC |= !address << PC0; 
    transmitByte(PINC & (1 << PC2 | 1 << PC1));
    PORTC &= ~(!address << PC0); 
}

void slaveLoop(){
    uint8_t received = receiveByte();
    switch (address)
    {
    case 1:
        if(received & (1<<PC1)){
            PORTB &= ~(1<<PB5);
        }
        else
        {
            PORTB |= (1<<PB5);
        }
        break;
    case 2:
        if(received & (1<<PC2)){
            PORTB &= ~(1<<PB5);
        }
        else
        {
            PORTB |= (1<<PB5);
        }
    default:
        break;
    }
}

int main(){
    setup();

    if(!address) while (1) masterLoop();
    else while(1) slaveLoop();

    return 0;
}