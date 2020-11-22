#include <avr/io.h>
#include <avr/interrupt.h>

#define UBRRH_VAL   ((F_CPU / (16 * baud)) - 1) >> 8
#define UBRRL_VAL   ((F_CPU / (16 * baud)) - 1) & 0xff

uint8_t volatile address;

ISR(USART_RX_vect){
    uint16_t received = receiveData();
    if(received & 1){
        if((received >> 1) == address)
            UCSR0A &= ~(1 << MPCM0); //await data frame by disabling Multi-processor mode
        
    }
    else
    {
        if(received >> 1)
            PORTB &= ~(1<<PB5);
        else
            PORTB |= (1<<PB5);

        UCSR0A |= (1 << MPCM0); //reactivate Multi-processor mode
    }
}

void initUSART_ISR(uint32_t baud, uint8_t isTX) {
    UBRR0H = UBRRH_VAL;
    UBRR0L = UBRRL_VAL;

    UCSR0B = (isTX << TXEN0) | (!isTX << RXEN0) | (!isTX << RXCIE0) | (1 << UPM00) | (1 << UPM01);//interrupt enable and parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
} 

void transmitData(uint16_t data) { 
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UCSR0B &= ~(1<<TXB80);
    if (data & 8)
        UCSR0B |= (1<<TXB80);
    UDR0 = (uint8_t) data;
}

uint16_t receiveData(void) {
    uint8_t status, resh, resl;

    status = UCSR0A;
    resh = UCSR0B;
    resl = UDR0;
    if (status & (1<<FE0)|(1<<DOR0)|(1<<UPE0))
    return -1;

    resh = (resh >> 1) & 0x01;
    return ((resh << 8) | resl);
}

void setup(){
    PORTB |= (1 << PB3 | 1 << PB2 | 1 << PB1 | 1 << PB0);
    PORTC |= (1 << PC2 | 1 << PC1);

    DDRB |= (1 << PB5);
    DDRC |= (1 << PC0);
    DDRD |= (1 << PD1);
 
    address = PINB & (1 << PIN3 | 1 << PIN2 | 1 << PIN1 | 1 << PIN0);

    initUSART_ISR(9600, !address);

    sei();
}

void slaveLoop(){

    while(1);
}

void masterLoop(){
    uint8_t buttonStatus, prevButtonStatus = 0;

    while(1){
        buttonStatus = PINC & (1 << PC2 | 1 << PC1);
        if(buttonStatus != prevButtonStatus){
            if((buttonStatus&1) != (prevButtonStatus&1)){ //address 1
                PORTC |= !address << PC0; 
                transmitData((1 << 1) + 1);                 //address
                transmitData((buttonStatus & 1) << 1 + 0);  //data frame
                PORTC &= ~(!address << PC0); 
            }
            else if((buttonStatus&2) != (prevButtonStatus&2)){ //address 2
                PORTC |= !address << PC0; 
                transmitData((2 << 1) + 1);                 //address frame
                transmitData((buttonStatus & 2) << 0 + 0);  //data frame
                PORTC &= ~(!address << PC0); 
            }
        } 
    }
}

void main(){
    setup();
    if(!address){
        masterLoop();
    }
    else
    {
        slaveLoop();
    }
}