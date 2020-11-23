#include <avr/io.h>

#define UBRRH_VAL   ((F_CPU / (16 * baud)) - 1) >> 8
#define UBRRL_VAL   ((F_CPU / (16 * baud)) - 1) & 0xff

#define ADDRESSFRAME 1
#define DATAFRAME 0
#define MASTERADDRESS 0

volatile uint8_t address;
volatile uint8_t rxData;

void transmit(uint8_t isAddress, uint8_t data){
    while (!(UCSR0A & (1 << UDRE0)));

    if(isAddress)
        UCSR0B |= (1 << TXB80);
    else
        UCSR0B &= ~(1 << TXB80);
    UDR0 = data;
}

uint8_t receive(){
    while (!(UCSR0A & (1<<RXC0)));
    
    uint8_t resh, resl;

    resh = UCSR0B;
    resh = (resh >> RXB80) & 0x01;

    rxData = UDR0;
    return resh;
}

void initUSART(uint32_t baud, uint8_t isTX){

    UBRR0H = UBRRH_VAL;
    UBRR0L = UBRRL_VAL;

    UCSR0B |= (1 << UCSZ02);//| (1 << UPM00) | (1 << UPM01); //parity
    UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00); //set 9 bit data, and 1 stop bit

    if(isTX)
        UCSR0B |= (1 << TXEN0);
    else{
        UCSR0B |= (1 << RXEN0);
        UCSR0A = (1 << MPCM0);
    }
}

void setup(){
    PORTB |= (1 << PB3 | 1 << PB2 | 1 << PB1 | 1 << PB0); //Switches
    PORTC |= (1 << PC2 | 1 << PC1); //buttons

    DDRB |= (1 << PB5); //led
    DDRC |= (1 << PC0); //tx line
 
    address = PINB & (1 << PIN3 | 1 << PIN2 | 1 << PIN1 | 1 << PIN0);

    initUSART(9600, (address == MASTERADDRESS));
}

void slaveLoop(){
    while (1)
    {
        uint8_t isAddress = receive();

        if(isAddress){ //if addressFrame
            if(address == rxData){
                UCSR0A &= ~(1 << MPCM0); //wait for data frame
            }
            else
            {
                UCSR0A |= (1 << MPCM0); //make sure it looks for address fram
            }
        }
        else // if dataFrame
        {
            if(rxData == 0){
                PORTB |= (1<<PB5);
            }
            else{
                PORTB &= ~(1<<PB5);
            }
        }
    }
}

void masterLoop(){
    uint8_t prevButton1 = 1, prevButton2 = 1;
    uint8_t button1, button2;

    while (1)
    {
        button1 = (PINC & (1<<PC1)) >> PC1;
        button2 = (PINC & (1<<PC2)) >> PC2;

        if(button1 != prevButton1){
            transmit(ADDRESSFRAME, 1);
            transmit(DATAFRAME, button1);
        }
        if(button2 != prevButton2){
            transmit(ADDRESSFRAME, 2);
            transmit(DATAFRAME, button2);
        }

        prevButton1 = button1;
        prevButton2 = button2;
    }
}

int main(){
    setup();
    if(address == MASTERADDRESS){
        masterLoop();
    }
    else
    {
        slaveLoop();
    }
    return 0;
}