#include <avr/io.h>
#include <avr/interrupt.h>
#include "USART.h"

#define UBRRH_VAL   ((F_CPU / (16 * baud)) - 1) >> 8
#define UBRRL_VAL   ((F_CPU / (16 * baud)) - 1) & 0xff

void initUSART(uint32_t baud) {
    UBRR0H = UBRRH_VAL;
    UBRR0L = UBRRL_VAL;
#if USE_2X
    UCSR0A |= (1 << U2X0);
#else
    UCSR0A &= ~(1 << U2X0);
#endif
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void initUSART_RX_ISR(uint32_t baud) {
    UBRR0H = UBRRH_VAL;
    UBRR0L = UBRRL_VAL;
#if USE_2X
    UCSR0A |= (1 << U2X0);
#else
    UCSR0A &= ~(1 << U2X0);
#endif
    UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void transmitByte(uint8_t data) {
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = data;
}

uint8_t receiveByte(void) {
    loop_until_bit_is_set(UCSR0A, RXC0);
    return UDR0;
}

void printString(const char myString[]) {
    uint8_t i = 0;
    while (myString[i]) {
        transmitByte(myString[i]);
        i++;
    }
}

void readString(char myString[], uint8_t maxLength) {
    char response;
    uint8_t i = 0;
    while (i < (maxLength - 1)) {
        response = receiveByte();
        transmitByte(response);
        if (response == '\r') {
            break;
        } else {
        myString[i] = response;                       /* add in a letter */
        i++;
        }
    }
    myString[i] = 0;                          /* terminal NULL character */
}

void transmitASCII_8(uint8_t byte) {
    transmitByte('0' + (byte / 100));
    transmitByte('0' + ((byte / 10) % 10));
    transmitByte('0' + (byte % 10));
}

void transmitASCII_16(uint16_t word) {
    transmitByte('0' + (word / 10000));
    transmitByte('0' + ((word / 1000) % 10));
    transmitByte('0' + ((word / 100) % 10));
    transmitByte('0' + ((word / 10) % 10));
    transmitByte('0' + (word % 10));
}

void transmitBIN(uint8_t byte) {
    uint8_t bit;
    for (bit = 7; bit < 255; bit--) {
        if (bit_is_set(byte, bit)) {
            transmitByte('1');
        } else {
            transmitByte('0');
        }
    }
}

void transmitHEX_4(uint8_t nibble) {
    if (nibble < 10) {
        transmitByte('0' + nibble);
    } else {
        transmitByte('A' + nibble - 10);
    }
}

void transmitHEX_8(uint8_t byte) {
    uint8_t nibble;
    nibble = (byte & 0b11110000) >> 4;
    transmitHEX_4(nibble);
    nibble = byte & 0b00001111;
    transmitHEX_4(nibble);
}

uint8_t getNumber(void) {
    char hundreds = '0';
    char tens = '0';
    char ones = '0';
    char thisChar = '0';
    do {
        hundreds = tens;
        tens = ones;
        ones = thisChar;
        thisChar = receiveByte();
        transmitByte(thisChar);
    } while (thisChar != '\r');
    return (100 * (hundreds - '0') + 10 * (tens - '0') + ones - '0');
}
