/* #ifndef USART_BAUD
#define USART_BAUD 9600UL
#endif */

void initUSART(uint32_t baud);
void initUSART_RX_ISR(uint32_t baud);

void transmitByte(uint8_t data);
uint8_t receiveByte(void);

void printString(const char myString[]);
void readString(char myString[], uint8_t maxLength);

void transmitASCII_8(uint8_t byte);
void transmitASCII_16(uint16_t word);

void transmitBIN(uint8_t byte);

void transmitHEX_4(uint8_t nibble);
void transmitHEX_8(uint8_t byte);

uint8_t getNumber(void);

#define   USART_HAS_DATA    bit_is_set(UCSR0A, RXC0)
#define   USART_BUF_EMPTY   bit_is_set(UCSR0A, UDRE0)
