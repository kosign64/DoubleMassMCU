#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>

typedef unsigned char uchar;

#define ADC_VREF_TYPE 0x40

#define LED    2
#define TOGGLE 4
#define POT    7
#define HALL   6
#define DIR    2

int potentiometr = 0;
int control = 0;
bool manual;

unsigned int readAdc(unsigned char channel);
unsigned int readEncoder(void);
void sendByteUART(uchar byte);

int main(void)
{
    DDRB = _BV(0) | _BV(1) | _BV(2);
    DDRD = _BV(LED);

    // Timer 1 Fast PWM 8-bit, 11.0592 MHz
    //TCCR1A=0x81;
    TCCR1A = _BV(COM1A1) | _BV(WGM10);
    //TCCR1B=0x09;
    TCCR1B = _BV(WGM12) | _BV(CS10);

    //Timer 2 CTC, prescaler 64, interrupt freq ~= 1 kHz
    TCCR2 = _BV(WGM21) | _BV(CS22);
    OCR2 = 173;

    TIMSK = _BV(OCIE2);

    // UART
    UCSRA=0x00;
    UCSRB=0x98;
    UCSRC=0x86;
    UBRRH=0x00;
    UBRRL=0x05;

    // Disable Analog Comparator
    ACSR=0x80;

    // ADC AVcc with external cap and division factor 16
    ADMUX = _BV(REFS0);
    ADCSRA = _BV(ADEN) | _BV(ADPS2);
    //ADCSRA = 0x87;

    sei();

    while(1)
    {
        // Blink led
        PORTD ^= _BV(LED);
        if(bit_is_clear(PIND, 4))
        {
            unsigned int pot = readAdc(POT);
            pot = (pot + 1) / 2;
            cli();
            potentiometr = pot - 256;
            sei();
            manual = true;
        }
        else
        {
            manual = false;
        }
        _delay_ms(100);
    }
}

ISR(USART_RXC_vect)
{
    uchar data = UDR;
    control = ((int)data + 1) * 2;
    control -= 256;
}

// Control loop
ISR(TIMER2_COMP_vect)
{
    if(manual)
    {
        if(potentiometr >= 0)
        {
            PORTB |= _BV(DIR);
        }
        else
        {
            PORTB &= ~_BV(DIR);
            potentiometr = -potentiometr;
        }
        if(potentiometr > 0) potentiometr -= 1;
        OCR1AL = (uchar)potentiometr;
    }
    else
    {
        if(control >= 0)
        {
            PORTB |= _BV(DIR);
        }
        else
        {
            PORTB &= ~_BV(DIR);
            control = -control;
        }
        if(control > 0) control -= 1;
        OCR1AL = (uchar)control;
    }
    sendByteUART(255);
    unsigned int encoder = readEncoder();
    unsigned int current = readAdc(HALL);
    unsigned int tmp = current >> 7;
    sendByteUART(tmp);
    tmp = current - (tmp << 7);
    sendByteUART(tmp);
    tmp = encoder >> 7;
    sendByteUART(tmp);
    tmp = encoder - (tmp << 7);
    sendByteUART(tmp);
}


unsigned int readAdc(unsigned char channel)
{
    ADMUX = channel | (ADC_VREF_TYPE & 0xff);
    _delay_us(10);
    ADCSRA |= 0x40;
    while((ADCSRA & 0x10) == 0);
    ADCSRA |= 0x10;

    return ADCW;
}

unsigned int readEncoder(void)
{
    char tmp,tmp2;
    unsigned int result;

    PORTB &= ~_BV(0);
    _delay_us(1);
    tmp = PINC;
    tmp = tmp & 63;
    PORTB |= _BV(0);
    _delay_us(1);
    tmp2 = PINC;
    tmp2 = tmp2 & 15;
    result = (unsigned int)tmp2 * 64 + tmp;

    return result;
}

void sendByteUART(uchar byte)
{
    while(!(UCSRA & _BV(UDRE)));
    UDR = byte;
}
