/*
 * main.c
 *
 * Created: 9/4/2025 10:22:27 AM
 * Author: felip
 */

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//==================== LCD ====================
// Bit definitions based on the 74HC595 connections in the circuit
// Q0 = RS, Q1 = EN
#define LCD_RS  (1 << 0)
#define LCD_EN  (1 << 1)

#define LATCH (1 << PB2) // PB2 as the Latch pin

//============ ADC =========
void init_adc() {
    ADMUX = (1 << REFS0); // AVcc como referência
    ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADIE) |
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler 128
    ADCSRB = 0; // Free running
    DIDR0 = (1 << ADC0D);
}

//========= SPI =================
void spi_init(void) {
    DDRB |= (1 << PB3) | (1 << PB5) | LATCH; // MOSI, SCK, and LATCH as outputs
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0)|| (1 << SPR1); // Master mode, Fosc/16
	
}

void spi_send(uint8_t data) {
    SPDR = data;
    while (!(SPSR & (1 << SPIF)));
    
    // Pulse the latch pin
    PORTB |= LATCH;
    _delay_us(10); // A small delay is important for the simulator
    PORTB &= ~LATCH;
}

//========= LCD Low-Level =========
void lcd_send(uint8_t data, uint8_t mode) {
    uint8_t high_nibble, low_nibble;
    
    // Prepare the high nibble with control bits
    high_nibble = (data & 0xF0) | mode;
    
    // Prepare the low nibble with control bits
    low_nibble = ((data << 4) & 0xF0) | mode;

    // Send high nibble with EN pulse
    spi_send(high_nibble | LCD_EN);
    _delay_us(500); // Increased delay for stability
    spi_send(high_nibble & ~LCD_EN);
    _delay_us(500);

    // Send low nibble with EN pulse
    spi_send(low_nibble | LCD_EN);
    _delay_us(500);
    spi_send(low_nibble & ~LCD_EN);
    _delay_us(500);
}

void lcd_cmd(uint8_t cmd) {
    lcd_send(cmd, 0); // Mode 0 for Command
    _delay_ms(2);
}

void lcd_write(char letra) {
    lcd_send(letra, LCD_RS); // Mode LCD_RS for Data
}

void lcd_clr() {
    lcd_cmd(0x01); // Clear display
    _delay_ms(2);  // Delay is needed
}

void lcd_init() {
    _delay_ms(50); // Wait for LCD to power up
    
    // The following commands are for 4-bit mode initialization
    lcd_send(0x30, 0);
    _delay_ms(5);
    lcd_send(0x30, 0);
    _delay_us(200);
    lcd_send(0x30, 0);
    _delay_us(200);
    
    lcd_send(0x20, 0); // Set to 4-bit mode
    _delay_us(200);
    
    // Final configuration commands
    lcd_cmd(0x28); // 4 bits, 2 lines, 5x8 dots
    lcd_cmd(0x0C); // Display ON, Cursor OFF
    lcd_cmd(0x06); // Cursor increment mode
    lcd_clr();
}

//================ Main ====================
int main() {
    spi_init();
    lcd_init();

    // Test writing
    lcd_write('A');
    lcd_cmd(0xC0); // second line
    lcd_write('B');
    lcd_write('C');

    while(1) {
        // main loop
    }
}