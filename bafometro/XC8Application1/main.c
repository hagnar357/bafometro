/*
 * main.c
 * ATmega328P @ 8 MHz
 * HD44780 via 74HC595 (4 bits)
 */

#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <stdio.h>

// ==================== DEFINIÇÕES DE HARDWARE ====================
#define LCD_RS (1 << 0)
#define LCD_EN (1 << 1)

#define LATCH (1 << PB2)   // STCP do 74HC595 no PB2
#define MOSI  (1 << PB3)
#define SCK   (1 << PB5)

// ==================== DEFINIÇÕES DO SENSOR MQ-3 ====================
#define VCC     5.0
#define RL      200000.0
#define R0      60000.0

#define A 0.66
#define B 1.25

// ==================== VARIÁVEIS GLOBAIS ====================
#define NUM_MED 10
uint16_t medidas[NUM_MED] = {0};
uint8_t start_test = 1;       // lógica invertida, 0 inicia
uint8_t tempo = 1;            // flag do timer
uint8_t unidade = 0;          // 0:mg/L, 1:BAC, 2:copos
uint8_t att_display = 0;      // flag de atualização do display
uint8_t med_index = 0;        // índice de medidas

// ==================== SPI ====================
void spi_init(void) {
    DDRB |= MOSI | SCK | LATCH;
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
    SPSR &= ~(1 << SPI2X);
    PORTB &= ~LATCH;
}

void latch_pulse(void) {
    PORTB |= LATCH;
	PORTB |= LATCH;
    PORTB &= ~LATCH;
}

void spi_shift(uint8_t b) {
    SPDR = b;
    while (!(SPSR & (1 << SPIF)));
    latch_pulse();
}

// ==================== LCD ====================
void lcd_send_nibble(uint8_t nibble_hi, uint8_t mode) {
    uint8_t payload = (nibble_hi & 0xF0) | (mode & LCD_RS);
    spi_shift(payload | LCD_EN);
    spi_shift(payload & ~LCD_EN);
}

void lcd_send(uint8_t data, uint8_t mode) {
    lcd_send_nibble(data & 0xF0, mode);
    lcd_send_nibble((data << 4) & 0xF0, mode);
}

void lcd_cmd(uint8_t cmd) {
    lcd_send(cmd, 0);
    if (cmd == 0x01 || cmd == 0x02);
}

void lcd_write(char c) {
    lcd_send((uint8_t)c, LCD_RS);
}

void lcd_clr(void) {
    lcd_cmd(0x01);
}

void lcd_print(const char texto[]) {
    uint8_t i = 0;
    while (texto[i] != '\0') {
        lcd_write(texto[i]);
        i++;
    }
}

void lcd_init(void) {
    _delay_ms(50);
    lcd_send_nibble(0x30, 0);
    lcd_send_nibble(0x30, 0);
    lcd_send_nibble(0x30, 0);
    lcd_send_nibble(0x20, 0);

    lcd_cmd(0x28); // 4 bits, 2 linhas, 5x8
    lcd_cmd(0x0C); // display ON, cursor OFF
    lcd_cmd(0x06); // entry mode
    lcd_clr();
}

// ==================== TIMER1 ====================
void init_timer1(void) {
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10); // CTC, prescaler 1024
    OCR1A = 39060;  // ~0,5s @ 8MHz
    TIMSK1 |= (1 << OCIE1A);
}

// ==================== BOTÕES ====================
void init_buttons(void) {
    DDRD &= ~((1 << PD2) | (1 << PD3));
    PORTD |= (1 << PD2) | (1 << PD3);
    EICRA |= (1 << ISC01) | (1 << ISC11);
    EICRA &= ~((1 << ISC00) | (1 << ISC10));
    EIMSK |= (1 << INT0) | (1 << INT1);
}

// ==================== ADC ====================
void adc_init(void) {
    ADMUX = (1 << REFS0); // AVCC como referência, canal ADC0
    ADCSRA = (1 << ADEN)  // habilita ADC
           | (1 << ADIE)  // habilita interrupção
           | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler 128
}

// ==================== CONVERSÕES ====================
double converte_mgl(double adc_value) {
    double vout, rs, ratio, ppm, mgL;
    vout = (adc_value / 1023.0) * VCC;
    if (vout == 0) return 0;
    rs = RL * (VCC - vout) / vout;
    ratio = rs / R0;
    ppm = pow(10, (-A * log10(ratio) + B));
    mgL = (ppm * 46.07) / 24450.0;
    return mgL;
}

double converte_bac(double mgl) {
    return (mgl * 0.21);
}

double converte_copos(double bac, double weight_kg, double r_factor) {
    double weight_g = weight_kg * 1000.0;
    double a = bac * r_factor * weight_g;
    return a / 13.8;
}

void double_to_string(double val, int prec, char* s) {
	// Extrair a parte inteira
	long int_part = (long)val;
	double dec_part = val - (double)int_part;

	// Converte a parte inteira para string
	if (int_part == 0) {
		*s++ = '0';
		} else {
		int i = 0;
		char temp[10];
		while (int_part > 0) {
			temp[i++] = (int_part % 10) + '0';
			int_part /= 10;
		}
		while (i > 0) {
			*s++ = temp[--i];
		}
	}

	// Adiciona o ponto decimal, se a precisão for maior que 0
	if (prec > 0) {
		*s++ = '.';

		// Converte a parte decimal
		while (prec-- > 0) {
			dec_part *= 10;
			long digit = (long)dec_part;
			*s++ = digit + '0';
			dec_part -= digit;
		}
	}

	// Adiciona o terminador nulo
	*s = '\0';
}
// ==================== INTERRUPÇÕES ====================
ISR(ADC_vect) {
    medidas[med_index++] = ADC;
    if (med_index >= NUM_MED) med_index = 0;
}

ISR(TIMER1_COMPA_vect) {
    tempo = 0;
}

ISR(INT0_vect) {
    unidade = (unidade + 1) % 3;
    att_display = 1;
}

ISR(INT1_vect){
    start_test = 1 - start_test;
}

// ==================== DISPLAY ====================
void lcd_show_selected(double mgl, double bac, double copos, uint8_t unidade) {
	char buffer[5];
	lcd_clr();
	switch (unidade) {
		case 0:
		lcd_print("mg/L: ");
		double_to_string(mgl, 3, buffer);
		lcd_print(buffer);
		break;
		case 1:
		lcd_print("BAC: ");
		double_to_string(bac, 3, buffer);
		lcd_print(buffer);
		lcd_cmd(0xC0);
		lcd_print("g/dL");
		break;
		case 2:
		lcd_print("Copos: ");
		double_to_string(copos, 3, buffer);
		lcd_print(buffer);
		break;
		default:
		lcd_print("Unidade invalida");
		break;
	}
}

//==========================WATCHDOG======================
void wdt_enable() {
	// Define o prescaler para 1s (com F_CPU=8MHz)
	WDTCSR |= (1 << WDCE) | (1 << WDE);
	WDTCSR = (1 << WDE) | (1 << WDP3) | (1 << WDP0); //8s
}

void wdt_reset() {
	__asm__ __volatile__ ("wdr");
}

void wdt_desable(){
	WDTCSR |= (1 << WDCE) | (1 << WDE);
	WDTCSR = 0x00;
}

// ==================== INICIALIZAÇÃO ====================
void init(void) {
    spi_init();
    lcd_init();
    init_buttons();
    init_timer1();
    adc_init();
}

// ==================== MAIN ====================
int main(void) {
    init();
    sei();

    double valor_mgl, valor_bac, valor_copos;



    while (1) {
		wdt_desable();
		lcd_clr();
		lcd_print("Bafometro");
        while (start_test); // espera botão
        lcd_clr();
        lcd_print("Iniciando");

        // reseta medidas
        for (uint8_t i = 0; i < NUM_MED; i++) medidas[i] = 0;
        med_index = 0;

        // inicia conversões ADC contínuas
        ADCSRA |= (1 << ADSC);

        TCNT1 = 0;
        tempo = 1;
		wdt_enable();
        while (tempo) {
            // dispara ADC continuamente enquanto espera o timer
            if (!(ADCSRA & (1 << ADSC))) ADCSRA |= (1 << ADSC);
        }

        // calcula média
        double soma = 0;
        for (uint8_t i = 0; i < NUM_MED; i++) soma += medidas[i];
        double media_adc = soma / NUM_MED;

        valor_mgl = converte_mgl(media_adc);
        valor_bac = converte_bac(valor_mgl);
        valor_copos = converte_copos(valor_bac, 70.0, 0.68);

        att_display = 1;

        while (!start_test) {
            if (att_display) {
                lcd_show_selected(valor_mgl, valor_bac, valor_copos, unidade);
                att_display = 0;
				wdt_reset();
            }
        }
    }
}
