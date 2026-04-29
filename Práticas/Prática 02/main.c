// ======================================================================
// Universidade Federal do Ceará (Campus) Sobral
// Prof.: Me. Alan Rocha
// Disciplina: Microprocessadores
// Semestre: 2025.2
// ======================================================================

// ======================================================================
// "MICROS" na linha 1 e "PROF: ALAN" na linha 2
// PIC18F4520 + LCD 16x2 (8 bits, barramento inteiro em PORTD)
// RS = RB0, EN = RB1, RW = GND
// Cristal externo 20 MHz (OSC = HS)
// ======================================================================

#include <xc.h>

// ----------------------------------------------------------------------
// CONFIG BITS
// ----------------------------------------------------------------------
#pragma config OSC = HS        // Oscilador externo HS (20 MHz)
#pragma config FCMEN = OFF     // Fail-Safe Clock Monitor off
#pragma config IESO = OFF      // Sem troca INT/EXT automática

#pragma config PWRT = OFF      // Power-up Timer off
#pragma config BOREN = SBORDIS // Brown-out Reset hardware only
#pragma config BORV = 3        // Nível de Brown-out

#pragma config WDT = OFF       // Watchdog Timer off
#pragma config WDTPS = 32768   // Pós-escalonador do WDT (irrelevante com WDT off)

#pragma config CCP2MX = PORTC  // CCP2 em RC1
#pragma config PBADEN = OFF    // PORTB<4:0> como digital após reset
#pragma config LPT1OSC = OFF
#pragma config MCLRE = ON      // /MCLR habilitado

#pragma config STVREN = ON     // Reset em stack overflow/underflow
#pragma config LVP = OFF       // Desativa programação em baixa tensão (libera RB5)
#pragma config XINST = OFF     // Modo estendido off
#pragma config DEBUG = OFF     // Debugger off

// ----------------------------------------------------------------------
// Frequência de clock para __delay_ms / __delay_us
// Cristal externo de 20 MHz
// ----------------------------------------------------------------------
#define _XTAL_FREQ 20000000UL

// ----------------------------------------------------------------------
// Mapeamento de pinos LCD:
// LCD_RS -> RB0
// LCD_EN -> RB1
// LCD_RW -> GND
// LCD_D0..D7 -> RD0..RD7
// ----------------------------------------------------------------------
#define LCD_RS_LAT   LATBbits.LATB0
#define LCD_EN_LAT   LATBbits.LATB1
#define LCD_DATA_LAT LATD

// ----------------------------------------------------------------------
// Protótipos
// ----------------------------------------------------------------------
void lcd_pulseEnable(void);
void lcd_command(unsigned char cmd);
void lcd_data(unsigned char c);
void lcd_init(void);
void lcd_setLine1(void);
void lcd_setLine2(void);
void lcd_print(const char *msg);

// ----------------------------------------------------------------------
// lcd_pulseEnable: pulso em EN (LCD lê no flanco de descida)
// ----------------------------------------------------------------------
void lcd_pulseEnable(void) {
    LCD_EN_LAT = 1;
    __delay_us(50);
    LCD_EN_LAT = 0;
    __delay_us(50);
}

// ----------------------------------------------------------------------
// lcd_command: envia comando para o LCD (RS = 0)
// ----------------------------------------------------------------------
void lcd_command(unsigned char cmd) {
    LCD_RS_LAT = 0;
    LCD_DATA_LAT = cmd;
    lcd_pulseEnable();

    if (cmd == 0x01 || cmd == 0x02) {
        __delay_ms(2);
    } else {
        __delay_us(100);
    }
}

// ----------------------------------------------------------------------
// lcd_data: envia caractere para o LCD (RS = 1)
// ----------------------------------------------------------------------
void lcd_data(unsigned char c) {
    LCD_RS_LAT = 1;
    LCD_DATA_LAT = c;
    lcd_pulseEnable();
    __delay_us(100);
}

// ----------------------------------------------------------------------
// lcd_init: sequência de inicialização HD44780 em modo 8 bits
// ----------------------------------------------------------------------
void lcd_init(void) {
    __delay_ms(20);
    __delay_ms(20);

    lcd_command(0x38); // 8 bits, 2 linhas, 5x8
    lcd_command(0x0C); // Display ON, cursor OFF, blink OFF
    lcd_command(0x01); // Clear display
    lcd_command(0x06); // Entry mode (cursor++)
}

void lcd_setLine1(void) { lcd_command(0x80); }
void lcd_setLine2(void) { lcd_command(0xC0); }

void lcd_print(const char *msg) {
    while (*msg != '\0') {
        lcd_data(*msg);
        msg++;
    }
}

// ----------------------------------------------------------------------
// main
// ----------------------------------------------------------------------
void main(void) {

    INTCON = 0x00;     // Desabilita interrupções
    ADCON1 = 0x0F;     // Todos os pinos AN como digitais

    LATB = 0x00;
    LATD = 0x00;

    TRISB = 0x00;      // PORTB saída (RB0 = RS, RB1 = EN)
    TRISD = 0x00;      // PORTD saída (D0..D7)

    lcd_init();

    lcd_setLine1();
    lcd_print("MICROS");

    lcd_setLine2();
    lcd_print("PROF: ALAN");

    while (1) {
        // mantém a mensagem na tela
    }
}
