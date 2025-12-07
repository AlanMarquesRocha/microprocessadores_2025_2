/*--------------------------------------------------------------- 
 * UNIVERSIDADE FEDERAL DO CEARÁ ? UFC
 * CAMPUS DE SOBRAL - CURSO DE ENGENHARIA ELÉTRICA E COMPUTAÇÃO
 * DISCIPLINA: SBL0082 - Microprocessadores (2025.2)
 *
 * PRÁTICA 08 - RX com leitura local e UART remota exibidas no LCD
 * Microcontrolador: PIC18F4520
 * Display: LCD Alfanumérico 16x2 (modo 4 bits)
 * Comunicação: Serial UART (RX) + ADC local
 *
 * AUTOR: Prof. Me. Alan Rocha
 * DATA: 07/12/2025
 * VERSÃO: 2.0
 *--------------------------------------------------------------*/

// === CONFIGURAÇÕES DE FUSES ===
// CONFIG1H
#pragma config OSC = XT
#pragma config FCMEN = OFF
#pragma config IESO = OFF

// CONFIG2L
#pragma config PWRT = OFF
#pragma config BOREN = OFF
#pragma config BORV = 3

// CONFIG2H
#pragma config WDT = OFF
#pragma config WDTPS = 32768

// CONFIG3H
#pragma config CCP2MX = PORTC
#pragma config PBADEN = OFF
#pragma config LPT1OSC = OFF
#pragma config MCLRE = ON

// CONFIG4L
#pragma config STVREN = ON
#pragma config LVP = OFF
#pragma config DEBUG = OFF

// CONFIG5L/5H, CONFIG6L/6H, CONFIG7L/7H
#pragma config CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF
#pragma config CPB = OFF, CPD = OFF
#pragma config WRT0 = OFF, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF
#pragma config WRTC = OFF, WRTB = OFF, WRTD = OFF
#pragma config EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF
#pragma config EBTRB = OFF

// === BIBLIOTECAS ===
#include <xc.h>
#include <stdio.h>
#include <string.h>

// === DEFINIÇÕES ===
#define _XTAL_FREQ 20000000

// === LCD ===
#define lcd_rs     PORTDbits.RD0
#define lcd_enable PORTDbits.RD1
#define lcd_db4    PORTDbits.RD4
#define lcd_db5    PORTDbits.RD5
#define lcd_db6    PORTDbits.RD6
#define lcd_db7    PORTDbits.RD7

// === VARIÁVEIS GLOBAIS ===
char Rx_buffer[16];
char Rx_lock = 1;         // Trava de leitura
float V_remota = 0.0;     // Tensão recebida do TX

// === PROTÓTIPOS ===
void escreve_lcd(char c);  // printf ? LCD
void limpa_lcd(void);
void inicializa_lcd(void);
void caracter_inicio(char linha, char coluna);

// printf direcionado para LCD
void putch(char c) {
    escreve_lcd(c);
}

// === LCD ===
void envia_nibble_lcd(char dado) {
    lcd_db4 = (dado >> 0) & 1;
    lcd_db5 = (dado >> 1) & 1;
    lcd_db6 = (dado >> 2) & 1;
    lcd_db7 = (dado >> 3) & 1;

    lcd_enable = 1;
    __delay_us(50);
    lcd_enable = 0;
    __delay_us(50);
}

void envia_byte_lcd(char endereco, char dado) {
    lcd_rs = endereco;
    envia_nibble_lcd(dado >> 4);
    envia_nibble_lcd(dado & 0x0F);
    __delay_us(50);
}

void escreve_lcd(char c) {
    envia_byte_lcd(1, c);
}

void limpa_lcd(void) {
    envia_byte_lcd(0, 0x01);
    __delay_ms(2);
}

void inicializa_lcd(void) {
    lcd_enable = 0;
    lcd_db4 = lcd_db5 = lcd_db6 = lcd_db7 = 0;
    __delay_ms(50);
    envia_nibble_lcd(0x03); __delay_ms(5);
    envia_nibble_lcd(0x03); __delay_us(150);
    envia_nibble_lcd(0x03);
    envia_nibble_lcd(0x02); // Modo 4 bits

    envia_byte_lcd(0, 0x28); // 4 bits, 2 linhas
    envia_byte_lcd(0, 0x0C); // display ON, cursor OFF
    envia_byte_lcd(0, 0x06); // incremento do cursor
    limpa_lcd();
}

void caracter_inicio(char linha, char coluna) {
    char pos = (linha == 1) ? 0x80 : 0xC0;
    pos += coluna;
    envia_byte_lcd(0, pos);
}

// === INTERRUPÇÃO UART ===
void __interrupt() rx_isr(void) {
    static char *p = Rx_buffer;
    if (RCIF) {
        char c = RCREG;

        if (c != '\n') {
            Rx_lock = 1;
            *p++ = c;
        } else {
            *p = '\0';  // final da string
            p = Rx_buffer;

            // Espera formato: "V=3.25"
            if (strncmp(Rx_buffer, "V=", 2) == 0) {
                sscanf(Rx_buffer + 2, "%f", &V_remota);
                Rx_lock = 0;
            }
        }
    }
}

// === FUNÇÃO PRINCIPAL ===
void main(void) {
    // UART
    TRISCbits.TRISC7 = 1; // RX
    SYNC = 0;
    BRGH = 1;
    BRG16 = 0;
    SPBRG = 129;
    SPEN = 1;
    CREN = 1;

    RCIE = 1;
    PEIE = 1;
    GIE = 1;

    // ADC - RA0 como analógica
    ADCON1bits.PCFG = 0b1110;
    ADCON2bits.ADFM = 1;
    ADCON2bits.ADCS = 0b100;
    ADCON0bits.ADON = 1;

    // LCD
    TRISD = 0x00;
    PORTD = 0;
    inicializa_lcd();

    // Variáveis
    unsigned int valor_adc = 0;
    float V_local = 0.0;

    while (1) {
        // Leitura local (RA0)
        ADCON0bits.GO = 1;
        while (ADCON0bits.GO);
        valor_adc = ADRES;
        V_local = 5.0 * valor_adc / 1023.0;

        // Linha 1: tensão local
        caracter_inicio(1, 0);
        printf("Vloc=%.2fV", V_local);

        // Linha 2: tensão remota
        if (!Rx_lock) {
            caracter_inicio(2, 0);
            printf("Vrem=%.2fV", V_remota);
        }

        __delay_ms(200);
    }
}
