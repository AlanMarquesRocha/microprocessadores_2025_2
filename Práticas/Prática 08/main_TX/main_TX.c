/*--------------------------------------------------------------- 
 * UNIVERSIDADE FEDERAL DO CEARÁ ? UFC
 * CAMPUS DE SOBRAL - CURSO DE ENGENHARIA ELÉTRICA E COMPUTAÇÃO
 * DISCIPLINA: SBL0082 - Microprocessadores (2025.2)
 *
 * PRÁTICA 08 - Transmissão UART de tensão analógica para exibição remota
 * Microcontrolador: PIC18F4520
 * Comunicação: Serial UART (9600 bps)
 * Cristal: 20 MHz
 *
 * AUTOR: Prof. Me. Alan Rocha
 * DATA: 07/12/2025
 * VERSÃO: 2.0
 *--------------------------------------------------------------*/

// === CONFIGURAÇÕES DE FUSE BITS ===
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

// === DEFINIÇÕES ===
#define _XTAL_FREQ 20000000  // Frequência do cristal (20 MHz)

// Função necessária para printf() com UART
void putch(char txData) {
    while (!TXIF);   // Espera buffer estar pronto
    TXREG = txData;  // Envia caractere
}

// === FUNÇÃO PRINCIPAL ===
void main(void) {
    // Configuração da UART
    TRISCbits.TRISC6 = 0;   // RC6 (TX) como saída
    SYNC = 0;               // Modo assíncrono
    BRGH = 1;               // Alta velocidade
    BRG16 = 0;              // Registrador SPBRG de 8 bits
    SPBRG = 129;            // 9600 bps para Fosc = 20 MHz
    TXEN = 1;               // Habilita transmissor
    SPEN = 1;               // Habilita módulo serial

    // Configuração do ADC (leitura de RA0/AN0)
    ADCON1bits.PCFG = 0b1110;   // RA0 como entrada analógica
    ADCON2bits.ADFM = 1;        // Resultado justificado à direita
    ADCON2bits.ADCS = 0b100;    // Clock do ADC = Fosc/4
    ADCON0bits.ADON = 1;        // Liga o conversor A/D

    // Variáveis
    unsigned int leitura_adc = 0;
    float voltagem = 0.0;

    // Loop principal
    while (1) {
        // Inicia conversão A/D
        ADCON0bits.GO = 1;
        while (ADCON0bits.GO);      // Espera término

        leitura_adc = ADRES;        // Lê resultado
        voltagem = 5.0 * leitura_adc / 1023.0;  // Converte para volts

        // Envia tensão via UART no formato "V=3.25"
        printf("V=%.2f\n", voltagem);

        __delay_ms(500);  // Aguarda meio segundo antes de próxima leitura
    }
}
