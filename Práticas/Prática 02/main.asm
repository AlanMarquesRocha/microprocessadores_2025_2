; =====================================================================
; "MICROS" na linha 1 e "PROF: ALAN" na linha 2
; PIC18F4520 + LCD 16x2 (8 bits)
; =====================================================================

            LIST      P=18F4520, F=INHX32
            INCLUDE   "p18f4520.inc"
	  
; =====================================================================
; CONEX�ES (MONTAGEM F�SICA)
; =====================================================================
; LCD pino  1 (VSS) -> GND
; LCD pino  2 (VDD) -> +5 V
; LCD pino  3 (VEE) -> cursor do potenci�metro de ~10k (outros lados em 5 V e GND)
; LCD pino  4 (RS)  -> RB0 do PIC (pino f�sico 33)
; LCD pino  5 (RW)  -> GND
; LCD pino  6 (E)   -> RB1 do PIC (pino f�sico 34)
; LCD pino  7 (D0)  -> RD0 do PIC
; LCD pino  8 (D1)  -> RD1 do PIC
; LCD pino  9 (D2)  -> RD2 do PIC
; LCD pino 10 (D3)  -> RD3 do PIC
; LCD pino 11 (D4)  -> RD4 do PIC
; LCD pino 12 (D5)  -> RD5 do PIC
; LCD pino 13 (D6)  -> RD6 do PIC
; LCD pino 14 (D7)  -> RD7 do PIC
;
; Cristal ~4 MHz entre OSC1 e OSC2, com capacitores de ~22pF pra GND.
; MCLR (pino 1 do PIC) puxado para +5 V via resistor de 10k.
;
; IMPORTANTE: O c�digo assume:
;   RS = RB0
;   EN = RB1
;   RW = GND (escrita apenas)
;   D0..D7 = RD0..RD7
; =====================================================================

; ---------------------------------------------------------------------
; CONFIG BITS
; ---------------------------------------------------------------------
    CONFIG  OSC = XT          ; Oscilador externo tipo XT (cristal ~4 MHz)
    CONFIG  FCMEN = OFF       ; Fail-Safe Clock Monitor desabilitado
    CONFIG  IESO = OFF        ; Sem troca INT/EXT autom�tica

    CONFIG  PWRT = OFF        ; Power-up Timer OFF
    CONFIG  BOREN = SBORDIS   ; Brown-out Reset hardware only
    CONFIG  BORV = 3          ; N�vel de Brown-out

    CONFIG  WDT = OFF         ; Watchdog Timer OFF
    CONFIG  WDTPS = 32768     ; P�s-escalonador do WDT (irrelevante com WDT OFF)

    CONFIG  CCP2MX = PORTC    ; CCP2 em RC1
    CONFIG  PBADEN = OFF      ; PORTB<4:0> digitais ap�s reset
    CONFIG  LPT1OSC = OFF
    CONFIG  MCLRE = ON        ; /MCLR habilitado (reset externo ativo em RE3/MCLR)

    CONFIG  STVREN = ON       ; Reset em stack overflow/underflow
    CONFIG  LVP = OFF         ; Desativa programa��o em baixa tens�o (libera RB5)
    CONFIG  XINST = OFF       ; Conjunto estendido de instru��es OFF
    CONFIG  DEBUG = OFF       ; Debug OFF

; ---------------------------------------------------------------------
; DEFINI��ES DE PINOS DO LCD
; ---------------------------------------------------------------------
LCD_RS      EQU     0         ; RB0 -> RS
LCD_EN      EQU     1         ; RB1 -> EN

; ---------------------------------------------------------------------
; VARI�VEIS
; ---------------------------------------------------------------------
            CBLOCK  0x20
d0          ; contador de delay grosso
d1          ; contador interno de delay
            ENDC

; ---------------------------------------------------------------------
; VETOR DE RESET
; ---------------------------------------------------------------------
            ORG     0x0000
            GOTO    START

; ---------------------------------------------------------------------
; DELAYS
; ---------------------------------------------------------------------

; Delay curto (~centenas de microssegundos @4MHz, aproximado)
DelayShort:
            MOVLW   0xFF
            MOVWF   d1, ACCESS
DelayShortLoop:
            DECFSZ  d1, F, ACCESS
            BRA     DelayShortLoop
            RETURN

; Delay longo (~alguns ms) chamando v�rios DelayShort
DelayLong:
            MOVLW   0x04
            MOVWF   d0, ACCESS
DelayLongLoop:
            CALL    DelayShort
            DECFSZ  d0, F, ACCESS
            BRA     DelayLongLoop
            RETURN

; ---------------------------------------------------------------------
; ROTINAS DE LCD
; ---------------------------------------------------------------------

; Gera pulso no pino EN do LCD.
; Pr�-condi��o: LATD j� tem o byte e RS j� est� ajustado em LATB.
LCD_PulseEN:
            BSF     LATB, LCD_EN, ACCESS   ; EN = 1
            CALL    DelayShort
            BCF     LATB, LCD_EN, ACCESS   ; EN = 0
            CALL    DelayShort
            RETURN

; Envia comando (RS=0). WREG cont�m o comando (ex: 0x01 = clear).
LCD_Command:
            MOVWF   LATD, ACCESS           ; LATD <- comando
            BCF     LATB, LCD_RS, ACCESS   ; RS = 0 (comando)
            CALL    LCD_PulseEN
            CALL    DelayLong              ; tempo pro LCD executar
            RETURN

; Envia dado (caractere ASCII) (RS=1). WREG cont�m o caractere.
LCD_Data:
            MOVWF   LATD, ACCESS           ; LATD <- caractere ASCII
            BSF     LATB, LCD_RS, ACCESS   ; RS = 1 (dado)
            CALL    LCD_PulseEN
            CALL    DelayShort
            RETURN

; Inicializa��o padr�o HD44780 em 8 bits.
; 0x38: interface 8 bits, 2 linhas, fonte 5x8
; 0x0C: display ON, cursor OFF, blink OFF
; 0x01: clear display
; 0x06: entry mode, cursor avan�a
LCD_Init:
            CALL    DelayLong
            CALL    DelayLong          ; atraso inicial p�s-power-up

            MOVLW   0x38               ; Function Set
            CALL    LCD_Command

            MOVLW   0x0C               ; Display ON, cursor OFF, blink OFF
            CALL    LCD_Command

            MOVLW   0x01               ; Clear display
            CALL    LCD_Command

            MOVLW   0x06               ; Entry mode: cursor incrementa
            CALL    LCD_Command

            RETURN

; Cursor no in�cio da linha 1.
; Linha 1 = DDRAM addr 0x00 -> comando 0x80
LCD_SetLine1:
            MOVLW   0x80
            CALL    LCD_Command
            RETURN

; Cursor no in�cio da linha 2.
; Linha 2 = DDRAM addr 0x40 -> comando 0xC0
LCD_SetLine2:
            MOVLW   0xC0
            CALL    LCD_Command
            RETURN

; Imprime string ROM terminada em 0x00.
; Sup�e TBLPTR apontando para o 1� caractere da string.
LCD_PrintString:
NextChar:
            TBLRD*+                     ; TABLAT <- [TBLPTR], TBLPTR++
            MOVF    TABLAT, W, ACCESS   ; W = caractere lido
            BNZ     SendChar            ; se W != 0x00, imprime
            RETURN                      ; se W == 0x00, acabou a string

SendChar:
            CALL    LCD_Data            ; manda caractere em W
            BRA     NextChar

; ---------------------------------------------------------------------
; PROGRAMA PRINCIPAL
; ---------------------------------------------------------------------
START:
            ; Desativa interrup��es globais
            CLRF    INTCON, ACCESS

            ; For�a PORTA/PORTB digitais
            MOVLW   0x0F
            MOVWF   ADCON1, ACCESS

            ; Zera as sa�das antes de configurar TRIS
            CLRF    LATB, ACCESS
            CLRF    LATD, ACCESS

            ; PORTB como sa�da (RB0=RS, RB1=EN, etc.)
            MOVLW   0x00
            MOVWF   TRISB, ACCESS       ; RB7..RB0 = 0 -> sa�da

            ; PORTD como sa�da (D0..D7 do LCD)
            MOVLW   0x00
            MOVWF   TRISD, ACCESS       ; RD7..RD0 = 0 -> sa�da

            ; Inicializa LCD
            CALL    LCD_Init

            ; ===== Linha 1: "MICROS" =====
            CALL    LCD_SetLine1

            ; Carrega TBLPTR para MsgLinha1
            MOVLW   HIGH(MsgLinha1)
            MOVWF   TBLPTRH, ACCESS
            MOVLW   UPPER(MsgLinha1)
            MOVWF   TBLPTRU, ACCESS
            MOVLW   LOW(MsgLinha1)
            MOVWF   TBLPTRL, ACCESS

            CALL    LCD_PrintString

            ; ===== Linha 2: "PROF: ALAN" =====
            CALL    LCD_SetLine2

            ; Carrega TBLPTR para MsgLinha2
            MOVLW   HIGH(MsgLinha2)
            MOVWF   TBLPTRH, ACCESS
            MOVLW   UPPER(MsgLinha2)
            MOVWF   TBLPTRU, ACCESS
            MOVLW   LOW(MsgLinha2)
            MOVWF   TBLPTRL, ACCESS

            CALL    LCD_PrintString

MainLoop:
            BRA     MainLoop            ; loop infinito

; ---------------------------------------------------------------------
; STRINGS EM FLASH (terminadas com 0x00)
; colocadas depois do c�digo inicial pra n�o bagun�ar o vetor de reset
; ---------------------------------------------------------------------
            ORG     0x300
MsgLinha1:
            DB      "MICROS",0x00
MsgLinha2:
            DB      "PROF: ALAN",0x00

; ---------------------------------------------------------------------
; END OF FILE
; ---------------------------------------------------------------------
            END
