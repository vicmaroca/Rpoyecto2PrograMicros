/*
 * File:   mainProyecto.c
 * Author: Amilcar Rodriguez
 *
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#define _tmr0_value 243
#define TMR1H_VALUE 255
#define TMR1L_VALUE 247
#define _XTAL_FREQ 500000
#define PRESCALER_TMR0 4
//Variables del programa
char contador = 0;
int preload;
int i;
char adc1, adc2, adc3, adc4;
char work_value1, work_value4;
char eeprom_mode = 0;
char eeprom_data = 0;

//Subrutina de interrupción
void __interrupt() isr(void)
{

    if (INTCONbits.T0IF == 1)
    {

        contador++;

        if (contador > 37)
        {
            contador = 0;
        }
        if (contador <= work_value1)
        {
            PORTCbits.RC0 = 1;
        }
        else
        {
            PORTCbits.RC0 = 0;
        }
        if (contador <= work_value4)
        {
            PORTCbits.RC3 = 1;
        }
        else
        {
            PORTCbits.RC3 = 0;
        }

        if (!eeprom_mode)
        {
            ADCON0bits.GO = 1;
        }

        TMR0 = _tmr0_value;
        INTCONbits.T0IF = 0;
    }
}

void setup(void);
void main(void)
{
    setup();
    ADCON0bits.GO = 1; //Inicia conversión ADC
    while (1)
    {
        //Interrupción cambio en entradas Analógicas
        if (ADIF == 1)
        {
            ADIF = 0;           //Limpiamos bandera
            ADCON0bits.GO = 1; //Iniciamos la conversión

            if (ADCON0bits.CHS == 0b0000)
            {

                __delay_us(4);
                adc1 = ADRESH;

                __delay_us(4);
                work_value1 = (3 * adc1 / 255);
                __delay_us(4);
                ADCON0bits.CHS = 1; //Cambiamos la entraga analógica
                ADCON0bits.GO = 1;
            }
            if (ADCON0bits.CHS == 0b0001)
            {
                __delay_us(4);
                adc2 = ADRESH;

                __delay_us(4);
                //
                char pot_value2 = (adc2 * (8.25 / 255)) + 7.75;

                CCPR2L = pot_value2;

                __delay_us(4);
                ADCON0bits.CHS = 2; //Cambiamos la entraga analógica
                ADCON0bits.GO = 1;
            }
            if (ADCON0bits.CHS == 0b0010)
            {
                __delay_us(4);
                adc3 = ADRESH;

                __delay_us(4);
                char pot_value3 = (adc3 * (8.25 / 255)) + 7.75;

                CCPR1L = (pot_value3);

                __delay_us(4);
                ADCON0bits.CHS = 3; //Cambiamos la entrada analógica
                ADCON0bits.GO = 1;
            }
            if (ADCON0bits.CHS == 0b0011)
            {
                __delay_us(4);
                adc4 = ADRESH;

                __delay_us(4);
                work_value4 = (adc4 / 255) + 2;
                PORTD = work_value4;
                __delay_us(4);
                ADCON0bits.CHS = 0; //Cambiamos la entrada analógica
                ADCON0bits.GO = 1;
            }
        }

        // Botón 1 - Cambiar modo EEPROM
        if (!PORTBbits.RB0)
        {
            __delay_ms(20); // Debounce
            if (!PORTBbits.RB0)
            {
                eeprom_mode = !eeprom_mode;
                if (eeprom_mode)
                {
                    PORTEbits.RE0 = 1; // Encender LED
                }
                else
                {
                    PORTEbits.RE0 = 0; // Apagar LED
                }
            }
        }

        // Botón 2 - Guardar datos en EEPROM
        if (!PORTBbits.RB1)
        {
            __delay_ms(20); // Debounce
            if (!PORTBbits.RB1)
            {
                if (eeprom_mode)
                {
                    eeprom_data = work_value1;
                    // Guardar datos en la dirección de memoria deseada
                    eeprom_write(0x00, eeprom_data); // Cambia la dirección (0x00) si es necesario
                }
            }
        }

        // Botón 3 - Reproducir datos guardados
        if (!PORTBbits.RB2)
        {
            __delay_ms(20); // Debounce
            if (!PORTBbits.RB2)
            {
                if (eeprom_mode)
                {
                    eeprom_data = eeprom_read(0x00); // Cambia la dirección (0x00) si es necesario
                    work_value1 = eeprom_data;
                }
            }
        }
    }
}

void setup(void)
{
    //Entradas y salidas
    ANSEL = 0;
    ANSELH = 0;
    //Entradas y Pull-ups
    OPTION_REGbits.nRBPU = 0;
    TRISB = 255;
    WPUB = 255;
    //Salidas
    TRISD = 0;
    TRISC = 0b0000;
    TRISEbits.RE0 = 0; // Puerto E0 como salida

    //Oscilador
    OSCCONbits.IRCF = 0b011; //500kHz
    OSCCONbits.SCS = 1;

    // Configuración del puerto CCP1 y CCP2 PWM
    CCP1CON = 0b00001100; // Modo PWM, salida activa alta
    CCP2CON = 0b00001100; // Modo PWM, salida activa alta
    T2CON = 0b00000111;   // Timer2 configurado para 1:16
    TMR2 = 0;             // Reiniciar timer2
    PR2 = 155.25;         // Valor de período para 20kHz (500kHz / (4 * (PR2 + 1)))
    CCPR1L = 11.6;
    CCPR2L = 11.6;

    //Configuracion TMR0
    OPTION_REGbits.T0CS = 0;
    OPTION_REGbits.PSA = 1;
    OPTION_REGbits.PS = 0b000;
    TMR0 = _tmr0_value;

    //Configuracion de interrupciones
    INTCONbits.T0IF = 0;
    INTCONbits.T0IE = 1;
    INTCONbits.GIE = 1;
    ADIF = 0; //Bandera inicia en 0

    //Configuracion EUSART
    TXSTAbits.SYNC = 0;    //Modo asincrono
    TXSTAbits.BRGH = 1;    //High speed baud rate
    BAUDCTLbits.BRG16 = 1; //16-bit Baud Rate
    SPBRG = 25;
    SPBRGH = 0;
    RCSTAbits.SPEN = 1;   //Serial port enable
    RCSTAbits.RX9 = 0;    //8 bits de datos
    RCSTAbits.CREN = 1;   //Habilitar para recibir datos
    TXSTAbits.TXEN = 1;   //Habilitar para enviar datos

    //Interrupciones EUSART
    PIR1bits.RCIF = 0; //Bandera RX
    PIE1bits.RCIE = 1; //INT EUSART RC

    //config ADC
    ANSEL = 0b00001111; //Entradas analógicas (AN0-AN3)
    TRISA = 0b00001111;
    PORTA = 0b00001111;
    ADCON0bits.ADCS = 0;
    ADCON0bits.CHS = 1;
    ADCON1bits.ADFM = 0;
    __delay_ms(10);
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;
    ADCON0bits.ADON = 1;
}

// Funciones de escritura y lectura de EEPROM
void eeprom_write(unsigned char address, unsigned char data)
{
    EEADR = address;    // Escribir dirección de memoria
    EEDATA = data;      // Escribir datos
    EECON1bits.EEPGD = 0; // Seleccionar memoria de datos
    EECON1bits.WREN = 1;  // Habilitar escritura en EEPROM
    INTCONbits.GIE = 0;  // Deshabilitar interrupciones
    EECON2 = 0x55;       // Secuencia de escritura
    EECON2 = 0xAA;       // Secuencia de escritura
    EECON1bits.WR = 1;   // Iniciar escritura
    while (EECON1bits.WR)
        ;                  // Esperar a que termine la escritura
    EECON1bits.WREN = 0; // Deshabilitar escritura en EEPROM
    INTCONbits.GIE = 1;  // Habilitar interrupciones
}

unsigned char eeprom_read(unsigned char address)
{
    EEADR = address;       // Escribir dirección de memoria
    EECON1bits.EEPGD = 0;  // Seleccionar memoria de datos
    EECON1bits.RD = 1;     // Iniciar lectura
    return EEDATA;         // Devolver datos leídos
}
