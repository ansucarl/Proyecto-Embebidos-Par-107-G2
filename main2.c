// Proyecto sistemas embebidos
//Andres Suarez Par 107
//Matriz 5x5x5
#include <avr/io.h>
#include <util/delay.h>
#define PIN_SER      PB1 // Puerto B1 Pin que controla Entrada de datos
#define PIN_RCLK     PB0 // Puerto B0 Pin que controla el conteo del reloj
#define PIN_SRCLK    PB2 // pin 11 Pin que funciona como disparador del 74HC595
// write digital "high" to pin <pn> on portb
#define GOHI(pn) PORTB |= (1<<pn)
// write digital "low" to pin <pn> on portb
#define GOLO(pn) PORTB &= ~(1<<pn)
#include <avr/interrupt.h>
#include <stdbool.h>

// Funciones del 74HC595
void put(uint8_t);
void putBit(uint8_t);
void latch();
void cycleClock();
void longDelay(uint16_t);
void setup();
uint8_t rotateLeft(uint8_t);
uint8_t rotateRight(uint8_t);


void setup() {
  
   // Configuracion de la Salidas del puerto B
    DDRB |= (1 << PIN_SER) | (1 << PIN_SRCLK) | (1 << PIN_RCLK);
}



void longDelay(uint16_t ms) {
    for(ms /= 10; ms>0; ms--) _delay_ms(10);
}

/*
 * Shifts bits left, but wraps the left-most bit back around
 */
uint8_t rotateLeft(uint8_t toRotate) {
    uint8_t carry = (toRotate & 0b10000000) == 0b10000000;
    return (toRotate << 1 | carry);
}

/*
 * Shifts bits right, but wraps the right-most bit back around
 */
uint8_t rotateRight(uint8_t toRotate) {
    uint8_t carry = ( toRotate & 1 ) ? 0b10000000 : 0;
    return (toRotate >> 1 | carry);
}

/*
 * Funcion que escribe por byte  hacia la salida del pin de datos
 */
void put(uint8_t toPut) {
    uint8_t i;
    for(i = 0; i < 8; i++) {
        putBit(toPut & 1);
        toPut >>= 1;
    }

    latch();
}

/*
 * Writes a bit out serially to PIN_SER
 */
void putBit(uint8_t bit) {
    if(bit == 0) GOLO(PIN_SER);
    else         GOHI(PIN_SER);

    cycleClock();
}

/*
 * Funcion que dispara los datos almacenados en el registro
 */
void latch() {
    GOHI(PIN_RCLK);
    GOLO(PIN_RCLK);
}

/*
 *  Funcion que dice al registro leer la señal de reloj
 */
void cycleClock() {
    GOHI(PIN_SRCLK);
    GOLO(PIN_SRCLK);
}

static volatile bool abajo1 = false; // static variable global // volatile diferentes hilos o interrupciones + programa principal
static volatile bool abajo2 = false;
static volatile bool abajo3 = false;
static volatile bool abajo4 = false;
static volatile bool abajo5 = false;
static volatile bool abajoR = false;

//Funcion que verifica que boton esta presionado
void isPress(){
  if((PIND&(1<<PIND0))==0){ //si está presionado entra al if
 while((PIND&(1<<PIND0))==0){} //espera a que se suelte la botonera
 abajo1 = !abajo1;
 }
 if((PIND&(1<<PIND1))==0){ //si está presionado entra al if
 while((PIND&(1<<PIND1))==0){} //espera a que se suelte la botonera
 abajo2 = !abajo2;
 }
 if((PIND&(1<<PIND2))==0){ //si está presionado entra al if
 while((PIND&(1<<PIND2))==0){} //espera a que se suelte la botonera
 abajo3 = !abajo3;
 }
 if((PIND&(1<<PIND3))==0){ //si está presionado entra al if
 while((PIND&(1<<PIND3))==0){} //espera a que se suelte la botonera
 abajo4 = !abajo4;
 }
 if((PIND&(1<<PIND4))==0){ //si está presionado entra al if
 while((PIND&(1<<PIND4))==0){} //espera a que se suelte la botonera
 abajo5 = !abajo5;
 }
 if((PIND&(1<<PIND5))==0){ //si está presionado entra al if
 while((PIND&(1<<PIND5))==0){} //espera a que se suelte la botonera
 abajoR = !abajoR;
 }
}

//MAIN
int main()
{
 setup();
 uint8_t left1 = 0b00000000;
 uint8_t right1 = 0b00000001;
 uint8_t left2 = 0b11111111;
 uint8_t right2 = 0b11111111;
 uint8_t left3 = 0b1010101010;
 uint8_t right3 = 0b00000000;
 uint8_t left4 = 0b111111111;
 uint8_t right4 = 0b00000000;
 uint8_t left5 = 0b10101010;
 uint8_t right5 = 0b00000000;
 uint8_t leftR = 0b000000000;
 uint8_t rightR = 0b00000000;
   
   
   
//  Resistencias PULLUP para las entradas de los botones
 PORTD |= (1<<PD0); //Activar resistencia de PULLUP en PIN0 del puerto D
 PORTD |= (1<<PD1); //Activar resistencia de PULLUP en PIN1 del puerto D
 PORTD |= (1<<PD2); //Activar resistencia de PULLUP en PIN2 del puerto D
 PORTD |= (1<<PD3); //Activar resistencia de PULLUP en PIN2 del puerto D
 PORTD |= (1<<PD4); //Activar resistencia de PULLUP en PIN3 del puerto D
 PORTD |= (1<<PD5); //Activar resistencia de PULLUP en PIN4 del puerto D
   
   
 while (1){

    
 //Juego luces 1
 
 if(abajo1){
put(left1 | right1);
_delay_ms(10);
left1 = rotateRight(left1);
right1 = rotateLeft(right1);
 }
 
 //Juego luces 2
 //Prender todos los Leds
 if(abajo2){
put(left2 | right2);
_delay_ms(10);

 }
 
  //Juego luces 3
 //Prender por diagonales
 if(abajo3){
put(left3 | right3);
_delay_ms(100);
left3 = rotateRight(left3);
right3 = rotateLeft(right3);
 }
 
 //Juego luces 4
 if(abajo4){
put(left4 | right4);
_delay_ms(100);
left4 = rotateRight(left4);
right4 = rotateLeft(right4);
 }
 
 //Juego luces 5
 if(abajo5){
put(left5 | right5);
_delay_ms(100);
left5 = rotateRight(left5);
right5 = rotateLeft(right5);
 }
 
 //Juego Reset
 if(abajoR){
put(leftR | rightR);
leftR = rotateRight(leftR);
rightR = rotateLeft(rightR);
 }
 
 else{

 }
 isPress();
 }
 return 0;
}
