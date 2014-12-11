#include <AFMotor.h>
AF_DCMotor motor(2, MOTOR12_64KHZ); // Indicamos el motor que vamos a usar y la frecuencia del PWM

//Tiempo de referencia para mover el motor y relizar las paradas 
int tiempo = 240;

//Potencia del motor de 0-255   (0%-100%) 
int potencia = 170;

void setup() {

  motor.setSpeed(potencia); // establecemos la potencia al motor
  pinMode(13,OUTPUT);       // Inicializamos el pin del led para ver cuando funciona el motor en un sentido u otro
}

void loop() {

  digitalWrite (13,HIGH); //Encendemos LED
  motor.run(FORWARD); // Mover motor en un sentido (adelante)
  delay(tiempo);      // Esperar  tiempo establecido con el motor encendido (adelante)
  
  motor.run(RELEASE); // dejamos el motor suelto
  delay(tiempo/4);    // esperamos un carto del tiempo anterior para que se vaya frenando
  
  digitalWrite( 13,LOW); //Apagamos led
  motor.run(BACKWARD); // Encendemos el motor en sentido contrario (atras)
  delay(tiempo);       // Esperamos con el motor encendido

  motor.run(RELEASE); // Dejamos el motor sin potencia
  delay(tiempo/4);    //Esperamos 
}
