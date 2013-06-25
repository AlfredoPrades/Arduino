/***

Autor: Alfredo Prades
       aprades gmail.com

Control de un servo mendiante envío de caracteres a través de un puerto serie bluetooth

La posicion del servo se controla manualmente mediante una de las entradas analogicas
del arduino, en este caso "turnPostionPinIN" .

Mediante dos señales digitales de salida del arduino, se controlan las puertas (gate)
de los transistores del puente H que controla el motor que mueve el "servo" (motor) de 
la dirección de las ruedas. turnLeftPinOUT  turnRightPinOUT

Con señales digitales a los pines forwardPinOUT  backwardPinOUT controlamos los "gates" del
puente H del motor princiapl

El movimiento que queremos hacer se recibe por el puerto serie mediante caracteres, 
que indican hacia que posicion nos queremos mover.
En nuestro caso el puerto serie esta conectado a un módulo bluetooth el cual recibe los commandos
(caracteres) desde un telefono android con un app de puerto serie bluetooth. En nuestro caso usamos el app
Bluetooth serial controller disponible en el playstore.


TODO: Se podria usar una variable target direction para transmittir directamente por serie el valor 
deseado de giro, de momento nos desplazaremos a izq, der, centro 

TODO: Buscar un app que use el acelerometro del movil para trasmitir direcciones y/o velocidad.


**/

#include <Arduino.h>

const int turnPostionPinIN = 0; //VERDER : GND   -- Amarillo: 5V -- Blanco: ref

const int turnLeftPinOUT  = 5;   //DOWN   //naranja
const int turnRightPinOUT = 3;  //UP      //NEGRO 

const int forwardPinOUT = 9;
const int backwardPinOUT = 10;

const int marginalDirection = 30; // Margen al cual si nos acercamos suficiente dejamos de alimentar el servo



char command;            // variable to receive data from the serial port
const char CMD_RIGHT = 'R';
const char CMD_LEFT  = 'L';
const char CMD_CENTER  = 'S';
const char CMD_FORWARD = 'F';
const char CMD_BACKWARD = 'B';
const int POS_MAX_RIGHT = 1024;
const int POS_CENTER = POS_MAX_RIGHT / 2;
const int POS_MIN_LEFT = 0;

const int MAX_SPEED = 255;   //Speed goes from -255 to 255
const int SPEED_STEP = 16;
int currentSpeed =  0;
int safetyTime = 7000;

long timeMark,timeMark2;


void setup() {

	pinMode(turnLeftPinOUT,OUTPUT);
	pinMode(turnRightPinOUT,OUTPUT);

	pinMode(11, OUTPUT);
	analogWrite(11,130);

	Serial.begin(9600);       // start serial communication at 9600bps
	Serial.println("OK __________________________ START");

	analogWrite( forwardPinOUT , 0 );

	timeMark = millis();
}

void loop() {
	
	//Parpadear cada byte recibido
	if( Serial.available() )       // if data is available to read
	{
		command = Serial.read();         // read it and store it in 'val'

		//Serial.print( command );           // echo the command back for debuggin purposes
		Serial.print( getTurnPosition ( turnPostionPinIN ) );
		timeMark2 = timeMark;
		timeMark = millis();
		if( command == CMD_RIGHT ) 
		{
			moveRight();
		} else  if ( command == CMD_LEFT ){ 
			moveLeft();
		} else  if ( command == CMD_CENTER ){ 
			moveCenter();
			stop();
		} else if ( command == CMD_FORWARD ) {
			accelerate();
		} else if ( command == CMD_BACKWARD ) {
			decelerate();
		}
		else {
			timeMark = timeMark2;
		}

	}
	//Si no llega ningun comando vamos a la posicion central si no estamos ya ahí
	//O NO hacemos nada ?
	else {
		if ( inCenter( ) != 1 ) {
			//moveCenter();
		}
		stopTurn();

		
		if ( timeMark + safetyTime < millis() ) {
			slowDown();
		}
	}
	//Serial.println(currentSpeed);
	delay(25);           
} 

void stop () {
	currentSpeed = 0;
	setSpeed ( currentSpeed );
}


void accelerate () {
	if (  currentSpeed < MAX_SPEED ) {
		currentSpeed = currentSpeed + SPEED_STEP;
		
	}
	setSpeed ( currentSpeed );
}

void decelerate() {
	if (  currentSpeed > -MAX_SPEED ) {
		currentSpeed = currentSpeed - SPEED_STEP;
		
	}
	setSpeed ( currentSpeed );
}


void slowDown(){
	if (currentSpeed > 0 ) {
		currentSpeed -= 1;
	}
	else {
		currentSpeed += 1;
	}
	setSpeed(currentSpeed);
	//Serial.println(currentSpeed);
}

void setSpeed(int speed ) {
	if (speed > MAX_SPEED ) speed = MAX_SPEED;
	if (speed < -MAX_SPEED ) speed = -MAX_SPEED;


	//Serial.println(speed);
	if ( speed > SPEED_STEP ) {
		digitalWrite( backwardPinOUT , LOW );
		analogWrite( forwardPinOUT , speed );
	}
	else if (speed < -SPEED_STEP ) {
		digitalWrite( forwardPinOUT , LOW );
		analogWrite( backwardPinOUT , -speed );
	}
	else {
		digitalWrite( forwardPinOUT , LOW );
		digitalWrite( backwardPinOUT , LOW );
	}
}

void moveRight( ) {
	if ( ! isRightMost () ){
		right();
	}
}

void moveLeft ( ) {
	if ( ! isLeftMost() ) {
		left();
	}
}

void moveCenter  ()  {
	Serial.println("CENTER ");
	if ( ! inCenter () ){
		if ( inLeftSide() ) {
			right();
			Serial.println("right ");
		}
		else{
			left();
			Serial.println("left ");
		}
	}
}

int inCenter () {
	if ( getTurnPosition ( turnPostionPinIN ) - marginalDirection < POS_CENTER &&
		 getTurnPosition ( turnPostionPinIN ) + marginalDirection > POS_CENTER ) {
		return 1;	
	}
	else {
		return 0;
	}
}


int isRightMost () {
	if ( getTurnPosition ( turnPostionPinIN ) + marginalDirection > POS_MAX_RIGHT ) {
		return 1;	
	}
	else {
		return 0;
	}
}


int isLeftMost () {
	if ( getTurnPosition ( turnPostionPinIN ) - marginalDirection < POS_MIN_LEFT  ) {
		return 1;	
	}
	else {
		return 0;
	}
}

//Posicion actual del servo ( 0 - MAX_RIGHT)
int getTurnPosition (int  pin  ) {
	int raw  =analogRead( pin );
	int directionServoReading = map ( raw , 260 , 870 ,POS_MIN_LEFT , POS_MAX_RIGHT) ;
	Serial.print("RAW: " ); 
	Serial.print(raw);
	Serial.print(" Pos:");
	Serial.println(directionServoReading);
	return  directionServoReading;
}


int inLeftSide ( ) {
	return getTurnPosition ( turnPostionPinIN ) < POS_CENTER ;
}

void right () {
	Serial.println("RIGHT");
	digitalWrite( turnLeftPinOUT , LOW);
	delay(1);
	digitalWrite( turnRightPinOUT , HIGH);
	delay(40);
}

void left () {
	Serial.println("LEFT");
	digitalWrite( turnRightPinOUT , LOW);
	delay(1);
	digitalWrite( turnLeftPinOUT , HIGH);
	delay(40);
}

void stopTurn() {
	//Serial.println("STOP");
	digitalWrite( turnRightPinOUT , LOW);
	digitalWrite( turnLeftPinOUT , LOW);
}

