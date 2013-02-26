/* Infrared trigger for pentax KX

Pulse patern:
13ms on 
3ms off
7X( 1 ms on , 1 ms off)
On duty cicle 50% 38Khz


Connect an infrared led to the signalPin (default = 3). 
This code will make your pentax KX camera trigger the shoot button 

*/

const int signalPin = 3;
const long carrierFreq = 38000; /* Frequency of the modulated signal*/

long carrierPeriod ;

void setup() {
	pinMode(signalPin, OUTPUT);

	/*Period of carrier pulses in us */
	carrierPeriod = 2000000 / carrierFreq  ;
}


/*13ms on, 3ms off, 7X( 1 ms on , 1 ms off) */
void loop()
{
	signalPeriod( signalPin, 13 , carrierPeriod );
	offPeriod( signalPin, 3);
	for ( int i = 0 ; i < 7 ; i++ ) {
		signalPeriod( signalPin, 1 , carrierPeriod );
		offPeriod(signalPin, 1 );
	}

	/*Time between shutter signal*/
	delay ( 300);
}
 
/* Signals at specified amount of time in ms, c_Period pulse period in us */
int  signalPeriod ( int pin, long timeLimit, long c_Period) {
	long time=millis();
	while (time + timeLimit > millis()) {
		digitalWrite(pin, HIGH);
		delayMicroseconds(c_Period);
		digitalWrite(pin, LOW);
		delayMicroseconds(c_Period);
	}

}

/* Stop signal for a period in ms*/
int  offPeriod ( int pin, long timeLimit) {
	digitalWrite(pin, LOW);
	delay( timeLimit );
}


