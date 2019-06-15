/**
 * /github/AlfreodPrades
 * 
 * Necesita un access point y le ha de dar ip normalmente 192.168.43.XXX (android AP)
 * La pantalla LCD nos deberia decir la IP una vez conectado.
 * 
 * La aplicacion android "shareGPS" (com.jillybunch.shareGPS) (u otra) nos tiene que mandar al puerto 2300 la posicion GPS 
 * del telefono en formato NMEA
 * 
 * 
 * Por sofware serial tenemos que decodificar nuestro propio GPS 
 * 
 * Hay un magnetometro que nos indica nuestra orientacion. Es conveniente darle un par de vueltas despacio al coche 
 * para que la magentometro se inicialice y sepa cuales son los valores maximos y minimos en cada eje.
 * 
 * En base a las dos posiciones y nuestra orientacion giramos o avanzamos para acercarnos a la 
 * posicion del telefono
 * 
 * Hay un pin definido (demo_pin) que se si se pone a HIGH el Coche se pone en modo demo y se pone a mover los motores.
 * 
 * El ESP32 crea una web en el puerto 80 que permite ver el estado de todo lo conectado usando websockets.  
 * La pagina web, que deberia estar en la carpeta data dentro de la carpeta del proyecto, contiene css, hmtl y javascript
 * que se sube al ESP32 mediante el menu: tools -> "ESP32 Sketch data upload". Hay que tener en cuenta que hay que poner 
 * la ip que el AP del telefono le ha dado a tu ESP32 (se muestra en el LCD) quiza tu telefono tambien te diga la ip 
 * de los cliente conectados.
 * 
 * Toda la parte del LCD y el servidor web que muestra informacion es opcional, pero se deberia quitar con cuidado.
 * 
 */

//NETWORKING
#include "WiFi.h"
#include <WifiConfig.h> // This file is for you to create with your credentials or you can just delete this include
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include <ArduinoJson.h>

//MAGNETOMETER
#include <Wire.h>
#include <LIS3MDL.h>
#include <SPI.h>

//LCD
#include <PCD8544.h>
#include "driver/gpio.h"

//GPS
//#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define DEBUG false // flag to turn on/off debugging
#define Serial if(DEBUG)Serial 




#define mtrA_pin1 19
#define mtrA_pin2 18
#define mtrB_pin1 33
#define mtrB_pin2 25
#define gps_serial_rx 16
#define gps_serial_tx 17
#define lcd_pin1 14
#define lcd_pin2 13
#define lcd_pin3 27
#define lcd_pin4 26
#define lcd_pin5 15
#define mag_pin1 21
#define mag_pin2 22
#define demo_pin 36

TinyGPSPlus remoteGps;
TinyGPSPlus selfGps;
#define SelfGpsSerial Serial2

//MAGNETOMETER
//https://github.com/Anirudhvl/Esp32-Nokia-5110-Interfacing-and-NTP-Sync-IST-Digital-Clock
//https://www.instructables.com/id/Esp32-5110-LCD-Interfacing-and-Digital-Clock/

#define contrast 50
static PCD8544 display = PCD8544(lcd_pin1, lcd_pin2, lcd_pin3, lcd_pin4, lcd_pin5);
//ARDUINO
//CLK  14
//DIN  13  (MOSI)
//DC   27
//RST  26
//CE   15   (CS)
//VCC  3V3
//BL   3V3

//Hardware serial

LIS3MDL mag;
LIS3MDL::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};
const int pPI = 314159;
const float radToDegs = 180 / PI;
char angleString[4];

int minx; //-3300;
int maxx; //2300;
int miny; //-2000;
int maxy; //3700;
bool mangnetometerInitialized = false;

//how many clients should be able to telnet to this ESP8266
#define MAX_SRV_CLIENTS 1
//essid3 => ANDROID essid4 => TALLER
const char *ssid = essid3;
const char *password = passwd3;

// Create AsyncWebServer object on port 80
AsyncWebServer wsServer(80);
AsyncWebSocket ws("/ws");
AsyncWebSocketClient *globalClient = NULL;
char jsonString[100];
StaticJsonDocument<100> doc;

const int acceptableAngleError = 30;

WiFiServer server(2300);
WiFiClient serverClients[MAX_SRV_CLIENTS];

String rawLat, rawLon;
long lat, lon;
float latMin, lonMin;

int actualDirection = 137; // LEER DE LA BRUJULA
int targetDirection;       // CALCULAR EL COURSE
char motorTurn;            // I R  D
int turn;                  // DIFERENCIA MODULAR(%360) del real a target direction
int dist = -2;

const int motorPinsR[] = {mtrA_pin1, mtrA_pin2};
const int motorPinsL[] = {mtrB_pin1, mtrB_pin2};
const int motorChanR[] = {1, 2};
const int motorChanL[] = {3, 4};
int ledState = HIGH;
#define HALF 128
#define FULL 255
//NO SoftwareSerial SelfGpsSerial(4, 10); // RX, TX
int speed = 192;

void setup()
{
  // ACTIVACION PINES MOTORES MODO SALIDA
  for (int i = 0; i < sizeof(motorPinsR); i++)
  {
    ledcAttachPin(motorPinsR[i], motorChanR[i]);
    ledcAttachPin(motorPinsL[i], motorChanL[i]); // assign RGB led pins to channels

    
    //Para utilizar la salida PWM del ESP32 se usan instrucciones ledc y se mapean los puertos
    //en un canal con unos numero determinados. no se puede usar analogWrite
    // Initialize channels
    // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
    // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
    ledcSetup(motorPinsR[i], 12000, 8); // 12 kHz PWM, 8-bit resolution
    ledcSetup(motorPinsL[i], 12000, 8);

    // VIEJO DIGITAL
    //pinMode(motorPinsR[i], OUTPUT);
    //pinMode(motorPinsL[i], OUTPUT);
  }

  Serial.begin(115200);

  // LECTURA PIN MODO DEMO
  pinMode(demo_pin, INPUT);
  int demoPinHigh = digitalRead(demo_pin);
  if (demoPinHigh)
  {
    motorsDemo();
  }

  // Initialize SPIFFS donde esta guardada la pagina web completa
  Serial.print(" Init SPIFSS:  ");
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  else
  {
    ok();
  }

  //initialize magnetometer
  Serial.print(" Init Magnetometer:  ");
  Wire.begin(mag_pin1, mag_pin2);
  if (!mag.init())
  {
    Serial.println("Failed to detect and initialize magnetometer!");
    while (1)
      ;
  }
  mag.enableDefault();
  ok();

  //Start local gps serial port
  SelfGpsSerial.begin(9600, SERIAL_8N1, gps_serial_rx, gps_serial_tx);

  display.begin();
  display.clear();
  display.setContrast(contrast);
  display.setCursor(0, 2);
  display.print("Conectando... ");
  display.setCursor(0, 4);
  display.print(ssid);

  //Connect to WIFI AP
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to ");
  Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20)
    delay(500);
  if (i >= 21)
  {
    Serial.print("Could not connect to");
    Serial.println(ssid);
    while (1)
      delay(500);
  }
  else
  {
    showWifiInfo();
  }

  // Route for root / web page
  wsServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    lo("/");
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  // Route to load style.css file
  wsServer.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
    lo("style.css");
    request->send(SPIFFS, "/style.css", "text/css");
  });

  // Route to set GPIO to HIGH
  wsServer.on("/on", HTTP_GET, [](AsyncWebServerRequest *request) {
    lo("/on");
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  // Route to set GPIO to HIGH
  wsServer.on("/off", HTTP_GET, [](AsyncWebServerRequest *request) {
    lo("/off");
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  ws.onEvent(onWsEvent);
  wsServer.addHandler(&ws);
  wsServer.begin();

  //Start Server
  server.begin();
  server.setNoDelay(true);
}

void loop()
{
  actualDirection = getOrientationDegrees();
  sendCompassWebSocket(actualDirection);

  if (ledState == HIGH)
  {
    ledState = LOW;
  }
  else
  {
    ledState = HIGH;
  }

  uint8_t i;
  //check if there are any new clients
  if (server.hasClient())
  {
    for (i = 0; i < MAX_SRV_CLIENTS; i++)
    {
      //find free/disconnected spot
      if (!serverClients[i] || !serverClients[i].connected())
      {
        if (serverClients[i])
          serverClients[i].stop();
        serverClients[i] = server.available();
        Serial.print("New client: ");
        Serial.print(i);
        continue;
      }
    }
    //no free/disconnected spot so reject
    WiFiClient serverClient = server.available();
    serverClient.stop();
  }

  //Se comprueba se hay alguien conectado y si no estan mandando datos de GPS
  for (i = 0; i < MAX_SRV_CLIENTS; i++)
  {
    if (serverClients[i] && serverClients[i].connected())
    {
      if (serverClients[i].available())
      {
        //get data from the telnet client and push it to the UART
        while (serverClients[i].available())
        {
          if (remoteGps.encode(serverClients[i].read()))
          {
            displayInfo(remoteGps);
            sendGPSSWebSocket("remotegps", remoteGps);
            /*
                Serial.print("Distance to 40 0 ");
                Serial.print(TinyGPSPlus::distanceBetween(remoteGps.location.lat(),remoteGps.location.lng(),40,0));
                Serial.print("  course to 40 0 ");
                Serial.println(TinyGPSPlus::courseTo(remoteGps.location.lat(),remoteGps.location.lng(),40,0));
                */
            break; //No abusamos de una fuente de gps, saltamos a la otra
          }
        }
      }
    }
  }

  // Comprobamos si estamos recibiendo ya nuestras propias coordenadas desde el puerto serie
  // Hay que tener en cuenta que el GPS no es inmediato y tambien se puede perder temporalmente 
  while (SelfGpsSerial.available())
  {
    Serial.print(".");
    if (selfGps.encode(SelfGpsSerial.read()))
    {
      Serial.println("\n----Software Serial-----");
      displayInfo(selfGps);
      sendGPSSWebSocket("selfgps", selfGps);
      Serial.println("----Software Serial end-----\n");
    }
  }
  Serial.print(".");


  // Si ambas coodenadas GPS son validas ya podemos empezar a decidir si nos debemos mover o girar  
  if ((selfGps.location.isValid() && remoteGps.location.isValid()))
  {

    dist = TinyGPSPlus::distanceBetween(remoteGps.location.lat(), remoteGps.location.lng(), selfGps.location.lat(), selfGps.location.lng());
    Serial.print("Distancia GPSs: ");
    Serial.print(dist);
    Serial.println("m");

    targetDirection = TinyGPSPlus::courseTo(selfGps.location.lat(), selfGps.location.lng(), remoteGps.location.lat(), remoteGps.location.lng());
    sendRumboSWebSocket(targetDirection, dist);
    Serial.print("Course GPSs: ");
    Serial.print(targetDirection);
    Serial.println("deg");
    //turn = calculaGiroAIzq(actualDirection, targetDirection);
    turn = calculaGiroClockWise(actualDirection, targetDirection);
    Serial.print("giro: ");
    Serial.println(turn);
    if (dist > 2)
    {
      if (abs(turn) < acceptableAngleError)
      {
        motorTurn = 'R';
        Serial.println("RECTO");
        sendMotorSWebSocket("forward");
        goForward(speed);
        // VAMOS BIEN
      }
      else if (turn > 0)
      {
        motorTurn = 'D';
        Serial.println("DER");
        sendMotorSWebSocket("right");
        rotateRight(speed);
        //GIRO DER
      }
      else if (turn < 0)
      {
        motorTurn = 'I';
        sendMotorSWebSocket("left");
        Serial.println("IZQ");
        rotateLeft(speed);
        //GIRO IZQ
      }
    }
    else
    {
      Serial.println("STOP");
      stopMotors();
      motorTurn = 'S';
    }
  }
  displayLCD();
  yield();
  delay(50);
}

int calculaGiroClockWise(int actual, int target)
{
  int der = (target - actual);
  if (abs(der) <= 180)
  {
    return der;
  }
  else
  {
    return inverseSign(der) * (360 - abs(der));
  }
}

int inverseSign(int x)
{
  if (x > 0)
    return -1;
  else
    return 1;
}

void rotateLeft(int speed)
{
  Serial.println("LEFT");
  sideForward(motorChanR, speed);
  sideBackward(motorChanL, 0);
}

void rotateRight(int speed)
{
  Serial.println("RIGHT");
  sideForward(motorChanL, speed);
  sideBackward(motorChanR, 0);
}

void goForward(int speed)
{
  Serial.println("FORWARD");
  sideForward(motorChanR, speed);
  sideForward(motorChanL, speed);
}

void goBackward(int speed)
{
  Serial.println("BACKWARD");
  sideBackward(motorChanR, speed);
  sideBackward(motorChanL, speed);
}

void sideForward(const int *motorPins, int speed)
{
  ledcWrite(motorPins[0], 0);
  ledcWrite(motorPins[1], speed);
}

void sideBackward(const int *motorPins, int speed)
{
  ledcWrite(motorPins[0], speed);
  ledcWrite(motorPins[1], 0);
}

void stopMotors()
{
  ledcWrite(motorChanR[0], 0);
  ledcWrite(motorChanR[1], 0);
  ledcWrite(motorChanL[0], 0);
  ledcWrite(motorChanL[1], 0);
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  if (type == WS_EVT_CONNECT)
  {
    Serial.println("Websocket client connection received");
    globalClient = client;
  }
  else if (type == WS_EVT_DISCONNECT)
  {
    Serial.println("Client disconnected");
    globalClient = NULL;
  }
}



void showWifiInfo()
{
  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("IP Mask: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("Gateway: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("DNS: ");
  Serial.println(WiFi.dnsIP());
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("BSSID: ");
  Serial.println(WiFi.BSSIDstr());
  Serial.print("Channel: ");
  Serial.println(WiFi.channel());
}

void displayInfo(TinyGPSPlus gps)
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}


/**
 * Dados los dos valores de intensidad el campo magnetico la funcion atan2 te da el rumbo.
 * Ademas actualizamos los valores maximos de intensidad si es necesario.
 * 
 * Se usa la funcion meuMap porque el map de arduino hace overflow en en los datos intermedios
 * 
 * ES necesario al iniciar el sketch darle un par de vueltas al manetometro para inicializarlo bien
 * 
 * 
 * */

int getOrientationDegrees()
{

  char buffer[120];
  mag.read();

  if (mangnetometerInitialized)
  {
    minx = min((int)mag.m.x, (int)minx);
    maxx = max((int)mag.m.x, (int)maxx);
    miny = min((int)mag.m.y, (int)miny);
    maxy = max((int)mag.m.y, (int)maxy);
  }
  else
  {
    minx = (int)mag.m.x;
    maxx = (int)mag.m.x + 1;
    miny = (int)mag.m.y;
    maxy = (int)mag.m.y + 1;
    mangnetometerInitialized = true;
  }
  sprintf(buffer, "X: %d  minx: %d  maxx: %d     Y: %d  miny: %d  maxy: %d", mag.m.x, minx, maxx, mag.m.y, minx, maxy);
  Serial.println(buffer);

  int rx = (int)mag.m.x + abs(minx);
  int ry = (int)mag.m.y + abs(miny);

  int offsetMaxx = maxx + abs(minx);
  int offsetMaxy = maxy + abs(miny);

  int x = meuMap(rx, 0, offsetMaxx, 0, 2 * pPI);
  int y = meuMap(ry, 0, offsetMaxy, 0, 2 * pPI);

  x = x - pPI;
  y = y - pPI;

  double dx = (double)((double)x / 100000.0);
  double dy = (double)((double)y / 100000.0);

  double rads = atan2(dx, dy);
  int degr = rads * radToDegs + 180;
  Serial.print("Rads: ");
  Serial.print(rads);
  Serial.print(" Deg: ");
  Serial.println(degr);

  delay(100);
  return degr;
}

void displayLCD()
{
/**
 *   BRU: Direccion hacia la que apunta el coche (0 Norte, 90 Este , 180 SUR, 270 OESTE)
 *   RGPS: latitud del gps remoto  (recibido por la red wifi)
 *   LGPS: latitud del gps local   (recibido por puerto serie)
 *   D: Distante entre los dos GPSs
 *   C: Rumbo a segir desde un punto al otro( pe:  si el RGPS(movil) esta al SUR-ESTE de LGPS(coche)  C debria ser 135 aprox
 *   MT: indica que es lo que queremos que el motor gire Izq Der Recto  (por si los motores estan mal conectados)
 *   T: indica la diferencia entre hacia donde apuntamos y hacia adonde deberiamos apuntar.
 *   IP: La ip que nos ha dado el punto de acceso del movil
 * 
 * */  


  //display.setTextSize(2);
  display.clear();
  display.setCursor(1, 0);
  display.print("BRU ");
  sprintf(angleString, "%03d", actualDirection);
  display.print(angleString);

  display.setCursor(0, 1);
  display.print("RGPS:");
  if (remoteGps.location.isValid())
  {
    display.print(remoteGps.location.lat() * 10000);
  }
  else
  {
    display.print("rNO valid");
  }

  display.setCursor(0, 2);
  display.print("LGPS:");
  if (selfGps.location.isValid())
  {
    display.print(selfGps.location.lat() * 10000);
  }
  else
  {
    display.print("s invalid");
  }

  display.setCursor(0, 3);
  display.print("D ");
  display.print(dist);
  display.print(" C: ");
  display.print(targetDirection);

  display.setCursor(0, 4);
  display.print("MT ");
  display.print(motorTurn);
  display.print(" T ");
  display.print(turn);

  display.setCursor(0, 5);
  display.println(WiFi.localIP());

  //display.display();
}
/*
void drawAngle2( float rads) {
  float hx = display.width()/2;
  float hy = display.height()/2;
    
  int dX = sin (rads) * hx * -1;
  int dY = cos (rads) * hy ;

  display.drawLine(hx, hy , hx + dX , hy + dY, BLACK); 
  display.drawLine(hx + 1, hy  , hx + dX +1 , hy + dY, BLACK); 
  display.display();
}

*/

void sendDebugData(String data)
{
  buildCompassEventData("debug", data);
  sendWebSocket();
}

void sendMotorSWebSocket(char *side)
{
  buildMotorEventData(String(side));
  sendWebSocket();
}

void buildMotorEventData(String side)
{
  JsonArray arr = doc.to<JsonArray>();
  JsonObject evt1 = doc.createNestedObject();
  evt1["type"] = "turn";
  evt1["side"] = side;
}

void sendRumboSWebSocket(int rumbo, int dist)
{
  buildRumboEventData("rumbo", String(rumbo), String(dist));
  sendWebSocket();
}

void buildRumboEventData(char *eventType, String rumbo, String dist)
{
  JsonArray arr = doc.to<JsonArray>();
  JsonObject evt1 = doc.createNestedObject();
  evt1["type"] = eventType;
  evt1["dist"] = dist;
  evt1["rumbo"] = rumbo;
}

void sendCompassWebSocket(int dir)
{
  buildCompassEventData("compass", String(dir));
  sendWebSocket();
}

void buildCompassEventData(char *eventType, String eventData)
{
  JsonArray arr = doc.to<JsonArray>();
  JsonObject evt1 = doc.createNestedObject();
  evt1["type"] = eventType;
  evt1["data"] = eventData;
}

void sendGPSSWebSocket(char *gpsName, TinyGPSPlus gps)
{
  if (gps.location.isValid())
  {
    buildGPSEventData(gpsName, String(gps.location.lat(), 6), String(gps.location.lng(), 6));
  }
  else
  {
    buildGPSEventData(gpsName, String("location not ready"), String("location not ready"));
  }
  sendWebSocket();
}

void buildGPSEventData(char *eventType, String lat, String lon)
{
  JsonArray arr = doc.to<JsonArray>();
  JsonObject evt1 = doc.createNestedObject();
  evt1["type"] = eventType;
  evt1["lat"] = lat;
  evt1["lon"] = lon;
}

void sendWebSocket()
{
  serializeJson(doc, jsonString, sizeof(jsonString));
  if (globalClient != NULL && globalClient->status() == WS_CONNECTED)
  {
    Serial.print("Send data: ");
    globalClient->text(jsonString);
  }
  else
  {
    Serial.print("No enviamos porque no hay cliente WS: ");
  }
  Serial.println(jsonString);
}

void lo(char *msg)
{
  Serial.print("requested: ");
  Serial.println(msg);
}

// Hace la division antes de la multiplicacion para evitar overflow
int meuMap(int x, int in_min, int in_max, int out_min, int out_max)
{
  int resp = (x - in_min) * ((out_max - out_min) / (in_max - in_min)) + out_min;
  return resp;
}

// Replaces placeholder with LED state value
String processor(const String &var)
{

  return String();
}

/*            String line = serverClients[i].readStringUntil('\r');
            if ( line.indexOf("$GPRMC") == -1) {
                continue;
            }
           
            parseNMEA(line);            
  */
void ok()
{
  Serial.println(" OK!");
}

void motorsDemo()
{
  int initialSpeed = 127;
  int demoSpeed = initialSpeed;
  while (true)
  {
    demoSpeed += 16;
    if (demoSpeed > 255)
    {
      demoSpeed = initialSpeed;
    }

    goForward(demoSpeed);
    delay(600);
    stopMotors();
    delay(300);

    rotateRight(demoSpeed);
    delay(600);
    stopMotors();
    delay(300);


    rotateLeft(demoSpeed);
    delay(600);
    stopMotors();
    delay(300);

    goBackward(demoSpeed);
    delay(600);
    stopMotors();
    delay(1200);
  }
}
