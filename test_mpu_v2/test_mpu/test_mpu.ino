// Librerias I2C para controlar el mpu6050
// la libreria MPU6050.h necesita I2Cdev.h, I2Cdev.h necesita Wire.h
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

//Para la red y comunicación
#include <SoftwareSerial.h>
#define DEBUG true

//para pantalla y sonido
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h> 

//Creo los objetos a utilizar
hd44780_I2Cexp lcd;
SoftwareSerial esp8266(5,4); // make RX Arduino line is pin 5, make TX Arduino line is pin 4.
                             // This means that you need to connect the TX line from the esp to the Arduino's pin 2
                             // and the RX line from the esp to the Arduino's pin 3
MPU6050 sensor; // La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
                // del estado de AD0. Si no se especifica, 0x68 estará implicito

int LED = 10; 
volatile int estadoSensorSonido= LOW; //utilizado en la interrupción del sensor de sonido
bool estado = false ;
bool limpiar = true;
long int acum = 0;
String content = "";
int pinNumber = 0;
int secondNumber = 0;

//Utilizadas para saber si el alerta es por sonido, movimiento o ambas
bool monitoring = false;
int alertmov = 0; //estados 0 y 1
int alertsnd = 0; //estados 0 y 2
int alerta = 0; //puede ser: 0=Sin Alerta; 1=Alerta por movimiento; 2=Alerta por sonido; 3=Alerta por movimiento y sonido

// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
//Variables para las lecturas de acelerómetro y giróscopo
int axi, ayi, azi;
float inicialAcel;
int gxi, gyi, gzi;
float inicialGiro;
int acumAcel=0;
int acumGiro=0;

int umbralAcel = 4;
int umbralGiro = 45;

int ax, ay, az;
int gx, gy, gz;
//****************************************************************************

int connectionId = -1;

String linea = "";

void setup() {
  Serial.begin(9600);     //Iniciando puerto serial
  esp8266.begin(9600);    //Iniciando conexión con módulo WIFI
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor

  //sonido y led
  Wire.begin();
  lcd.begin(16, 2);
  lcd.display();
  attachInterrupt(0, fnc_cambio_estado, CHANGE); //Seteo interrupción para cuando se detecten sonidos considerables
  pinMode( LED, OUTPUT) ;
  analogWrite(LED , 0) ; // Apagamos el LED al empezar  

  //Inicializando red WIFI
  //sendCommand("AT+RST\r\n",2000,DEBUG); // reset module
  sendCommand("AT+CWMODE=1\r\n",1000,DEBUG); // configure as access point
  //sendCommand("AT+CWJAP=\"SOa-IoT-N750\",\"ioteshumo\"\r\n",3000,DEBUG);
  //delay(20000);
  sendCommand("AT+CIFSR\r\n",1000,DEBUG); // get ip address
  if(esp8266.find("STAIP,\"")){
    String response = "";
    for(int i=0; i<15; i++)
      {
        char c = esp8266.read(); // read the next character.
        response+=c;
      }  
    lcd.setCursor(0,0);
    lcd.print(response);
  }
  sendCommand("AT+CIPMUX=1\r\n",1000,DEBUG); // configure for multiple connections
  sendCommand("AT+CIPSERVER=1,80\r\n",1000,DEBUG); // turn on server on port 80

  Serial.println("Server Ready");
  
  // Leer las aceleraciones y velocidades angulares iniciales de giróscopo y acelerómtetro
  sensor.getAcceleration(&axi, &ayi, &azi);
  sensor.getRotation(&gxi, &gyi, &gzi);

  //Conversión de lecturas a valores positivos
  float axi_m_s2 = axi * (9.81/16384.0)+19.62;
  float ayi_m_s2 = ayi * (9.81/16384.0)+19.62;
  float azi_m_s2 = azi * (9.81/16384.0)+19.62;
  float gxi_deg_s = gxi * (250.0/32768.0)+250;
  float gyi_deg_s = gyi * (250.0/32768.0)+250;
  float gzi_deg_s = gzi * (250.0/32768.0)+250;

  inicialAcel= axi_m_s2+ayi_m_s2+azi_m_s2;
  inicialGiro= gxi_deg_s+gyi_deg_s+gzi_deg_s;

  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");
}

void loop() {
  //Leer peticiones al módulo WIFI**********************************************************************************************************************
  
  if(esp8266.available()) // check if the esp is sending a message 
     {      
      if(esp8266.find("+IPD,"))
        {
         //Serial.print("if IPD");
         delay(1000); // wait for the serial buffer to fill up (read all the serial data)
         // get the connection id so that we can then disconnect
         connectionId = esp8266.read()-48; // subtract 48 because the read() function returns 
                                           // the ASCII decimal value and 0 (the first decimal number) starts at 48
         esp8266.find("pin="); // advance cursor to "pin="
          
         pinNumber = (esp8266.read()-48); // get first number i.e. if the pin 13 then the 1st number is 1
         secondNumber = (esp8266.read()-48);
         if(secondNumber>=0 && secondNumber<=9)
          {
          pinNumber*=10;
          pinNumber +=secondNumber; // get second number, i.e. if the pin number is 13 then the 2nd number is 3, then add to the first number
          }

          Serial.print("pin number: "+pinNumber);

       if(pinNumber >= 90)
             switch(pinNumber){
              case 99:
                if(limpiar){
                  alertmov = 0;
                  alertsnd = 0;              
                  limpiar = false;
                }
                Serial.print("Monitoreando...");
                alerta = alertmov;
                  if(estadoSensorSonido == HIGH)
                    {
                      contabilizarSonido();
                      alerta = alerta + alertsnd;
                    }
                content = alerta;
                sendHTTPResponse(connectionId,content);

                //Limpio las variables para un nuevo ciclo y apago el LED de alerta
                alerta = 0;
                alertmov = 0;
                alertsnd = 0;
                digitalWrite(LED, LOW);
                pinNumber = 0; //para que no vuelva a entrar en el próximo loop si no hubo otro request
                break;
                
              case 98:
                content = "ACK";
                limpiar = true;
                Serial.print("Monitoreo detenido");
                sendHTTPResponse(connectionId,content);
                break;
                
              case 97: //temperatura
                content = "Temperatura ambiente";
                Serial.print("Temperatura leída");
                sendHTTPResponse(connectionId,content);
                break;
             }
          }
      }
  //**********************************************************************************************************************************************************
    // Leer las aceleraciones y velocidades angulares
    sensor.getAcceleration(&ax, &ay, &az);
    sensor.getRotation(&gx, &gy, &gz);
    float ax_m_s2 = ax * (9.81/16384.0)+19.62;
    float ay_m_s2 = ay * (9.81/16384.0)+19.62;
    float az_m_s2 = az * (9.81/16384.0)+19.62;
    float gx_deg_s = gx * (250.0/32768.0)+250;
    float gy_deg_s = gy * (250.0/32768.0)+250;
    float gz_deg_s = gz * (250.0/32768.0)+250;

    //Revisar esta parte; hay que considerar el valor absoluto de cada componente y después hacer la suma.
    acumAcel= ax_m_s2+ay_m_s2+az_m_s2;
    acumGiro= gx_deg_s+gy_deg_s+gz_deg_s;

    if(abs(acumAcel - inicialAcel) > umbralAcel || 
      abs(acumGiro - inicialGiro) > umbralGiro)
      {
        alertaVisual();
        alertaMovimiento();
        delay(1000);
        digitalWrite(LED, LOW);
        alertmov = 1;
        acumAcel = 0;
        acumGiro = 0;
      }
}

void fnc_cambio_estado(){
      estadoSensorSonido = HIGH;
}

void alertaMovimiento(){
      lcd.setCursor(0,0);
      lcd.print("A: ");
      lcd.print(abs(acumAcel - inicialAcel));
      lcd.print(", G: ");
      lcd.print(abs(acumGiro - inicialGiro));
      Serial.print("Umbral Superado! ");Serial.print( "Acel: ");Serial.print( abs(acumAcel - inicialAcel));Serial.print(" GIRO: ");Serial.print( abs(acumGiro - inicialGiro));
      Serial.print("\n");  
}

void contabilizarSonido(){
    acum = acum + analogRead(A0);
    //lcd.setCursor(0,1);
    //lcd.print(acum);
    //lcd.setCursor(,1);
    //lcd.print(", ");
    //lcd.print(millis()/1000);
    //delay (1000);
    //analogWrite(LED,0); //luego de 10 segundos lo apagamos
    estadoSensorSonido = LOW; //El cambio de estado se da con la interrupción. No debería cambiarlo yo de nuevo acá.
    //if(acum/(millis()/1000) > 600)
    //{
      alertsnd = 2;
      alertaVisual();
      //digitalWrite(LED, HIGH);
      //delay(5000); //voy a apagar el LED después de mandar el alerta a la APP.
    //}
    //digitalWrite(LED, LOW);
}

void alertaVisual(){
  analogWrite(LED, 30) ; // prendemos el sensor con PWM
}

//Funciones de Comunicación***************************************************************************************************************************************
/*
* Name: sendData
* Description: Function used to send data to ESP8266.
* Params: command - the data/command to send; timeout - the time to wait for a response; debug - print to Serial window?(true = yes, false = no)
* Returns: The response from the esp8266 (if there is a reponse)
*/
String sendData(String command, const int timeout, boolean debug)
{
    String response = "";
    
    int dataSize = command.length();
    char data[dataSize];
    command.toCharArray(data,dataSize);
           
    esp8266.write(data,dataSize); // send the read character to the esp8266
    if(debug)
    {
      Serial.println("\r\n====== HTTP Response From Arduino ======");
      Serial.write(data,dataSize);
      Serial.println("\r\n========================================");
    }
    
    long int time = millis();
    
    while( (time+timeout) > millis())
    {
      while(esp8266.available())
      {
        
        // The esp has data so display its output to the serial window 
        char c = esp8266.read(); // read the next character.
        response+=c;
      }  
    }
    
    if(debug)
    {
      Serial.print(response);
    }
    
    return response;
}
 
/*
* Name: sendHTTPResponse
* Description: Function that sends HTTP 200, HTML UTF-8 response
*/
void sendHTTPResponse(int connectionId, String content)
{
     // build HTTP response
     String httpResponse;
     String httpHeader;
     // HTTP Header
     httpHeader = "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=UTF-8\r\n"; 
     httpHeader += "Content-Length: ";
     httpHeader += content.length();
     httpHeader += "\r\n";
     httpHeader +="Connection: close\r\n\r\n";
     httpResponse = httpHeader + content + " "; // There is a bug in this code: the last character of "content" is not sent, I cheated by adding this extra space
     for(int i=0; i<= connectionId; i++){
      sendCIPData(i,httpResponse);
      delay(1500);
     }
}
 
/*
* Name: sendCIPDATA
* Description: sends a CIPSEND=<connectionId>,<data> command
*
*/
void sendCIPData(int connectionId, String data)
{
   String cipSend = "AT+CIPSEND=";
   cipSend += connectionId;
   cipSend += ",";
   cipSend +=data.length();
   cipSend +="\r\n";
   sendCommand(cipSend,1000,DEBUG);
   sendData(data,1000,DEBUG);
}
 
/*
* Name: sendCommand
* Description: Function used to send data to ESP8266.
* Params: command - the data/command to send; timeout - the time to wait for a response; debug - print to Serial window?(true = yes, false = no)
* Returns: The response from the esp8266 (if there is a reponse)
*/
String sendCommand(String command, const int timeout, boolean debug)
{
    String response = "";
           
    esp8266.print(command); // send the read character to the esp8266
    
    long int time = millis();
    
    while( (time+timeout) > millis())
    {
      while(esp8266.available())
      {
        
        // The esp has data so display its output to the serial window 
        char c = esp8266.read(); // read the next character.
        response+=c;
      }  
    }
    
    if(debug)
    {
      Serial.print(response);
    }
    
    return response;
}
