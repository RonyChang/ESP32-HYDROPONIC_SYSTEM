#include <Wire.h> //Protocolo I2C sensor de luz
#include <BH1750.h> //Librería sensor de luz
#include <OneWire.h>  // Biblioteca para la comunicación OneWire
#include <DallasTemperature.h>  // Biblioteca para los sensores de temperatura DS18B20
#include <DHT.h>//https://github.com/adafruit/DHT-sensor-library
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <string.h>

// Variables WiFi
const char *ssid = "SSID";
const char *password = "PASSWORD";

// Nombre del dominio o ruta de dirección IP
const char *serverName = "http://159.112.136.97:8080/";
// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastTime = 0;
// Set timer to 20 seconds (20000)
unsigned long timerDelay = 20000;

BH1750 lightMeter; // Sensor luz

// Define pines y el DHT22
#define DHTPIN 33
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
float h; // HUMEDAD DHT22
float t; // TEMPERATURA DHT22

//#define pinDatos 32  // Define el pin de datos para el sensor termocupla
#define TdsSensorPin 35  // Define el pin de datos para el sensor TDS
#define VREF 3.3  // Voltaje de referencia analógica del ADC
#define SCOUNT  30  // Número de muestras para el filtro

// DHT sensorTH (pinDatos, DHT22);    //Crea objeto sensorDHT

int analogBuffer[SCOUNT];  // Almacena los valores analógicos leídos del ADC
int analogBufferTemp[SCOUNT];  // Almacena temporalmente los valores para el filtrado
int analogBufferIndex = 0;  // Índice del buffer
int copyIndex = 0;  // Índice para copia de buffer

//TDS
float averageVoltage = 0;  // Voltaje promedio calculado
float tdsValue = 0;  // Valor de TDS calculado
float temperature = 25;  // Temperatura actual para compensación

//PH
int pHSense= 34; // Define el pin de PH
int samples = 10;
float adc_resolution = 4096.0;
//ds18B20
// Configuración para el sensor DS18B20 **
//#define ONE_WIRE_BUS 32  // Define el pin de datos para el sensor DS18B20 **
const int oneWireBus = 32;     // Define el pin de datos para el sensor DS18B20
//OneWire oneWire(ONE_WIRE_BUS);  // Crea un objeto OneWire **
OneWire oneWire(oneWireBus);   // Crea un objeto OneWire
DallasTemperature sensors(&oneWire);  // Crea un objeto para manejar los sensores DS18B20 **
float waterTemp = 0;  // Variable para almacenar la temperatura del agua **

// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen){
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++)
  bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0){
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

void setup() {

  Serial.begin (115200);

  //Configuración WIFI
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("..............");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
  

  dht.begin ();   //Inicializa DHT
  Wire.begin(21,22);   //Inicializa pines del sensor
  sensors.begin();//Inicializa sensor
  // On esp8266 you can select SCL and SDA pins using Wire.begin(D4, D3);
  // For Wemos / Lolin D1 Mini Pro and the Ambient Light shield use Wire.begin(D2, D1);
  pinMode(TdsSensorPin,INPUT);  //Inicializa sensor TDS
  lightMeter.begin(); //Inicializa sensor de luz
  Serial.println(F("BH1750 Test begin"));
 
}
//Calculo PH
float ph (float voltage) {
  return 7 + ((1.65 - voltage) / 0.236);
}

void loop() {
  delay (2000);

  static unsigned long analogSampleTimepoint = millis();
  if(millis()-analogSampleTimepoint > 40U){     //every 40 milliseconds,read the analog value from the ADC
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if(analogBufferIndex == SCOUNT){ 
      analogBufferIndex = 0;
    }
  }   
  
  static unsigned long printTimepoint = millis();
  if(millis()-printTimepoint > 800U){
    printTimepoint = millis();
    for(copyIndex=0; copyIndex<SCOUNT; copyIndex++){
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      
      // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 4096.0;
      
      //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0)); 
      float compensationCoefficient = 1.0+0.02*(temperature-25.0);
      //temperature compensation
      float compensationVoltage=averageVoltage/compensationCoefficient;
      
      //convert voltage value to tds value
      tdsValue = (133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5;
      tdsValue = tdsValue*0.00156;
      Serial.print("TDS Value:");
      Serial.print(tdsValue);
      Serial.println("dS/m");
    }
  }

  // Send an HTTP POST request every 20 seconds
  if ((millis() - lastTime) > timerDelay)
  {
    // Check WiFi connection status
    if (WiFi.status() == WL_CONNECTED)
    {
      WiFiClient client;
      HTTPClient http; 

      //POST Temperatura DHT22
      http.begin(client, "http://159.112.136.97:8080/metric/create-metric/");
      //Procedimiento para obtener el value
      t = dht.readTemperature();
      temperature = t;
      Serial.print(F("%  Temperature: "));
      Serial.print(t);
      Serial.println(F("°C "));
      // Specify content-type header
      http.addHeader("Content-Type", "application/json");
      // Send HTTP POST request
      int httpResponseCode1 = http.POST("{\"value\":\"" + String(t) + "\",\"sensor_type\":\"TAMB\"}");      
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode1);
      http.end();

      //POST Humedad DHT22
      http.begin(client, "http://159.112.136.97:8080/metric/create-metric/");
      //Procedimiento para obtener el value
      h = dht.readHumidity();
      Serial.print(F("%  Humidity: "));
      Serial.print(h);
      Serial.println(F("% "));
      // Specify content-type header
      http.addHeader("Content-Type", "application/json");
      // Send HTTP POST request
      int httpResponseCode2 = http.POST("{\"value\":\"" + String(h) + "\",\"sensor_type\":\"HAMB\"}");      
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode2);
      http.end();

      //POST PH
      http.begin(client, "http://159.112.136.97:8080/metric/create-metric/");
      //Procedimiento para obtener el value
      int measurings=0;
      for (int i = 0; i < samples; i++)
      {
          measurings += analogRead(pHSense);
          delay(10);
      }
      float voltage = 3.3 / adc_resolution * measurings/samples;
      Serial.print("pH= ");
      float pht = ph(voltage);
      Serial.println(pht);
      // Specify content-type header
      http.addHeader("Content-Type", "application/json");
      // Send HTTP POST request
      int httpResponseCode3 = http.POST("{\"value\":\"" + String(pht) + "\",\"sensor_type\":\"PH\"}");      
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode3);
      http.end();

      //POST Sensor Luz
      http.begin(client, "http://159.112.136.97:8080/metric/create-metric/");
      //Procedimiento para obtener el value
      float lux = lightMeter.readLightLevel();
      Serial.print("Light: ");
      Serial.print(lux);
      Serial.println(" lx");
      // Specify content-type header
      http.addHeader("Content-Type", "application/json");
      // Send HTTP POST request
      int httpResponseCode4 = http.POST("{\"value\":\"" + String(lux) + "\",\"sensor_type\":\"LUM\"}");      
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode4);
      http.end();

      //POST TDS
      http.begin(client, "http://159.112.136.97:8080/metric/create-metric/");
      //Procedimiento para obtener el value
      Serial.print("TDS Value:");
      Serial.print(tdsValue,0);
      Serial.println("ppm");
      float tds = tdsValue;
      // Specify content-type header
      http.addHeader("Content-Type", "application/json");
      // Send HTTP POST request
      int httpResponseCode5 = http.POST("{\"value\":\"" + String(tds) + "\",\"sensor_type\":\"TDS\"}");      
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode5);
      http.end();

      //POST Temperatura termocucpla
      http.begin(client, "http://159.112.136.97:8080/metric/create-metric/");
      //Procedimiento para obtener el value
      sensors.requestTemperatures(); 
      float tsol = sensors.getTempCByIndex(0);
      Serial.print("TSOL Value:");
      Serial.print(tsol);
      Serial.println("ºC");
      // Specify content-type header
      http.addHeader("Content-Type", "application/json");
      // Send HTTP POST request
      int httpResponseCode6 = http.POST("{\"value\":\"" + String(tsol) + "\",\"sensor_type\":\"TSOL\"}");      
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode6);
      http.end();


      delay(5000);
      
  
    }
  }
}
