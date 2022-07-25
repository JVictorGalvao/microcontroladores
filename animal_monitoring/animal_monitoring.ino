#include <ArduinoJson.h>
#define TINY_GSM_MODEM_SIM808
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <HTTPClient.h>
#include <WiFi.h>


#define SerialMon Serial
#define SerialAT Serial1
#define TINY_GSM_DEBUG SerialMon
#define TINY_GSM_YIELD() { delay(2); }
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
#define GSM_PIN ""

const char apn[] = "gprs.oi.com.br";
const char gprsUser[] = "oi";
const char gprsPass[] = "oi";

const char* ssid = "S20FE";
const char* password = "yskg3597";


//<MPU> 
const uint16_t MPU_addr=0x68;  // endereço I2C do MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
//</MPU>

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);

#define RXD2 17
#define TXD2 16

void setup() {
  SerialMon.begin(115200);

  WiFi.begin(ssid,password);
  
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
 
  
  //<MPU>
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // Registro PWR_MGMT_1
  Wire.write(0);     // definido como zero (ativa o MPU-6050)
  Wire.endTransmission(true);
  // </MPU>
  
  SerialMon.println("Waiting...");
  SerialAT.begin(9600, SERIAL_8N1, RXD2, TXD2, false);
  delay(3000);

    SerialMon.println("Initializing modem...");
    modem.init();
    
    String modemInfo = modem.getModemInfo();
    
    SerialMon.print("Modem Info: ");
    SerialMon.println(modemInfo);
  
    SerialMon.print("Waiting for network...");
    if (!modem.waitForNetwork()) {
      SerialMon.println(" fail");
      delay(1000);
      return;
    }
    SerialMon.println(" success");
  
    if (modem.isNetworkConnected()) {
      SerialMon.println("Network connected");
    }
  
    SerialMon.print(F("Connecting to "));
    SerialMon.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
      SerialMon.println(" fail");
      delay(10000);
      return;
    }
    SerialMon.println(F("success"));
  
    bool res = modem.isGprsConnected();
    DBG("GPRS status:", res ? "connected" : "not connected");
  
    String ccid = modem.getSimCCID();
    SerialMon.println("CCID:" + ccid);
  
    String imei = modem.getIMEI();
    SerialMon.println("IMEI:" + imei);
  
    String cop = modem.getOperator();
    SerialMon.println("Operator:" + cop);
  
    IPAddress local = modem.localIP();
    SerialMon.println("Local IP:" + local.toString());
  
    int csq = modem.getSignalQuality();
    SerialMon.println("Signal quality:" + csq);

    if (modem.enableGPS()) {
    SerialMon.println("GPS Enabled");
  }
  
}

unsigned char gps_data_delay = 0;

void envia_dados(float Velocidade, float Latitude, float Longitude){

  String local_json;
  
  StaticJsonDocument<200> doc;

  doc["Animal"] = "Animal_Higor";
  doc["Longitude"] = Longitude;
  doc["Latitude"] = Latitude;
  doc["Velocidade"] = Velocidade;
  doc["Mordido"] = true;
  
  serializeJson(doc, local_json);

  Serial.println(local_json);

  if(WiFi.status()==WL_CONNECTED){

      HTTPClient http;

      http.begin("https://cirgcnpfxvxlzlmbubjk.supabase.co/rest/v1/Dados");
      http.addHeader("apikey", "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImNpcmdjbnBmeHZ4bHpsbWJ1YmprIiwicm9sZSI6ImFub24iLCJpYXQiOjE2NTg2ODM4NTgsImV4cCI6MTk3NDI1OTg1OH0.iWUNacpLTe9A17xFjNCXq_R3AOd4qt1Gc4Ft498654g");
      http.addHeader("Authorization", "Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImNpcmdjbnBmeHZ4bHpsbWJ1YmprIiwicm9sZSI6ImFub24iLCJpYXQiOjE2NTg2ODM4NTgsImV4cCI6MTk3NDI1OTg1OH0.iWUNacpLTe9A17xFjNCXq_R3AOd4qt1Gc4Ft498654g");
      http.addHeader("Content-Type", "application/json");
      http.addHeader("Prefer", "return=representation");


      int httpResponseCode = http.POST(local_json);

      if(httpResponseCode >0){

         String response = http.getString();
      
        Serial.println(httpResponseCode);
        Serial.println(response);
      }else{
        Serial.println("Error");
      }

      http.end();
  }
    
  }

void mpu_read(){
  
 Wire.beginTransmission(MPU_addr);
 Wire.write(0x3B);  // começando com o registro 0x3B (ACCEL_XOUT_H)
 Wire.endTransmission(false);
 Wire.requestFrom(MPU_addr,(uint8_t)14,true);  // solicitar um total de 14 registros
 
 AcX=Wire.read()<<8|Wire.read();      // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
 AcY=Wire.read()<<8|Wire.read();      // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
 AcZ=Wire.read()<<8|Wire.read();      // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
 GyX=Wire.read()<<8|Wire.read();      // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
 GyY=Wire.read()<<8|Wire.read();      // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
 GyZ=Wire.read()<<8|Wire.read();      // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

 // CALIBRAGEM 
 if( ((AcX <= 100) and (AcY <= 100) and ( AcZ <= 100)) or (( GyX <= 100) and (GyY <= 100) and (GyZ <= 100)) ){
     Serial.print("Animal não foi mordido \n");
     delay(3000);
  }else{
     Serial.print("Animal foi mordido\n\nLocalizacao:");
     
    float lat, lng, vel;
    modem.getGPS(&lat, &lng, &vel);
    String msg = String(lat) + "," + String(lng) + "," + String(vel);
    SerialMon.println("Latitude, Longitude e velocidade:");
    SerialMon.println(msg);
    delay(1000);
    envia_dados(vel,lat,lng);
    delay(2000);
    }
  
}
void loop() {

  // <MPU>
  mpu_read();
  // </MPU>
  delay(1000);

 }
