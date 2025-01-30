#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "ICM42605.h"
#include "ArduinoTimer.h"
#include "MegunoLink.h"
// Set WiFi credentials

#define WIFI_SSID "iPhone"
#define WIFI_PASS "Athletics1.0"
String newHostname = "Left-Foot";
String ConvertFloat;
time_t Timestamp = 0;
long TimeStamp = 0;
uint32_t nextTime;
uint32_t lastTime;
const unsigned SendInterval = 1;  // [ms]
float Ax, Ay, Az;
//stuttura dati sensore------
struct Byte {
  unsigned char f3 : 8;
  unsigned char f2 : 8;
  unsigned char f1 : 8;
  unsigned char f0 : 8;
};
union Bits {
  float SensorData;
  struct Byte bit;
} ShiftFloat;
//------------------------------
// UDP indirizzo e porta
WiFiUDP UdpConnection;
ArduinoTimer Timer;

const IPAddress DestinationIp(255, 255, 255, 255);
const int DestinationPort = 52128;
const int SourcePort = 52127;
// config SPI
ICM42605 IMU(SPI, 15);
//-----------------------------
void setup() {
  //plot parameter

  // Setup serial port
  Serial.begin(115200);
  Serial.println();
  // start communication with IMU
  int status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
  IMU.setAccelODR(ICM42605::odr1k);  // set accelerometer frequency 1000hz.
  IMU.setGyroODR(ICM42605::odr1k);   // set gyro frequency 1000hz.
  IMU.setAccelFS(ICM42605::gpm16);   //set g-forze scale
   IMU.setGyroFS(ICM42605::dps2000); // set gyro scale DPS
  IMU.setFilters(false, false);      // set filtres gyro - accelerometer
  IMU.calibrateAccel();              // calibrate offset
  IMU.calibrateGyro();               // calibrate offset 
  //----------------------------------------------------------------------------------------
  IMU.enable_wom(true,28,28,28, WOM_MODE_OR, WOM_MODE_PREV); // enable wake-on-motion
  //IMU.enable_wom(true, X,Y,Z, (WOM_MODE_OR, WOW_MODE_AND), (WOM_MODE_PREV, WOW_MODE_INIT);
  // X, y, Z  set micro g-forces scale from 0 to 255 
  // WOM_MODE_OR makes a (or) on all three axes
  // WOM_MODE_AND makes a (and) on all three axes
  //----------------------------------------------------------------------------------------
  
  
  // Begin WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  /* UdpConnection.begin(DestinationPort);
  UdpConnection.beginPacket(DestinationIp, DestinationPort);
  //Get Current Hostname
  UdpConnection.printf("Default hostname: %s\n", WiFi.hostname().c_str());
  //Set new hostname
  WiFi.hostname(newHostname.c_str());
  //Get Current Hostname
  UdpConnection.printf("New hostname: %s\n", WiFi.hostname().c_str());
  // Connecting to WiFi...
  UdpConnection.print("Connecting to ");
  UdpConnection.print(WIFI_SSID);
  UdpConnection.println();
  UdpConnection.print("RRSI: ");
  UdpConnection.println(WiFi.RSSI());
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    UdpConnection.print(".");
  }
  UdpConnection.println();
  UdpConnection.print("Connected! IP local: ");
  UdpConnection.println(WiFi.localIP());
  UdpConnection.print("Connected! IP remote: ");
  UdpConnection.println(WiFi.broadcastIP());
  UdpConnection.print("Opening UDP port ");
  UdpConnection.println(DestinationPort);
  UdpConnection.endPacket();*/

  //Get Current Hostname
  Serial.printf("Default hostname: %s\n", WiFi.hostname().c_str());
  //Set new hostname
  WiFi.hostname(newHostname.c_str());
  //Get Current Hostname
  Serial.printf("New hostname: %s\n", WiFi.hostname().c_str());
  // Connecting to WiFi...
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);
  Serial.println();
  Serial.print("RRSI: ");
  Serial.println(WiFi.RSSI());

  // Loop continuously while WiFi is not connected
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println();
  // Connected to WiFi

  Serial.print("Connected! IP local: ");
  Serial.println(WiFi.localIP());
  Serial.print("Connected! IP remote: ");

  // Begin UDP port
  UdpConnection.begin(DestinationPort);
  Serial.print("Opening UDP port ");
  Serial.println(DestinationPort);

}  //fine void setup


void loop() {
  IMU.getAGT();
  IMU.get_Wom();
 // && Timer.TimePassed_Milliseconds(1)
  if (WiFi.isConnected() ) {
   

    UdpConnection.beginPacket(DestinationIp, DestinationPort);

    XYPlot PlotAcc("Left-Foot(Acc)", UdpConnection);
    XYPlot TPlotGiro("Left-Foot(Giro)", UdpConnection);
    PlotAcc.SendData("Left-Foot(AccX)",IMU.counter(), IMU.accX(), 6);
    PlotAcc.SendData("Left-Foot(AccY)", IMU.counter(),IMU.accY(), 6);
    PlotAcc.SendData("Left-Foot(AccZ)",IMU.counter(), IMU.accZ(), 6);


    TPlotGiro.SendData("Left-Foot(GiroX)",IMU.counter(), IMU.gyrX(), 6);
    TPlotGiro.SendData("Left-Foot(GiroY)",IMU.counter(), IMU.gyrY(), 6);
    TPlotGiro.SendData("Left-Foot(GiroZ)",IMU.counter(), IMU.gyrZ(), 6);

    /*ConvertFloat = String(Ax, 6);
  UDP.write(ConvertFloat.c_str(), 8);
  UDP.write(",");
  ConvertFloat = String(Ay, 6);
  UDP.write(ConvertFloat.c_str(), 8);
  UDP.write(",");
  ConvertFloat = String(Az, 6);
  UDP.write(ConvertFloat.c_str(), 8);
  UDP.write(",");*/
    UdpConnection.endPacket();
    //Timestamp++;
  }

 // Serial.print(IMU.on_xyz_wom());
 // Serial.print("\t");
  
  Serial.print(IMU.counter());
  Serial.print("\t");
  Serial.print(IMU.accX(),6);
  Serial.print("\t");
  Serial.print(IMU.accY(),6);
  Serial.print("\t");
  Serial.println(IMU.accZ(),6);
   //delay(1000);
}


//Serial.print(IMU.tmst(),10);
//Serial.print("\t");
//Serial.print(IMU.accX(),6);
//Serial.print("\t");
//Serial.print(IMU.accY(),6);
//Serial.print("\t");
//Serial.println(IMU.accZ(),6);
//Serial.print("\t");
// Serial.print(IMU.gyrX(),6)*
// Serial.print("\t");
// Serial.print(IMU.gyrY(),6);
//Serial.print("\t");
//Serial.print(IMU.gyrZ(),6);
//Serial.print("\t");
//Serial.println(IMU.temp(),6);
//UDP.println("A wireless hello");
/* ShiftFloat.SensorData = IMU.accX();
  UDP.write(ShiftFloat.bit.f0);
  UDP.write(ShiftFloat.bit.f1);
  UDP.write(ShiftFloat.bit.f2);
  UDP.write(ShiftFloat.bit.f3);
  ShiftFloat.SensorData = IMU.accY();
  UDP.write(ShiftFloat.bit.f0);
  UDP.write(ShiftFloat.bit.f1);
  UDP.write(ShiftFloat.bit.f2);
  UDP.write(ShiftFloat.bit.f3);
  ShiftFloat.SensorData = IMU.accZ();
  UDP.write(ShiftFloat.bit.f0);
  UDP.write(ShiftFloat.bit.f1);
  UDP.write(ShiftFloat.bit.f2);
  UDP.write(ShiftFloat.bit.f3);*/