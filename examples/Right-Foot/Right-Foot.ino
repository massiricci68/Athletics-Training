#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "ICM42605.h"
#include "ArduinoTimer.h"
#include "MegunoLink.h"
// Set WiFi credentials
#define WIFI_SSID "iPhone"
#define WIFI_PASS "Athletics1.0"
String newHostname = "Right-Foot";
String ConvertFloat;
time_t Timestamp = 0;
long LastSent;
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
//const IPAddress LocalIP (192,168,137,10);
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
  IMU.setAccelODR(ICM42605::odr1k);
  IMU.setGyroODR(ICM42605::odr1k);
  IMU.setFilters(false, false);
  IMU.calibrateAccel();
  IMU.calibrateGyro();
  ////////////////////////////////////////////////////////////
  // Begin WiFi
  // Wifi.LocalIP(192,168,137,10);
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

  if (WiFi.isConnected() && Timer.TimePassed_Milliseconds(1)) {

    UdpConnection.beginPacket(DestinationIp, DestinationPort);
   // XYPlot PlotAccX("Right-Foot(AccX)", UdpConnection);
    XYPlot PlotAccY("Right-Foot(AccY)", UdpConnection);
   // XYPlot PlotAccZ("Right-Foot(AccZ)", UdpConnection);

    //XYPlot TPlotGiro("Right-Foot(Giro)", UdpConnection);
    //PlotAccX.SendData("Right-Foot(AccX)", Timestamp, IMU.accX(), 6);
    PlotAccY.SendData("Right-Foot(AccY)", Timestamp, IMU.accY(), 6);
    //PlotAccZ.SendData("Right-Foot(AccZ)", Timestamp, IMU.accZ(), 6);
    //TPlotGiro.SendData("Right-Foot(GiroX)", Timestamp, IMU.gyrX(), 6);
   // TPlotGiro.SendData("Right-Foot(GiroY)", Timestamp, IMU.gyrY(), 6);
   // TPlotGiro.SendData("Right-Foot(GiroZ)", Timestamp, IMU.gyrZ(), 6);

    /* TimePlot PlotAcc("Right-Foot(Acc)", UdpConnection);
    TimePlot TPlotGiro("Right-Foot(Giro)", UdpConnection);

    PlotAcc.SendFloatData("Right-Foot(AccX)", IMU.accX(), 6);
    PlotAcc.SendFloatData("Right-Foot(AccY)", IMU.accY(), 6);
    PlotAcc.SendFloatData("Right-Foot(AccZ)", IMU.accZ(), 6);

    TPlotGiro.SendFloatData("Right-Foot(GiroX)", IMU.gyrX(), 6);
    TPlotGiro.SendFloatData("Right-Foot(GiroY)", IMU.gyrY(), 6);
    TPlotGiro.SendFloatData("Right-Foot(GiroZ)", IMU.gyrZ(), 6);*/

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
    Timestamp++;
   if(Timestamp == 10000) 
   {
    Timestamp =0;
    
   } 
  }
   

  /* Serial.print("Millisecond: ");
  Serial.print(Timestamp);
  Serial.print("\t");*/
  //Serial.print(IMU.accX(), 6);
 // Serial.print("\t");
 // Serial.print(IMU.accY(), 6);
 // Serial.print("\t");
 // Serial.println(IMU.accZ(), 6);
  //delayMicroseconds(1);
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