//#include "time.h"
#include <WiFi.h>
//#include <HTTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

#define STASSID "JioFiber 1"
#define STAPSK "Dexiatoungjio"
//
const char* ntpServer = "0.in.pool.ntp.org";
const long  gmtOffset_sec = 16196;
const int   daylightOffset_sec = 3600;

    HardwareSerial Receiver(2);
//    String serverName = "http://192.168.29.212:8090/sensorData";
    const char* host = "192.168.29.212";
    unsigned int udpPort = 8090;
    const char* ssid     = STASSID;
    const char* password = STAPSK;

    unsigned long myTime;
//      uint8_t myTime[2];
    
    WiFiUDP Udp;

char buffer[80];

//void printLocalTime()
//{
//  time_t rawtime;
//  struct tm * timeinfo;
//
//  time (&rawtime);
//  timeinfo = localtime (&rawtime);
//
//  strftime (buffer,80," %d %B %Y %H:%M:%S ",timeinfo);
//  //struct tm timeinfo;
//  //time_t now = time(nullptr);
//  Serial.println(buffer);
//  //Serial.print(ctime(&now));
//  //Serial.print(&timeinfo, " %d %B %Y %H:%M:%S ");
//}


unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
 const unsigned long period = 1;

void setup () {

   Serial.begin(115200);
    Receiver.begin(115200, SERIAL_8N1, 16, 17); 
    WiFi.begin(ssid, password);
    Udp.begin(udpPort);

//    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

 
void loop() {

//  if (WiFi.status() == WL_CONNECTED) { //Check WiFi connection status
//      float Ax=0;
//      uint8_t aData[2];
//       uint8_t bData[12];
////      int16_t acc_x_raw;  
//       uint8_t counter = 0;
//          
//while(1){  
////     Receiver.readBytes(aData,2);
//
//////      Serial.print("Time: ");
////      myTime = millis();
//      
////      Serial.println(myTime); // prints time since program started
////      bData[counter] = myTime[0];
////      bData[counter+1] = myTime[1];
////      bData[counter+2] = aData[0];
////      bData[counter+3] = aData[1];
//      
////     bData[counter] = aData[0];
////     bData[counter+1] = aData[1];
//
//      counter+=4;
//      
////    counter+= 2;
//
//  printLocalTime();
//
////      uint8_t test[10]={0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA };
//        
////      acc_x_raw = (int16_t)((aData[0]<<8) | aData[1]);
////      Ax = acc_x_raw/16384.000;
////      Serial.println(Ax);
////      String serverPath = serverName + "?Ax=" + String(Ax);
////      http.begin(serverPath.c_str()); //Specify request destination
////      Udp.begin(serverPath.c_str());
//      if (counter == 12)
//      {
//        Udp.beginPacket(host, udpPort);
//        Udp.write(bData,12);
////        Udp.write(buffer,2);
//        Udp.endPacket();   //Close connection
//        counter = 0;
//      }
//
//      } 
//  }



      float Ax=0;
      uint8_t aData[2];
       uint8_t bData[12];
       int16_t acc_x_raw;  

 Receiver.readBytes(aData,2);
      acc_x_raw = (int16_t)((aData[0]<<8) | aData[1]);
      Ax = acc_x_raw/16384.000;


//         time_t rawtime;
//        struct tm * timeinfo;
//        time (&rawtime);
//        timeinfo = localtime (&rawtime);
         currentMillis = millis();
//        strftime (buffer,80," %d %B %Y %H:%M:%S ",timeinfo);

        DynamicJsonBuffer jBuffer;
        JsonObject& root = jBuffer.createObject();
        root["sensor"] = Ax;
//        root["Time"] = buffer;

//        if (currentMillis - startMillis == period){
        root["millisec"] = currentMillis;
//        }

//        root.prettyPrintTo(Serial);
//        Serial.println();

        // Send UDP packet
        Udp.beginPacket(host, udpPort);
        root.printTo(Udp);
        Udp.println();
        Udp.endPacket();

}
