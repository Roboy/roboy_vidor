#include <wiring_private.h>

// uncomment for USB-blaster functionality
#define BLAST

#ifdef BLAST
  #include <SPI.h>
  #include "Blaster.h"
  extern void enableFpgaClock();
  void setup(){
    USBBlaster.setOutEpSize(60);
    USBBlaster.begin(true);
    // also start the 48MHz clock feed for the FPGA, in case we need to run the bitstream and need this clock
    enableFpgaClock();
  }
  
  void loop() {
    USBBlaster.loop();
  }

#else 
  #include <SPI.h>
  #include <WiFiNINA.h>
  #include <WiFiUdp.h>
  #include <DualMAX14870MotorShield.h>
  DualMAX14870MotorShield motors;
  
  int status = WL_IDLE_STATUS;
  ///////please enter your sensitive data in the Secret tab/arduino_secrets.h
  char ssid[] = "why-fi";        // your network SSID (name)
  char pass[] = "gotohell11880";    // your network password (use for WPA, or use as key for WEP)
  
  unsigned int localPort = 2390;      // local port to listen on
  IPAddress remoteIP(192,168,2,113); 
  
  WiFiUDP Udp;
  
  const int transmissionPin = 6;
  const int slaveSelectPin = 7;

  union COM_FRAME_READ{
    struct{
      int32_t pos[4];
      int16_t vel[4];
      int16_t dis[4];
      int16_t cur[4];
      int16_t pwmRef[4];
    }values;
    uint8_t data[48];
  }com_frame_read;
  
  union COM_FRAME_WRITE{
    struct{
      int16_t Kp[4];
      int16_t Ki[4];
      int16_t Kd[4];
      int32_t sp[4];
      int16_t outputPosMax[4];
      int16_t outputNegMax[4];
      int16_t IntegralPosMax[4];
      int16_t IntegralNegMax[4];
      int16_t deadBand[4];
      uint8_t conf;
      uint8_t control_mode;
      uint8_t outputDivider[4];
    }values = {.Kp = {1,1,1,1}, .Ki = {0,0,0,0}, .Kd = {0,0,0,0}, .sp = {0,0,0,0}, .outputPosMax = {500,500,500,500}, .outputNegMax = {-500,-500,-500,-500},
      .IntegralPosMax = {0,0,0,0}, .IntegralNegMax = {0,0,0,0}, .deadBand = {0,0,0,0}, .conf =  0x40,  .control_mode=0, .outputDivider  = {0,0,0,0}
    };
    uint8_t data[86];
  }com_frame_write;

  void printWifiStatus() {
    // print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
  
    // print your board's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);
  
    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
  }

  void setup() {
    //Initialize serial and wait for port to open:
    Serial.begin(115200);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
  
    // check for the WiFi module:
    if (WiFi.status() == WL_NO_MODULE) {
      Serial.println("Communication with WiFi module failed!");
      // don't continue
      while (true);
    }
  
    // attempt to connect to Wifi network:
    while (status != WL_CONNECTED) {
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(ssid);
      // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
      status = WiFi.begin(ssid, pass);
    }
    Serial.println("Connected to wifi");
    printWifiStatus();
    Udp.begin(localPort);
    SPI.begin();
    pinMode (slaveSelectPin, OUTPUT);
    pinMode (transmissionPin, OUTPUT);  

    motors.enableDrivers();
    motors.setM1Speed(100);
  }

  void loop() {
    digitalWrite(transmissionPin, LOW);
    for(int i=0;i<87;i++){
      digitalWrite(slaveSelectPin, LOW);
      SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
      if(i<48)
        com_frame_read.data[i] = SPI.transfer(com_frame_write.data[i]);
      SPI.endTransaction();
      digitalWrite(slaveSelectPin, HIGH);
    }
    digitalWrite(transmissionPin, HIGH);

    // if there's data available, read a packet
    int packetSize = Udp.parsePacket();
    if (packetSize) {
      Serial.print("Received packet of size ");
      Serial.println(packetSize);
      Serial.print("From ");
      IPAddress remoteIp = Udp.remoteIP();
      Serial.print(remoteIp);
      Serial.print(", port ");
      Serial.println(Udp.remotePort());
  
      // read the packet into packetBufffer
      int len = Udp.read(com_frame_write.data, packetSize);
    }

    // send a reply, to the IP address and port that sent us the packet we received
    Udp.beginPacket(remoteIP, 8000);
    Udp.write(com_frame_read.data,48);
    Udp.endPacket();
//    char str[200];
//    sprintf(str,"pos:\t%d\t%d\t%d\t%d\n", com_frame_read.values.pos[0], com_frame_read.values.pos[1], com_frame_read.values.pos[2], com_frame_read.values.pos[3] );
//    Serial.print(str);
//    sprintf(str,"vel:\t%d\t%d\t%d\t%d\n", com_frame_read.values.vel[0], com_frame_read.values.vel[1], com_frame_read.values.vel[2], com_frame_read.values.vel[3] );
//    Serial.print(str);
//    sprintf(str,"dis:\t%d\t%d\t%d\t%d\n", com_frame_read.values.dis[0], com_frame_read.values.dis[1], com_frame_read.values.dis[2], com_frame_read.values.dis[3] );
//    Serial.print(str);
//    sprintf(str,"cur:\t%d\t%d\t%d\t%d\n", com_frame_read.values.cur[0], com_frame_read.values.cur[1], com_frame_read.values.cur[2], com_frame_read.values.cur[3] );
//    Serial.print(str);
//    sprintf(str,"pwm:\t%d\t%d\t%d\t%d\n", com_frame_read.values.pwmRef[0], com_frame_read.values.pwmRef[1], com_frame_read.values.pwmRef[2], com_frame_read.values.pwmRef[3] );
//    Serial.print(str);
//    delay(1000);
  }
  
#endif
