#include <wiring_private.h>
#include "jtag.h"
#include <SPI.h>
//#include <WiFiNINA.h>
//#include <WiFiUdp.h>

#define TDI                               12
#define TDO                               15
#define TCK                               13
#define TMS                               14
#define MB_INT                            28
#define MB_INT_PIN                        31
#define SIGNAL_IN                         33 //B2 N2

#define SPI_WRITE_COMMAND                 (1<<7)
#define ADDR_PERIPH_PWM                   (1<<3)
#define ADDR_SPI_REG_1                    0
#define ADDR_SPI_REG_2                    1

#define no_data    0xFF, 0xFF, 0xFF, 0xFF, \
          0xFF, 0xFF, 0xFF, 0xFF, \
          0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
          0xFF, 0xFF, 0xFF, 0xFF, \
          0x00, 0x00, 0x00, 0x00  \

#define NO_BOOTLOADER   no_data
#define NO_APP        no_data
#define NO_USER_DATA    no_data

__attribute__ ((used, section(".fpga_bitstream_signature")))
const unsigned char signatures[4096] = {
  //#include "signature.ttf"
  NO_BOOTLOADER,

  0x00, 0x00, 0x08, 0x00,
  0xA9, 0x6F, 0x1F, 0x00,   // Don't care.
  0x20, 0x77, 0x77, 0x77, 0x2e, 0x73, 0x79, 0x73, 0x74, 0x65, 0x6d, 0x65, 0x73, 0x2d, 0x65, 0x6d, 0x62, 0x61, 0x72, 0x71, 0x75, 0x65, 0x73, 0x2e, 0x66, 0x72, 0x20, 0x00, 0x00, 0xff, 0xf0, 0x0f,
  0x01, 0x00, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x00,   // Force

  NO_USER_DATA,
};
__attribute__ ((used, section(".fpga_bitstream")))
const unsigned char bitstream[] = {
  #include "app.h"
};

const int slaveSelectPin = 7;

unsigned char PWM_Puls;
char PWM_IncDir;

//int status = WL_IDLE_STATUS;
//char ssid[] = "why-fi";        // your network SSID (name)
//char pass[] = "gotohell11880";    // your network password (use for WPA, or use as key for WEP)
//
//void printWifiStatus() {
//  // print the SSID of the network you're attached to:
//  Serial.print("SSID: ");
//  Serial.println(WiFi.SSID());
//
//  // print your board's IP address:
//  IPAddress ip = WiFi.localIP();
//  Serial.print("IP Address: ");
//  Serial.println(ip);
//
//  // print the received signal strength:
//  long rssi = WiFi.RSSI();
//  Serial.print("signal strength (RSSI):");
//  Serial.print(rssi);
//  Serial.println(" dBm");
//}

// the setup function runs once when you press reset or power the board
void setup() {

  //Initialize serial and wait for port to open:
  Serial.begin(115200);
//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }
//
//  // check for the WiFi module:
//  if (WiFi.status() == WL_NO_MODULE) {
//    Serial.println("Communication with WiFi module failed!");
//    // don't continue
//    while (true);
//  }
//
//  String fv = WiFi.firmwareVersion();
//  if (fv < "1.0.0") {
//    Serial.println("Please upgrade the firmware");
//  }
//
//  // attempt to connect to Wifi network:
//  while (status != WL_CONNECTED) {
//    Serial.print("Attempting to connect to SSID: ");
//    Serial.println(ssid);
//    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
//    status = WiFi.begin(ssid, pass);
//
//    // wait 10 seconds for connection:
//    delay(10000);
//  }
//  Serial.println("Connected to wifi");
//  printWifiStatus();


  int ret;
  uint32_t ptr[1];

  pinPeripheral(30, PIO_AC_CLK);
  clockout(0, 1);
  delay(1000);

  //Init Jtag Port
  ret = jtagInit();
  mbPinSet();

  // Load FPGA user configuration
  ptr[0] = 0 | 3;
  mbEveSend(ptr, 1);

  // Give it delay
  delay(1000);

  // Configure onboard LED Pin as output
  pinMode(LED_BUILTIN, OUTPUT);

  // Disable all JTAG Pins (usefull for USB BLASTER connection)
  pinMode(TDO, INPUT);
  pinMode(TMS, INPUT);
  pinMode(TDI, INPUT);
  pinMode(TCK, INPUT);

  // Configure other share pins as input too
  pinMode(SIGNAL_IN, INPUT);
  pinMode(MB_INT, INPUT);


  // Positionne le SS de la liaison SPI vers le FPGA en sortie
  pinMode (slaveSelectPin, OUTPUT);

  // initialise la liaison SPI
  SPI.begin();

  // Rapport cyclique
  PWM_Puls = 0;
  // Increment / Decrement
  PWM_IncDir = 1;

  // SS : Par defaut on adresse pas le FPGA
  digitalWrite(slaveSelectPin, HIGH);

  // ADDR_SPI_REG_2[7:0] = Rapport cyclique. Initialise a 0%
  SPIFPGAWrite(SPI_WRITE_COMMAND | ADDR_PERIPH_PWM | ADDR_SPI_REG_2 , 0);


  // ADDR_SPI_REG_1[7] = PWM ON/OFF, ADDR_SPI_REG_1[3:0] = Prediviseur d'horloge
  // Pour une frequence FPGA = 80Mhz (non fiable). Prediviseur =
  // 0  : 156.25 Khz
  // 1  : 78.125 Khz
  // ...
  // 15 : 4.77 Hz
  SPIFPGAWrite(SPI_WRITE_COMMAND | ADDR_PERIPH_PWM | ADDR_SPI_REG_1, 128 + 1);


}


void SPIFPGAWrite(int adresse, int valeur) {

  digitalWrite(slaveSelectPin, LOW);
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  uint8_t data0 = SPI.transfer(adresse);
  uint8_t data1 = SPI.transfer(valeur);
  
  SPI.endTransaction();

  digitalWrite(slaveSelectPin, HIGH);
}

// the loop function runs over and over again forever
void loop() {

       delay(50);
       PWM_Puls+= PWM_IncDir;

       // Change de direction a 75% de rapport cyclique
       if ((PWM_Puls == 0) || (PWM_Puls==191)) PWM_IncDir *= -1;

       // Ecriture du nouveau rapport cyclique
       SPIFPGAWrite(SPI_WRITE_COMMAND | ADDR_PERIPH_PWM | ADDR_SPI_REG_2, PWM_Puls);

}
