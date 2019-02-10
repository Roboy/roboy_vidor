#include <wiring_private.h>
#include <SPI.h>
#include "Blaster.h"
#include <stdio.h>
extern void enableFpgaClock();
const int transmissionPin = 6;
const int slaveSelectPin = 7;

//#define BLAST

void setup() {
  #ifdef BLAST
    USBBlaster.setOutEpSize(60);
    USBBlaster.begin(true);
  #else
    Serial.begin(115200);
    SPI.begin();
    pinMode (slaveSelectPin, OUTPUT);
    pinMode (transmissionPin, OUTPUT);
  #endif
  
  // also start the 48MHz clock feed for the FPGA, in case we need to run the bitstream and need this clock
  enableFpgaClock();
}

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
  }values;
  uint8_t data[86];
}com_frame_write;

void loop() {
  #ifdef BLAST
    USBBlaster.loop();
  #else
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
    char str[200];
    sprintf(str,"pos:\t%d\t%d\t%d\t%d\n", com_frame_read.values.pos[0], com_frame_read.values.pos[1], com_frame_read.values.pos[2], com_frame_read.values.pos[3] );
    Serial.print(str);
    sprintf(str,"vel:\t%d\t%d\t%d\t%d\n", com_frame_read.values.vel[0], com_frame_read.values.vel[1], com_frame_read.values.vel[2], com_frame_read.values.vel[3] );
    Serial.print(str);
    sprintf(str,"dis:\t%d\t%d\t%d\t%d\n", com_frame_read.values.dis[0], com_frame_read.values.dis[1], com_frame_read.values.dis[2], com_frame_read.values.dis[3] );
    Serial.print(str);
    sprintf(str,"cur:\t%d\t%d\t%d\t%d\n", com_frame_read.values.cur[0], com_frame_read.values.cur[1], com_frame_read.values.cur[2], com_frame_read.values.cur[3] );
    Serial.print(str);
    sprintf(str,"pwm:\t%d\t%d\t%d\t%d\n", com_frame_read.values.pwmRef[0], com_frame_read.values.pwmRef[1], com_frame_read.values.pwmRef[2], com_frame_read.values.pwmRef[3] );
    Serial.print(str);
    delay(1000);
  #endif
}
