#include <wiring_private.h>
#include "jtag.h"
#include <SPI.h>
#include <DualMAX14870MotorShield.h>

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

const int transmissionPin = A0;
const int slaveSelectPin = A1;

union COM_FRAME_READ{
  struct{
    int32_t pos[6];
    int16_t vel[6];
    int16_t dis[6];
    int16_t cur[6];
    int16_t pwmRef[6];
  }values;
  uint8_t data[72];
}com_frame_read;

union COM_FRAME_WRITE{
  struct{
    int16_t Kp[6];
    int16_t Ki[6];
    int16_t Kd[6];
    int32_t sp[6];
    int16_t outputPosMax[6];
    int16_t outputNegMax[6];
    int16_t IntegralPosMax[6];
    int16_t IntegralNegMax[6];
    int16_t deadBand[6];
    uint8_t conf[2];
    uint8_t control_mode[6];
    uint8_t outputDivider[6];
  }values = {.Kp = {1,1,1,1,1,1}, .Ki = {0,0,0,0,0,0}, .Kd = {0,0,0,0,0,0}, .sp = {0,0,0,0,0,0}, .outputPosMax = {500,500,500,500,500,500}, .outputNegMax = {-500,-500,-500,-500,-500,-500},
    .IntegralPosMax = {0,0,0,0,0,0}, .IntegralNegMax = {0,0,0,0,0,0}, .deadBand = {0,0,0,0,0,0}, .conf =  {0,1},  .control_mode= {0,0,0,0,0,0}, .outputDivider  = {0,0,0,0,0,0}
  };
  uint8_t data[134];
}com_frame_write;


#define M1DIR A3
#define M1PWM A4
#define M2DIR 99
#define M2PWM 99
#define nEN A5
#define nFAULT A6
DualMAX14870MotorShield motors(M1DIR,M1PWM,M2DIR,M2PWM,nEN,nFAULT);

// the setup function runs once when you press reset or power the board
void setup() {

  int ret;
  uint32_t ptr[1];

  //enableFpgaClock();
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
  
  SPI.begin();
  pinMode (slaveSelectPin, OUTPUT);
  pinMode (transmissionPin, OUTPUT); 

  motors.enableDrivers();
  motors.setM1Speed(100);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(transmissionPin, LOW);
  for(int i=0;i<134;i++){
    digitalWrite(slaveSelectPin, LOW);
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
    if(i<72)
      com_frame_read.data[i] = SPI.transfer(com_frame_write.data[i]);
    else
      SPI.transfer(com_frame_write.data[i]);
    SPI.endTransaction();
    digitalWrite(slaveSelectPin, HIGH);
  }
  digitalWrite(transmissionPin, HIGH);
}
