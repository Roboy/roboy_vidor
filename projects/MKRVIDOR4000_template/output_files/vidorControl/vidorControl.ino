#include <wiring_private.h>
#include "jtag.h"
#include <SPI.h>
#include "ardprintf.h"

#define TDI                               12
#define TDO                               15
#define TCK                               13
#define TMS                               14
#define MB_INT                            28
#define MB_INT_PIN                        31
#define SIGNAL_IN 33 //B2 N2

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

typedef union SPI_FRAME{
  struct{
    uint8_t motor;
    float Kp;
    float Ki;
    float Kd;
    float sp;
    int32_t outputLimit;
    uint8_t control_mode;
    uint16_t controlFlags;
    int32_t update_frequency;
    float pos_encoder_multiplier;
    float dis_encoder_multiplier;
    int16_t pwmRef;
    int32_t position;
    int16_t velocity;
    int16_t current;
    int32_t displacement;
    float position_raw;
    float velocity_raw;
    float displacement_raw;
    float position_conv;
    float velocity_conv;
    float displacement_conv;
    float displacement_myo_brick_conv;
    float position_err;
    float velocity_err;
    float displacement_err;
    float displacement_myo_brick_err;
    int32_t position_res;
    int32_t velocity_res;
    int32_t displacement_res;
    int32_t displacement_myo_brick_res;
    int32_t actual_update_frequency;
  }values;
  uint8_t data[114];
};

SPI_FRAME com_frame_write, com_frame_read;
const int NUMBER_OF_MOTORS = 6;

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("ready");
  
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

  // Configure onboard LED Pin as output
  pinMode(LED_BUILTIN, OUTPUT);
  
  SPI.begin();
  pinMode (slaveSelectPin, OUTPUT);
  pinMode (transmissionPin, OUTPUT);  

  com_frame_write.values.Kp = 1;
  com_frame_write.values.Ki = 0;
  com_frame_write.values.Kd = 0;
  com_frame_write.values.sp = 0;
  com_frame_write.values.outputLimit = 500;
  com_frame_write.values.control_mode=0;
  com_frame_write.values.controlFlags=0;
  com_frame_write.values.pos_encoder_multiplier=0.1;
  com_frame_write.values.dis_encoder_multiplier=0.1;
  com_frame_write.values.update_frequency=0;
  
}

void loop() {
  char str[400];
  for(int motor=0; motor<NUMBER_OF_MOTORS;motor++){
    com_frame_write.values.motor = motor;
    digitalWrite(transmissionPin, LOW);
    for(int i=0;i<114;i++){
      digitalWrite(slaveSelectPin, LOW);
      SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
      if(i<35)
        SPI.transfer(com_frame_write.data[i]);
      else
        com_frame_read.data[i] = SPI.transfer(com_frame_write.data[i]);
      SPI.endTransaction();
      digitalWrite(slaveSelectPin, HIGH);
    }
    digitalWrite(transmissionPin, HIGH);
    ardprintf("motor %d ------------", motor);
    ardprintf("motor %d, Kp %f, Ki %f, Kd %f, sp %f, outputLimit %d, control_mode %d, controlFlags %d, update_frequency %d ,pos_encoder_multiplier %f, dis_encoder_multiplier %f", 
    com_frame_write.values.motor, com_frame_write.values.Kp, com_frame_write.values.Ki, com_frame_write.values.Kd, com_frame_write.values.sp, com_frame_write.values.outputLimit,
    com_frame_write.values.control_mode, com_frame_write.values.controlFlags, com_frame_write.values.update_frequency, com_frame_write.values.pos_encoder_multiplier, com_frame_write.values.dis_encoder_multiplier); 
    ardprintf("pwmRef %d, position %d, velocity %d, displacement %d\n"
                 "position_raw %f, velocity_raw %f, displacement_raw %f\n"
                 "position_conv %f, velocity_conv %f, displacement_conv %f, displacement_myo_brick_conv %f\n" 
                 "position_err %f, velocity_err %f, displacement_err %f, displacement_myo_brick_err %f\n" 
                 "position_res %d, velocity_res %d, displacement_res %d, displacement_myo_brick_res %d\n" 
                 "actual_update_frequency %d",
    com_frame_write.values.pwmRef, com_frame_write.values.position, com_frame_write.values.velocity, com_frame_write.values.displacement, 
    com_frame_write.values.position_raw, com_frame_write.values.velocity_raw, com_frame_write.values.displacement_raw,
    com_frame_write.values.position_conv, com_frame_write.values.velocity_conv, com_frame_write.values.displacement_conv, com_frame_write.values.displacement_myo_brick_conv, 
    com_frame_write.values.position_err, com_frame_write.values.velocity_err, com_frame_write.values.displacement_err, com_frame_write.values.displacement_myo_brick_err, 
    com_frame_write.values.position_res, com_frame_write.values.velocity_res, com_frame_write.values.displacement_res, com_frame_write.values.displacement_myo_brick_res, 
    com_frame_write.values.actual_update_frequency); 
  }
  delay(1000);
};
