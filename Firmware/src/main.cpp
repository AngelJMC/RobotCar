// #include "Arduino.h"
// #include <FastSerial.h>
#include <SPI.h>
#include <I2C.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_InertialSensor.h>
#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_Compass.h>
#include <AP_Buffer.h>

#define APM2_HARDWARE
#define MPUREG_PRODUCT_ID_X                               0x0C    // Product ID Register

#include "Encoder.h"
#include "L298N.h"
#include "Servo.h"
#include "PID_v1.h"
#include <APM_RC.h> // ArduPilot Mega RC Library
#include "control.h"
#include "cli.h"
#include "rc_command.h"

#define _EN  8
#define _IN1 7
#define _IN2 6

#define _INT1 3
#define _INT2 2

# define RED_LED_PIN        27
# define YELLOW_LED_PIN     26
# define BLUE_LED_PIN       25
# define LED_ON           LOW
# define LED_OFF          HIGH
# define MAG_ORIENTATION  AP_COMPASS_APM2_SHIELD

typedef struct{
  uint32_t motor = 0;
  uint32_t steering = 0;
}delaysMs_t;

pidParameter_t _cval={
  .setPointMotor = 0,
  .input = 0,
  .output = 0,
  .Kp = 1.2,
  .Ki = 0.7,
  .Kd = 0.001,
};

refValue_t refVal;

cliHdlr_t _cliHdlr={
  .BufferSerialInput = "",
  .rcvNumberString = "",
  .FlagBufferInput = false,  // whether the string is complete
};

delaysMs_t lastMs;

Arduino_Mega_ISR_Registry _isr_registry,_isr_registry1;
AP_TimerProcess scheduler;

// AP_Compass_HMC5843 compass;
//
AP_InertialSensor_MPU6000 ins;

APM_PPMDecoder _rcPPM;
Servo servoSteering ;  // create servo object to control a servo
PID pidMtr(&_cval.input, &_cval.output , &_cval.setPointMotor, _cval.Kp, _cval.Ki, _cval.Kd, DIRECT);
L298N driverMtr(_EN, _IN1, _IN2);
Encoder encoderMtr(_INT1, _INT2);

void serialEvent();

// uint8_t register_read( uint8_t reg )
// {
//     uint8_t return_value;
//     uint8_t addr = reg | 0x80; // Set most significant bit
//
//     digitalWrite(53, LOW);
//
//     SPI.transfer(0b00001100);
//     return_value = SPI.transfer(0x00);
//
//     digitalWrite(53, HIGH);
//
//     return return_value;
// }

static void flash_leds(bool on)
{
  digitalWrite(BLUE_LED_PIN, on ? LED_OFF : LED_ON);
  digitalWrite(YELLOW_LED_PIN, on ? LED_OFF : LED_ON);
  digitalWrite(RED_LED_PIN, on ? LED_ON : LED_OFF);

}

void setup() {
  I2c.begin();
  I2c.timeOut(20);
  I2c.setSpeed(true);

  pinMode(40, OUTPUT);
  digitalWrite(40, HIGH);
 delay(100);
  SPI.begin();

  SPI.beginTransaction (SPISettings (1000000, MSBFIRST, SPI_MODE3));
  Serial.begin(115200);

  // we need to stop the barometer from holding the SPI bus
  delay(1);
 pinMode(BLUE_LED_PIN, OUTPUT);
 pinMode(YELLOW_LED_PIN, OUTPUT);
 pinMode(RED_LED_PIN, OUTPUT);

   _isr_registry.init();
  scheduler.init(&_isr_registry);


  ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ, delay, flash_leds, &scheduler);
  ins.init_accel(delay, flash_leds);

  initRC(&_rcPPM, &_isr_registry);
  initControl(&servoSteering,&pidMtr);
  _cliHdlr.BufferSerialInput.reserve(20);   // reserve 20 bytes for the inputString:

}

void loop() {

  if (_cliHdlr.FlagBufferInput) {
    updateCLI( &_cliHdlr, &refVal);
  }

  if (_rcPPM.GetState() == true ) {
    updateRC( &_rcPPM, &refVal);
  }

  if ((millis() - lastMs.motor) >= 100) {
    Vector3f accel;
    Vector3f gyro;

    lastMs.motor = millis();
    updateSpeed(&_cval, &encoderMtr, &pidMtr, &driverMtr, refVal);
    printInfoPID(&_cval);

    ins.update();


       delay(1000);
    accel = ins.get_accel();
    gyro = ins.get_gyro();
    // apm_imu_msg.ax=(accel.x+0.07262561)*0.99451745;
    //  apm_imu_msg.ay=(accel.y-0.34247160)*0.99348778;
    // apm_imu_msg.az=(accel.z-1.81453760)*0.98398465;
    Serial.println(accel.x);
    Serial.println(accel.y);
    Serial.println(accel.z);
    Serial.println(gyro.x);
    Serial.println(gyro.y);
    Serial.println(gyro.z);


  }

  if ((millis() - lastMs.steering) >= 10){
    lastMs.steering = millis();
    updateSteering( &servoSteering, refVal);
  }
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    _cliHdlr.BufferSerialInput += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      _cliHdlr.FlagBufferInput = true;
    }
  }
}
