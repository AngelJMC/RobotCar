#include "Arduino.h"
#include <SPI.h>
#include <I2C.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_InertialSensor.h>
#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_Compass.h>
#include <AP_Buffer.h>
#include <ROBC_motorControl.h>
#include <ROBC_IRsensors.h>

#include "Servo.h"
#include <APM_RC.h> // ArduPilot Mega RC Library
#include "cli.h"
#include "rc_command.h"

#define APM2_HARDWARE
#define MPUREG_PRODUCT_ID_X                               0x0C    // Product ID Register
# define MAG_ORIENTATION  AP_COMPASS_APM2_SHIELD

#define SERVO_INPUT         11
#define RED_LED_PIN        27
#define YELLOW_LED_PIN     26
#define BLUE_LED_PIN       25
#define LED_ON           LOW
#define LED_OFF          HIGH

#define  IR_SENSORS_N 5

typedef struct{
  uint32_t motor;
  uint32_t steering;
  uint32_t sensors;
}delaysMs_t;

double Kp=1.2, Ki=0.7, Kd=0.001;

const analog_inputs_t ir_inpus[ IR_SENSORS_N] = {AN0, AN1, AN2, AN3, AN4};

refValue_t refVal;

cliHdlr_t _cliHdlr={
  .BufferSerialInput = "",
  .rcvNumberString = "",
  .FlagBufferInput = false,  // whether the string is complete
};

delaysMs_t lastMs;

Arduino_Mega_ISR_Registry _isr_registry,_isr_registry1;
AP_TimerProcess scheduler;

AP_InertialSensor_MPU6000 ins;
APM_PPMDecoder _rcPPM;
Servo servoSteering ;  // create servo object to control a servo
motorControl motorCtrl(Kp, Ki, Kd);
ROBC_IRsensors IRsensors;


void serialEvent();


static void flash_leds(bool on)
{
  digitalWrite(BLUE_LED_PIN, on ? LED_OFF : LED_ON);
  digitalWrite(YELLOW_LED_PIN, on ? LED_OFF : LED_ON);
  digitalWrite(RED_LED_PIN, on ? LED_ON : LED_OFF);

}


void setup()
{
  I2c.begin();
  I2c.timeOut(20);
  I2c.setSpeed(true);

  pinMode(40, OUTPUT);
  digitalWrite(40, HIGH);
  delay(100);
  SPI.begin();

  SPI.beginTransaction (SPISettings (1000000, MSBFIRST, SPI_MODE3));
  Serial.begin(115200);

  delay(1);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);

   _isr_registry.init();
  scheduler.init(&_isr_registry);

  ins.init(AP_InertialSensor::WARM_START, AP_InertialSensor::RATE_100HZ, delay, flash_leds, &scheduler);
  ins.init_gyro(delay, flash_leds);
  ins.init_accel(delay, flash_leds);

  IRsensors.init(IR_SENSORS_N, ir_inpus);

  initRC(&_rcPPM, &_isr_registry);
  servoSteering.attach(SERVO_INPUT);
  motorCtrl.initialize();
  _cliHdlr.BufferSerialInput.reserve(20);   // reserve 20 bytes for the inputString:
  refVal.servoVal = 100;
}


void loop()
{

  if (_cliHdlr.FlagBufferInput) {
    updateCLI( &_cliHdlr, &refVal);
  }

  if (_rcPPM.GetState() == true ) {
    updateRC( &_rcPPM, &refVal);
  }

  if ( (millis() - lastMs.sensors) >=10){
    Vector3f accel;
    Vector3f gyro;
    lastMs.sensors = millis();
    ins.update();

    accel = ins.get_accel();
    gyro = ins.get_gyro();

    Serial.print(accel.x);
    Serial.print("  ,  ");
    Serial.print(accel.y);
    Serial.print("  ,  ");
    Serial.println(accel.z);
    Serial.print(gyro.x);
    Serial.print("  ,  ");
    Serial.print(gyro.y);
    Serial.print("  ,  ");
    Serial.println(gyro.z);

    IRsensors.update();
    IRsensors.print();
  }

  if ((millis() - lastMs.motor) >= 25) {

    lastMs.motor = millis();
    motorCtrl.updateSpeed(refVal.speedVal);
    motorCtrl.printInfoPID();

    lastMs.steering = millis();
    servoSteering.write( refVal.servoVal );
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
