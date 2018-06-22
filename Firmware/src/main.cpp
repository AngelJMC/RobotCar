#include "Arduino.h"
#include <SPI.h>
#include <I2C.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
// #include <ros.h>
// #include <std_msgs/String.h>
// #include <geometry_msgs/Twist.h>
// #include <std_msgs/UInt16MultiArray.h>
// #include <std_msgs/UInt16.h>
// #include <std_msgs/Float32.h>
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

// ros::NodeHandle  nh;
// std_msgs::String str_msg;
// ros::Publisher chatter("chatter", &str_msg);
//
// std_msgs::Float32 temp_msg;
// ros::Publisher pub_temp("temperature", &temp_msg);
//
// geometry_msgs::Twist imu_msg;
// ros::Publisher pub_imu("imu", &imu_msg);
//
// std_msgs::UInt16MultiArray ir_msg;
// ros::Publisher pub_ir("ir_sens", &ir_msg);


// char hello[13] = "hello world!";

delaysMs_t lastMs;

Arduino_Mega_ISR_Registry _isr_registry,_isr_registry1;
AP_TimerProcess scheduler;

AP_InertialSensor_MPU6000 ins;
APM_PPMDecoder _rcPPM;
Servo servoSteering ;  // create servo object to control a servo
motorControl motorCtrl(Kp, Ki, Kd);
ROBC_IRsensors IRsensors;

// uint16_t test[5]={1,2,3,4,5};

// void serialEvent();

// void servo_cb( const std_msgs::UInt16& cmd_msg){
//   refVal.servoVal = cmd_msg.data;
// }
//
// void speed_cb( const std_msgs::UInt16& cmd_msg){
//   refVal.speedVal = cmd_msg.data;
// }
//
// ros::Subscriber<std_msgs::UInt16> sub_servo("servo", servo_cb);
// ros::Subscriber<std_msgs::UInt16> sub_speed("speed", speed_cb);



static void flash_leds(bool on)
{
  digitalWrite(BLUE_LED_PIN, on ? LED_OFF : LED_ON);
  digitalWrite(YELLOW_LED_PIN, on ? LED_OFF : LED_ON);
  digitalWrite(RED_LED_PIN, on ? LED_ON : LED_OFF);

}


void setup()
{

  // nh.initNode();
  // nh.advertise(chatter);
  // nh.advertise(pub_temp);
  // nh.advertise(pub_imu);
  // nh.advertise(pub_ir);
  //
  //  nh.subscribe(sub_servo);
  //  nh.subscribe(sub_speed);

  I2c.begin();
  I2c.timeOut(20);
  I2c.setSpeed(true);

  pinMode(40, OUTPUT);
  digitalWrite(40, HIGH);
  delay(100);
  SPI.begin();

  SPI.beginTransaction (SPISettings (1000000, MSBFIRST, SPI_MODE3));
  Serial.begin(57600);



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
  refVal.speedVal = 0;

  // ir_msg.data_length = 5;

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
    IRsensors.update();

    accel = ins.get_accel();
    gyro = ins.get_gyro();

    // temp_msg.data = accel.x;
    // pub_temp.publish(&temp_msg);
    //
    // imu_msg.linear.x = accel.x;
    // imu_msg.linear.y = accel.y;
    // imu_msg.linear.z = accel.z;
    //
    // imu_msg.angular.x = gyro.x;
    // imu_msg.angular.y = gyro.y;
    // imu_msg.angular.z = gyro.z;
    //
    // pub_imu.publish(&imu_msg);

    // Serial.print(accel.x);
    // Serial.print("  ,  ");
    // Serial.print(accel.y);
    // Serial.print("  ,  ");
    // Serial.println(accel.z);
    // Serial.print(gyro.x);
    // Serial.print("  ,  ");
    // Serial.print(gyro.y);
    // Serial.print("  ,  ");
    // Serial.println(gyro.z);

    // for(uint8_t irIndex; irIndex < IR_SENSORS_N; ++irIndex){
    //   test[irIndex]  = IRsensors.getIR(irIndex);
    // }

    // ir_msg.data = test;
    // pub_ir.publish(&ir_msg);
    // IRsensors.print();
  }

  // if ((millis() - lastMs.steering) >= 500) {
  //   lastMs.steering = millis();
  //
  //   str_msg.data = hello;
  //   chatter.publish( &str_msg );
  //
  // }
  // nh.spinOnce();

  if ((millis() - lastMs.motor) >= 25) {

    lastMs.motor = millis();
    motorCtrl.updateSpeed(refVal.speedVal);
    servoSteering.write( refVal.servoVal );

     motorCtrl.printInfoPID();

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
