#include "Arduino.h"
#include <SPI.h>
#include <I2C.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <ROBC_rosMsg/RobcOdom.h>
#include <AP_InertialSensor.h>
#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_Compass.h>
#include <AP_Buffer.h>
#include <ROBC_motorControl.h>
#include <ROBC_IRsensors.h>
#include <rosserial_arduino/Adc.h>
#include "Servo.h"
#include <APM_RC.h> // ArduPilot Mega RC Library

#define SERIAL_DEBUG 0

#define APM2_HARDWARE
#define MPUREG_PRODUCT_ID_X     0x0C    // Product ID Register
#define MAG_ORIENTATION        AP_COMPASS_APM2_SHIELD

#define SERVO_INPUT             11
#define RED_LED_PIN             27
#define YELLOW_LED_PIN          26
#define BLUE_LED_PIN            25
#define LED_ON                  LOW
#define LED_OFF                 HIGH
#define IR_SENSORS_N            5
#define RC_CHANNELS             6

#define FACTOR                  10
#define P1                      1.9*FACTOR
#define P2                      97.8*FACTOR
#define MAX_STEERING_ANGLE      22

typedef struct{
  uint8_t servo;
  int16_t speed;
}refValue_t;

typedef struct{
  int16_t steeringAngle; //Degree
  int16_t speed;
}rosSubs_t;

const double Kp=2.2, Ki=1.5, Kd=0.05;
const analog_inputs_t ir_inpus[ IR_SENSORS_N] = {AN0, AN1, AN2, AN3, AN4};

ros::NodeHandle             nh;
geometry_msgs::Twist        imu_msg;
std_msgs::Int16MultiArray   ir_msg;
std_msgs::Int16MultiArray   ctrl_mtr_msg;
std_msgs::Int16MultiArray   rc_msg;
robc_controller::RobcOdom   odom_msg;

Arduino_Mega_ISR_Registry   isr_registry;
AP_TimerProcess             scheduler;
AP_InertialSensor_MPU6000   ins;
APM_PPMDecoder              rcPPM;
Servo                       servoSteering ;  // create servo object to control a servo
motorControl                motorCtrl(Kp, Ki, Kd);
ROBC_IRsensors              IRsensors;

uint16_t test[ IR_SENSORS_N ]={};
uint16_t rcVal[ RC_CHANNELS ]={};
int16_t motor[3]={};


static uint32_t _tmrMotionControl;
static uint32_t _tmrRangeSensor;
static uint32_t _tmrInertialSensors;
static uint32_t tnow;

static refValue_t  inputVal={   .servo = 100,
                                .speed = 0 };

static rosSubs_t   rosVal={ .steeringAngle = 0,
                            .speed = 0 };

static void inertialSensors_task( uint32_t const period );
static void rangeSensors_task( uint32_t const period );
static void motionControl_task( uint32_t const period );
static void rcPPM_task( );
static void servo_cb( const std_msgs::Int16& cmd_msg);
static void speed_cb( const std_msgs::Int16& cmd_msg);
static void flash_leds(bool on);
static int16_t _steeringAngle_degreeToServoInput( int16_t angle );
static int16_t _steeringAngle_ServoInputToDegree( int16_t servoSignal );
static int16_t _motor_countsToLinearMovement( int16_t counts );


ros::Publisher pub_imu("imu", &imu_msg);
ros::Publisher pub_ir("ir_sens", &ir_msg);
ros::Publisher pub_motor("motor", &ctrl_mtr_msg);
ros::Publisher pub_rc("rc", &rc_msg);
ros::Publisher pub_odom("odometry", &odom_msg);
ros::Subscriber<std_msgs::Int16> sub_servo("servo", servo_cb);
ros::Subscriber<std_msgs::Int16> sub_speed("speed", speed_cb);


void setup()
{
  nh.initNode();
  nh.advertise(pub_imu);
  nh.advertise(pub_ir);
  nh.advertise(pub_motor);
  nh.advertise(pub_rc);
  nh.advertise(pub_odom);

  nh.subscribe(sub_servo);
  nh.subscribe(sub_speed);

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

  isr_registry.init();
  scheduler.init(&isr_registry);

  ins.init(AP_InertialSensor::WARM_START, AP_InertialSensor::RATE_100HZ, delay, flash_leds, &scheduler);
  ins.init_gyro(delay, flash_leds);
  ins.init_accel(delay, flash_leds);

  IRsensors.init(IR_SENSORS_N, ir_inpus);

  rcPPM.Init( &isr_registry );
  servoSteering.attach(SERVO_INPUT);
  motorCtrl.initialize();

  ir_msg.data_length = IR_SENSORS_N;
  rc_msg.data_length =  RC_CHANNELS;
  ctrl_mtr_msg.data_length = 3;

  _tmrMotionControl = micros();
  _tmrInertialSensors = micros();
  _tmrRangeSensor = micros();

}



void loop()
{
  rcPPM_task( );
  motionControl_task ( 24900 );
  inertialSensors_task( 9900 );
  rangeSensors_task( 9900 );
  nh.spinOnce();
}



static void rcPPM_task( )
{
  if( !rcPPM.GetState() ) { return; }

  for(uint8_t rcCh = 0; rcCh < 6; ++rcCh){
    rcVal[rcCh] = rcPPM.InputCh(rcCh);
  }
  if( rcVal[4] > 1000){
    inputVal.servo = map( rcVal[3], 980, 2035, 140, 60);
    inputVal.speed = map( rcVal[1], 920, 2100, -80, 80);
  }
  else{
    inputVal.servo =_steeringAngle_degreeToServoInput( (int16_t) rosVal.steeringAngle );
    inputVal.speed = rosVal.speed;
  }
#if SERIAL_DEBUG
  Serial.println("rcPPM_task");
#endif
}


static void inertialSensors_task( uint32_t const period )
{
  // Inertial sensors read rate to 100hz maximum. We use 10000us
  // the read rate will end up at exactly 100hz because the Periodic Timer fires at 1khz
  tnow = micros();
  if (  tnow - _tmrInertialSensors < period ) {
    return;
  }
  _tmrInertialSensors = tnow;

  ins.update();
  Vector3f const accel = ins.get_accel();
  Vector3f const gyro = ins.get_gyro();

  imu_msg.linear.x = accel.x;
  imu_msg.linear.y = accel.y;
  imu_msg.linear.z = accel.z;
  imu_msg.angular.x = gyro.x;
  imu_msg.angular.y = gyro.y;
  imu_msg.angular.z = gyro.z;

  pub_imu.publish(&imu_msg);
#if SERIAL_DEBUG
  Serial.print("IntertialSensors_task: ");
#endif
}


static void rangeSensors_task( uint32_t const period )
{
  // range Sensor rate to 100hz maximum. We use 10000us
  // the read rate will end up at exactly 100hz because the Periodic Timer fires at 1khz
  tnow = micros();
  if (  tnow - _tmrRangeSensor < period ) {
    return;
  }
  _tmrRangeSensor = tnow;

  IRsensors.update();
  for(uint8_t irIndex; irIndex < IR_SENSORS_N; ++irIndex){
    test[irIndex]  = IRsensors.getIR(irIndex);
  }
  ir_msg.data = (int*)test;
  pub_ir.publish(&ir_msg);

#if SERIAL_DEBUG
  Serial.println("RangeSensors_task: ");
  IRsensors.print();
#endif
}


static void motionControl_task( uint32_t const period )
{
  // motion control rate to 40hz maximum. We use 25000us
  // the read rate will end up at exactly 40hz because the Periodic Timer fires at 1khz
  tnow = micros();
  if (  tnow - _tmrMotionControl < period ) {
    return;
  }
  _tmrMotionControl = tnow;
  motorCtrl.updateSpeed( inputVal.speed );
  servoSteering.write( inputVal.servo );
  odom_msg.encoderCounts = motorCtrl.getDiffCounts( );
  odom_msg.steeringAngle = _steeringAngle_ServoInputToDegree( inputVal.servo );
  odom_msg.timestamp = nh.now();
  pub_odom.publish(&odom_msg);

  motor[0] = motorCtrl.getDiffCounts( );
  motor[1] = motorCtrl.getAcumCounts( );
  motor[2] = motorCtrl.getOutputPID( );
  ctrl_mtr_msg.data = motor;
  pub_motor.publish(&ctrl_mtr_msg);

  rc_msg.data = (int*)rcVal;
  pub_rc.publish(&rc_msg);

#if SERIAL_DEBUG
  Serial.println("motionControl_task: ");
  motorCtrl.printInfoPID();
#endif
}


static void servo_cb( const std_msgs::Int16& cmd_msg){
  rosVal.steeringAngle = cmd_msg.data;
}


static void speed_cb( const std_msgs::Int16& cmd_msg){
  rosVal.speed = cmd_msg.data;
}


static void flash_leds(bool on)
{
  digitalWrite(BLUE_LED_PIN, on ? LED_OFF : LED_ON);
  digitalWrite(YELLOW_LED_PIN, on ? LED_OFF : LED_ON);
  digitalWrite(RED_LED_PIN, on ? LED_ON : LED_OFF);
}


static int16_t _steeringAngle_degreeToServoInput( int16_t angle )
{
  int16_t servoSignal;

  if( angle > MAX_STEERING_ANGLE)         angle =   MAX_STEERING_ANGLE;
  else if( angle < -MAX_STEERING_ANGLE)   angle = - MAX_STEERING_ANGLE;
  servoSignal = angle * P1 + P2;

  return ( int16_t ) servoSignal / FACTOR;
}


static int16_t _steeringAngle_ServoInputToDegree( int16_t servoSignal )
{
  int16_t angle = (int16_t)( servoSignal / P1 - P2 / P1 );

  return angle;
}


static int16_t _motor_countsToLinearMovement( int16_t counts )
{
  return counts;
}
