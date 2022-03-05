// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */


// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Store previous IMU value
float ypr_prev[3];

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// Force sensor define
#define FORCE_SENSOR_PIN_HEEL A0 // the FSR and 10K pulldown are connected to A0
#define FORCE_SENSOR_PIN_TOE A1 // the FSR and 10K pulldown are connected to A1

// Define LED
#define LED_PIN_HEEL 13
#define LED_PIN_TOE 12
bool blinkState_heel = false;
bool blinkState_toe = false;

// Define slope variables
unsigned long t;
unsigned long t_cur;
unsigned long t_prev;

float slope;

bool rising_edge = true;
bool zero_crossing = true;

// Motor shield #1
// PWM is connected to pin 3.
const int pinPwm_1 = 3;

// DIR is connected to pin 2.
const int pinDir_1 = 4;

// Pot sensor pin connected to A3
int sensorPin_1 = A2;

// PWM is connected to pin 3.
const int pinPwm_2 = 6;

// DIR is connected to pin 2.
const int pinDir_2 = 8;

// Pot sensor pin connected to A3
int sensorPin_2 = A3;

/*******************************************************************************
 * PRIVATE GLOBAL VARIABLES                                                     *
 *******************************************************************************/

// Speed of the motor.
int Speed = 255;

int sensorVal_1;
int sensorVal_2;

int req_analogVal;

float strokeLength = 2.0;
float extensionLength_1;
float extensionLength_2;

int maxAnalogReading;
int minAnalogReading;

float pot_diff = 0;
float speed_increment = 2;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN_HEEL, OUTPUT);
    pinMode(LED_PIN_TOE, OUTPUT);

    ypr_prev[1] = ypr[1];

    // Initialize the PWM and DIR pins as digital outputs.
    pinMode(pinPwm_1, OUTPUT);
    pinMode(pinDir_1, OUTPUT);
    pinMode(sensorPin_1, INPUT);

    pinMode(pinPwm_2, OUTPUT);
    pinMode(pinDir_2, OUTPUT);
    pinMode(sensorPin_2, INPUT);

    maxAnalogReading = 220;
    minAnalogReading = 80;
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

// The loop routine runs over and over again forever.
void loop() {
  Serial.println("Extending...");
  sensorVal_1 = analogRead(sensorPin_1);
  while(sensorVal_1 < maxAnalogReading){
    driveActuator_1(1, Speed);
    displayOutput();  
    delay(20);
  }
  driveActuator_1(0, Speed);
  delay(1000);

  Serial.println("Extending...");
  sensorVal_2 = analogRead(sensorPin_2);
  while(sensorVal_2 < maxAnalogReading){
    driveActuator_2(1, Speed);
    displayOutput();  
    delay(20);
  }
  driveActuator_2(0, Speed);
  delay(1000);
  
  Serial.println("Retracting...");
  sensorVal_1 = analogRead(sensorPin_1);
  while(sensorVal_1 > minAnalogReading){
    driveActuator_1(-1, Speed);
    displayOutput();  
    delay(20);
  }
  driveActuator_1(0, Speed);
  delay(1000);

  Serial.println("Retracting...");
  sensorVal_2 = analogRead(sensorPin_2);
  while(sensorVal_2 > minAnalogReading){
    driveActuator_2(-1, Speed);
    displayOutput();  
    delay(20);
  }
  driveActuator_2(0, Speed);
  delay(1000);

  Serial.println("Extending to 0.5 inches");
  req_analogVal = mapLength2Analog(0.5, float(minAnalogReading), float(maxAnalogReading), 0.0, strokeLength);
  sensorVal_1 = analogRead(sensorPin_1);
  while(sensorVal_1 < req_analogVal){
    driveActuator_1(1, Speed);
    displayOutput();  
    delay(20);
  }
  driveActuator_1(0, Speed);
  delay(1000);

  Serial.println("Extending to 0.5 inches");
  //req_analogVal = mapLength2Analog(0.5, float(minAnalogReading), float(maxAnalogReading), 0.0, strokeLength);
  sensorVal_2 = analogRead(sensorPin_2);
  while(sensorVal_2 < req_analogVal){
    driveActuator_2(1, Speed);
    displayOutput();  
    delay(20);
  }
  driveActuator_2(0, Speed);
  delay(1000);

  Serial.println("Retracting...");
  sensorVal_1 = analogRead(sensorPin_1);
  while(sensorVal_1 > minAnalogReading){
    driveActuator_1(-1, Speed);
    displayOutput();  
    delay(20);
  }
  driveActuator_1(0, Speed);
  delay(1000);

  Serial.println("Retracting...");
  sensorVal_2 = analogRead(sensorPin_2);
  while(sensorVal_2 > minAnalogReading){
    driveActuator_2(-1, Speed);
    displayOutput();  
    delay(20);
  }
  driveActuator_2(0, Speed);
  delay(1000);
}

float mapfloat(float x, float inputMin, float inputMax, float outputMin, float outputMax){
  return (x-inputMin)*(outputMax - outputMin)/(inputMax - inputMin)+outputMin;
}

float mapLength2Analog (float y, float inputMin, float inputMax, float outputMin, float outputMax) {
  return (y-outputMin)/((outputMax - outputMin)/(inputMax - inputMin)) + inputMin;
}

void displayOutput(){
  sensorVal_1 = analogRead(sensorPin_1);
  extensionLength_1 = mapfloat(sensorVal_1, float(minAnalogReading), float(maxAnalogReading), 0.0, strokeLength);
  Serial.print("Analog Reading 1: ");
  Serial.print(sensorVal_1);
  Serial.print("\tActuator extension length 1: ");
  Serial.print(extensionLength_1);
  Serial.println(" inches");

  sensorVal_2 = analogRead(sensorPin_2);
  extensionLength_2 = mapfloat(sensorVal_2, float(minAnalogReading), float(maxAnalogReading), 0.0, strokeLength);
  Serial.print("Analog Reading 2: ");
  Serial.print(sensorVal_2);
  Serial.print("\tActuator extension length 2: ");
  Serial.print(extensionLength_2);
  Serial.println(" inches");  
}

// Wrapper function to drive actuation
void driveActuator_1(int Direction, int Speed){
  switch(Direction){
    case 1:       //extension
      analogWrite(pinPwm_1, Speed);
      digitalWrite(pinDir_1, HIGH);
      break;
   
    case 0:       //stopping
      analogWrite(pinPwm_1, 0);
      digitalWrite(pinDir_1, HIGH);
      break;

    case -1:      //retraction
      analogWrite(pinPwm_1, Speed);
      digitalWrite(pinDir_1, LOW);
      break;
  }
}

// Wrapper function to drive actuation
void driveActuator_2(int Direction, int Speed){
  switch(Direction){
    case 1:       //extension
      analogWrite(pinPwm_2, Speed);
      digitalWrite(pinDir_2, HIGH);
      break;
   
    case 0:       //stopping
      analogWrite(pinPwm_2, Speed);
      digitalWrite(pinDir_2, HIGH);
      break;

    case -1:      //retraction
      analogWrite(pinPwm_2, Speed);
      digitalWrite(pinDir_2, LOW);
      break;
  }
}
