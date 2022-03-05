#include <elapsedMillis.h>
elapsedMillis timeElapsed;

/*******************************************************************************
 * IO DEFINITION                                                                *
 *******************************************************************************/

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

/*******************************************
************************************
 * PRIVATE GLOBAL VARIABLES                                                     *
 *******************************************************************************/

// Speed of the motor.
static int Speed = 255;

int sensorVal_1;
int sensorVal_2;

int req_analogVal;
float strokeLength = 2.0;
float extensionLength_1;
float extensionLength_2;

int maxAnalogReading;
int minAnalogReading;

/*******************************************************************************
 * FUNCTIONS                                                                    *
 *******************************************************************************/

// The setup routine runs once when you press reset.
void setup() {                
  // Initialize the PWM and DIR pins as digital outputs.
  pinMode(pinPwm_1, OUTPUT);
  pinMode(pinDir_1, OUTPUT);
  pinMode(sensorPin_1, INPUT);

  pinMode(pinPwm_2, OUTPUT);
  pinMode(pinDir_2, OUTPUT);
  pinMode(sensorPin_2, INPUT);

  Serial.begin(9600);

  //wait for ready
//  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//  while (Serial.available() && Serial.read()); // empty buffer
//  while (!Serial.available());                 // wait for data
//  while (Serial.available() && Serial.read()); // empty buffer again

  delay(2000);

  maxAnalogReading = 220;
  minAnalogReading = 80;
}

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

//// Function to get the maximum potentiometer output
//int moveToLimit(int Direction) {
//  int prevReading = 0;
//  int currReading = 0;
//
//  driveActuator(Direction, Speed);
//  delay(2000);
//  currReading = analogRead(sensorPin);
//
//  return currReading;
//
////  do {
////    prevReading = currReading;
////    driveActuator(Direction, Speed);
////    timeElapsed = 0;
////    while(timeElapsed < 200){ delay(1);}           //keep moving until analog reading remains the same for 200ms
////    currReading = analogRead(sensorPin);
////  }while(prevReading != currReading);
////  return currReading;
//}

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
      analogWrite(pinPwm_2, 0);
      digitalWrite(pinDir_2, HIGH);
      break;

    case -1:      //retraction
      analogWrite(pinPwm_2, Speed);
      digitalWrite(pinDir_2, LOW);
      break;
  }
}
