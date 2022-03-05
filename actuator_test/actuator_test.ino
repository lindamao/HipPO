#include <elapsedMillis.h>
elapsedMillis timeElapsed;

/*******************************************************************************
 * IO DEFINITION                                                                *
 *******************************************************************************/

// PWM is connected to pin 3.
const int pinPwm = 3;

// DIR is connected to pin 2.
const int pinDir = 4;

// Pot sensor pin connected to A3
int sensorPin = A3;

/*******************************************
************************************
 * PRIVATE GLOBAL VARIABLES                                                     *
 *******************************************************************************/

// Speed of the motor.
static int Speed = 255;

int sensorVal;
float strokeLength = 2.0;
float extensionLength;

int maxAnalogReading;
int minAnalogReading;

/*******************************************************************************
 * FUNCTIONS                                                                    *
 *******************************************************************************/

// The setup routine runs once when you press reset.
void setup() {                
  // Initialize the PWM and DIR pins as digital outputs.
  pinMode(pinPwm, OUTPUT);
  pinMode(pinDir, OUTPUT);
  pinMode(sensorPin, INPUT);

  Serial.begin(9600);

  delay(2000);

  //maxAnalogReading = moveToLimit(1); //250
  minAnalogReading = moveToLimit(-1); //44

}

// The loop routine runs over and over again forever.
void loop() {
  driveActuator(0, Speed);
  displayOutput();
//  Serial.println("Extending...");
//  driveActuator(1, Speed);
//  delay(1000);
//  driveActuator(0, Speed);
//  delay(1000);
//  
//  Serial.println("Retracting...");
//  driveActuator(-1, Speed);
//  delay(1000);
//  driveActuator(0, Speed);
//  delay(1000);
}

// Function to get the maximum potentiometer output
int moveToLimit(int Direction) {
  int prevReading = 0;
  int currReading = 0;

  driveActuator(Direction, Speed);
  delay(2000);
  currReading = analogRead(sensorPin);

  return currReading;

//  do {
//    prevReading = currReading;
//    driveActuator(Direction, Speed);
//    timeElapsed = 0;
//    while(timeElapsed < 200){ delay(1);}           //keep moving until analog reading remains the same for 200ms
//    currReading = analogRead(sensorPin);
//  }while(prevReading != currReading);
//  return currReading;
}

float mapfloat(float x, float inputMin, float inputMax, float outputMin, float outputMax){
  return (x-inputMin)*(outputMax - outputMin)/(inputMax - inputMin)+outputMin;
}

void displayOutput(){
  sensorVal = analogRead(sensorPin);
  extensionLength = mapfloat(sensorVal, float(minAnalogReading), float(maxAnalogReading), 0.0, strokeLength);
  Serial.print("Analog Reading: ");
  Serial.print(sensorVal);
  Serial.print("\tActuator extension length: ");
  Serial.print(extensionLength);
  Serial.println(" inches");  
}

// Wrapper function to drive actuation
void driveActuator(int Direction, int Speed){
  switch(Direction){
    case 1:       //extension
      analogWrite(pinPwm, Speed);
      digitalWrite(pinDir, HIGH);
      break;
   
    case 0:       //stopping
      analogWrite(pinPwm, 0);
      digitalWrite(pinDir, HIGH);
      break;

    case -1:      //retraction
      analogWrite(pinPwm, Speed);
      digitalWrite(pinDir, LOW);
      break;
  }
}
