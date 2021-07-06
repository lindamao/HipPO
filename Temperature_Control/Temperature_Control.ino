//Thermistor temperature values
int ThermistorPin = 0;
int Vo;
float R1 = 10000; //Change depending on resistance of thermistor
float logR2, R2, T;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

//Pin defines
int PWM_pin = 3; //output PWM pin to vary DC voltage

//Temperature control buttons pin defines (temporary)
int low_button = 0;
int med_button = 1;
int high_button = 2;

//PID Variables
float temperature;
float temperature_set; //changes depending on mode
float error;
float prev_error = 0;
float elapsedTime, currTime;
float prevTime = 0;
float cumError, rateError;
int PID_value;

int PID_p;
int PID_i;
int PID_d;

//PID constants
// We will need to tune the kp/ki/kd values once we test
int kp = 2;
int ki = 0.1;
int kd = 0.1;

//Constants
int low_status = 0;
int med_status = 0;
int high_status = 0;

float low_mode = 44.4;
float medium_mode = 51.71;
float high_mode = 58.52;

void setup() {
  //Set up PWM output pin (for heating element)
  pinMode(PWM_pin, OUTPUT);

  //Set up digital I/O input pin for buttons
  pinMode(low_button, INPUT);
  pinMode(medium_button, INPUT);
  pinMode(high_button, INPUT);
  
  Serial.begin(9600); //For reading temperature in Serial Monitor
  currTime = millis();
}

float getTemp() {
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  T = T - 273.15;

  Serial.print("Temperature: "); 
  Serial.print(T);
  Serial.println(" C"); 

  return T;
}

float PID_control(float temperature, float temperature_set) {
  //Calculate time
  currTime = millis();
  elapsedTime = currTime - prevTime;

  //Calculate error
  error = temperature_set - temperature;

  cumError += error * elapsedTime; //integral
  rateError = (error - prev_error)/elapsedTime; //derivative

  //Calculate P,I,D values
  PID_p = kp * error;
  PID_i = ki * cumError;
  PID_d = kd * rateError;

  //Total PID value
  PID_value = PID_p + PID_i + PID_d;

  prevTime = currTime;
  prev_error = error; 

  return  PID_value;
}

void pwm_output(float PID_value) {
  // PWM output range is defined between 0 and 255
  if (PID_value < 0) {
    PID_value = 0;
  }
  else if (PID_value > 255) {
    PID_value = 255;
  }
  
  Serial.print("PID value: "); 
  Serial.print(PID_value);
  
  analogWrite(PWM_pin, 255-PID_value);
}

void loop() {
  // Get mode
  low_status = digitalRead(low_button);
  med_status = digitalRead(med_button);
  high_status = digitalRead(high_button);
  
  if (low_status == HIGH) {
    temperature_set = low_mode;
  }
  else if (med_status == HIGH) {
    temperature_set = medium_mode;
  }
  else if (high_status == HIGH) {
    temperature_set = high_mode;
  }
  
  temperature = getTemp();
  PID_value = PID_control(temperature, temperature_set);

  pwm_output(PID_value);
  delay(500);
}
