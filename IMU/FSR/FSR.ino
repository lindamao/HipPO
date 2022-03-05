/* FSR testing sketch. 
 
Connect one end of FSR to power, the other end to Analog 0.
Then connect one end of a 10K resistor from Analog 0 to ground 
 
For more information see www.ladyada.net/learn/sensors/fsr.html */
// https://datasheet.lcsc.com/lcsc/1912111437_LEANSTAR-MD30-60-20kg_C406745.pdf

int fsrPin = 0;     // the FSR and 10K pulldown are connected to a0
int fsrReading;     // the analog reading from the FSR resistor divider
int fsrVoltage;     // the analog reading converted to voltage
unsigned long fsrResistance;  // The voltage converted to resistance, can be very big so make "long"
unsigned long extra_buffer_R;
unsigned long fsrConductance; 
long fsrForce;       // Finally, the resistance converted to force
float fsrForce_kg;

void setup(void) {
  Serial.begin(9600);   // We'll send debugging information via the Serial monitor
}
 
void loop(void) {
  fsrReading = analogRead(fsrPin);  
  Serial.print("Analog reading = ");
  Serial.println(fsrReading);
 
  // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
  fsrVoltage = map(fsrReading, 0, 1023, 0, 3300);
  Serial.print("Voltage reading in mV = ");
  Serial.println(fsrVoltage);  
 
  if (fsrVoltage == 0) {
    Serial.println("No pressure");
    //Serial.println("0");  
  } else {
    // The voltage = Vcc * R / (R + FSR) where R = 10K and Vcc = 5V
    // so FSR = ((Vcc - V) * R) / V        yay math!
    fsrResistance = 3300 - fsrVoltage;     // fsrVoltage is in millivolts so 5V = 5000mV
    extra_buffer_R = 5000; // adjust this as needed
    fsrResistance *= 10000 + extra_buffer_R;                // 10K resistor
    fsrResistance /= fsrVoltage;
    Serial.print("FSR resistance in ohms = ");
    Serial.println(fsrResistance);

    if (fsrResistance < 30000) {
      fsrForce_kg = 146591*pow((fsrResistance/1000), -1.15);
      Serial.print("FSR force in kg = ");
      Serial.println(fsrForce_kg/1000);
    } else {
      fsrForce_kg = -38*(fsrResistance/1000) + 4235;
      Serial.print("FSR force in kg = ");
      Serial.println(fsrForce_kg/1000);
    }
  }
  //Serial.println("--------------------");
  delay(1000);
}
