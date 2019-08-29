/*
  Analog input, analog output, serial output

 Reads an analog input pin, maps the result to a range from 0 to 255
 and uses the result to set the pulsewidth modulation (PWM) of an output pin.
 Also prints the results to the serial monitor.

 The circuit:
 * potentiometer connected to analog pin 0.
   Center pin of the potentiometer goes to the analog pin.
   side pins of the potentiometer go to +5V and ground
 * LED connected from digital pin 9 to ground

 created 29 Dec. 2008
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */

// These constants won't change.  They're used to give names
// to the pins used:

#define countNumber = 1000



const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int analogInPin_ref = A1;  // Analog input pin that the potentiometer is attached to

int sensorValue = 0;        // value read from the pot


void setup() {
  // initialize serial communications at 9600 bps:
  pinMode(13, OUTPUT);
  //pinMode(4, OUTPUT);
  Serial.begin(115200);
}

void loop() {
        unsigned int ADCValue, ADCValue1;
        double Voltage, Voltage_ref;
        double Vcc;

        int i;
        
        Vcc = readVcc()/1000.0;
        ADCValue = analogRead(analogInPin);
        ADCValue1 = analogRead(analogInPin_ref);
       // Voltage = (ADCValue / 1023.0) * Vcc;
       // Voltage_ref = (ADCValue1 / 1023.0) * Vcc;

       Voltage = ADCValue;
       Voltage_ref = ADCValue1;


        Serial.print("Value :[");
        Serial.print(Voltage);
        Serial.print("] [");
        Serial.print(Voltage_ref);
        Serial.println("]");

        if(Voltage > 2.8) {
          digitalWrite(13, HIGH);
          delay(50);
          
        Serial.print("Value :[");
        Serial.print(Voltage);
        Serial.print("] [");
        Serial.print(Voltage_ref);
        Serial.println("]");
        
          digitalWrite(13, LOW);
        }


        delay(100);

}



long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
