/*
  24GHz Heartbeat monitoring sensor version 2
  Just only toggle HIGH when detected by Lidar Sensor throug A0 analog PIN
  After delay time just made LOW 

  Created by Arnold Kim (arnold@fvn.co.kr)
  Created  2018. 01. 04. 00:47 
  Modified 2018. 01. 09. 16:31  //Logic changed. Looping and toggle count
  Modified 2018. 04. 21. 19.12  // Output added for voltage value analog writing
  Modified 2018. 05. 28. 16:57  // Existing sensor version. ON logic and OFF duration logic changed. 
                                // 5초간 tresHold 이하 이면 제어핀 LOW(OFF), [[1]] 로직
                                // 최초 tresHold 이상 값이 detect 되고나면, 5초 내에 추가 값이 없으면 OFF / 한번이라도 들어오면 ON  [[2]] 로직
                                // V2로 버젼 독립시킴. 

 */

#define thresHold    3.0    // thresHold power level
#define sysDelayTime 100     // Looping delay time (100 : 0.1 Sec)

//PIN definition
const int analogInPin     =  A3;  // Analog input pin that the potentiometer is attached to
const int digitalControl  =  A0;  // Analog output to backend system
const int ledPin          =  A1;  // led indicator pin
const int outputVoltage   =  A4;  // Analog write reading voltage
//const int ledPin          =  13;  // Digital led pin (usual arduino board connected to LED on board)

//Variables for Logic
int sensorValue = 0;        // value read from the pot
int nowStatus   = 0;        // 0: OFF, 1: ON , 2: ON time (3 초 내에 움직임이나 재실 Detected)
int contCount   = 50;       // sysDelayTime X contCount = continuing count for ON over this count and time when it reached nowStatus toggle to OFF
                            // 20 : 2sec [sysDelayTime : 100]
                            // 50 : 5sec , 5초 동안 다른 신호 없으면 OFF 로 전환. 
int onCount     = 0;
int offCount    = 0;        // off 유지 count & time 이용한 OFF 제어 
int pinStatus   = 0;


void setup() {
  
  // Initiation
  pinMode(ledPin,         OUTPUT);
  pinMode(digitalControl, OUTPUT);
  pinMode(analogInPin,    INPUT);
  pinMode(outputVoltage,  OUTPUT);
  pinMode(ledPin,         OUTPUT);
  
  // initialize serial communications at 115200 bps:
  Serial.begin(57600);

  Serial.println("---------------------START");
}


void loop() {
  
        unsigned int  ADCValue;
        double        Voltage;
        double        Vcc;
        
        Vcc = readVcc()/1000.0;
        ADCValue = analogRead(analogInPin);
        Voltage = (ADCValue / 1023.0) * Vcc;

        // Monitoring Reading voltage from ADC3
        //Serial.print(ADCValue/500.0);
        //Serial.print(",");
        Serial.print(Voltage);
        Serial.print(",");

        // Just write ADC value to ouputVoltage port.(2018.04.21)
        analogWrite(outputVoltage, ADCValue);

        if(Voltage >= thresHold) {
              // Over thresHold 
              // Check first is it ON ? or OFF
              if(nowStatus == 0){
                  // It's OFF now. 
                  // Wait during this status 3 sec continued. 
                  offCount ++;
                  if(offCount > contCount){
                      //Turn ON / CHANGE to HIGH and reset whole the values.
                      nowStatus = 1;
                      offCount = 0;                
                      digitalWrite(digitalControl, HIGH);    
                  }                
                  pinStatus = 5;                                      
              } else {
                pinStatus = 5;
                // 특정 시간 이상으로 ON 유지되면 onCount reset.
                onCount ++;
                if(onCount > contCount) onCount = 0;
              }
              Serial.println(pinStatus);   

        } else {
              // Low thresHold
              if(nowStatus == 1){
                  // Detected status, Low thresHold
                  onCount++;
                  if(onCount > contCount) {
                          // Toggle mode 0 to 1;
                          nowStatus = 0;
                          onCount = 0;
                          
                          digitalWrite(digitalControl, LOW);
                          pinStatus = 0;
                          Serial.println(pinStatus); 
                  }else{
                    
                    Serial.println(pinStatus);
                  }
              }else{

                  // 특정시간 이상으로 OFF 유지되면 카운트 Reset. 
                  offCount++;
                  if(offCount > contCount) offCount = 0;
                  Serial.println(pinStatus); 
                
              }

        }        
    delay(sysDelayTime);
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

