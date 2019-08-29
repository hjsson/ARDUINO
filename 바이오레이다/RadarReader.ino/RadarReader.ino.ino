
//////////////////////////////////////////////////////////////////////////////////////////////////////
// Radar module Reader V0.9
// Author : Arnold Kim (arnold@fvn.co.kr)
// Created : 2015. 11. 26. First coding.
// Modified : 2017.02.15  Comparison with heartbeat sensor (IR) and RADAR display. 
//                        Heartbeat counting logic applied.
// Revision : Without LCD just heartbeat display via Serial. 2017.01.25
// Revision : 전시회 출품용으로 추이 그래프를 제외한 Ploter data disable version. only disable in section "DEBUG ON"
//            
//          Modified 2019. 03. 18. 18:05  // DEBUGH ON 상태에서 심박수 Serial out 적용 (초당 1회 ~ 2회 적용예정)
//          Modified 2019. 03. 28. 09:57  // 특정 thresHold 이하이면 A0 digital pin OFF. default OFF
//                   2019. 04. 03.        // delay time settings added. 
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <SoftwareSerial.h>

#define radarPin        A3     //Anallog Pin defininition
#define heartPin        A4     //Heart beat sensor reading pin
#define controlPin      A0     //Digital OUT pin for control (HIGH/LOW). 2019.03.28
#define ledPin          A1     //installed LED pin.
#define thresHold       3.15    //ON thresHold ( max 3.3) by anold 3.28
#define BUFFER_SIZE     1024
#define opDelaytime     1500    //ms. 1500: 1.5sec delay time. 

#define HBweight        0.9   // old data
#define HBREFRESH       100   // Count reset
#define HBHIGH          1000  // 60bpm
#define HBLOW           600   // 100bpm
#define BTSERIAL        0     //ON:1 OFF:0
#define FASTADC         1     //ON:1 OFF:0
#define DEBUG_ON        1     //ON:1 OFF:0
#define DEBUG_TIME      0     //ON:1 OFF:0
#define DEBUG_PULSE     0     //ON:1 OFF:0 
#define NUM_AVERAGE     16    //Count for Average 8
#define NUM_AVERAGE2    25    //Sliding Average  .28
#define PULSE_COUNT     14    //Pulse Array count.for check same value count. ** .. 11--> 14 (2016.09.08)
#define LED_OFF_COUNT   200
#define PULSE_OFF_COUNT 50    //Count for PULSE check disable. origin : 30, down it cause not changed to 20.


int       sensorValue = 0;
float     voltage     = 0.0;


// Heart Beat detection related various.
float preValue  = 0;
float vArray[NUM_AVERAGE];
float sArray[NUM_AVERAGE2];
float pArray[PULSE_COUNT];
  int averageCNT  = 0;
  int saverageCNT = 0;
  int paverageCNT = 0;
  int totAverage  = 0;
float tmpSum      = 0;
  int pulseCount  = 0;
float pAverage    = 0;
  int ledCNT      = 0;

  int heartBEAT   = 0;
  int tmpBEAT     = 0;
   
 bool isLEDON     =  true; 
 bool isHIGH      =  false;

float prePulseValue   = 0.0;
  int forthefirst     = 0;

  int dispBEAT        = 857;
 long riseTime        = 0;
 long dropTime        = 0;
 long intervalTime    = 0;
 long deltaTime       = 0;
  int timeIndex       = 0;
 long timeArray[3][3] = {(0,0,0),(0,0,0)};
  int hbCount         = 0;
  int hearBEAT        = 0;
  int isEMPTY         = 0;
  int noBODY          = 0;

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//Bluetooth Serial PIN define. RX : 4, TX : 5
SoftwareSerial btSerial(4,9);


//LED related program variables

int nowStatus   = 0;        // 0: OFF, 1: ON
int contCount   = 3000;     //2019-05-13 원본 : 500; // sysDelayTime X contCount = continuing count for ON over this count and time when it reached nowStatus toggle to OFF
                            // 20 : 2sec [sysDelayTime : 100]
                            // 400: 2sec [sysDelayTime : 5]
                            // 20000 : 10 sec
                            // 2000  : 1sec
int onCount     = 0;
int pinStatus   = 0;


void setup() {

    #if FASTADC   // set prescale to 16     
      sbi(ADCSRA,ADPS2) ;
      cbi(ADCSRA,ADPS1) ;
      cbi(ADCSRA,ADPS0) ;
    #endif

    // put your setup code here, to run once:
    Serial.begin(115200);
    btSerial.begin(9600);

    for (int tmpCount = 0; tmpCount < NUM_AVERAGE; tmpCount ++) {
          vArray[tmpCount] = 0;
    }
    //Digital control pin setup & initiation (2019.03.28)
    pinMode(controlPin, OUTPUT);
    digitalWrite(controlPin, LOW);
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(100);
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(100);

    Serial.println("CIS4U program start !!!");
    delay(100);
     Serial.println("CIS4U program start !!!");
         delay(100);
      Serial.println("CIS4U program start !!!");
          delay(100);
       Serial.println("CIS4U program start !!!");
           delay(100);
        Serial.println("CIS4U program start !!!");
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


int countBEAT(int count) {
    float countPerSec = 0.0;
    float countPerMin = 0.0;

    countPerSec = count/30.0;
    countPerMin = 60.0/countPerSec;
   
    return int(countPerMin);

}

int calculBEAT() {

    return int(60000/dispBEAT);
}


void ledControl(double signLevel) {
  
        unsigned int  ADCValue;
        double        Voltage;

        Voltage = signLevel;
        
        // Monitoring Reading voltage from ADC3
        //Serial.print(ADCValue/500.0);
        //Serial.print(",");
        //Serial.print(Voltage);
        //Serial.print(",");

        if(Voltage >= thresHold) {
              // Over thresHold
              if(nowStatus == 0){
                  // For the first time or ON start point. Toggle 0 to 1.
                  nowStatus = 1;
                  digitalWrite(controlPin, HIGH);
                  digitalWrite(ledPin, HIGH);

                  //delay(opDelaytime); // 2019.04.03. added. del by ks.
                  pinStatus = 5;                                      
              } else {
                pinStatus = 5;
                onCount = 0;
              }
              //Serial.print(",");
              //Serial.println(pinStatus);         
        } else {
              // Low thresHold

              if(nowStatus == 1){
                  // Detected status, Low thresHold
                  onCount++;
              }

        } 
        
        if(onCount > contCount) {
          nowStatus = 0;
          onCount = 0;
          
          digitalWrite(controlPin, LOW);
          digitalWrite(ledPin, LOW);
          pinStatus = 0;

          delay(opDelaytime); // 2019.04.03. added.
        }
        //Serial.print(onCount);  
        //Serial.print(",");  

}

void loop() {

    unsigned int  ADCValue;
          double  Voltage;
          double  Vcc;
           float  average    = 0;
           float  sAverage   = 0;
           float  nowValue   = 0;
           float  nowValueH  = 0;
             int  cntHIGH    = 0;
             int  downCNT    = 0;

             
    //ANALOG READ [Reference voltage 1.0v]  , RADAR SENSOR
    Vcc = readVcc()/1000.0;
    //DEBUG
    //ADCValue = analogRead(heartPin);
    ADCValue = analogRead(radarPin);
    Voltage = (ADCValue / 1023.0) * Vcc;

    //Calculation average 
    nowValue = Voltage;

    ledControl(Voltage);

    //vArray processing [values Average]
    vArray[averageCNT] = nowValue; 
    for (int tmpCount = 0; tmpCount < NUM_AVERAGE; tmpCount ++) {
        average += vArray[tmpCount];
    }
    average /= NUM_AVERAGE;
    averageCNT++;

    if((averageCNT - (NUM_AVERAGE-1)) >= 0) averageCNT = 0;     //Average count reset

    //ANALOG READ [Reference voltage 1.0v] , HEARTBEAT SENSOR
    Vcc = readVcc()/1000.0;
    ADCValue = analogRead(heartPin);
    Voltage = (ADCValue / 1023.0) * Vcc;

    //Calculation average 
    nowValueH = Voltage;

 
    #if DEBUG_ON
      // 2018.11.13 전시회용으로 1st value disable. 
      //Serial.print(average);
      //Serial.println(average);
    #endif

    //IDLE check logic during idle time it should display 0
   
    if(average > 1.9) isEMPTY++;
    else isEMPTY = 0;

    if(isEMPTY > 1000) {
        //Mode Change to 0
        noBODY = 1;
        dispBEAT = 0;
        
        #if DEBUG_TIME
        Serial.println("mode changed... to idle");
        //delay(1000);
        #endif

        for (int i=0; i<3 ; i++) {
          timeArray[i][0] = 0;
        }
        Serial.println("");
        
        for (int i=0; i<3 ; i++) {
          timeArray[1][i] = 0;
        }
        
    }
    else if (noBODY == 1) {
        //Mode Change from noBODY --> oneBODY
        noBODY = 0;
        dispBEAT = 857;
        
        #if DEBUG_TIME
        Serial.println("mode changed... to active");
        delay(1000);
        #endif
    }

/**
     #if DEBUG_TIME
      Serial.print(average);
      Serial.print(" ");
      Serial.print(isEMPTY);
      Serial.print(" ");
      Serial.println(noBODY);
    #endif
**/
    //sArray processing. [Sliding Average]
    sArray[saverageCNT] = nowValue;
    
    for (int stmpCount = 0; stmpCount < NUM_AVERAGE2; stmpCount ++) {
        sAverage += sArray[stmpCount];
    }

    sAverage /= NUM_AVERAGE2;
    saverageCNT++;
    
    if((saverageCNT - (NUM_AVERAGE2-1)) >= 0) saverageCNT = 0;
    
    #if DEBUG_ON
    //Serial.print(" "); //2018.11.13일 위 1st value disable 관련 
    //Serial.print(sAverage*3+6);
    Serial.print(sAverage);
    //Serial.print(",");
    //Serial.print(nowValueH);
    //Serial.print(heartBEAT);
    //Serial.print(" ");
    //Serial.print(nowValueH);  //HeartBeat sensor Value
    #endif

    //HEART BEAT COUNT DISTINGUISH HIGH PHASE and LOW PHASE

    if((sAverage - average) <= 0) {
      //HIGH case
      #if DEBUG_ON
      // 2018.11.13 전시회용 추이 그래프만 보여줌 
      //Serial.print(" ");
      //Serial.print(3.7);
      #endif

      if(isHIGH == false) {
        //pulse count print.                   
        // TOTAL count value making paverage. 
        // 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25 . 26 . 27. 28. Array's room.
        // 0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10 . 11.  12. 13. Address of ROOM.
        
        if(cntHIGH < 25  and cntHIGH > 15 ) {
            pArray[cntHIGH - 15]++;
        }
        cntHIGH = 0;      // From LOW comes HIGH point. Starging point
        
        isHIGH = true;   

        //rising HIGH time marking. rising time settings.
        riseTime = millis();
        
      } else cntHIGH++;
    } else {
      //LOW case
      #if DEBUG_ON
      // 2018.11.13 전시회용 추이 그래프만 보여줌
      //Serial.print(" ");
      //Serial.print(-1);
      #endif
      
      if(isHIGH == true) {
              isHIGH = false;

              //dropping Time. dropTime setting and calcualte interval time.
              
              intervalTime = millis() - riseTime;
              
              riseTime = 0;
                      
              if(intervalTime > 100) {
                  //Write array this value
                  timeArray[(timeIndex%3)][0] = intervalTime;
                  timeArray[1][(timeIndex%3)] = millis() - deltaTime;
    
                  //Set now to deltaTime.
     
                  deltaTime = millis();
                  
                  #if DEBUG_TIME
                  //Serial.println(timeIndex%3);
                  //Serial.println(intervalTime);
                  #endif
                  
                  timeIndex++;
              }
              
              if(timeIndex > 2){

                  #if DEBUG_TIME
                      //Serial.println(dispBEAT);
                      Serial.println("----------------------3 HIGH display ");
                      Serial.println("");
                      for (int i=0; i<3 ; i++) {
                        Serial.print(timeArray[i][0]);
                        Serial.print(" ");
                      }
                      Serial.println("");
                      
                      for (int i=0; i<3 ; i++) {
                        Serial.print(timeArray[1][i]);
                        Serial.print(" ");
                      }
                      Serial.println("");
                  #endif

                  
                  /**  
                  if (abs(timeArray[0][0] - timeArray[1][0]) < 120 )
                            if (timeArray[1][1] >=  HBLOW && timeArray[1][1] <= HBHIGH ) dispBEAT = (dispBEAT + timeArray[1][1])/2;
                  
                  else if (abs(timeArray[1][0] - timeArray[2][0]) < 120 )
                            if (timeArray[1][2] >=  HBLOW && timeArray[1][2] <= HBHIGH ) dispBEAT = (dispBEAT + timeArray[1][2])/2;
                  **/

                  if (timeArray[1][1] >=  HBLOW && timeArray[1][1] <= HBHIGH ) {
 
                        dispBEAT = (dispBEAT*HBweight + timeArray[1][1]*(1-HBweight));
                  } else 
                  
                   #if DEBUG_TIME 
                      Serial.print(dispBEAT);
                      Serial.print(" ");
                  #endif
                  
                  if (timeArray[1][2] >=  HBLOW && timeArray[1][2] <= HBHIGH ) {
                       
                        dispBEAT = (dispBEAT*HBweight + timeArray[1][2]*(1-HBweight));
                  } else 

                  #if DEBUG_TIME
                      Serial.print(dispBEAT);
                      Serial.print(" ");
                  #endif

                  //Caculation BEAT and Init;
                  heartBEAT = calculBEAT();

                  #if DEBUG_TIME
                      Serial.println(heartBEAT);
                  #endif
                  
                  //Init all values
                  timeIndex = 0;
                  dropTime = 0;
              }

              
      }
     
      cntHIGH++;
    }


    #if BTSERIAL
      btSerial.print(average);
      //btSerial.print(3.2);
      btSerial.print(":");
      btSerial.print(nowValueH);
      //btSerial.print(2.5);
      btSerial.print(":");
      btSerial.print(heartBEAT);
      btSerial.print("\r\n");
    #endif

    #if DEBUG_ON
    // 2018.11.13 전시회용 추이 그래프만 보여줌
    //Serial.print(" ");
    //Serial.print(pAverage);
    //Serial.print(" ");
    //Serial.print(cntHIGH);
    #endif




   if(ledCNT > PULSE_OFF_COUNT) {
          // Whole the PULSE value = 0. It's not valid values. JUST only NOISE.
          //tmpBEAT = countBEAT(0);
         // tmpBEAT = 0;
    } else {
          // Update values. 
          #if DEBUG_PULSE
              for(int i=0; i< PULSE_COUNT; i++) {
                Serial.print (pArray[i]);
                Serial.print(",");
              }
              Serial.print(" ");
          #endif
          int totalSUM = 0;
          int countSUM = 0;
          for(int i=0; i< PULSE_COUNT; i++) {
            countSUM += pArray[i];
            pArray[i] = pArray[i] * (15+i);
            totalSUM += pArray[i];
          }
          float pulseValue = totalSUM / countSUM;
          #if DEBUG_PULSE
            Serial.println(pulseValue);
          #endif
          if(countSUM >= 3) {
              
              if(prePulseValue != 0) {
                  float finaValue = (prePulseValue + pulseValue) / 2.0;
                  tmpBEAT = countBEAT(finaValue);
                
                  #if DEBUG_PULSE
                  Serial.print(",");
                  Serial.print(prePulseValue);
                  Serial.print(",");
                  Serial.print(pulseValue);
                  Serial.print(",");
                  Serial.print(finaValue);
                  Serial.println("");

                  #endif
    
                  prePulseValue = finaValue;
              
              }else{
    
                  if(forthefirst != 0){
                  //float pulseValue = totalSUM / countSUM;
                  tmpBEAT = countBEAT(pulseValue); 
                  prePulseValue = pulseValue;
                  }else forthefirst++;
              }
          }
    }
 

    for (int count=0; count< PULSE_COUNT; count++){
        pArray[count]=0;
    }

    #if DEBUG_ON
      Serial.println("");
    #endif
    
    delay(5);
    
} //loop end.
