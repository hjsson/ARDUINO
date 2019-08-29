        /*  2019-04-02  mrSson ,  First coding. 
         *  UPPERN 확인으로 상판인지 하판인지 구분하여 구동 
         *  Modified DATE. Author. Description............
         *  2019.04.05     Arnold. Arrange document style and hand-over from MrSson. New naming samrtFAN-v10.ino
         *  2019.04.06     Pin change A0 <--> A5 for communication test.
         *  2019.04.13     Change Naming to smartFanV2.ino
         *  2019.04.15     BUGFIX. PulseIn function time out setting. SIGNAL gabage filtering function and some bugfix aaded.  
         *  2019.04.17     FAN status detection logic changed.
         *  2019.04.19     * FAIL LIST & MODIFY LIST
         *                 - [**1] 상판 하판 레벨 확인. 왔다 갔다 함 [OK]
         *                 - [**2] PIN FAULT 신호가 HIGH 유지여야 하는데, 펄스 신호처럼 나옴. // 전역에서 핀에 대한 상태 모니터링으로 처리. fanFAULT 변수를 참조하게 함. []
         *                 - [**3] 자체 온도에 의해 팬이 돌 경우에도 장애 표현이 됨. []
         *  2019.04.21     *V3, V2.1 version (04.20) 폐기.
         *                 - [**4] 작동 온도 제어 기능 . 최초 기동 온도는 20도가 되지 않으면 작동 안함. 작동이 시작되면 10도까지는 작동함.
         *                 - [**5] ANALOG로 핀 읽어서 데이타 처리하는 방식으로 바꿈. 팬이 STOP할 경우 두가지의 상태 나타남. 
         *                         * 500 이상의 긴 PULSE 주기적으로 나타남.(5개 이상이 지속되면) - 알람 울림.
         *                         * 특정 VALUE로 지속적으로 나타남. (50개 이상이면)          - 알람 울림.
         *                         * 900 --> 1000 수정 본.
         * 2019.04.22      *V4 LOGIC 변경. 
         *                 - [**6] ADC --> DIGITAL, PULSE IN reading & count. RPM 측정 및 특정 시간당 HIGH COUNT.
         *                 
         * 2019.04.23      *V5 CALIBRATION VALUE 저장
         *                 - [**7] INIT VALUE 측정 하고 저장 .
         *                 
         * 2019.04.25      *V10 ANALOG BASED SOLUTION APPLIED. SPEED & HALT detection possible. 
         *                 - [**8] PULSE READING and ANALYZING LOGIC applied.. Very SIMPLE !!!!
         *                 
         * 2019.04.27      *V11 TEMPERATURE SHOULD BE APPLIED. 
         *                 - [**9] ALL OK. TEMPERATURE APPLIED. 3PHASE.  LOW TEMP//NORMAL//HIGH TEMP
         *                 - [LOW TEMP] : 최초기동 : 20도 이하에서는 작동 안함. 작동중 : 본 모드로 들어가도 10도 까지는 작동 진행 함.
         *                 - [NORMAL] : 그냥 작동
         *                 - [HIGH TEMP] : 최초 진입 -> 255로 기동 . 진입상태에서 35도 까지 255로 기동.
         * 
         * 2019.04.27      *V12 FINALIZING
         *                 - [OPERATION TEST] completed.
         *                 - [FAN FAULT TEST] completed.
         *                 - [TEMP OPERATION TEST] completed.
         *                 - [SOURCE CLEAN] completed.
         * 
         *                 
        */
        
        #include <PinChangeInt.h>
 
        #define FAST_ADC      1           // FAST ADC    ON:1 OFF:0
        #define sysDelay      20          // System delay time
        #define THERSHOLD_F   410        // Threshold Front FAN NORMAL 410
        #define THERSHOLD_B   200         // Threshold Back  FAN NORMAL, 200
        
        #define countFILTER   30         // FILTER for Release counter.
        #define countFILTER_O 30          // FILTER for FAULT ON count.
        #define countHIGH     100         // count MAX HIGH change period.
        
        #define UPPER         1         // UPPER FAN flag.
        #define LOWER         0         // LOWER FAN flag.
        #define speedFULL     250       // FAN FULL SPEED value.
        #define speedZERO     160         // FAN ZERO SPEED value.
        #define pwmMAX        2000      // PWM max signal value.
        #define fanFAIL       0         // FAN Fail value. 
        //#define MY_PIN      5         // PORT CHANGE INTERRUPT USING PIN we could choose any pin
        
        #define arrayCount    30        // arrayCount. SHOULD pairing NUM_AVERAGE !!!!
        #define NUM_AVERAGE   30        // average count
        #define NUM_TEMP_AVERAGE 100   // number of temperature average CNT
        
        // Temperature related variables. [**4]온도제어가 추가됨.
        #define tempMIN     21        // 21
        #define tempMAX     40        // 40

        // PULSE COUNT define.
        #define readingTIME   100     // 100mm sec duration
        
        
        // Interrupt variables.
        volatile int pwm_value = 0;
        volatile int prev_time = 0;
        uint8_t latest_interrupted_pin;
        
        // Pin mapping and variables.
        int alarmCNT=      0;
        int releaseCNT=    0;
        int debugCount =   0;
        int P_NOW_TEMP =  A7;          //온도센서                      //T_Sen : Input : Temperature Sensor /ADC7  PIN No A7    ---- Device TMP36
        int P_LED_ALM =   A0;          //팬 이상시 알람 LED             //LED_ALM : Output : Alarm  LED Turn On while FAN Fail /PC0 Pin No A0    ---- Normal Low / Fail High(Turn On)
        int P_LED_PWR =   A1;          //전체 파워 LED                 //LED_PWR : Output : Power LED Yrun On while Power On /PC1 Pin No A1     ---- Normal High / Fail Low(Turn Off- No Power)
        int P_F_FAN_RPM = A2;          //상위 팬 속도 확인               //FAN1_Mon : Input : Fan operation Monitor as Frequency Counter /PC2 Pin No A2  --- 2 pulse per one turn
        int P_B_FAN_RPM = A3;          //하위 팬 속도 확인               //FAN2_Mon : Input : Fan operation Monitor as Frequency Counter /PC3 Pin No A3  --- 2 Pulse per one turn
        int P_FAN_FAULT = A4;          //나의 팬 정상여부 전달 포트         //** FAN-Fault : Output : Fan Fail Alarm Output /PC6 Pin No A6 --- Fault High / Normal Low ==> Destination to MAIN monitoring system.  
                                                                     // 2019.04.18. P_FAN_FAULT pin mapping changed. /PC4 Pin No A4. Whole program download again !!! 
        int P_UPPERN =    A5;          //현재 상판인지 하판인지 구분 플레그   //UPPERN : Upper Tag : Upper status Check /PC5 Pin No A5  -----  Low is Upper / High is Lower 
                                                                                                                                    // HIGH is Upper / LOW is Lower . changed. 2019.04.19 FINAL !!!! 
        int P_F_FAN_PWM =  3;          //상위 팬 속도 주기               //PWM_FAN1 : Output : Fan#1 Speed Control PWM Signal /PD3 Pin No 3 --- 40% at 25C   100% at 31 / when No signal from other Unit, 100%
        int P_B_FAN_PWM =  5;          //하위 팬 속도 주기               //PWM_FAN2 : Output : Fan#1 Speed Control PWM Signal /PD5 Pin No 5 Same as FAN1
        
        int P_CONN_SIGNAL_OUT = 9;     //연결된 판과의 데이터 전송          //SENSE : Output & Input : Output PWN Signal when Upper Chhecked: Input Monitor when Lower checked / PB0(Output)Pin No 8  
        int P_CONN_SIGNAL_IN =  9;     //연결된 판과의 데이터 수신          //SENSE : Output & Input : Output PWN Signal when Upper Chhecked: Input Monitor when Lower checked / PB1(Input)Pin No 9
        boolean UP_DOWN =     false;       //false=하판, true=상판
        boolean fanFAULT =    false;      //false=NORMAL true=FAULT
        int tempOffset =  20;
        int fanSPEED=      0;
        int fanSPEEDCNT =  0;
        
        // fanFAULT 고도화 [**3], 자체 signal 신호가(MAX) 있으면서 알람 모드일 경우 3회 까지는 Reset. 연속 4회일 경우는 바로 알람 !!!
        int maxFanFaultCNT=   0;
        
        // [**4]온도관련 전역변수 : * 온도값을 return 형태는 작동을 위한 온도만 전달. 최저는 21도. 실제온도 관련 로직에서는 [**4]로직을 따름. 별도 관리용 변수.
        //                     * 온도값에 따라 20도 이상이면 작동 시작. 이때 tempOperation이 true가 됨. false가 되는 시점은 10도 이하로 내려가면.
        // [**9]온도 전체 관리를 위한 변수 적용. LOW, NORMAL, HIGH 모드로 적용.
        float nowTemperature =    0.0;
        boolean tempOperation = false;  //온도 작동에 대한 전체 작동 통제 변수. true : 무조건 작동.  false : 무조건 스톱.
        boolean tempHIGH      = false;  // 40도 이상
        boolean tempLOW       = false;  // 20도 이하
        boolean tempNORMAL    = false;  // 정상 온도

        // 온도 테스트를 위한 변수 정리
//        int  testTMPCNT     = 0;
//        int  testCNT        = 0;
        
        
        // PULSE Check variables.
        long startTimeF     = 0;  // RISING TIME F
        long endTimeF       = 0;  // FALLING TIME F
        long startTimeB     = 0;  // RISING TIME B
        long endTimeB       = 0;  // FALLING TIME B
        long durationTimeF  = 0;  // RISE - FALLING simple calculation.
        long durationTimeB  = 0;  // RISE - FALLING simple calculation.
        long durationError  = 0;  // Continued Error Time. specific time reached from Error occurred check there's Error continuing or NOT. (10sec / 10,000 milisec)
        boolean     isHIGHF = false;
        boolean     isHIGHB = false;
        int highCountF      = 0;  // 500 이상값을 count.
        int highCountB      = 0;  // 500 이상값을 count.
        long preValueF      = 0;
        int  sameCountF     = 0;
        long preValueB      = 0;
        int  sameCountB     = 0;
        int  continueFAULT  = 0;
        int  preContinueFAULT = 0;
        int  noFaultCNT       = 0;
        int  confirmError     = 0;      // Spot성 에러는 제거. 
        int  reportALAM       = 0;      // 상위 REPORT 알람은 더 신중하게
        boolean frontFAN       = true ;  // false : BACK true : FRONT
        int  frontFANCNT      = 0;
        int  lowerZeroCNT     = 0;      // Lower 0 count 3 이상이어야 0 detection으로 간주.
        boolean uppderCMDZero = false;

        //[**7] 최초 기동시에 PULSE값을 측정해서 관리.
        //      해당 값으로 장애 처리 및 관리.
        boolean       isFirstTime         = true;

        // VAULES
        unsigned int  rpmMINvalueF        = 0;
        unsigned int  rpmMAXvalueF        = 0;
        unsigned int  rpmMINvalueB        = 0;
        unsigned int  rpmMAXvalueB        = 0;

        // THRESHOLD VALUE
        unsigned int  rpmMINthresF        = 0;
        unsigned int  rpmMAXthresF        = 0;
        unsigned int  rpmMINthresB        = 0;
        unsigned int  rpmMAXthresB        = 0;
        
        int           errorContinueCNT    = 0;      // 3회 연속으로 threshold 이하 또는 이상의 값이 나오면,,, 장애 알림.
        int           errorReleaseCNT     = 0;      // 3회 연속으로 normal 상태이면 장애 해제.
        int           verifiedSegnal      = 0;      // 이전값 기억
        int           signalCNT           = 0;      // signal 3회 이상 0 값이 들어와야 
        
        
        // 통계처리 변수
        int             averageCNT        = 0;      // STATISTICS COUNT
        int             averageTCNT       = 0;      // STATISTICS COUNT FOR TEMPERATURE
        unsigned long   fFArray[arrayCount];        // Array for FRONT FAN
        unsigned long   fBArray[arrayCount];        // Array for BACK FAN
        float           fTArray[NUM_TEMP_AVERAGE];  // Array for TEMPERATURE, 따로 구성 해야 함 50개 sampling은 너무 많아 (10개 정도로
        
        // Direct ADC 
        unsigned long readSIGNAL;       // reading value MAIN pulse value. 


        // V4 TIMER 관련
        boolean           readPulse       = false;
        long              readPulseTime   = 0.0;
        unsigned long     pulseCNTF       = 0;
        unsigned long     pulseCNTB       = 0;
             
        
        // defines for setting and clearing register bits
        #ifndef cbi
            #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
        #endif
        #ifndef sbi
            #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
        #endif
        
        void setup(){
          
                  //Serial start !! for the first time. 
                  Serial.begin(115200);
        
                  Serial.println("smartFAN controller V12.00....... ");
                  Serial.println("            Created by FVN, Corp.");
                  delay(100);
        
                  #if FAST_ADC   // set prescale to 16     
                    sbi(ADCSRA,ADPS2) ;
                    cbi(ADCSRA,ADPS1) ;
                    cbi(ADCSRA,ADPS0) ;
                  #endif
            
                  
                  //PIN DEFINITION
                  //OUTPUT
                  pinMode(P_LED_ALM,          OUTPUT); 
                  pinMode(P_LED_PWR,          OUTPUT);  
                  pinMode(P_FAN_FAULT,        OUTPUT); 
                  pinMode(P_F_FAN_PWM,        OUTPUT); 
                  pinMode(P_B_FAN_PWM,        OUTPUT); 
                  pinMode(P_CONN_SIGNAL_OUT,  OUTPUT); 
                  
                  //INPUT          
                  pinMode(P_NOW_TEMP,         INPUT); 
                  pinMode(P_CONN_SIGNAL_IN,   INPUT); 
                  pinMode(P_F_FAN_RPM,        INPUT); 
                  digitalWrite(P_F_FAN_RPM,     LOW);
                  pinMode(P_B_FAN_RPM,        INPUT);
                  digitalWrite(P_B_FAN_RPM,     LOW);
                  pinMode(P_UPPERN,           INPUT);
                  digitalWrite(P_UPPERN,       HIGH);    // Pullup settings.
        
                  //INITIATION
                  UP_DOWN = false;
                  fanFAULT = false;
                  digitalWrite(P_FAN_FAULT,     LOW);
                  digitalWrite(P_LED_ALM,       LOW);

                  delay(300);
        
                  // SCENARIO 1 : For the first time BOOTING signal. POWER LED ON
                  // SCENARIO 2 : When it started.Two fan which installed on this should operate full speed (speedFULL) 
        
                  // [**1] UPPER / LOWER STATUS definition changed. FINAL and confirmed.
                  if(digitalRead(P_UPPERN) == HIGH){ //상판에 조립된 경우

                        UP_DOWN = true;
        
                        //SCENARIO 1 (POWER LED ON)
                        digitalWrite(P_LED_PWR,   HIGH);
        
                        //SCENARIO 2 (FAN OPERATION FULL SPEED during 3 SEC)
                        analogWrite(P_F_FAN_PWM,  255);
                        delay(1500);
                        analogWrite(P_B_FAN_PWM,  100); 
                        
                  }else{
                        //SCENARIO 1 (POWER LED blink twice)
                        digitalWrite(P_LED_PWR,   HIGH);
                        delay(500);   
                        digitalWrite(P_LED_PWR,   LOW);
                        delay(500);   
                        digitalWrite(P_LED_PWR,   HIGH);
        
                        //SCENARIO 2 (FAN OPERATION FULL SPEED during 3 SEC)
                        delay(1000);
                        analogWrite(P_F_FAN_PWM,  255);
                        delay(1500);
                        analogWrite(P_B_FAN_PWM,  100); 
                       
                  }
        
                  delay(500); 
                  
                  #if DEBUG
                      Serial.println("SETUP FINISHED......");
                      delay(500);
                      Serial.println("NOW IT STARTED !!!!!");
                  #endif
        
                  //INTERRUPT START
                  #if INTERRUPT
                      pinMode(MY_PIN, INPUT); digitalWrite(MY_PIN, HIGH);
                      Serial.begin(115200);
                      PCintPort::attachInterrupt(MY_PIN, &rising, RISING);
                  #endif

                  // INITIATION for TIMER
                  readPulseTime = millis();
                
        }

        
        
        void loop(){
        
                  float           nowTemp;
                  float           averageFront  = 0;
                  float           averageBack   = 0;
                  unsigned int    ADCValueF;
                  unsigned int    ADCValueB;   
                  double          VoltageF;
                  double          VoltageB;
                  double          Vcc;
                  float           averageTemp;

                  // FINAL
                  //FRONT FAN
                  unsigned long   timeDeltaF    = 0;
                  boolean         isHIGHF       = false;
                  int             highCNTF      = 0;
                  unsigned long   inPulsCNTF    = 0;
                  
                  //BACK FAN
                  unsigned long   timeDeltaB    = 0;
                  boolean         isHIGHB       = false;
                  int             highCNTB      = 0;
                  unsigned long   inPulsCNTB    = 0;
        
                 
        
                  // INIT
                  readSIGNAL= 0.0;

                  // PULSE READING & PROCESSING SEPERATION

                  pulseCNTF = 0;
                  pulseCNTB = 0;
                  readPulseTime = millis();

                  // FAN CALIBRATION & SETTING VALUE REGISTER
                  // JUST 1 time EXECUTE.
                  if(isFirstTime == true) {
                         // MAX VALUE CALCULATION
                         Serial.println(" For the First TIME !!! =============");

                         // PRODUCTION CODE , 양산을 위하여 온도값을 먼저 확인 해야 함.
                         Serial.println(readNowTemp());
                         delay(500);
                         Serial.println(readNowTemp());
                         // PRODUCTION CODE END

                         countPulse(100, 5000);  
                         delay(1000);
                         countPulse(255, 7000);

                         //CALIBRATION VALUE PHASE END
                         isFirstTime = false;
                  }
 
                  
                  while((millis() - readPulseTime) < 2000){
   
                        // 특정 시간단위 PULSE HIGH COUNT.
                        ADCValueF   = analogRead(P_F_FAN_RPM); // P_B_FAN_RPM
                        ADCValueB   = analogRead(P_B_FAN_RPM); // P_F_FAN_RPM

                        if(ADCValueF > 100 && ADCValueF ) {
                              if(isHIGHF == true ) {
                                    highCNTF ++;
                                    //Serial.print(30);
                              } else {
                                    isHIGHF = true;
                                    highCNTF ++;
                                    timeDeltaF = micros();
                              }
                              
                        } else {
                              if(isHIGHF == true && highCNTF >= 4) {
                                    // FINAL TIME STAMP
                                    // Value INPUT
                                    inPulsCNTF = micros() - timeDeltaF;

                                    #if DEBUG_PULSE
                                          Serial.print("COUNT : ");
                                          Serial.print(highCNTF);
                                          Serial.print(" TIMESTAMP : ");
                                          Serial.print((micros() - timeDeltaF));  
                                          Serial.println(""); 
                                    #endif
                              } isHIGHF = false; 
                              highCNTF = 0;
                              timeDeltaF = micros();
                        }


                         if(ADCValueB > 100 && ADCValueB ) {
                              if(isHIGHB == true ) {
                                    highCNTB ++;
                                    //Serial.print(30);
                              } else {
                                    isHIGHB = true;
                                    highCNTB ++;
                                    timeDeltaB = micros();
                              }
          
                        } else {
                              if(isHIGHB == true && highCNTB >= 4) {
                                    // FINAL TIME STAMP

                                    inPulsCNTB = micros() - timeDeltaB;

                                    #if DEBUG_PULSE
                                            Serial.print("COUNT : ");
                                            Serial.print(highCNTB);
                                            Serial.print(" TIMESTAMP : ");
                                            Serial.print((micros() - timeDeltaB));  
                                            Serial.println(""); 
                                    #endif 
                              } isHIGHB = false; 
                              highCNTB = 0;
                              timeDeltaB = micros();
                        }


                        // STATISTICS for VALUES.

                        fFArray[averageCNT] = inPulsCNTF;
                        for (int tmpCountF = 0; tmpCountF < NUM_AVERAGE; tmpCountF ++) {
                            averageFront += fFArray[tmpCountF];
                        }
                        averageFront  /= NUM_AVERAGE;

                        fBArray[averageCNT] = inPulsCNTB;
                        for (int tmpCountB = 0; tmpCountB < NUM_AVERAGE; tmpCountB ++) {
                            averageBack += fBArray[tmpCountB];
                        }
                        averageBack  /= NUM_AVERAGE;
              
                        if((averageCNT - (NUM_AVERAGE-1)) >= 0) averageCNT = 0;     //Average count reset
                        else averageCNT++;


                  } // while END.

      
                  // [**9] 하위 모든 로직은 그대로 활용하기 위해 변수에 넣어 줌.
                  pulseCNTF = averageFront;
                  pulseCNTB = averageBack;

                  delay(100);
  
                  // 팬 동작에 대한 상하판 통신 Pulse READ. 
                  // 10000 timeout 무조건 주어야 함.
                  readSIGNAL  =   pulseIn(P_CONN_SIGNAL_IN,HIGH,10000); 

                  // [**4]Read Temperature. Everytime in this loop.
                  //      This return temperature should applied for FAN operation.


                  // READ TEMPERATURE
                  nowTemp =  readNowTemp();


                  
                  //[**4] 온도에 따른 작동 방식을 제어함.
                  //      - 기동시에 온도가 20도 이하일 경우 무조건 작동 금지.
                  //      - 20도 이상이면 [한번이라도] flag 설정. 작동 시작.
                  //      - 작동 시작하면 10도 되는 최초 순간까지 작동. 다시 시작 하려면 최소 한번 20도가 되어야 함.
                  //      - [**9] ALL OK. TEMPERATURE APPLIED. 3PHASE.  LOW TEMP//NORMAL//HIGH TEMP
                  //      - [LOW TEMP] : 최초기동 : 20도 이하에서는 작동 안함. 작동중 : 본 모드로 들어가도 10도 까지는 작동 진행 함.
                  //      - [NORMAL] : 그냥 작동
                  //      - [HIGH TEMP] : 최초 진입 -> 255로 기동 . 진입상태에서 35도 까지 255로 기동.
                  // [**9]온도 전체 관리를 위한 변수 적용. LOW, NORMAL, HIGH 모드로 적용.
         
                  // LOW TEMP CASE CHECK
                  // 최종 확인 할 경우는 아래의 다양한 변수로 확인.
                  // 온도가 20도 이상인 경우 (작동 함)
                  if(nowTemperature >= 20.0) { 
                        tempOperation = true;
                        tempNORMAL    = true;
                        tempLOW       = false;
                  }
                  // 온도가 20도 이하인 경우 (작동 안함)
                  if (nowTemperature  < 20 && tempOperation != true) { 
                        tempOperation = false; 
                        tempNORMAL    = false;
                        tempLOW       = true;
                  }
                  // 온도가 20도 이하인 경우 (작동중일 경우 플래그만 변경 / 작동 함)
                  if (nowTemperature  < 20 && tempOperation == true) { 
                        tempNORMAL    = false;
                        tempLOW       = true;
                  }
                  // 작동하다가 10도 이하로 떨어진 경우. (작동 안함)
                  if (nowTemperature  <= 10.0 && tempOperation == true) {   
                        tempOperation = false;
                        tempNORMAL    = false;
                        tempLOW       = true;
                  }

                  // HIGH TEMP CASE CHECK
                  // 40도 이상이면 최고속도로 돌기 시작
                  if(nowTemperature >= 40) {  
                        tempHIGH      = true;
                        tempNORMAL    = false;
                        tempOperation = true;
                  }
                  // 40도 이상이었다가 35도로 내려 왔으므로 정상 작동
                  if (nowTemperature <= 35 && tempHIGH == true ){ 
                        tempHIGH      = false;
                        tempNORMAL    = true;
                        tempOperation = true;
                  }



                  // CHECK for ERROR CASE. 
                  // tempOperation
                  if((tempOperation == true && pulseCNTF > rpmMINvalueF )|| (tempOperation == true  && pulseCNTB > rpmMINvalueB ) || (tempOperation == true && pulseCNTF == 0 && uppderCMDZero != true) || (tempOperation == true && pulseCNTB == 0 && uppderCMDZero != true)) {
        
                      if(pulseCNTF < rpmMINthresF) frontFAN = true;
                      else frontFAN = false; 
    

                      // FAN FAULT DETECTED for the first time.
                      // 6 TIMES REPEAT CHECK
                      if(fanFAULT != true && errorContinueCNT >= 3) {  //[**2] P_FAN_FAULT signal change from PWM type to continuous type
                              fanFAULT = true;   
                               
                              digitalWrite(P_FAN_FAULT, HIGH);
                              digitalWrite(P_LED_ALM,   HIGH);

                              //Error occurred for the first time. Timer start !!
                              durationError = millis();

                              noFaultCNT = 0;
                              
                      } else {
                              errorContinueCNT++;
                              noFaultCNT = 0;    
                      } // if end
        
                  } else {
                          errorContinueCNT = 0;
                          noFaultCNT++;
                  } // if end

                  // CHECK FOR RELEASE CASE
                  if(noFaultCNT >= 2 && fanFAULT == true ) { 
        
                          if(fanFAULT == true) { 
                                // 에러 복구로 확인하고, 상태 복귀
                                fanFAULT = false; 
                                   
                                digitalWrite(P_FAN_FAULT, LOW);
                                digitalWrite(P_LED_ALM,   LOW);

                                durationError = 0;
                          }
                  }

                  
                  //####################### 상판일 경우 #######################
                  if(UP_DOWN){
                           
                          // 상판의 경우는 마스터가 되고. 온도에 따른 값을 전달 해 주면 됨
                          Serial.print("UPPER |");
                           
                          int targetRpm = map(nowTemp, tempMIN, tempMAX, speedZERO, speedFULL); //현재의 온도값을 읽어 팬의속도를 결정한다
        
                          // speedFULL option added. When fanFAULT == true.
                          // if(readSIGNAL == 0 ||fanFAULT == true) targetRpm = speedFULL;
                          
                          if(fanFAULT == true) targetRpm = speedFULL;
                              
                           //[**4] targetRpm을 10으로 전달함. 하판에 온도때문에 작동 중지를 전달 하기 위함. pulse값을 전달 해야 함. 0과는 구분해야 함.
                           //      70 - 85 사이로 나옴. SIGNAL.
                           if(tempOperation == false) targetRpm = 10;   //대전제 작동 하냐 / 안하냐 !!! 
                           else if (tempHIGH == true) targetRpm = speedFULL; //다 필요없고 온도가 40도 이상이면 풀로 돌아라. (35도 올때까지)
                           
                           commandFan(targetRpm,UPPER);
                    
                  //####################### 하판일 경우 #######################
                  }else{
                            
                           // 상판의 경우는 마스터가 되고. 온도에 따른 값을 전달 해 주면 됨
                           Serial.print("LOWER |");
 
        
                           // DEBUG SCENARIO SIGNAL value filtering. 2019.04.15 EMERGENCY!!!
                           // [**4]비교값을 수정해야 함. 10의 펄스값을 확인하고, 적용 해야 함.

                 
                           //상판이 정상작동 할 경우 상판의 팬속도와 동기화
                           if(readSIGNAL > 0){

                                lowerZeroCNT = 0;
                                
                                int out = map(readSIGNAL, 0, pwmMAX, speedZERO, speedFULL);
          
                                // FAN fail check
                                if(fanFAULT == true) out = speedFULL;
          
                                //[**4] 펄스값이 작동 중지 상태일 경우 스피드 0 으로 전달.
                                //      스피트 10으로 전달된 펄스값의 범위는 70 - 85 사이임.
                                if(readSIGNAL > 70 && readSIGNAL < 85) {
                                  uppderCMDZero = true;
                                  out = 0;
                                } else uppderCMDZero = false;
          
                                // Command to FAN
                                commandFan(out,LOWER);
                                
                                Serial.print("NORMAL MODE |");
                              
                            }else{            
                                //상판 팬 오류 또는 통신오류일경우 풀가동
                                //[**10] 상판 에러일 경우. 온도값을 보고 작동 중지. 10도 이하일 경우.
                                int out = 0;
                                
                                if(lowerZeroCNT >= 3) {
                                      out = 0; 
                                }

                                if(tempOperation == false  && lowerZeroCNT >= 3 ) out = 0;  // 중지
                                else out = speedFULL;

                                if(tempOperation == false) out = 0;

                                // lowerZeroCNT INIT.

                                if(lowerZeroCNT > 5) {
                                  lowerZeroCNT = 0;
                                } else lowerZeroCNT++;
                                
                                commandFan(out ,LOWER);   // 최대로 돈다.

                                Serial.print("ERROR MODE |");
                            }
                  } 

                  
                  // 2019.04.15 Just reporting 

                  Serial.print("|PULSE[INIT] ");
                  Serial.print("|100 |F|");
                  Serial.print(rpmMINvalueF);
                  Serial.print(" |B|");
                  Serial.print(rpmMINvalueB);
                  Serial.print("| 250 |F|");
                  Serial.print(rpmMAXvalueF);
                  Serial.print(" |B|");
                  Serial.print(rpmMAXvalueB);
                  Serial.print("| pulseCNTF |");
                  Serial.print(pulseCNTF);
                  Serial.print("| pulseCNTB |");
                  Serial.print(pulseCNTB); 
                  Serial.print("| nowTemp |");
                  Serial.print(nowTemperature);
                  Serial.print("| tempOPR |");
                  if(tempOperation == true) Serial.print("ON");
                  else Serial.print("OFF");
                  Serial.print("| STATUS |");
                  if(tempLOW    == true) Serial.print("tmpLOW");
                  if(tempHIGH   == true) Serial.print("tmpHIGH");
                  if(tempNORMAL == true) Serial.print("tmpNORMAL");
                  
                  Serial.println("");

                  delay(sysDelay); 

                 
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
        
        
        //fan 가동, 1: UPPER , 0:LOWER
        void commandFan(int fan_rpm,int flag) {

                  // 10 으로도 상판의 팬이 도는 버그 수정
                  if(flag == UPPER && fan_rpm == 10) {
                          analogWrite(P_B_FAN_PWM, 0);
                          analogWrite(P_F_FAN_PWM, 0);
                  } else {
                          analogWrite(P_B_FAN_PWM, fan_rpm);
                          analogWrite(P_F_FAN_PWM, fan_rpm);
                  }

                  //fanSPEED 변수에 값을 설정, 전역에서 팬 스피트 체크 로직 추가를 위해 [**3]
                  fanSPEED = fan_rpm;
        
                  //상판일 경우 하판과의 동기화를 위해 나의 팬 속도 전송. 상판의 팬 이상일 경우 PWM 데이타를 0으로 제공
                  if(flag == UPPER) {
                        if(fanFAULT == true && fan_rpm == 10)   analogWrite(P_CONN_SIGNAL_OUT, fan_rpm);  //FAN FAULT 상황이어도 팬 작동 정지시에는 정지.
                        else if (fanFAULT == true ) analogWrite(P_CONN_SIGNAL_OUT, 0);
                        else analogWrite(P_CONN_SIGNAL_OUT, fan_rpm);
     
                  } 
        }
        
        //온도측정
        float readNowTemp(){
                unsigned int  ADCValue    = 0;
                double        Voltage     = 0;
                double        Vcc         = 0;
                float         averageTemp = 0;
                float         averageVolt = 0.0;
                int           tmpCNT      = 0;

                while( tmpCNT < NUM_TEMP_AVERAGE){
                        tmpCNT++;
                        ADCValue = 0.0;
                        Vcc = readVcc()/1000.0;
                        ADCValue = analogRead(P_NOW_TEMP);   
                        Voltage = (ADCValue / 1023.0) * Vcc;
                
                        // RAW DATA의 통계 처리로 바꿈.=================================
                
                         // 누적통계 - TEMPERATURE STATISTICS applying.
                        fTArray[averageTCNT] = Voltage;
                        
                        for (int tmpCountT = 0; tmpCountT < NUM_TEMP_AVERAGE; tmpCountT ++) {
                            averageVolt += fTArray[tmpCountT];
                        }
                        
                        averageVolt  /= NUM_TEMP_AVERAGE;
                  
                        if((averageTCNT - (NUM_TEMP_AVERAGE-1)) >= 0) averageTCNT = 0;     //Average count reset
                        else averageTCNT++; // 누적통계 - TEMPERATURE STATISTICS applying.

                        delay(10);

                }
    
                // 통계처리 완료.=============================================
          
                float temperC = (averageVolt - 0.5) * 100 ; 
                averageTemp = temperC;
        
                // 통계 처리된 온도를 전역변수에 항상 넣어줌.[**4]
                nowTemperature = averageTemp;
        
                if(averageTemp < tempMIN){
                    averageTemp = tempMIN;
                }
                
                if(averageTemp > tempMAX){
                    averageTemp = tempMAX;
                }

                // RETURN Temperature
                return averageTemp;

        }

        void countPulse(int rpm, int duration) {

                // FINAL
                //FRONT FAN
                unsigned long   timeDeltaF    = 0;
                boolean         isHIGHF       = false;
                int             highCNTF      = 0;
                unsigned long   inPulsCNTF    = 0;
                
                //BACK FAN
                unsigned long   timeDeltaB    = 0;
                boolean         isHIGHB       = false;
                int             highCNTB      = 0;
                unsigned long   inPulsCNTB    = 0;
          
                unsigned long   averageFront  = 0;
                unsigned long   averageBack   = 0;
                unsigned int    ADCValueF;
                unsigned int    ADCValueB;  

                unsigned int    readPulseTime = millis();

                analogWrite(P_B_FAN_PWM, rpm);
                analogWrite(P_F_FAN_PWM, rpm);
                
                for(int i=0;i<=30;i++)  {
                  fFArray[i] = 0;
                }
                
                for(int i=0;i<=30;i++)  {
                  fBArray[i] = 0;
                }

                while((millis() - readPulseTime) < duration){

                        // 특정 시간단위 PULSE HIGH COUNT.
                        ADCValueF   = analogRead(P_F_FAN_RPM); // P_B_FAN_RPM
                        ADCValueB   = analogRead(P_B_FAN_RPM); // P_F_FAN_RPM

                        Serial.print(ADCValueF);
                        Serial.print(" ");
                        Serial.print(ADCValueB);

                        if(ADCValueF > 100 ) {
                              if(isHIGHF == true ) {
                                
                                    highCNTF ++;
                                    
                              } else {
                                    isHIGHF = true;
                                    highCNTF ++;
                                    timeDeltaF = micros();
                              }
                              
                        } else {
                              if(isHIGHF == true && highCNTF >= 2) {
                                    // FINAL TIME STAMP
                                    // Value INPUT
                                    
                                    inPulsCNTF = micros() - timeDeltaF;

                                    #if DEBUG_PULSE
                                          Serial.print("FFFFFF COUNT : ");
                                          Serial.print(highCNTF);
                                          Serial.print(" TIMESTAMP : ");
                                          Serial.print((micros() - timeDeltaF));  
                                          Serial.println(""); 
                                    #endif
                              } isHIGHF = false; 
                              highCNTF = 0;
                              //timeDeltaF = micros();
                        }


                         if(ADCValueB > 100 ) {
                              if(isHIGHB == true ) {
                                
                                    highCNTB ++;
                                    
                              } else {
                                    isHIGHB = true;
                                    highCNTB ++;
                                    timeDeltaB = micros();
                              }
          
                        } else {
                              if(isHIGHB == true && highCNTB >= 2) {
                                    
                                    // FINAL TIME STAMP
                                    inPulsCNTB = micros() - timeDeltaB;

                                    #if DEBUG_PULSE
                                            Serial.print("BBBB COUNT : ");
                                            Serial.print(highCNTB);
                                            Serial.print(" TIMESTAMP : ");
                                            Serial.print((micros() - timeDeltaB));  
                                            Serial.println(""); 
                                    #endif 
                              } isHIGHB = false; 
                              highCNTB = 0;
                        }


                        // STATISTICS for VALUES.

                        fFArray[averageCNT] = inPulsCNTF;
                        for (int tmpCountF = 0; tmpCountF < NUM_AVERAGE; tmpCountF ++) {
                            averageFront += fFArray[tmpCountF];
                        }
                        averageFront  /= NUM_AVERAGE;

                        fBArray[averageCNT] = inPulsCNTB;
                        for (int tmpCountB = 0; tmpCountB < NUM_AVERAGE; tmpCountB ++) {
                            averageBack += fBArray[tmpCountB];
                        }
                        averageBack  /= NUM_AVERAGE;
              
                        if((averageCNT - (NUM_AVERAGE-1)) >= 0) averageCNT = 0;     //Average count reset
                        else averageCNT++;

                        Serial.print(" CNTF [");
                        Serial.print(inPulsCNTF);
                        Serial.print("] CNTB [");
                        Serial.print(inPulsCNTB);
                        Serial.print("]");
                        Serial.println("");

                        delay(1);

                  } // while END


                // VALUE REGISTER
                if(rpm >= speedFULL) {
                      // MAX VALUE REGISTER
                      rpmMAXvalueF = averageFront;
                      rpmMAXvalueB = averageBack;
                } else {
                      // MIN VALUE REGISTER
                      rpmMINvalueF = averageFront;
                      rpmMINvalueB = averageBack;   
                }               
        }
        
        // Timer triggering functions (rising(), falling())
        void rising()
        {
                latest_interrupted_pin=PCintPort::arduinoPin;
                PCintPort::attachInterrupt(latest_interrupted_pin, &falling, FALLING);
                prev_time = micros();
        }
         
        void falling() {
                latest_interrupted_pin=PCintPort::arduinoPin;
                PCintPort::attachInterrupt(latest_interrupted_pin, &rising, RISING);
                pwm_value = micros()-prev_time;
                Serial.println(pwm_value);
        }
