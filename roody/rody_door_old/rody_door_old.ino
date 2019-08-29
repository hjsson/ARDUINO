#include <EEPROM.h>
#include <SoftwareSerial.h>

#define MAGNETIC_FAR   1 //자석이 멀리있으면 1
#define MAGNETIC_CLOSE 0 //자석이 가까이 있으면 0

#define MAX_ADC_DIF    300
#define MOTICE_CNT_MAX 600
#define BLE_MODE        1
#define SMARTDOOR_MODE  2

/*
 * FIN 0/ RIN 1 :데드볼트 열림
 * FIN 1/ RIN 0 :데드볼트 닫힘
 */

/************ PIN NUMBER DEFINE ****************/
//---> MOTISE
#define FIN            4
#define RIN            5
#define HALL_IC_VOUT   8
#define DEADVOLT_CLOSE 9
#define DEADVOLT_OPEN  10

//---> SMARTDOORLOCK READ SIGNAL
#define S_FIN          A2
#define S_RIN          A3
  
//---> TOW LED
#define LED1           18
#define LED2           19

//---> ADC FOR READ TOUCH
#define ADC_IN        A0

//---> BLE
#define BT_RX         2 // BT모듈의 Tx와 연결 됨.
#define BT_TX         3 // BT모듈의 Rx와 연결 됨.
/************** DEFINE INLINE *********************/
#define LED1_ON     digitalWrite(LED1, LOW);
#define LED1_OFF      digitalWrite(LED1, HIGH);
#define LED2_ON     digitalWrite(LED2, LOW);
#define LED2_OFF      digitalWrite(LED2, HIGH);

#define FIN_ON       digitalWrite(FIN, HIGH);
#define FIN_OFF      digitalWrite(FIN, LOW);
#define RIN_ON       digitalWrite(RIN, HIGH);
#define RIN_OFF      digitalWrite(RIN, LOW);

/**************** DEFINE FUNCTION ****************/
void initLed(void);
void initAdc(void);
void initMotise(void);
void initBLE(void);
void initSmartDoorOutput(void);
void initSystem(void);
void printMotiseVal(void);
void printAdcVal(void);
void printS_Motise(void);
void magSensorCheck(void);
void deadboltCheck(void);
void doorStatusCheck(void);
void motiseOpen(void);
void motiseClose(void);
void motiseStop(void);
void autoMotiseOpen(void);
void autoMotiseClose(void);
void BottonMotiseClose(void);
void scanBLE(void);
void parsingBLE(void);
void readNumOfRegisteredPeople(void);
void readUuidOfRegisteredPeople(void);

/*****************전역 변수 선언*********************/
int MagSensor_val;  //마그네틱 센서값 저장할 변수
int DeadboltOpen_val; //데드볼트 오픈값 저장할 변수
int DeadvoltClose_val; //데드볼트 클로즈값 저장할 변수

int S_Hall_Ic_Vin_val; //스마트 도어락이 모티스의 홀센서에 전원을 주는 값
int S_Photo_Vpp_val; //스마트 도어락이 모티스의 포토센서에 전원 주는 값
int S_Fin_val; //스마트 도어락이 모티스 모터 구동 F
int S_Rin_val; //스마트 도어락이 모티스 모터 구동 R

int ADC_init_val = 0;
int ADC_val = 0;

int MODE;

int btComTime = 0;        //스캔 타이머
int uuidNewCnt = 0;       //신규 유효 id
int uuidCnt = 0;          //유효 id
bool btOpen = false;      //유효bt 검색성공시 open 대기상태
bool newUserYn = false;   //유효bt 검색성공시 open 대기상태
bool userYn = false;      //유효bt 검색성공시 open 대기상태

int eepUserCnt = 0;       //초기화 원할경우 EEPROM.write(1000, 0); 을 셋팅해준다.
int eepUuidAreaCnt = 0;
int uuidEqualCnt = 0;     //저장 uuid와 일치할경우 증가
char userUuid[300];       //max 10명정도
int equalCnt = 0;
int uuidCloseCnt = 0;

bool adcOnOff = true;     //ADC센서 선택유무

int finVal, rinVal;//스마트 도어락의 도어락 구동 신호를 저장할 변수.
int diff;
int MotorCntTimer = 0;

/***************************************************/

SoftwareSerial blueToothSerial(BT_RX,BT_TX);//setup함수 안에서 선언시 컴파일 에러남.

void setup() {
  initSystem();
  initMotise();

  Serial.begin(19200);
  blueToothSerial.begin(153600);

  scanBLE(); delay(15);
  readNumOfRegisteredPeople(); //등록된 사람수 읽기
  readUuidOfRegisteredPeople();//등록된 사람의 UUID 읽기
  
  //MODE = SMARTDOOR_MODE;
  
 
}

void loop() {

  //********* 스마트 도어락 모드 ************
  finVal = analogRead(S_FIN);
  rinVal = analogRead(S_RIN);
  diff = finVal - rinVal;
  
  #if 0
  printS_Motise();
  Serial.println(MotorCntTimer);
  Serial.println(ADC_init_val);
  ADC_val = analogRead(ADC_IN);
  printAdcVal();
  #endif
  
  if(diff > 1000) 
  {
    //Serial.println("OPEN");
    autoMotiseOpen();
  }
  
  //***************** end ********************
#if 1
 //************* 블루투스 모드 ***************
  
  if(btOpen) 
  {
    //Serial.println("scan ok");
    
    ADC_val = analogRead(ADC_IN);
    //printAdcVal();
    //Serial.print("adcOnOff:");
    //Serial.println(adcOnOff);
    if(adcOnOff)
    {
          if((ADC_init_val - ADC_val) > 400)
          {
                autoMotiseOpen();
                btOpen = false;
          }
    }
    else
    {
          autoMotiseOpen();
          btOpen = false;
         // adcOnOff = true;
    }
  }
  if(blueToothSerial.available()) parsingBLE();
  
  //if(btComTime > 4000)
  if(btComTime > 200)
  {  //4초마다 BLE 스캔
          //LED1_ON; delay(200); LED1_OFF;
          blueToothSerial.print("AT+DISI?");  //복제품: AT+DISI?  정품: AT+DISI
          btOpen = false;
          btComTime = 0;
          adcOnOff = true;//hkcho 추가
          //Serial.println("scan");
  }
  btComTime++;
  
 //********************************************   
 #endif
 //delay(2);
 
//***** 마그네틱이 가까우면 닫는 것 공통 ******
  doorStatusCheck();
 if(MagSensor_val == MAGNETIC_CLOSE)
 {
      autoMotiseClose();
 }
 else MotorCntTimer = 0;

}
/******************************************************************/
/************************* [FUNTION] ******************************/
void initLed(void)
{
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  LED1_OFF;
  LED2_OFF;
}

void initAdc(void)
{
  pinMode(ADC_IN, INPUT);
  ADC_init_val = analogRead(ADC_IN);
  delay(2);
  ADC_init_val = analogRead(ADC_IN);
  delay(2);
  ADC_init_val = analogRead(ADC_IN);
}

void initMotise(void)
{
   pinMode(FIN, OUTPUT);
   pinMode(RIN, OUTPUT);

   pinMode(HALL_IC_VOUT, INPUT_PULLUP);
   pinMode(DEADVOLT_CLOSE, INPUT_PULLUP);
   pinMode(DEADVOLT_OPEN, INPUT_PULLUP);
}

void initSmartDoorOutput(void)
{
  pinMode(S_FIN, INPUT);
  pinMode(S_RIN, INPUT);
}

void initBLE(void)
{
  pinMode(BT_RX, INPUT);
  pinMode(BT_TX, OUTPUT);
}
void initSystem(void)
{
  initLed();
  initAdc();
  //initMotise();
  initBLE();
  initSmartDoorOutput();
}

void  printMotiseVal(void)
{
  Serial.print("마그네틱센서:");
  Serial.println(MagSensor_val);
  Serial.print("데드볼트오픈:");
  Serial.println(DeadboltOpen_val);
  Serial.print("데드볼트 클로즈:");
  Serial.println(DeadvoltClose_val);
}

void printAdcVal(void)
{
  Serial.print("ADC값:");
  Serial.println(ADC_val);
}

void printS_Motise(void)
{
  Serial.print("S_FIN:");
  Serial.println(finVal);
  Serial.print("S_RIN:");
  Serial.println(rinVal);
  Serial.print("diff:");
  Serial.println(diff);
  Serial.println();
}

void magSensorCheck(void)
{
  MagSensor_val = digitalRead(HALL_IC_VOUT);
}
void deadboltCheck(void)
{
  DeadboltOpen_val = digitalRead(DEADVOLT_OPEN);
  DeadvoltClose_val = digitalRead(DEADVOLT_CLOSE);
}

void doorStatusCheck(void)
{
   magSensorCheck();
   deadboltCheck();
}

void motiseOpen(void)
{
   FIN_ON;
   RIN_OFF;
}

void motiseClose(void)
{
   FIN_OFF;
   RIN_ON;
}

void motiseStop(void)
{
  FIN_OFF; RIN_OFF;
}

void autoMotiseOpen(void)
{
  doorStatusCheck();
  if(DeadboltOpen_val == 1 && DeadvoltClose_val == 0) return;//이미 데드볼트가 열려있다면,
  LED1_ON;
  LED2_ON;
  motiseOpen();
  while(1)//데드볼트 오픈 == 1, 데드볼트 클로즈 0 이 될때까지 오픈.
  {
        deadboltCheck();
        if(DeadboltOpen_val == 1 && DeadvoltClose_val == 0) 
        {
          motiseStop();
          LED1_OFF;
          LED2_OFF;
          adcOnOff = true;//hkcho 추가
          break;
            
        }
  }

  motiseClose();
  delay(20);
  motiseStop();
  
}

void autoMotiseClose(void)
{
  doorStatusCheck();
  if(DeadboltOpen_val == 0 && DeadvoltClose_val == 1) return;//이미 데드볼트가 닫혀있다면,
  
  MotorCntTimer++;
  if(MotorCntTimer++ < MOTICE_CNT_MAX) return;
  LED1_ON;
  LED2_ON;
  motiseClose();
  while(1)//데드볼트 오픈 == 1, 데드볼트 클로즈 0 이 될때까지 오픈.
  {
        deadboltCheck();
        if(DeadboltOpen_val == 0 && DeadvoltClose_val == 1) 
        {
          motiseStop();
          MotorCntTimer = 0;
          LED1_OFF;
          LED2_OFF;
          break;
            
        }
        delay(10);
  }
  motiseOpen();
  delay(20);
  motiseStop();
}

void BottonMotiseClose(void)
{
  doorStatusCheck();
  if(DeadboltOpen_val == 0 && DeadvoltClose_val == 1) return;//이미 데드볼트가 닫혀있다면,
  
  LED1_ON;
  LED2_ON;
  motiseClose();
  while(1)//데드볼트 오픈 == 1, 데드볼트 클로즈 0 이 될때까지 오픈.
  {
        deadboltCheck();
        if(DeadboltOpen_val == 0 && DeadvoltClose_val == 1) 
        {
          motiseStop();
          LED1_OFF;
          LED2_OFF;
          break;
            
        }
        delay(10);
  }
  motiseOpen();
  delay(20);
  motiseStop();
}

void scanBLE(void)
{
      blueToothSerial.print("AT+DISI?");  //복제품: AT+DISI?  정품: AT+DISI
      btOpen = false;
      btComTime = 0;
}

void parsingBLE(void)
{
  char readBtUUid = blueToothSerial.read();  //HM-11 블루투스 신호를 읽는다.
      
            //APP에서 신규등록요청이 들어왔을때
            if(readBtUUid == 'A') uuidNewCnt++; 
            else uuidNewCnt = 0;

            if(newUserYn)
            {  //신규유저등록 신호 확인후 등록
                    if(eepUuidAreaCnt < 25)
                          { //UUID 25자리만 등록한다
                            EEPROM.write((eepUserCnt*25)+eepUuidAreaCnt, readBtUUid);
                            userUuid[(eepUserCnt*25)+eepUuidAreaCnt] = readBtUUid;
                            eepUuidAreaCnt++;
                    }
                    else
                    {
                            eepUserCnt++; //등록된 총 유저수를 증가
                            EEPROM.write(1000, eepUserCnt);
                            newUserYn = false;
                            eepUuidAreaCnt = 0; //저장소 자릿수 초기화
                            LED1_ON;
                            LED2_ON;
                            delay(100);
                            LED1_OFF;
                            LED2_OFF;
                    }
            }
            else 
            {  //주변 등록된 사용자 탐색
                    //uuid scan 일치 되는지 확인(임시UUID설정 DDDDD >> TODO: 보안 UUID로 교체)
                    if(readBtUUid == 'D') uuidCnt++;
                    else uuidCnt = 0;

                    if(userYn)
                    {
                            if(readBtUUid == 'B')
                            { //ADC 인식 배제 로직 추가(UUID만 일치하면 DOOR OPEN) 20180821 조소장님 요청사항
                                    adcOnOff = false;
                            }
                            for (int i = 0;  i < eepUserCnt;  i++) 
                            {
                                    //저장소 처음부터 비교해야함
                                    if(eepUserCnt == 1)
                                    {
                                            if(userUuid[equalCnt] == readBtUUid) uuidEqualCnt++;
                                                      
                                    }else
                                    {
                                            if(userUuid[25+equalCnt] == readBtUUid) uuidEqualCnt++;
                                                      
                                    }
                            }
                            equalCnt++;
                            //Serial.println();
                            //Serial.print("equalCnt=");
                            //Serial.println(equalCnt);
                            //Serial.print("uuidEqualCnt=");
                            //Serial.println(uuidEqualCnt);
                            if(uuidEqualCnt > 20)
                            {
                                    //Serial.println(readBtUUid);
                                    btOpen = true;
                                    userYn = false;
                                    equalCnt = 0;
                            }
                    }
                    else
                    {
                            uuidEqualCnt = 0;
                    }
                    //순서가 중요 5이상이고 다음에 들어오는것부터 저장
                    if(uuidCnt > 4) userYn = true;                   
            }
            //순서가 중요 5이상이고 다음에 들어오는것부터 저장
            if(uuidNewCnt > 4) newUserYn = true;

            
            //도어락 닫힘 로직 추가(APP에서 닫힘 버튼 추가)20180821 조소장님 요청사항 ========
            if(readBtUUid == 'F') uuidCloseCnt++;
            else uuidCloseCnt = 0;
      
            if(uuidCloseCnt > 4)
            {       
                    //Serial.println("닫힘 버튼");
                    BottonMotiseClose();
                    uuidCloseCnt = 0;
            }
          //=== END ======== 도어락 닫힘 로직 추가 ========            
}

void readNumOfRegisteredPeople(void)
{
        eepUserCnt = EEPROM.read(1000);     //1000번째 주소에 등록된 가족 수가 저장됨. byte 48 초기값일경우 초기화
        if(eepUserCnt > 9)  // 9명 이상 등록시 초기화.
        {
              EEPROM.write(1000, 0);
              eepUserCnt = 0;
        }
}

void readUuidOfRegisteredPeople(void)
{
         for(int i = 0; i < (25*eepUserCnt); i++) 
         {               
               userUuid[i] = EEPROM.read(i);     // EEPROM에 저장된 가족 UUID 데이터 읽기
                //Serial.println("EEPROM Address : " + String(i) + "\t Value : " + userUuid[i]);
         }
}



