/* 일자   : 2018-07-25
 * 작성자 : 개발 2팀 HJ-SON, R&D HK-CHO
 * NOTI   : 비콘연동 AUTO DOOR SYSTEM
 * USE    : Bluetooth LE device scanner using HM-11
 */
#include <EEPROM.h>
#include <SoftwareSerial.h>   //Software Serial Port

//HK-CHO 
int FIN = 4;              //모터 열림방향
int RIN = 5;              //모터 닫힘방향
int HALL_IC_VOUT = 6;     //open, close 인식
int Close = 7;            //잠금장치 닫힘 상태 인식
int Open = 8;             //잠금장치 열림 상태 인식

int sensorPin = A0;       //ADC SENSOR FIN
int ledPin = 13;
int sensorValue = 0;      //ADC SENSOR VAL 0~1023

bool Open_flag;       
bool Close_flag;
bool HALL_flag;           //mag non Detection:1, mag Detection:0

// HJ-SON
int btRx = 2;             //bt 모듈 Tx와 연결
int btTx = 3;             //bt 모듈 Rx와 연결
int btComTime = 0;        //스캔 타이머
int uuidNewCnt = 0;       //신규 유효 id
int uuidCnt = 0;          //유효 id
bool btOpen = false;      //유효bt 검색성공시 open 대기상태
bool newUserYn = false;   //유효bt 검색성공시 open 대기상태
bool userYn = false;      //유효bt 검색성공시 open 대기상태
SoftwareSerial blueToothSerial(btRx,btTx);//the software serial port 
int eepUserCnt = 0;       //초기화 원할경우 EEPROM.write(1000, 0); 을 셋팅해준다.
int eepUuidAreaCnt = 0;
int uuidEqualCnt = 0;     //저장 uuid와 일치할경우 증가
char userUuid[300];       //max 10명정도
int equalCnt = 0;
int adcInItVal = 0;       //ADC센서 진입시 저장하여 비교(가변저항)
bool adcOnOff = true;     //ADC센서 선택유무

void setup() {
  //HK-CHO 
  Serial.begin(9600);
  pinMode(FIN, OUTPUT);
  pinMode(RIN, OUTPUT);
  pinMode(HALL_IC_VOUT, INPUT_PULLUP);
  pinMode(Close, INPUT_PULLUP);
  pinMode(Open, INPUT_PULLUP);
  
  // HJ-SON
  pinMode(btRx, INPUT);    //UART pin for Bluetooth
  pinMode(btTx, OUTPUT);   //UART pin for Bluetooth
  blueToothSerial.begin(9600);
  delay(200);
  blueToothSerial.print("AT+BAUD0");  // HM-11 기준임 0 : 9600
  delay(200);
  blueToothSerial.print("AT+IMME1");  //자동연결여부 1:수동연결
  delay(200);
  blueToothSerial.print("AT+ROLE1");  //마스터 모드로 시작
  delay(200);
  blueToothSerial.print("AT+RESET");   //적용
  delay(2000);
  blueToothSerial.print("AT+DISI?");   //SCAN
  //EEPROM.write(1000, 0);
  eepUserCnt = EEPROM.read(1000);     //1000번째 주소에 등록된 가족 수가 저장됨. byte 48 초기값일경우 초기화
  if(eepUserCnt > 9){
   EEPROM.write(1000, 0);
   eepUserCnt = 0;
  }
  //Serial.println(eepUserCnt);
  //Serial.println();
  for(int i = 0; i < (25*eepUserCnt); i++) {               
    userUuid[i] = EEPROM.read(i);     // EEPROM에 저장된 가족 UUID 데이터 읽기
    //Serial.println("EEPROM Address : " + String(i) + "\t Value : " + userUuid[i]);
  }
  
  adcInItVal = analogRead(sensorPin);
  //TEST 추후 삭제 요망 닫힘상태로 만들어주기 ============== START
  //Serial.println(Open_flag);
  Open_flag = digitalRead(Open);
  if(Open_flag == true){
    digitalWrite(FIN, LOW);         
    digitalWrite(RIN, HIGH);
    delay(3000);  
    digitalWrite(FIN, LOW);         
    digitalWrite(RIN, LOW);
  }
  //TEST 추후 삭제 요망 ============== END
}

void loop() {
  //HK-CHO
  if(btOpen){
    sensorValue = analogRead(sensorPin);
    Open_flag = digitalRead(Open);
    Close_flag = digitalRead(Close);
    HALL_flag = digitalRead(HALL_IC_VOUT);
    //Serial.println(sensorValue);
    //DOOR 가 닫혀있고 손잡이 ADC 센서 인식되었을떄 Mag Detection 상관없음
    if(adcOnOff){
      if(Open_flag == false && sensorValue < (adcInItVal-20)){
        digitalWrite(FIN, HIGH);         
        digitalWrite(RIN, LOW);
        digitalWrite(ledPin, HIGH);
        delay(3000);  //도어락 ADC센서 인식 했을때 잡자 마자 때는 경우 Exception 완전히 열리지 않았지만 멈추는 현상이 있음.
        digitalWrite(FIN, LOW);         
        digitalWrite(RIN, LOW);
        digitalWrite(ledPin, LOW);
        btOpen = false;
        //TEST를 위해서 다시 닫아준다. 추후 삭제 요망 ============== START
        delay(1000);
        digitalWrite(FIN, LOW);         
        digitalWrite(RIN, HIGH);
        digitalWrite(ledPin, HIGH);
        delay(3000);
        digitalWrite(FIN, LOW);         
        digitalWrite(RIN, LOW);
        digitalWrite(ledPin, LOW);
        //TEST 추후 삭제 요망 ============== END
      }
    }else{
      if(Open_flag == false){
        digitalWrite(FIN, HIGH);         
        digitalWrite(RIN, LOW);
        digitalWrite(ledPin, HIGH);
        delay(3000);  //도어락 ADC센서 인식 했을때 잡자 마자 때는 경우 Exception 완전히 열리지 않았지만 멈추는 현상이 있음.
        digitalWrite(FIN, LOW);         
        digitalWrite(RIN, LOW);
        digitalWrite(ledPin, LOW);
        btOpen = false;
        adcOnOff = true;
        //TEST를 위해서 다시 닫아준다. 추후 삭제 요망 ============== START
        delay(1000);
        digitalWrite(FIN, LOW);         
        digitalWrite(RIN, HIGH);
        digitalWrite(ledPin, HIGH);
        delay(3000);
        digitalWrite(FIN, LOW);         
        digitalWrite(RIN, LOW);
        digitalWrite(ledPin, LOW);
        //TEST 추후 삭제 요망 ============== END
        }
    }
  } 
  /*  HJ-SON  Serial.println();  Serial.write(readBtUUid);
   *  UUID SAMPLE
   *  OK+DISIS
      OK+DISC:00000000:00000000000000000000000000000000:0000000000:14AD0CDD15EA:-085
      OK+DISC:4C000215:0778A49157C4466E8638F2B8BB073142:1E6122B8BF:47C5F3CBBD59:-078
      OK+DISC:00000000:00000000000000000000000000000000:0000000000:0BA35BFFE27F:-083
      OK+DISC:4C000215:3916E933D17E449AAC5179ECCB6D86E0:115C022BBF:612AD035F61D:-044
      OK+DISC:00000000:00000000000000000000000000000000:0000000000:67402F04B2F7:-089
      OK+DISCE
  */
  if(blueToothSerial.available()){
      char readBtUUid = blueToothSerial.read();

      //신규등록요청
      if(readBtUUid == 'A'){  
        uuidNewCnt++;
      }else{
        uuidNewCnt = 0;
      }
      
      if(newUserYn){  //신규유저등록 신호 확인후 등록
        if(eepUuidAreaCnt < 25){ //UUID 25자리만 등록한다
          EEPROM.write((eepUserCnt*25)+eepUuidAreaCnt, readBtUUid);
          userUuid[(eepUserCnt*25)+eepUuidAreaCnt] = readBtUUid;
          eepUuidAreaCnt++;
        }else{
          eepUserCnt++; //등록된 총 유저수를 증가
          EEPROM.write(1000, eepUserCnt);
          newUserYn = false;
          eepUuidAreaCnt = 0; //저장소 자릿수 초기화
          digitalWrite(ledPin, HIGH);
          delay(500);
          digitalWrite(ledPin, LOW);
        }
      }else{  //주변 등록된 사용자 탐색
        //uuid scan 일치 되는지 확인
        if(readBtUUid == 'D'){  
          uuidCnt++;
        }else{
          uuidCnt = 0;
        }
        
        if(userYn){
          if(readBtUUid == 'B'){
            adcOnOff = false;
          }
          for (int i = 0;  i < eepUserCnt;  i++) {
            //저장소 처음부터 비교해야함
            if(eepUserCnt == 1){
              if(userUuid[equalCnt] == readBtUUid){
                uuidEqualCnt++;
              };
            }else{
              if(userUuid[25+equalCnt] == readBtUUid){
                uuidEqualCnt++;
              };
            };
          };
          equalCnt++;
          //Serial.println();
          //Serial.print("equalCnt=");
          //Serial.println(equalCnt);
          //Serial.print("uuidEqualCnt=");
          //Serial.println(uuidEqualCnt);
          if(uuidEqualCnt > 20){
            //Serial.println(readBtUUid);
            btOpen = true;
            userYn = false;
            equalCnt = 0;
            //Serial.println("true");
          }
        }else{
          uuidEqualCnt = 0;
        }
        //순서가 중요 5이상이고 다음에 들어오는것부터 저장
        if(uuidCnt > 4){
          userYn = true;
        }
      }
      //순서가 중요 5이상이고 다음에 들어오는것부터 저장
      if(uuidNewCnt > 4){
        newUserYn = true;
      }
      //Serial.write(readBtUUid);
    }
    
    if(btComTime > 4000){
      blueToothSerial.print("AT+DISI?");  
      btOpen = false;
      btComTime = 0;
      //Serial.println();
    }
    delay(2);
    btComTime++;
}

