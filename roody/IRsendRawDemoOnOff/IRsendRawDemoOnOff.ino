/* PIR sensor tester*/
#include <IRremote.h>
#include <EEPROM.h>

int recvPin = 11;         // 리모컨 수신부
int inputPin = 2;         // 동작감지기 센서
int pirState = LOW;       // 동작감지기 기본값
int val = 0;              // 동작감지기 값 임시 저장소
int khz = 38;             // 38kHz carrier frequency for the NEC protocol
boolean onOffYn = true;  // false:Off, true:On
int saveOffCnt = 0;
int saveOnCnt = 0;
int dataLength[3] = {0,0,0};
//SWITCH PIN
int swOne = 4;
int swTwo = 5;
int swThr = 6;
//SWITCH PIN FLAG
int swOnOffNum = 0; //스위치 눌림상태 값
//LED PIN
int ledOne = 7;
int ledTwo = 8;
int ledThr = 9;
int ledPow = 10;
int ledWrite = 12;
//State Flag
int stateFlag = 0; //0:저장값 없어서 대기 모드, 1:하드웨어 데이터 쏘기 모드, 2: 리모컨 데이터 입력모드 
int allReset = 0;
unsigned int oneRowDtArr[220];
unsigned int twoRowDtArr[220];
unsigned int thrRowDtArr[220];

IRrecv irrecv(recvPin);
IRsend irsend;
decode_results  results;        // Somewhere to store the results

void setup() {
    Serial.begin(9600);
    irrecv.enableIRIn();  // Start the receiver
    pinMode(inputPin, INPUT); // declare sensor as input
    //LED SW SET
    pinMode(ledOne, OUTPUT);
    pinMode(ledTwo, OUTPUT);
    pinMode(ledThr, OUTPUT);
    pinMode(ledPow, OUTPUT);
    pinMode(ledWrite, OUTPUT);
    //switch
    pinMode(swOne, INPUT_PULLUP);
    pinMode(swTwo, INPUT_PULLUP);
    pinMode(swThr, INPUT_PULLUP);
    digitalWrite(ledPow, HIGH); //POWER ON
    dataLength[0] = EEPROMReadInt(1000);
    dataLength[1] = EEPROMReadInt(1002);
    dataLength[2] = EEPROMReadInt(1004);
    if(dataLength[0] > 1){
      for (int i = 0;  i < (dataLength[0]*2)+2;  i=i+2) {
        oneRowDtArr[i/2] = EEPROMReadInt(i);
      }
      digitalWrite(ledOne, HIGH); // ON
      stateFlag = 1;
      
    };
    if(dataLength[1] > 1){
      for (int i = 300;  i < 300+(dataLength[1]*2)+2;  i=i+2) {
        twoRowDtArr[(i-300)/2] = EEPROMReadInt(i);
      }
      digitalWrite(ledTwo, HIGH); // ON
      stateFlag = 1;
    };
    if(dataLength[2] > 1){
      for (int i = 600;  i < 600+(dataLength[2]*2)+2;  i=i+2) {
        thrRowDtArr[(i-600)/2] = EEPROMReadInt(i);
      }
      digitalWrite(ledThr, HIGH); // ON
      stateFlag = 1;
    };
}
//=========EEPROM===ReadInt, EEPROM===WriteInt=========start
unsigned int EEPROMReadInt(int p_address){
    byte lowByte = EEPROM.read(p_address);
    byte highByte = EEPROM.read(p_address + 1);
    return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}
void EEPROMWriteInt(int p_address, int p_value){
     byte lowByte = ((p_value >> 0) & 0xFF);
     byte highByte = ((p_value >> 8) & 0xFF);
     EEPROM.write(p_address, lowByte);
     EEPROM.write(p_address + 1, highByte);
}
//=========EEPROM===ReadInt, EEPROM===WriteInt=========end   eepromCnt
void  EEPROMWrite (decode_results *results){
  if(results->rawlen < 110){  //MAX BYTE
    if(swOnOffNum == 1){
      EEPROMWriteInt(1000, results->rawlen-1);
      digitalWrite(ledOne, HIGH);
    }
    if(swOnOffNum == 2){
      EEPROMWriteInt(1002, results->rawlen-1);
      digitalWrite(ledTwo, HIGH);
    }
    if(swOnOffNum == 3){
      EEPROMWriteInt(1004, results->rawlen-1);
      digitalWrite(ledThr, HIGH);
    }
    dataLength[swOnOffNum-1] = results->rawlen-1;
    for (int i = 1;  i < results->rawlen;  i++) {
        unsigned int  rowDt = results->rawbuf[i] * USECPERTICK;
        if(swOnOffNum == 1){
            if(i == 1){
                EEPROMWriteInt(0, rowDt);
            }
            if(i == 2){
                EEPROMWriteInt(2, rowDt);
            }
            if(i > 2){
                EEPROMWriteInt(((i*1)+(i-2)), rowDt);
            }
            oneRowDtArr[i] = rowDt;
            digitalWrite(ledOne, HIGH);
        }
        if(swOnOffNum == 2){
            if(i == 1){
                EEPROMWriteInt(300, rowDt);
            }
            if(i == 2){
                EEPROMWriteInt(302, rowDt);
            }
            if(i > 2){
                EEPROMWriteInt((300 + ((i*1)+(i-2))), rowDt);
            }
            twoRowDtArr[i] = rowDt;
            digitalWrite(ledTwo, HIGH);
        }
        if(swOnOffNum == 3){
            if(i == 1){
                EEPROMWriteInt(600, rowDt);
            }
            if(i == 2){
                EEPROMWriteInt(602, rowDt);
            }
            if(i > 2){
                EEPROMWriteInt((600 + ((i*1)+(i-2))), rowDt);
            }
            if(i > 150){
              return;
            }
            thrRowDtArr[i] = rowDt;
            digitalWrite(ledThr, HIGH);
        }
    }
    swOnOffNum = 0;
    ledNumOnFn(swOnOffNum);
    digitalWrite(ledWrite, LOW);
    stateFlag = 1;
  }
}
void ledNumOnFn(int ledNum){
  for (int a = 0;  a < ledNum;  a++) {
    digitalWrite(ledWrite, HIGH);
    delay(100); 
    digitalWrite(ledWrite, LOW);
    delay(100); 
    digitalWrite(ledWrite, HIGH);
  }
}

void loop() {
  if (digitalRead(swOne) == LOW) {  //1번 눌림상태
    swOnOffNum = 1;
    stateFlag = 2;
    ledNumOnFn(1);
    allReset++;
  }; 
  if (digitalRead(swTwo) == LOW) {  //2번 눌림상태
    swOnOffNum = 2;
    stateFlag = 2;
    ledNumOnFn(2);
    allReset++;
  }; 
  if (digitalRead(swThr) == LOW) {  //3번 눌림상태
    swOnOffNum = 3;
    stateFlag = 2;
    ledNumOnFn(3);
    allReset++;
  }; 
  if(allReset > 2){
    dataLength[0] = 0;
    dataLength[1] = 0;
    dataLength[2] = 0;
    digitalWrite(ledOne, LOW);
    digitalWrite(ledTwo, LOW);
    digitalWrite(ledThr, LOW);
    digitalWrite(ledWrite, LOW);
    EEPROMWriteInt(1000, 0);
    EEPROMWriteInt(1002, 0);
    EEPROMWriteInt(1004, 0);
    stateFlag = 0;  //저장된 데이터 없으므로 대기모드
  }
  allReset = 0;
  
  if(stateFlag == 0){  //0: 버튼 입력 대기모드
    delay(300);
    digitalWrite(ledWrite, LOW);
    delay(300); 
    digitalWrite(ledWrite, HIGH);
  };
  if(stateFlag == 1){  //1: 하드웨어 데이터 쏘기 모드
    val = digitalRead(inputPin); // read input value
    if (val == LOW) { // check if the input is HIGH
      saveOffCnt++;
      saveOnCnt = 0;
      if(saveOffCnt > 30){ // 약 8초이상 움직임이 없을때
          if(onOffYn){
            //Serial.print(dataLength[0]);
            //Serial.print(",");
            //Serial.print(dataLength[1]);
            //Serial.print(",");
            //Serial.print(dataLength[2]);
              if(dataLength[0] > 1){
                irsend.sendRaw(oneRowDtArr, dataLength[0], khz); 
                irsend.sendRaw(oneRowDtArr, dataLength[0], khz); 
                irsend.sendRaw(oneRowDtArr, dataLength[0], khz); 
                digitalWrite(ledOne, LOW); //POWER ON
                delay(100); 
                digitalWrite(ledOne, HIGH); //POWER ON
              }
              if(dataLength[1] > 1){
                irsend.sendRaw(twoRowDtArr, dataLength[1], khz); 
                irsend.sendRaw(twoRowDtArr, dataLength[1], khz); 
                irsend.sendRaw(twoRowDtArr, dataLength[1], khz); 
                digitalWrite(ledTwo, LOW); //POWER ON
                delay(100); 
                digitalWrite(ledTwo, HIGH); //POWER ON
              }
              if(dataLength[2] > 1){
                irsend.sendRaw(thrRowDtArr, dataLength[2], khz); 
                irsend.sendRaw(thrRowDtArr, dataLength[2], khz); 
                irsend.sendRaw(thrRowDtArr, dataLength[2], khz); 
                digitalWrite(ledThr, LOW); //POWER ON
                delay(100); 
                digitalWrite(ledThr, HIGH); //POWER ON
              }
              onOffYn = false;
          }
          saveOffCnt = 0;
        }
      }else{  //val == HIGH
          digitalWrite(ledPow, LOW); //POWER ON
          delay(50); 
          digitalWrite(ledPow, HIGH); //POWER ON
          saveOnCnt++;
          saveOffCnt = 0;
          if(saveOnCnt > 50){
              onOffYn = true;
              saveOnCnt = 0;
          }
      }
  }
  //PIR SENSOR
  if(stateFlag == 2){  //2: 리모컨 데이터 입력대기 모드
    if (irrecv.decode(&results)) {  // Grab an IR code
      EEPROMWrite(&results);
      irrecv.resume();              // Prepare for the next value
      irrecv.resume(); 
    }
  }
  delay(100); 
}
