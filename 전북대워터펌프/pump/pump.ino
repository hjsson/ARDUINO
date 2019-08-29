int camSignal = 6;
int pumpSignal = A1;
int chFivePin = 9;
int chSixPin = 10;
int chFive;
int chSix;
int pumpState = 250;
int camState = 0; //0 대기, 1 Picture Mode, 2 Read Mode
void setup() {
  // put your setup code here, to run once:
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  Serial.begin(9600);
  pinMode(camSignal, OUTPUT);   
  pinMode(pumpSignal, OUTPUT);   
  delay(100);
  analogWrite(pumpSignal, 250);
}
void camNomal() {
  delay(1000);
  digitalWrite(camSignal, HIGH);
  delayMicroseconds(1000);
  digitalWrite(camSignal, LOW);
  delayMicroseconds(1000);
  digitalWrite(camSignal, HIGH);
  delayMicroseconds(1000);
  digitalWrite(camSignal, LOW);
}
void camOnOff() {
  digitalWrite(camSignal, HIGH);
  delayMicroseconds(2000);
  digitalWrite(camSignal, LOW);
  delayMicroseconds(2000);
  digitalWrite(camSignal, HIGH);
  delayMicroseconds(2000);
  digitalWrite(camSignal, LOW);
  camNomal();
}
void camReadOnOff() {
  digitalWrite(camSignal, HIGH);
  delayMicroseconds(1500);
  digitalWrite(camSignal, LOW);
  delayMicroseconds(1500);
  digitalWrite(camSignal, HIGH);
  delayMicroseconds(1500);
  digitalWrite(camSignal, LOW);
  camNomal();
}

void loop() {
  chFive = pulseIn(9, HIGH); // 980, 1480, 1800
  Serial.print("Channel 5:"); // Print the value of 
  Serial.println(chFive);
  delay(100);
  chSix = pulseIn(10, HIGH);

  Serial.print("Channel 6=====:"); // Print the value of 
  Serial.println(chSix);
  
  //ch5번 값이 1500 이하일 경우 PUMP OFF
  if(chFive < 1500){
    if(pumpState == 0){
      analogWrite(pumpSignal, 250);
      pumpState = 250;
      //Serial.println(chFive);
    }
  }
  //ch5번 값이 1500 이상일 경우 PUMP ON
  if(chFive > 1500){
    if(pumpState == 250){
      analogWrite(pumpSignal, 0);
      pumpState = 0;
      //Serial.println(chFive);
    }
  }
  //ch6번 값이 초기값 경우 CAM PICTURE MODE ON
  if(chSix > 800 && chSix < 1100){
    if(camState == 1){
      camOnOff();
      camState = 0;
    }
    if(camState == 2){
      camReadOnOff();
      camState = 0;
    }
  }
  //ch6번 값이 중간값일 경우 CAM PICTURE MODE ON
  if(chSix > 1300 && chSix < 1600){
    if(camState != 1){
      camOnOff();
      camState = 1;
      //Serial.println(chSix);
    }
  }
  //ch6번 값이 최대값일경우 CAM READ MODE ON
  if(chSix > 1700){
   if(camState != 2){
      camReadOnOff();
      camState = 2;
      //Serial.println(chSix);
    }
  }
  delay(100);
  /*if (Serial.available()){
    int inItVal  = Serial.read();
    if(inItVal == 49){ //1번 motor off
      analogWrite(pumpSignal, 250);
    }
    if(inItVal == 50){ //2번 motor on
      analogWrite(pumpSignal, 0);
    }
    if(inItVal == 51){ //3번 cam start
      camOnOff();
    }
    if(inItVal == 52){ //4번 cam stop
      camOnOff();
    }
    if(inItVal == 53){ //5번 cam read mode on
      camReadOnOff();
    }
    if(inItVal == 54){ //6번 cam read mode off
      camReadOnOff();
    }
  }*/
}
