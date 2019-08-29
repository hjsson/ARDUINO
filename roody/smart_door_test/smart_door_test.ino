//초음파 센서의 핀번호를 설정한다.
int echoPin = 11;
int trigPin = 12;
int motorPin = A1;
boolean manCheck = false;
int manCkCnt = 0;

void setup() {
  Serial.begin(9600);
  // trig를 출력모드로 설정, echo를 입력모드로 설정
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(motorPin, OUTPUT);
}
//2160.21cm
//28.03cm
void loop() {
  digitalWrite(trigPin, LOW);
  digitalWrite(echoPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH); 
  float distance = ((float)(340 * duration) / 10000) / 2;
  if(distance < 30){
    manCheck = true;
    if(manCkCnt < 20){
      manCkCnt++;
    }
  }else{
    manCheck = false;
    manCkCnt = 0;
  }
  if(manCkCnt == 10){
    Serial.print(manCheck);
  }
  //Serial.println(manCkCnt);
  if (Serial.available()){
    int inItVal  = Serial.read();
    Serial.println(inItVal);
    if(inItVal == 49){ //value: 1 
      digitalWrite(motorPin, HIGH);
      delay(1500);
      digitalWrite(motorPin, LOW);
    }
  }
  delay(100);
}
