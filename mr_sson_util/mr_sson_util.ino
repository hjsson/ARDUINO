
unsigned int tempVal;

void setup() {
  Serial.begin(9600);
  pinMode(PinNumber, INPUT);
  pinMode(PinNumber, OUTPUT);
  delay(100);
  delayMicroseconds(1000);
}

void loop() {
  Serial.print("--");    //한줄 출력
  Serial.println("--");  //줄바꿈 출력
  Serial.write(1);    //byte 쓰기 49
  analogWrite(PinNumber, 250);
  digitalWrite(PinNumber, HIGH);
  int drVal = digitalRead(PinNumber)
  float arVal = analogRead(PinNumber)
  if (Serial.available()){
    int inItVal  = Serial.read();
    if(inItVal == 49){ //1번 motor off
      //do 
    }
  }
}
