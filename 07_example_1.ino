// LED가 연결된 핀 번호
const int ledPin = 7;

void setup() {
  // ledPin을 출력 모드로 설정
  pinMode(ledPin, OUTPUT);
  
  // 처음 1초 동안 LED를 켠다
  delay(1000);
  
  // 다음 1초 동안 LED를 5번 깜빡인다
  for (int i = 0; i < 5; i++) {
    digitalWrite(ledPin, HIGH); 
    delay(100);                  
    digitalWrite(ledPin, LOW);  
    delay(100);                  
  }
  
  // LED를 끈다
  digitalWrite(ledPin, HIGH);
}

void loop() {
  // 무한 루프 종료를 위해 빈 루프
}
