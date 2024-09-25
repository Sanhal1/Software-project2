const int ledPin = 7; // LED가 연결된 핀 번호

void setup() {
  pinMode(ledPin, OUTPUT); // 핀을 출력 모드로 설정
}

void loop() {
  // LED 밝기를 0에서 255까지 증가
  for (int brightness = 0; brightness <= 255; brightness++) {
    analogWrite(ledPin, brightness); // LED 밝기 설정
    delay(1000); // 10ms 대기
  }

  // LED 밝기를 255에서 0까지 감소
  for (int brightness = 255; brightness >= 0; brightness--) {
    analogWrite(ledPin, brightness); // LED 밝기 설정
    delay(1000); // 10ms 대기
  }
}
