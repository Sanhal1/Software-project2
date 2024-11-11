#include <Servo.h>

// Arduino pin assignment
#define PIN_IR    A0       // IR sensor at Pin A0
#define PIN_LED   9
#define PIN_SERVO 10

#define _DUTY_MIN 1000     // servo full clock-wise position (0 degree)
#define _DUTY_NEU 1500     // servo neutral position (90 degree)
#define _DUTY_MAX 2000     // servo full counter-clockwise position (180 degree)

#define _DIST_MIN  100.0   // minimum distance 100mm
#define _DIST_MAX  250.0   // maximum distance 250mm

#define EMA_ALPHA  0.5     // for EMA Filter

#define LOOP_INTERVAL 20   // Loop Interval (unit: msec)

Servo myservo;
unsigned long last_loop_time = 0; // unit: msec

float dist_prev = _DIST_MIN;
float dist_ema = _DIST_MIN;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

  Serial.begin(1000000);    // 1,000,000 bps
}

void loop() {
  unsigned long time_curr = millis();
  int duty;
  float a_value, dist_raw;

  // wait until next event time
  if (time_curr < (last_loop_time + LOOP_INTERVAL))
    return;
  last_loop_time += LOOP_INTERVAL;

  a_value = analogRead(PIN_IR);
  dist_raw = ((6762.0 / (a_value - 9.0)) - 4.0) * 10.0 - 60.0;

  // Range Filter (only allow distances within _DIST_MIN ~ _DIST_MAX)
  if (dist_raw >= _DIST_MIN && dist_raw <= _DIST_MAX) {
    digitalWrite(PIN_LED, HIGH); // Turn on LED if within range
  } else {
    digitalWrite(PIN_LED, LOW); // Turn off LED if out of range
    dist_raw = (dist_raw < _DIST_MIN) ? _DIST_MIN : _DIST_MAX; // Clamp value to min/max range
  }

  // EMA filter to smooth distance readings
  dist_ema = EMA_ALPHA * dist_raw + (1 - EMA_ALPHA) * dist_prev;
  dist_prev = dist_ema;

  // Map distance to servo duty cycle manually (without using map() function)
  duty = _DUTY_MIN + ((dist_ema - _DIST_MIN) / (_DIST_MAX - _DIST_MIN)) * (_DUTY_MAX - _DUTY_MIN);

  // Write to servo
  myservo.writeMicroseconds(duty);

  // Serial output for plotter
  Serial.print("_DUTY_MIN:");  Serial.print(_DUTY_MIN);
  Serial.print("_DIST_MIN:");  Serial.print(_DIST_MIN);
  Serial.print(",IR:");        Serial.print(a_value);
  Serial.print(",dist_raw:");  Serial.print(dist_raw);
  Serial.print(",ema:");       Serial.print(dist_ema);
  Serial.print(",servo:");     Serial.print(duty);
  Serial.print(",_DIST_MAX:"); Serial.print(_DIST_MAX);
  Serial.print(",_DUTY_MAX:"); Serial.print(_DUTY_MAX);
  Serial.println("");
}
