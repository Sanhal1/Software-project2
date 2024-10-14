#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

// Configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 180.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360.0   // maximum distance to be measured (unit: mm)
#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficient to convert duration to distance

#define _EMA_ALPHA 0.5    // EMA weight of new sample (range: 0 to 1)

// Global variables
float dist_ema = _DIST_MAX; // unit: mm
unsigned long last_sampling_time; // unit: ms

Servo myservo;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  myservo.attach(PIN_SERVO); 
  myservo.write(0); // Start at 0 degrees

  Serial.begin(57600);
}

void loop() {
  if (millis() < (last_sampling_time + INTERVAL)) return;

  float dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // Apply range filter
  if (dist_raw == 0.0 || dist_raw > _DIST_MAX || dist_raw < _DIST_MIN) {
    dist_raw = dist_ema; // 유지하여 급격한 변화 방지
    digitalWrite(PIN_LED, HIGH); // LED OFF
  } else {
    digitalWrite(PIN_LED, LOW);  // LED ON      
  }

  // Apply EMA filter
  dist_ema = _EMA_ALPHA * dist_raw + (1 - _EMA_ALPHA) * dist_ema;

  // Calculate servo angle based on filtered distance
  int servo_angle;
  if (dist_ema <= _DIST_MIN) {
    servo_angle = 0;  // 0 degrees
  } else if (dist_ema >= _DIST_MAX) {
    servo_angle = 180; // 180 degrees
  } else {
    servo_angle = map(dist_ema, _DIST_MIN, _DIST_MAX, 0, 180);
  }

  myservo.write(servo_angle);

  // Serial output format for plotting
  Serial.print("Min:"); Serial.print(_DIST_MIN / 10.0, 1); // cm
  Serial.print(",dist:"); Serial.print(dist_raw / 10.0, 1); // cm
  Serial.print(",ema:"); Serial.print(dist_ema / 10.0, 1);  // cm
  Serial.print(",Servo:"); Serial.print(servo_angle);
  Serial.print(",Max:"); Serial.print(_DIST_MAX / 10.0, 1); // cm
  Serial.println("");

  last_sampling_time += INTERVAL;
}

// Get a distance reading from USS. Return value is in millimeters.
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
