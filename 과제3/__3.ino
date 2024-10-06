// Arduino pin assignment
#define PIN_LED  7
#define PIN_TRIG 12   // sonar sensor TRIGGER
#define PIN_ECHO 13   // sonar sensor ECHO

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25      // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficient to convert duration to distance

unsigned long last_sampling_time;   // unit: msec

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);  // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);   // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 
  
  // initialize serial port
  Serial.begin(57600);
}

void loop() { 
  float distance;

  // wait until next sampling time (polling)
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  distance = USS_measure(PIN_TRIG, PIN_ECHO); // read distance

  int brightness = 255; // Default LED off (max value)

  if (distance >= 300.0 || distance <= 100.0) {
      brightness = 255; // LED OFF (min brightness) for distance 100mm and 300mm
  } else if (distance == 200.0) {
      brightness = 0; // Max brightness at 200mm
  } else if (distance == 150.0 || distance == 250.0) {
      brightness = 128; // 50% brightness at 150mm and 250mm
  } else if (distance > 150.0 && distance < 200.0) {
      // Brightness increases linearly from 150mm (50% brightness) to 200mm (max brightness)
      brightness = map(distance, 150, 200, 128, 0);  
  } else if (distance > 200.0 && distance < 250.0) {
      // Brightness decreases linearly from 200mm (max brightness) to 250mm (50% brightness)
      brightness = map(distance, 200, 250, 0, 128);  
  } else if (distance > 250.0 && distance < 300.0) {
      // Brightness decreases linearly from 250mm (50% brightness) to 300mm (min brightness)
      brightness = map(distance, 250, 300, 128, 255);
  }

  analogWrite(PIN_LED, brightness);  // Control LED brightness

  // Output the distance and brightness value to the serial port
  Serial.print("Distance: "); Serial.print(distance);
  Serial.print(" mm, Brightness: "); Serial.println(brightness);

  // Update last sampling time
  last_sampling_time = millis();
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
