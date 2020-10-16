#define TIMER_INTERRUPT_DEBUG      0

#define USE_TIMER_1     true
#define USE_TIMER_2     true
#define USE_TIMER_3     false
#define USE_TIMER_4     false
#define USE_TIMER_5     false

#include "TimerInterrupt.h"

// Arduino pin assignment
#define PIN_LED 9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define TIMER1_INTERVAL_MS    25 // sampling interval (unit: ms)
#define TIMER2_INTERVAL_MS    25 // serial interval (unit: ms)
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300 // maximum distance to be measured (unit: mm)

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_center, dist_raw, dist_prev; // unit: mm
float scale; // used for pulse duration to distance conversion
volatile bool event_measure, event_serial;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW); 
  pinMode(PIN_ECHO,INPUT);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  dist_center = _DIST_MIN + _DIST_MAX / 2.0;
  timeout = (TIMER1_INTERVAL_MS / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = dist_prev = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;
  event_measure = event_serial = false;

// initialize serial port
  Serial.begin(57600);

// set up timer interrupt
  ITimer1.init();
  if(!ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS, TimerHandler1)) {
    Serial.println("Can't set ITimer1.");
    while(1) {}
  }

  ITimer2.init();
  if(!ITimer2.attachInterruptInterval(TIMER2_INTERVAL_MS, TimerHandler2)) {
    Serial.println("Can't set ITimer2.");
    while(1) {}
  }
  
}
  
void loop() {
  if(event_measure) {
    // get a distance reading from the USS
    dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);
    if(dist_raw == 0.0) {
      dist_raw = dist_prev;
    }
    else {
      dist_prev = dist_raw;
    }
    // turn on the LED if the distance is between dist_min and dist_max
    if(dist_raw < dist_min || dist_raw > dist_max) {
      analogWrite(PIN_LED, 255);
    }
    else {
      analogWrite(PIN_LED, 0);

//      analogWrite(PIN_LED, abs(dist_raw - dist_center) / (dist_max - dist_center) * 255);
    }
    event_measure = false;
  }

  if(event_serial) {
  // output the read value to the serial port
    Serial.print("Min:0,");
    Serial.print("raw:");
    Serial.print(dist_raw);
    Serial.print(",");
    Serial.println("Max:400");
    event_serial = false;
  }
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  reading = pulseInLong(ECHO, HIGH, timeout) * scale; // unit: mm
  if(reading < dist_min || reading > dist_max) reading = 0.0; // return 0 when out of range.
  return reading;
  // Pulse duration to distance conversion example (target distance = 17.3m)
  // - round trip distance: 34.6m
  // - expected pulse duration: 0.1 sec, or 100,000us
  // - pulseIn(ECHO, HIGH, timeout) * 0.001 * 0.5 * SND_VEL
  //           = 100,000 micro*sec * 0.001 milli/micro * 0.5 * 346 meter/sec
  //           = 100,000 * 0.001 * 0.5 * 346 * micro * sec * milli * meter
  //                                           ----------------------------
  //                                           micro * sec
  //           = 100 * 173 milli*meter = 17,300 mm = 17.3m
  // pulseIn() returns microseconds.
}

void TimerHandler1(void)
{
   event_measure = true;
}

void TimerHandler2(void)
{
   event_serial = true;
}
