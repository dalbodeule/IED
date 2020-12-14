#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0


// configurable parameters
#define INTERVAL_IR 20
#define INTERVAL_SERVO 20
#define INTERVAL_SERIAL 100

#define _DIST_MIN 100
#define _DIST_MAX 450

#define _DUTY_MIN 1380
#define _DUTY_NEU (_DUTY_MIN + _DUTY_MAX) / 2
#define _DUTY_MAX 1710

#define _DIST_ALPHA 0.22

#define _SERVO_SPEED 20

#define IR_RAW_MIN 92
#define IR_RAW_MAX 396

// initialize global variables
Servo myservo;

unsigned long last_sampling_time;

boolean event_ir, event_servo, event_serial;

float dist_min, dist_max, dist_raw, dist_cali, dist_prev, dist_ema, alpha; // unit: mm

int duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180.0 * INTERVAL_SERVO / 1000.0;
int duty_target, duty_curr;


void setup() {
  // initialize Servo object
  myservo.attach(PIN_SERVO);

  // initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  
  // initialize serial port
  Serial.begin(57600);

  // initialize variables

  // Filter and Sensor related variables
  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX;
  
  dist_raw = dist_prev = dist_ema = 0.0;
  alpha = _DIST_ALPHA;

  last_sampling_time = 0;
}

void loop() {
  unsigned long current_time = millis();

  // event ticket
  if (current_time > last_sampling_time + INTERVAL_IR) {
    event_ir = true;
  } else {
    event_ir = false;
  }

  if (current_time > last_sampling_time + INTERVAL_SERVO) {
    event_servo = true;
  } else {
    event_servo = false;
  }

  if (current_time > last_sampling_time + INTERVAL_SERIAL) {
    event_serial = true;
  } else {
    event_serial = false;
  }


  // event handler (IR Sensor)
  if (event_ir) {
    event_ir = false;

    dist_raw = ir_distance();
    dist_ema = alpha * dist_raw + (1 - alpha) * dist_ema;
    dist_cali = 100 + 300.0 / (IR_RAW_MAX - IR_RAW_MIN) * (dist_ema - IR_RAW_MIN);

    if (dist_cali > 255) {
        duty_target = map(duty_target, 255, _DUTY_NEU, _DIST_MAX, _DUTY_MAX);
    } else if (dist_cali < 255) {
        duty_target = map(duty_target, 255, _DUTY_NEU, _DIST_MIN, _DUTY_MIN);
    } else {
        duty_target = _DUTY_NEU;
    }
  }

  if (event_servo) {
    event_servo = false;

    // calculate duty_curr
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    } else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }
    // update servo position
    myservo.writeMicroseconds(duty_curr);
  }

  if (event_serial) {
    event_serial = false;

    Serial.print("Min:100,raw:");
    Serial.print(dist_raw);

    Serial.print(",ema:");
    Serial.print(dist_ema);

    Serial.print(",cali:");
    Serial.print(dist_cali);

    Serial.print(",servo:");
    Serial.print(myservo.read());

    Serial.print("duty_target:");
    Serial.print(duty_target);

    Serial.print(",duty_curr:");
    Serial.print(duty_curr);

    Serial.println(",Max:400");
  }
  

  // update last sampling time
  last_sampling_time = millis();
}


float ir_distance() { // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}