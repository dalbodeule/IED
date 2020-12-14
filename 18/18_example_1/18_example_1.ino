// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  
// initialize serial port
  Serial.begin(57600);

}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();

  if (raw_dist >= 200 && raw_dist <= 350) {
    digitalWrite(PIN_LED, 0); 
  } else {
    digitalWrite(PIN_LED, 1);
  }
  Serial.print("min:0,max:500,dist:");
  Serial.println(raw_dist);
  delay(20);
}
