#include <Servo.h>
#define PIN_SERVO 10

Servo myservo;

unsigned long beforetime = 0;
unsigned long aftertime = 0;

void setup() {
  Serial.begin(57600);
  Serial.println("=== Servo dps measure start! ===");
  Serial.println("if you type start, measure start!");
  
  myservo.attach(PIN_SERVO);
  myservo.write(0);
  delay(1000);
}

void loop() {
  if (Serial.available() > 0 && Serial.readStringUntil('\n') == "start") {
    Serial.println("Servo move start!");
    
    beforetime = millis();
    myservo.write(180);
  } else if (myservo.read() == 180) {
    aftertime = millis();

    Serial.print(aftertime - beforetime);
    Serial.println(" sec / 180 degree");
    Serial.println("\n===Servo dps measure end!===");

    myservo.write(0);
  }
}
