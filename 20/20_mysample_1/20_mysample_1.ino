#include <Servo.h>
#define PIN_SERVO 10

Servo myservo;

int servoDur[4] = { 1510, 1810, servoDur[0], 1210 };

void setup() {
  myservo.attach(PIN_SERVO);

   // while (1) { myservo.write(90); }
   
   while (1) { myservo.writeMicroseconds(servoDur[0]); }
   
   // while (1) { myservo.writeMicroseconds(1380); }
}

void loop() {
    for(int i = 0; i < 4; i++) {
      myservo.writeMicroseconds(servoDur[i]);
      delay(1000);
    }
}
