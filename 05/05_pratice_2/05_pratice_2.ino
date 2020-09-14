#define PIN_LED 7
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.println("Hello World!");
  count = toggle = 0;
  digitalWrite(PIN_LED, toggle); // turn on LED.

  delay(1000);

  Serial.println("LED Blank start!");
}

void loop() {
  if (count <= 10) {
    ++count;
    toggle = toggle_state(toggle); // toggle LED value.
    Serial.println(toggle);
    digitalWrite(PIN_LED, toggle); // update LED status.
    delay(100); // wait for 1,000 milliseconds
  }
}

int toggle_state(int toggle) {
  return !toggle;
}
