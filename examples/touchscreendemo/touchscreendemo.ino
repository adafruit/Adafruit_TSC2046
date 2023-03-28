#include <TSC2046.h>


// Differential resistance across Y- and Y+: 745 Ohms.
// Differential resistance across X- and X+: 400 Ohms.


Adafruit_TSC2046 touchscreen;

void setup() {
  Serial.begin(9600);
  touchscreen.begin(400);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {

  delay(200);
  digitalWrite(LED_BUILTIN, HIGH);

  if (touchscreen.isTouched()) {
    auto point = touchscreen.getPoint();

    Serial.print(point.xPercent());
    Serial.print("\t");
    Serial.print(point.yPercent());
    Serial.print("\t");
    Serial.print(point.z);
    Serial.print("\n");
  }

  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
}
