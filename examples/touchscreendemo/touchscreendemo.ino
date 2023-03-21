#include <TSC2046.h>


// 749Ω

Adafruit_TSC2046 touchscreen = Adafruit_TSC2046(2113);

void setup() {
  touchscreen.begin(10, 500);
}

void loop() {
  auto point = touchscreen.getPoint();

  Serial.print(point.x);
  Serial.print("\t");
  Serial.println(point.y);

  delay(200);
}
