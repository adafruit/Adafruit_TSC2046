#include <TSC2046.h>


// Differential resistance across Y- and Y+: 745 Ohms.
// Differential resistance across X- and X+: 400 Ohms.


Adafruit_TSC2046 touchscreen = Adafruit_TSC2046(2113);

void setup() {
  Serial.begin(9600);
  touchscreen.begin(10, 400);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {

  delay(200);
  digitalWrite(LED_BUILTIN, HIGH);

  auto point = touchscreen.getPoint();


  Serial.print(point.raw_x);
  Serial.print("\t");
  Serial.print(point.raw_y);
  Serial.print("\t");
  Serial.print(point.raw_z);
  Serial.print("\n");

  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
}
