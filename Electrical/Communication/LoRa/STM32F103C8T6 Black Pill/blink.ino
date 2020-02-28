// An example blink program on the STM32F103C8T6 Black Pill

/*
   Note that pins on the Black Pill follow the format P{bus letter}{pin number}
   For example here the LED is on bus B and pin 12, so we write PB12
   The Black Pill pins are labled with the bus letter and pin number,
   so if you want to use B11, you would write PB11
*/

int LED = PB12;

void setup() {
  pinMode(LED, OUTPUT);
}

void loop() {
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(1000);
}
