#include <Adafruit_AHTX0.h>

Adafruit_AHTX0 aht;
const int MOISTURE_PIN = A0; // Pin connected to your soil sensor
const int DELAY_TIME = 5000; // Read every 5 seconds (adjust as needed)


void setup() {
  Serial.begin(115200);
  pinMode(MOISTURE_PIN, INPUT);
  if (!aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
    while (1)
      delay(10);
  }
}

void loop() {
  sensors_event_t humidity, temp;
  int rawValue = analogRead(MOISTURE_PIN);
  int moisturePercent = map(rawValue, 1023, 0, 0, 100); 
  aht.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data
  Serial.print(moisturePercent);
  Serial.print(",");
  Serial.print(temp.temperature);
  Serial.print(",");
  Serial.println(humidity.relative_humidity);
  delay(5000);
}