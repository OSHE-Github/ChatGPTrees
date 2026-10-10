#include <Wire.h>
#include "Adafruit_LTR390.h"

Adafruit_LTR390 ltr = Adafruit_LTR390();
bool sensorActive = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Wire.begin();
  
  // Prevents the microcontroller from freezing if you yank the sensor wires out
  #if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(1000, true);
  #endif
}

void loop() {
  // 1. INITIALIZATION STATE
  if (!sensorActive) {
    if (!ltr.begin()) {
      Serial.println("ERR|SENSOR_MISSING");
      delay(1000);
      return;
    }
    // Sensor successfully found. Apply settings ONCE.
    ltr.setResolution(LTR390_RESOLUTION_18BIT);
    ltr.setGain(LTR390_GAIN_3);
    sensorActive = true;
  }

  // 2. READING STATE
  if (sensorActive) {
    // Ping the I2C address (0x53) to see if the sensor was physically unplugged
    Wire.beginTransmission(0x53);
    if (Wire.endTransmission() != 0) {
      // Sensor did not acknowledge the ping (disconnected)
      sensorActive = false;
      Serial.println("ERR|SENSOR_MISSING");
      delay(1000);
      return;
    }

    // Switch to UV mode and read
    ltr.setMode(LTR390_MODE_UVS);
    delay(100); // 100ms conversion time allows the 18-bit reading to finish safely
    uint32_t uvs = ltr.readUVS();

    // Switch to Ambient Light mode and read
    ltr.setMode(LTR390_MODE_ALS);
    delay(100); 
    uint32_t als = ltr.readALS();

    // Send the raw data to the Python script
    Serial.print("DATA|");
    Serial.print(uvs);
    Serial.print("|");
    Serial.println(als);
  }

  delay(800); // Adjusted to 800ms to maintain a ~1 second total polling rate
}