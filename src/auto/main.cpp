
#include <Arduino.h>
#include <DallasTemperature.h>
#include <GLCD.h>
#include <OneWire.h>
#include <RFM69.h>
#include <Ultrasonic.h>

#define ONE_WIRE_BUS PA0

OneWire oneWire(PA0);
DallasTemperature tempSensor(&oneWire);

Ultrasonic sensor(PB7, PB8);
GLCD lcd;
RFM69 radio(PA4, PA0);
unsigned long phaseEndLocal = 0; // local millis() when the current phase ends
int lastPhase = 0;

// Print state over serial (optional)
void sendState(int state, int remainingTime) {
  Serial.print("State: ");
  Serial.print(state);
  Serial.print(" | Remaining: ");
  Serial.println(remainingTime);
}

float calculateDistance() {
  float temp = tempSensor.getTempCByIndex(0);
  if (temp == DEVICE_DISCONNECTED_C)
    temp = 21;

  float echoDurationUs = sensor.measurePulse(); // in microseconds
  float distanceRoundTripCm = echoDurationUs * (331.3f + 0.606f * temp) /
                              10000.0f; // Calculate distance with temperature
                                        // due to sound speed changes
  return distanceRoundTripCm / 2.0f;
}

float getAverageDistance() {
  const int averageOf = 5;
  static float readings[averageOf] = {};
  static int index = 0;
  static int count = 0;
  static float sum = 0;

  float distanceCm = calculateDistance();
  // if (distanceCm <= 0 || distanceCm > 400)
  //     return -1.0f;

  sum -= readings[index];
  readings[index] = distanceCm;
  sum += distanceCm;

  index = (index + 1) % averageOf;

  if (count < averageOf)
    count++;

  return sum / count;
}

struct SpatMessage {
  int protocolVersion;
  int messageType;
  int stationId;
  int regionId;
  int intersectionId;
  int revision;
  int timeStampMs;
  int signalGroupId;
  int phase;
  int phaseEndsAt;
};

// Setup
void setup() {
  radio.initialize(RF69_433MHZ, 2, 255); // node id, network id
  radio.setPowerLevel(0);                // Low power for safety
  tempSensor.begin();
  tempSensor.setWaitForConversion(false);
  sensor.begin();
  lcd.begin();
  lcd.setColor(255, 255, 255);
  lcd.setTextSize(3.5);
}

// Main loop
void loop() {
  tempSensor.requestTemperatures();
  float distanceCm = getAverageDistance();
  if (distanceCm != -1.0f) {
    lcd.setCursor(20, 20);
    lcd.print("Distance: ");
    lcd.print((int)distanceCm);
    lcd.println("cm  ");
  }

  if (radio.receiveDone()) {
    if (radio.DATALEN == sizeof(SpatMessage)) {
      SpatMessage spat;
      memcpy(&spat, (const void *)radio.DATA, sizeof(spat));

      int remainingMs = (spat.phaseEndsAt - spat.timeStampMs + 60000) % 60000;

      lcd.setCursor(20, 60);
      lcd.print("Phase: ");
      lcd.print(spat.phase);
      lcd.print("   ");
      lcd.setCursor(20, 100);
      lcd.print("Ends: ");
      lcd.print(remainingMs);
      lcd.print("ms     ");
    }
  }

  delay(1);
}