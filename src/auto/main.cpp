
#include <Arduino.h>
#include <DallasTemperature.h>
#include <GLCD.h>
#include <OneWire.h>
#include <RFM69.h>
#include <HCSR04.h>

#define ONE_WIRE_BUS PA0

OneWire oneWire(PB0);
DallasTemperature tempSensor(&oneWire);

GLCD lcd;
RFM69 radio(PA4, PA0);

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

  long* echoDurationUs = HCSR04.measureMicroseconds(); // in microseconds
  float distanceRoundTripCm = echoDurationUs[0] * (331.3f + 0.606f * temp) /
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

struct Lane {
  int16_t positionXCm;
  int16_t positionYCm;
  uint8_t laneId;
  uint8_t direction;
  uint8_t maneuver;
  uint8_t connectsToLane;
  uint8_t signalGroupId;
};

struct MapMessage {
  float refLatitude;
  float refLongitude;
  uint16_t stationId;
  uint16_t laneWidthCm;
  uint8_t protocolVersion;
  uint8_t messageType;
  uint8_t regionId;
  uint8_t intersectionId;
  uint8_t revision;
  uint8_t laneCount;
  Lane lanes[4];
};

struct SpatMessage {
  uint16_t timeStampMs;
  uint16_t phaseEndsAt;
  uint16_t stationId;
  uint8_t protocolVersion;
  uint8_t messageType;
  uint8_t regionId;
  uint8_t intersectionId;
  uint8_t revision;
  uint8_t signalGroupId;
  uint8_t phase;
};

// Setup
void setup() {
  radio.initialize(RF69_433MHZ, 2, 255); // node id, network id
  radio.setPowerLevel(0); // Low power for safety

  tempSensor.begin();
  tempSensor.setWaitForConversion(false);

  byte echoPins[] = {PB8};
  HCSR04.begin(PB7, echoPins, 1);

  lcd.begin();
  lcd.setColor(255, 255, 255);
  lcd.setTextSize(2);
}

// Main loop
void loop() {
  tempSensor.requestTemperatures();
  float distanceCm = getAverageDistance();
  if (distanceCm != -1.0f) {
    lcd.setCursor(20, 10);
    lcd.print("Dist: ");
    lcd.print((int)distanceCm);
    lcd.println("cm  ");
  }
  float temp = tempSensor.getTempCByIndex(0);
  lcd.setCursor(200, 10);
  lcd.print(temp);
  lcd.println(" C");

  if (radio.receiveDone()) {
    if (radio.DATALEN == sizeof(SpatMessage)) {
      SpatMessage spat;
      memcpy(&spat, (const void *)radio.DATA, sizeof(spat));

      int remainingMs = (spat.phaseEndsAt - spat.timeStampMs + 60000) % 60000;

      lcd.setCursor(20, 50);
      lcd.print("Phase: ");
      lcd.print(spat.phase);
      lcd.print("   ");
      lcd.setCursor(20, 70);
      lcd.print("Ends: ");
      lcd.print(remainingMs / 1000.0);
      lcd.print("s     ");
    } else if (radio.DATALEN == sizeof(MapMessage)) {
      MapMessage map;
      memcpy(&map, (const void *)radio.DATA, sizeof(map));

      lcd.setCursor(20, 100);
      lcd.print("Lat: ");
      lcd.print(map.refLatitude, 4);
      lcd.print("   ");
      lcd.setCursor(20, 120);
      lcd.print("Lon: ");
      lcd.print(map.refLongitude, 4);
      lcd.print("   ");

      for (int i = 0; i < map.laneCount && i < 4; i++) {
        lcd.setCursor(20, 150 + i * 20);
        lcd.print("L");
        lcd.print((int)map.lanes[i].laneId);
        lcd.print(": ");
        lcd.print(map.lanes[i].positionXCm);
        lcd.print(",");
        lcd.print(map.lanes[i].positionYCm);
        lcd.print("   ");
      }
    }
  }
}