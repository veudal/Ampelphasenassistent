#include <Arduino.h>
#include <DallasTemperature.h>
#include <GLCD.h>
#include <HCSR04.h>
#include <Messages.h>
#include <OneWire.h>
#include <RFM69.h>

// Hardware
OneWire oneWire(PB0);
DallasTemperature tempSensor(&oneWire);
GLCD lcd;
RFM69 radio(PA4, PA0);

const int NODE_ID = 2;
const int NETWORK_ID = 255;
const short POWER_LEVEL = 0;

const float DEFAULT_TEMP_C = 21;
float temperature = DEFAULT_TEMP_C;

const float ALPHA = 0.2f;
float distanceCm = -1;
float speedCmPerS = 0; // cm/s, positive values mean:  approaching
unsigned long lastMs = 0;

enum class RecommendedAction {
  NONE,       // no signal data
  KEEP,       // current speed reaches the light while green
  ACCELERATE, // speed up to make it while green
  DECELERATE  // slow down to arrive at green
};

float calculateDistance(long echoDuration) {
  // Calculate distance with temperature due to sound speed changes
  float roundTripCm = echoDuration * (331.3f + 0.606f * temperature) / 10000.0f;
  if (roundTripCm > 300) // Ignore invalid readings (e.g. obstacles in front of the HCS R04 sensor)
    roundTripCm = 0;

  return roundTripCm / 2.0f;
}

float updateDistanceAndSpeed() {
  float rawCm = calculateDistance(HCSR04.measureMicroseconds()[0]);
  unsigned long currentMs = millis();

  if (distanceCm < 0) { // first reading
    distanceCm = rawCm;
    lastMs = currentMs;
    return distanceCm;
  }

  float newDistance = distanceCm + ALPHA * (rawCm - distanceCm);
  float dt = (currentMs - lastMs) / 1000.0f; // time diff in seconds

  speedCmPerS += ALPHA * ((distanceCm - newDistance) / dt - speedCmPerS);
  distanceCm = newDistance;
  lastMs = currentMs;

  return distanceCm;
}

void displayRecommendedAction() {}

void displayDistanceAndSpeed() {
  // if (distanceCm != -1.0f) {
  lcd.setCursor(20, 10);
  lcd.print("Dist: ");
  lcd.print((int)distanceCm);
  lcd.println("cm  ");
  // }

  lcd.setCursor(20, 30);
  lcd.print("Speed: ");
  lcd.print((int)speedCmPerS);
  lcd.println("cm/s  ");
}

void displayTemperature() {
  lcd.setCursor(180, 10);
  lcd.print(temperature);
  lcd.println(" C");
}

void displaySpatMessage(const SpatMessage &spat) {
  int remainingMs =
      (spat.phaseEndsAt - spat.timeStampMs + TIME_WRAP_MS) % TIME_WRAP_MS;

  lcd.setCursor(20, 50);
  lcd.print("Phase: ");
  lcd.print(spat.phase);
  lcd.print("   ");
  lcd.setCursor(20, 70);
  lcd.print("Ends: ");
  lcd.print(remainingMs / 1000.0);
  lcd.print("s     ");
}

void displayMapMessage(const MapMessage &map) {
  lcd.setCursor(20, 100);
  lcd.print("Lat: ");
  lcd.print(map.refLatitude, 4);
  lcd.print("   ");
  lcd.setCursor(20, 120);
  lcd.print("Lon: ");
  lcd.print(map.refLongitude, 4);
  lcd.print("   ");

  for (int i = 0; i < map.laneCount && i < MAX_LANES; i++) {
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

void receiveRadioMessage() {
  if (radio.receiveDone()) {
    if (radio.DATALEN == sizeof(SpatMessage)) {
      SpatMessage spat;
      memcpy(&spat, (const void *)radio.DATA, sizeof(spat));
      displaySpatMessage(spat);
    } else if (radio.DATALEN == sizeof(MapMessage)) {
      MapMessage map;
      memcpy(&map, (const void *)radio.DATA, sizeof(map));
      displayMapMessage(map);
    }
  }
}

void updateTemperature() {
  temperature = tempSensor.getTempCByIndex(0);

  if (temperature == DEVICE_DISCONNECTED_C)
    temperature = DEFAULT_TEMP_C;

  tempSensor.requestTemperatures();
}

// Setup
void setup() {
  radio.initialize(RF69_433MHZ, NODE_ID, NETWORK_ID); // node id, network id
  radio.setPowerLevel(POWER_LEVEL);                   // Low power for safety

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
  updateTemperature();
  updateDistanceAndSpeed();
  displayDistanceAndSpeed();
  displayTemperature();
  receiveRadioMessage();
}
