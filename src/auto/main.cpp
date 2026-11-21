#include <Arduino.h>
#include <DallasTemperature.h>
#include <GLCD.h>
#include <HCSR04.h>
#include <Messages.h>
#include <OneWire.h>
#include <RFM69.h>

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

int remainingMs = -1;
SignalPhase phase;

const float MIN_DISTANCE_CM = 10;  // closer than this: ignore
const float MIN_APPROACH_CMPS = 1; // slower than this: not really approaching
const float MAX_SPEED_CMPS = 50;
const long MARGIN_MS = 500; // safety buffer around phase changes

unsigned long spatReceivedMs = 0; // set when a SPaT message arrives

enum RecommendedAction {
  NONE,       // no signal data
  KEEP,       // current speed reaches the light while green
  ACCELERATE, // speed up to make it while green
  DECELERATE  // slow down to arrive at green
};

float calculateDistance(long echoDuration) {
  // Calculate distance with temperature due to sound speed changes
  float roundTripCm = echoDuration * (331.3f + 0.606f * temperature) / 10000.0f;
  if (roundTripCm > 300) // Ignore invalid readings (e.g. obstacles in front of
                         // the HCS R04 sensor)
    roundTripCm = 0;

  return roundTripCm / 2.0f;
}

float updateDistanceAndSpeed() {
  float rawCm = calculateDistance(HCSR04.measureMicroseconds()[0]);

  if (distanceCm < 0) { // first reading
    distanceCm = rawCm;
    lastMs = millis();
    return distanceCm;
  }

  unsigned long currentMs = millis();
  float newDistance = distanceCm + ALPHA * (rawCm - distanceCm);
  float dt = (currentMs - lastMs) / 1000.0f; // time diff in seconds

  speedCmPerS += ALPHA * ((distanceCm - newDistance) / dt - speedCmPerS);
  distanceCm = newDistance;
  lastMs = currentMs;

  return distanceCm;
}

RecommendedAction getRecommendedAction() {
  if (remainingMs < 0 || distanceCm < MIN_DISTANCE_CM)
    return NONE;

  long remaining = remainingMs - (long)(millis() - spatReceivedMs);

  // Standing still or very slow: accelerate on green, otherwise "="
  if (speedCmPerS < MIN_APPROACH_CMPS)
    return (phase == GREEN && remaining > MARGIN_MS) ? ACCELERATE : KEEP;

  if (remaining <= 0)
    return KEEP; // waiting for new SPaT: no change

  float arrivalMs = distanceCm / speedCmPerS * 1000.0f;

  if (phase == GREEN) {
    // Make it on current speed
    if (arrivalMs + MARGIN_MS <= remaining)
      return KEEP;

    // Not enough time left to make it
    if (remaining <= MARGIN_MS)
      return DECELERATE;

    // Speed needed to arrive before green ends
    float requiredCmps = distanceCm / ((remaining - MARGIN_MS) / 1000.0f);
    return (requiredCmps <= MAX_SPEED_CMPS) ? ACCELERATE : DECELERATE;
  }

  // Not green: remaining = time until the light turns green (assumption)
  if (arrivalMs >= remaining + MARGIN_MS)
    return KEEP;     // arrive after it turned green
  return DECELERATE; // would arrive on red/yellow
}

RecommendedAction stableAction() {
  static RecommendedAction shown = NONE, pending = NONE;
  static unsigned long pendingSince = 0;
  RecommendedAction raw = getRecommendedAction();
  if (raw != pending) {
    pending = raw;
    pendingSince = millis();
  }
  if (millis() - pendingSince >= 300)
    shown = pending;
  return shown;
}

void displayRecommendedAction(RecommendedAction action) {
  lcd.setCursor(225, 90);
  lcd.setTextSize(8);

  switch (action) {
  case KEEP:
    lcd.setColor(0, 190, 255);
    lcd.write("=");
    break;
  case ACCELERATE:
    lcd.setColor(70, 255, 120);
    lcd.write(24);
    break;
  case DECELERATE:
    lcd.setColor(255, 100, 100);
    lcd.write(25);
    break;
  default:
    lcd.setColor(180, 180, 180);
    lcd.write("?");
    break;
  }
  // Reset
  lcd.setColor(255, 255, 255);
  lcd.setTextSize(2);
}

void displayDistanceAndSpeed() {
  lcd.setCursor(20, 10);
  lcd.print("Dist: ");
  lcd.print((int)distanceCm);
  lcd.println("cm  ");

  lcd.setCursor(20, 30);
  lcd.print("Speed: ");
  lcd.print((int)speedCmPerS);
  lcd.println("cm/s  ");
}

void displayTemperature() {
  lcd.setCursor(220, 10);
  lcd.print(temperature);
  lcd.println(" C");
}

void displaySpatMessage(const SpatMessage &spat) {
  remainingMs =
      (spat.phaseEndsAt - spat.timeStampMs + TIME_WRAP_MS) % TIME_WRAP_MS;
  spatReceivedMs = millis();
  phase = (SignalPhase)spat.phase;

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
  // Antenna length: (299,792,458 / (868 * 10^6) / 4 = 0,0863m = 8,63cm
  radio.initialize(RF69_868MHZ, NODE_ID, NETWORK_ID); // node id, network id
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
  getRecommendedAction();

  displayTemperature();
  displayDistanceAndSpeed();
  displayRecommendedAction(stableAction());

  receiveRadioMessage();
}
