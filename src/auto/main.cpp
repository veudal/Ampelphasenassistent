#include <Arduino.h>
#include <DallasTemperature.h>
#include <HCSR04.h>
#include <Messages.h>
#include <OneWire.h>
#include <RFM69.h>
#include <UTFT.h>

OneWire oneWire(PB0);
DallasTemperature tempSensor(&oneWire);
RFM69 radio(PA4, PA3);
UTFT lcd(ILI9325D_16, PD11, PD5, PD7, PC1);

extern uint8_t BigFont[];
extern uint8_t Various_Symbols_32x32[];

const int NODE_ID = 2;
const int NETWORK_ID = 255;
const short POWER_LEVEL = 0;

const float DEFAULT_TEMP_C = 21;
float temperature = DEFAULT_TEMP_C;

const float ALPHA = 0.2f;
float distanceCm = -1;
float speedCmPerS = 0; // cm/s, positive values mean:  approaching
unsigned long lastMs = 0;

long remainingMs = -1;
int stopLineOffsetCm = 0;
SignalPhase phase;

const float MIN_DISTANCE_CM = 10;  // closer than this: ignore
const float MIN_APPROACH_CMPS = 1; // slower than this: not really approaching
const float MAX_SPEED_CMPS = 50;
const long MARGIN_MS = 500; // safety buffer around phase changes

unsigned long spatReceivedMs = 0; // set when a SPaT message arrives
const unsigned long MAX_MSG_AGE_MS = 2000;

enum RecommendedAction {
  NONE,       // no signal data
  KEEP,       // current speed reaches the light while green
  ACCELERATE, // speed up to make it while green
  DECELERATE  // slow down to arrive at green
};

void resetFont() {
  lcd.setFont(BigFont);
  lcd.setColor(255, 255, 255);
}

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
  float targetDistanceCm = max(0.0f, distanceCm - stopLineOffsetCm);

  // Standing still or very slow: accelerate on green, otherwise "="
  if (speedCmPerS < MIN_APPROACH_CMPS)
    return (phase == GREEN && remaining > MARGIN_MS) ? ACCELERATE : KEEP;

  if (remaining <= 0)
    return KEEP; // waiting for new SPaT

  float arrivalMs = targetDistanceCm / speedCmPerS * 1000.0f;

  if (phase == GREEN) {
    // Make it on current speed
    if (arrivalMs + MARGIN_MS <= remaining)
      return KEEP;

    // Not enough time left to make it
    if (remaining <= MARGIN_MS)
      return DECELERATE;

    // Speed needed to arrive before green ends
    float requiredCmps = targetDistanceCm / ((remaining - MARGIN_MS) / 1000.0f);
    return (requiredCmps <= MAX_SPEED_CMPS) ? ACCELERATE : DECELERATE;
  }

  // Not green: remaining = time until the light turns green (assumption)
  if (arrivalMs >= remaining + MARGIN_MS)
    return KEEP; // arrive after it turned green

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
  static RecommendedAction previous = NONE;
  static bool firstDraw = true;

  // Don't redraw if nothing changed
  if (!firstDraw && action == previous)
    return;

  firstDraw = false;
  previous = action;

  lcd.setColor(0, 0, 0);
  lcd.fillRect(200, 60, 319, 150);
  lcd.setFont(Various_Symbols_32x32);

  switch (action) {
  case KEEP: {
    lcd.setColor(0, 190, 255);
    lcd.print("L", 260, 100); // Happy smiley
    break;
  }

  case ACCELERATE:
    lcd.setColor(70, 255, 120);
    lcd.print("Q", 260, 100); // Up arrow
    break;

  case DECELERATE:
    lcd.setColor(255, 100, 100);
    lcd.print("P", 260, 100); // Down arrow
    break;

  default: {
    lcd.setColor(180, 180, 180);
    lcd.print("z", 260, 100); // Hourglass symbol (waiting for signal)
    break;
  }
  }
  resetFont();
}
void displayDistanceAndSpeed() {
  lcd.print(String("Dist: ") + (int)distanceCm + "cm  ", 20, 10);
  lcd.print(String("Speed: ") + (int)speedCmPerS + "cm/s   ", 20, 30);
}

void displayTemperature() { lcd.print(String(temperature, 1) + " C", 220, 10); }

void displaySpatMessage(const SpatMessage &spat) {
  spatReceivedMs = millis();
  phase = (SignalPhase)spat.phase;

  lcd.print(String("Phase: ") + (int)spat.phase + "   ", 20, 50);
  lcd.print(String("Ends: ") + String(remainingMs / 1000.0, 2) + "s     ", 20,
            70);
}

bool isTimestampFresh(uint32_t ts) {
  static uint32_t lastTs = 0;
  int32_t delta = ts - lastTs; // newer = positive

  // Duplicate or slightly older than the newest message: drop
  if (delta <= 0 && delta > -(int32_t)MAX_MSG_AGE_MS)
    return false;

  lastTs = ts;
  return true;
}

void displayMapMessage(const MapMessage &map) {
  lcd.print(String("Lat: ") + String(map.refLatitude, 4), 20, 100);
  lcd.print(String("Lon: ") + String(map.refLongitude, 4), 20, 120);

  for (int i = 0; i < map.laneCount && i < MAX_LANES; i++) {
    lcd.print(String("L") + (int)map.lanes[i].laneId + ": " +
                  (int)map.lanes[i].positionXCm + "," +
                  (int)map.lanes[i].positionYCm + "   ",
              20, 150 + i * 20);
  }

  if (map.laneCount > 0) {
    stopLineOffsetCm = map.lanes[0].stopLineOffsetCm;
  }
}

void receiveRadioMessage() {
  if (!radio.receiveDone())
    return;

  if (radio.DATALEN == sizeof(SpatMessage)) {
    SpatMessage spat;
    memcpy(&spat, (const void *)radio.DATA, sizeof(spat));
    if (spat.protocolVersion != PROTOCOL_VERSION ||
        spat.messageType != MESSAGE_SPAT)
      return;

    if (!isTimestampFresh(spat.timeStampMs))
      return;

    remainingMs = (long)(spat.phaseEndsAt - spat.timeStampMs);
    displaySpatMessage(spat);
  } else if (radio.DATALEN == sizeof(MapMessage)) {
    MapMessage map;
    memcpy(&map, (const void *)radio.DATA, sizeof(map));
    if (map.protocolVersion != PROTOCOL_VERSION ||
        map.messageType != MESSAGE_MAP)
      return;

    displayMapMessage(map);
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

  tempSensor.begin();
  tempSensor.setWaitForConversion(false);

  byte echoPins[] = {PB8};
  HCSR04.begin(PB7, echoPins, 1);

  lcd.InitLCD();

  // Rotate screen by 180 degrees
  digitalWrite(PD7, LOW);
  lcd.LCD_Write_COM_DATA(0x01, 0x0000); // mirror horizontally
  lcd.LCD_Write_COM_DATA(0x60, 0x2700); // mirror vertically
  digitalWrite(PD7, HIGH);

  lcd.setColor(0, 0, 0);
  lcd.fillRect(0, 0, lcd.getDisplayXSize() - 1, lcd.getDisplayYSize() - 1);

  resetFont();
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
