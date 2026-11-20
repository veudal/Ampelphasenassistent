#include <Arduino.h>
#include <GLCD.h>
#include <RFM69.h>

#define GREEN_PIN PA8
#define YELLOW_PIN PA9
#define RED_PIN PA10

#define POTENTIOMETER_PIN PB0

RFM69 radio(PA4, PA0);
GLCD lcd;
short direction = -1;
int SEND_INTERVAL_MS = 100;

const int RECEIVER_ID = 2;
const short powerLevel = 0;

const int MAP_EVERY_N_MESSAGES = 5;

const int PROTOCOL_VERSION = 2;
const int STATION_ID = 1001;
const int REGION_ID = 0;
const int INTERSECTION_ID = 1;
const int REVISION = 1;
const int SIGNAL_GROUP_ID = 1;

const int LANE_WIDTH_CM = 350;
const float REF_LATITUDE = 50.0000;
const float REF_LONGITUDE = 9.0000;

enum LightState { RED, YELLOW_RED, YELLOW, GREEN };
LightState state;

enum MessageType { MESSAGE_SPAT = 4, MESSAGE_MAP = 5 };

enum SignalPhase {
  PHASE_RED = 3,
  PHASE_RED_YELLOW = 4,
  PHASE_GREEN = 6,
  PHASE_YELLOW = 8
};

enum LaneDirection { LANE_INGRESS, LANE_EGRESS };

enum Maneuver { MANEUVER_NONE, MANEUVER_STRAIGHT };

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

const short STATE_DELAYS[4] = {6000, 1000, 3000, 6000};

Lane makeLane(int laneId, int direction, int maneuver, int xCm, int yCm,
              int connectsToLane, int signalGroupId) {
  Lane lane;
  lane.laneId = laneId;
  lane.direction = direction;
  lane.maneuver = maneuver;
  lane.positionXCm = xCm;
  lane.positionYCm = yCm;
  lane.connectsToLane = connectsToLane;
  lane.signalGroupId = signalGroupId;
  return lane;
}

void broadcastMapMessage() {
  MapMessage map;
  map.protocolVersion = PROTOCOL_VERSION;
  map.messageType = MESSAGE_MAP;
  map.stationId = STATION_ID;
  map.regionId = REGION_ID;
  map.intersectionId = INTERSECTION_ID;
  map.revision = REVISION;
  map.refLatitude = REF_LATITUDE;
  map.refLongitude = REF_LONGITUDE;
  map.laneWidthCm = LANE_WIDTH_CM;
  map.laneCount = 4;

  map.lanes[0] = makeLane(1, LANE_INGRESS, MANEUVER_STRAIGHT, -175, 500, 4,
                          SIGNAL_GROUP_ID);
  map.lanes[1] = makeLane(2, LANE_EGRESS, MANEUVER_NONE, 175, 500, 0, 0);
  map.lanes[2] = makeLane(3, LANE_INGRESS, MANEUVER_STRAIGHT, 175, -500, 2,
                          SIGNAL_GROUP_ID);
  map.lanes[3] = makeLane(4, LANE_EGRESS, MANEUVER_NONE, -175, -500, 0, 0);

  radio.send(RECEIVER_ID, &map, sizeof(MapMessage));
}

void broadcastSpatMessage(unsigned long timeRemainingMs) {
  SpatMessage spat;
  spat.protocolVersion = PROTOCOL_VERSION;
  spat.messageType = MESSAGE_SPAT;
  spat.stationId = STATION_ID;
  spat.regionId = REGION_ID;
  spat.intersectionId = INTERSECTION_ID;
  spat.revision = REVISION;
  spat.timeStampMs = millis() % 60000;
  spat.signalGroupId = SIGNAL_GROUP_ID;

  switch (state) {
  case RED:
    spat.phase = PHASE_RED;
    break;
  case YELLOW_RED:
    spat.phase = PHASE_RED_YELLOW;
    break;
  case YELLOW:
    spat.phase = PHASE_YELLOW;
    break;
  case GREEN:
    spat.phase = PHASE_GREEN;
    break;
  }

  spat.phaseEndsAt = (millis() + timeRemainingMs) % 60000;
  radio.send(RECEIVER_ID, &spat, sizeof(SpatMessage));
}

void applyLightState(bool red, bool yellow, bool green) {
  digitalWrite(RED_PIN, red);
  digitalWrite(YELLOW_PIN, yellow);
  digitalWrite(GREEN_PIN, green);

  unsigned long start = millis();
  unsigned long duration = STATE_DELAYS[state];
  unsigned long lastMap = 0;

  while (millis() - start < duration) {
    unsigned long remaining = duration - (millis() - start);

    SEND_INTERVAL_MS =
        analogRead(POTENTIOMETER_PIN) * 4 + 10; // 10 - 4102ms range

    broadcastSpatMessage(remaining);

    if (lastMap == 0 || millis() - lastMap >= 1000) {
      delay(50); // Important: without delay, the map message cannot be processed on the receiver's site, only one message at a time.
      broadcastMapMessage();
      lastMap = millis();
    }

    delay(SEND_INTERVAL_MS);
  }
}

void setup() {
  //   lcd.begin();
  //   lcd.setTextSize(2.5);
  radio.initialize(RF69_433MHZ, 1, 255); // node id, network id
  radio.setPowerLevel(powerLevel);       // low power for safety
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(POTENTIOMETER_PIN, INPUT);
}

void loop() {
  switch (state) {
  case RED:
    applyLightState(true, false, false);
    direction = -direction;
    break;
  case YELLOW_RED:
    applyLightState(true, true, false);
    state = (LightState)(state + direction);
    break;
  case YELLOW:
    applyLightState(false, true, false);
    if (direction == -1)
      state = (LightState)(state + direction);
    break;
  case GREEN:
    applyLightState(false, false, true);
    direction = -direction;
    break;
  }
  state = (LightState)(state + direction);
}