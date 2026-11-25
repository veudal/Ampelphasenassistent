#include <Arduino.h>
#include <UTFT.h>
#include <Messages.h>
#include <RFM69.h>

#define GREEN_PIN PA8
#define YELLOW_PIN PA9
#define RED_PIN PA10

#define POTENTIOMETER_PIN PB0

RFM69 radio(PA4, PA0);
UTFT lcd;
SignalPhase phase = RED;

const int NODE_ID = 1;
const int NETWORK_ID = 255;
const short POWER_LEVEL = 0;

int sendIntervalMs = 100;

const int STATION_ID = 1001;
const int REGION_ID = 0;
const int INTERSECTION_ID = 1;
const int REVISION = 1;
const int SIGNAL_GROUP_ID = 1;

const int LANE_WIDTH_CM = 350;
const float REF_LATITUDE = 50.0000;
const float REF_LONGITUDE = 9.0000;

const long RED_DURATION_MS = 7000;
const long RED_YELLOW_DURATION_MS = 1000;
const long YELLOW_DURATION_MS = 3000;
const long GREEN_DURATION_MS = 9000;


unsigned long getPhaseDuration() {
  switch (phase) {
  case RED:
    return 6000;
  case RED_YELLOW:
    return 1000;
  case YELLOW:
    return 3000;
  case GREEN:
    return 6000;
  }
  return 0;
}

Lane makeLane(int laneId, int laneDirection, int maneuver, int xCm, int yCm,
              int connectsToLane, int signalGroupId) {
  Lane lane;
  lane.laneId = laneId;
  lane.direction = laneDirection;
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

  radio.send(RF69_BROADCAST_ADDR, &map, sizeof(MapMessage));
}

void broadcastSpatMessage(unsigned long timeRemainingMs) {
  SpatMessage spat;
  spat.protocolVersion = PROTOCOL_VERSION;
  spat.messageType = MESSAGE_SPAT;
  spat.stationId = STATION_ID;
  spat.regionId = REGION_ID;
  spat.intersectionId = INTERSECTION_ID;
  spat.revision = REVISION;
  spat.timeStampMs = millis() % TIME_WRAP_MS;
  spat.signalGroupId = SIGNAL_GROUP_ID;
  spat.phase = phase;
  spat.phaseEndsAt = (millis() + timeRemainingMs) % TIME_WRAP_MS;

  radio.send(RF69_BROADCAST_ADDR, &spat, sizeof(SpatMessage));
}

void setLights() {
  digitalWrite(RED_PIN, phase == RED || phase == RED_YELLOW);
  digitalWrite(YELLOW_PIN, phase == YELLOW || phase == RED_YELLOW);
  digitalWrite(GREEN_PIN, phase == GREEN);
}

void applyPhase() {
  setLights();
  unsigned long start = millis();
  unsigned long duration = getPhaseDuration();
  unsigned long lastMapBroadcast = 0;

  while (millis() - start < duration) {
    unsigned long remaining = duration - (millis() - start);

    // Update interval (10 - 4102ms range)
    sendIntervalMs = analogRead(POTENTIOMETER_PIN) * 4 + 10;

    broadcastSpatMessage(remaining);

    if (lastMapBroadcast == 0 || millis() - lastMapBroadcast >= 1000) {
      // Important: without delay, the map message cannot be processed on the
      // receiver's site, only one message at a time.
      delay(100);
      broadcastMapMessage();
      lastMapBroadcast = millis();
    }

    // Switch to the next phase must be guaranteed
    delay(min(sendIntervalMs, remaining));
  }
}

SignalPhase getNextPhase() {
  switch (phase) {
  case RED:
    return RED_YELLOW;
  case RED_YELLOW:
    return GREEN;
  case GREEN:
    return YELLOW;
  case YELLOW:
    return RED;
  }
  return RED;
}

void setup() {
  // Antenna length: (299,792,458 / (868 * 10^6) / 4 = 0,0863m = 8,63cm 
  radio.initialize(RF69_868MHZ, NODE_ID, NETWORK_ID); // node id, network id
  radio.setPowerLevel(POWER_LEVEL);                   // low power for safety
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(POTENTIOMETER_PIN, INPUT);
}

void loop() {
  applyPhase();
  phase = getNextPhase();
}
