#include <Arduino.h>

const int PROTOCOL_VERSION = 1;
const int TIME_WRAP_MS = 60000; // timestamps wrap around every minute
const int MAX_LANES = 4;

enum MessageType { MESSAGE_SPAT = 4, MESSAGE_MAP = 5 };

enum SignalPhase {
  RED = 3,
  RED_YELLOW = 4,
  GREEN = 6,
  YELLOW = 8
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
  Lane lanes[MAX_LANES];
};