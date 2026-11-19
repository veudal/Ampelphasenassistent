#pragma once
// Funk-Schnittstelle (RFM69, 868 MHz). Bewusst als einfache Funktionen, keine Klasse.
#include "protocol.h"
namespace Link {
bool begin(uint8_t nodeId);
// Senden. Mit ack=true (Unicast) wird die Umlaufzeit in µs zurückgegeben, sonst 0.
uint32_t send(const SpatMsg& m, bool ack);
// Liefert true, wenn ein gültiges neues Paket empfangen wurde (und beantwortet ACK-Anfragen).
bool poll(SpatMsg& m, int16_t& rssi);
}
