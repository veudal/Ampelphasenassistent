#include "Link.h"
#include <Arduino.h>
#include <SPI.h>
#include <RFM69.h>

#if defined(ARDUINO_ARCH_STM32F1)
static SPIClass spiRf(2);                    // Maple/STM32duino-Kern wie im Laborskript
#else
static SPIClass spiRf(PB15, PB14, PB13);     // offizieller STM32-Kern: MOSI, MISO, SCK von SPI2
#endif
static RFM69 radio(PIN_RF_NSS, PIN_RF_IRQ, &spiRf);

bool Link::begin(uint8_t nodeId) {
  pinMode(PIN_RF_RESET, OUTPUT); digitalWrite(PIN_RF_RESET, LOW);   // wie Laborskript
  bool ok = radio.initialize(RF69_868MHZ, nodeId, NETWORK_ID);
  if (IS_RFM69HCW) radio.setHighPower();                            // nur für RFM69HW/HCW!
  return ok;
}

uint32_t Link::send(const SpatMsg& m, bool ack) {
  if (!ack) { radio.send(0, &m, sizeof(m), false); return 0; }      // Broadcast
  uint32_t t0 = micros();
  radio.send(NODE_ID_AUTO, &m, sizeof(m), true);                    // Unicast an das Fahrzeug
  uint32_t deadline = millis() + 50;
  while (millis() < deadline) if (radio.ACKReceived(NODE_ID_AUTO)) return micros() - t0;
  return 0;
}

bool Link::poll(SpatMsg& m, int16_t& rssi) {
  bool got = false;
  if (radio.receiveDone()) {
    if (radio.DATALEN == sizeof(SpatMsg)) {
      memcpy(&m, (const void*)radio.DATA, sizeof(m));
      if (m.version == 1 && m.msgType == 1) { rssi = radio.RSSI; got = true; }   // Formatprüfung
    }
    if (radio.ACKRequested()) radio.sendACK();
  }
  return got;
}
