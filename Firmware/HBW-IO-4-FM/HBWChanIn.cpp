#include "HBWChanIn.h"
#include <Arduino.h>


HBWChanIn::HBWChanIn(uint8_t _pin, hbw_config_io_in* _config)
  : pin(_pin), config(_config),
    lastReading(true), rawLastReading(true),
    lastChangeMillis(0), pressedSince(0), lastSentLong(0),
    keyPressNum(0), longSent(false)
{
  pinMode(pin, INPUT_PULLUP);
}


void HBWChanIn::afterReadConfig()
{
  pinMode(pin, INPUT_PULLUP);
  // Anfangszustand übernehmen, damit der erste Loop nicht sofort
  // einen "Druck" feuert.
  bool now = readPin();
  lastReading      = now;
  rawLastReading   = now;
  lastChangeMillis = millis();
  longSent         = false;
}


// liefert true = "nicht gedrückt" (HIGH), false = "gedrückt" (LOW, active-low)
bool HBWChanIn::readPin()
{
  return digitalRead(pin) != LOW;
}


uint8_t HBWChanIn::get(uint8_t* data)
{
  // Status für #S/#i: 0x00 = nicht aktiv, 0xC8 (200) = aktiv
  // (entspricht der Konvertierung boolean_integer threshold=1, true=200)
  *data = lastReading ? 0x00 : 0xC8;
  return 1;
}


void HBWChanIn::loop(HBWDevice* device, uint8_t channel)
{
  uint32_t now = millis();
  bool reading = readPin();

  // --- Software-Entprellung ---
  if (reading != rawLastReading) {
    rawLastReading   = reading;
    lastChangeMillis = now;
    return;
  }
  if ((uint32_t)(now - lastChangeMillis) < DEBOUNCE_MS) return;

  bool changed = (reading != lastReading);
  lastReading  = reading;

  if (isLocked()) {
    // gesperrt: keine Events senden, aber State weiterführen
    return;
  }

  if (config->input_type == IT_SWITCH) {
    // SWITCH: jede stabile Flanke -> ein SHORT-KeyEvent
    if (changed) {
      device->sendKeyEvent(channel, keyPressNum++, false /* longPress */);
    }
  }
  else { // PUSHBUTTON
    if (changed) {
      if (reading == false) {
        // gerade gedrückt
        pressedSince = now;
        longSent     = false;
      } else {
        // gerade losgelassen
        if (!longSent) {
          // war kurzer Druck
          device->sendKeyEvent(channel, keyPressNum++, false);
        }
        // bei langem Druck: kein "release"-Event nötig, der Empfänger
        // hat bereits LONG-Events erhalten.
      }
    }
    else if (reading == false) {
      // weiterhin gedrückt: prüfen ob LONG-Schwelle überschritten
      uint16_t longMs = longPressMs();
      if (!longSent) {
        if ((uint32_t)(now - pressedSince) >= longMs) {
          longSent     = true;
          lastSentLong = now;
          device->sendKeyEvent(channel, keyPressNum, true);
        }
      } else {
        // bereits "lang" -- alle longMs/2 wiederholen (typisches HBWired-Verhalten)
        if ((uint32_t)(now - lastSentLong) >= (uint32_t)(longMs / 2)) {
          lastSentLong = now;
          device->sendKeyEvent(channel, keyPressNum, true);
        }
      }
    }
  }
}
