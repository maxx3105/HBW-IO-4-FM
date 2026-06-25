/* HBWChanIn -- Input-Channel für HBW-IO-12-FM
 *
 * Bildet das INPUT-Konfig-Layout aus hmw_io_12_fm.xml exakt ab:
 *   byte 0 bit 0 : INPUT_TYPE       (0 = SWITCH, 1 = PUSHBUTTON)
 *   byte 0 bit 1 : INPUT_LOCKED_n   (gespeichert invertiert; 1 = nicht gesperrt)
 *   byte 1       : LONG_PRESS_TIME  (× 0,1 s; 0xFF wird auf 1,0 s gemappt)
 *
 * Verhalten:
 *   PUSHBUTTON: kurzer Druck -> KeyEvent SHORT,
 *               langer Druck -> wiederholte KeyEvents LONG (alle long_press_time / 2)
 *   SWITCH:     jede Flanke -> KeyEvent SHORT (klassischer "an/aus"-Schalter)
 *
 * Eingang ist active-low mit internem Pull-Up.
 */

#ifndef HBWChanIn_h
#define HBWChanIn_h

#include <inttypes.h>
#include "HBWired.h"


// EEPROM-Layout des INPUT-Modus, exakt nach hmw_io_12_fm.xml
struct hbw_config_io_in {
  uint8_t input_type        : 1;   // 0 = SWITCH, 1 = PUSHBUTTON
  uint8_t input_locked_n    : 1;   // invertiert: 1 = nicht gesperrt
  uint8_t                   : 6;
  uint8_t long_press_time;         // × 0,1 s; 0xFF -> 1,0 s
};


class HBWChanIn : public HBWChannel {
  public:
    HBWChanIn(uint8_t _pin, hbw_config_io_in* _config);
    virtual void afterReadConfig();
    virtual void loop(HBWDevice*, uint8_t channel);
    virtual uint8_t get(uint8_t* data);

    enum InputType {
      IT_SWITCH     = 0,
      IT_PUSHBUTTON = 1
    };

  private:
    uint8_t            pin;
    hbw_config_io_in*  config;

    bool      lastReading;        // entprellte Eingangslage
    bool      rawLastReading;     // letzte Roh-Lesung (für Entprellung)
    uint32_t  lastChangeMillis;   // Zeit letzter Pegel-Wechsel
    uint32_t  pressedSince;       // Zeitpunkt letzter HIGH->LOW-Übergang (nur Pushbutton)
    uint32_t  lastSentLong;       // Zeit des letzten LONG-Events
    uint8_t   keyPressNum;        // Sende-Counter (6 Bit, vom Receiver erwartet)
    bool      longSent;           // wurde im aktuellen Druck schon LONG gesendet?

    static const uint16_t DEBOUNCE_MS = 80;

    inline bool isLocked() const {
      // gespeichert invertiert: input_locked_n=1 -> nicht gesperrt
      return config->input_locked_n == 0;
    }
    inline uint16_t longPressMs() const {
      // Default 1,0 s wenn EEPROM frisch (0xFF)
      uint8_t v = config->long_press_time;
      if (v == 0xFF) v = 10;
      return (uint16_t)v * 100;
    }

    bool readPin();
};

#endif
