/* HBWSwitchHM -- Output-Channel fuer HBW-IO-12-FM (1:1-Klon HMW-IO-12-FM)
 *
 * Bildet das ORIGINAL eQ-3 28-Byte-Peering von hmw_switch_ch_link ab:
 *   - 2-Byte HM-config-time (value_size 1.6, Faktoren 0.1/1/60 s, little-endian)
 *   - SHORT/LONG je 11 Byte: modeByte + 4x uint16 Zeiten + uint16 Jump-Table
 * Im Gegensatz zur loetmeister-HBWLinkSwitchAdvanced (20-Byte, 1-Byte-Zeiten).
 *
 * Wiederverwendet die bewaehrte State-Machine aus HBWSwitchAdvanced
 * (setOutput, Timer, Feedback, get, afterReadConfig). Ueberschrieben werden
 * nur set()/loop() sowie die zeitabhaengigen Schritte (2-Byte convertTime).
 *
 * Gehoert zusammen mit dem 28-Byte Link-Receiver HBWLinkSwitchHM.
 */

#ifndef HBWSwitchHM_h
#define HBWSwitchHM_h

#include <inttypes.h>
#include "HBWSwitchAdvanced.h"   // Basisklasse, State-Machine, convertTime(), DELAY_*


class HBWSwitchHM : public HBWSwitchAdvanced {
  public:
    HBWSwitchHM(uint8_t _pin, hbw_config_switch* _config)
      : HBWSwitchAdvanced(_pin, _config), myPin(_pin) {};

    virtual void set(HBWDevice* device, uint8_t length, uint8_t const * const data);
    virtual void loop(HBWDevice* device, uint8_t channel);
    virtual void afterReadConfig();   // Workaround: Pin 0 == NOT_A_PIN (siehe .cpp)

    // Datenlaenge, mit der HBWLinkSwitchHM device->set() aufruft:
    // [0]=modeByte, [1..8]=4x uint16 Zeiten (LE), [9..10]=uint16 jt (LE),
    // [11]=keyPressNum, [12]=sameLastSender
    static const uint8_t PEER_SET_LEN = 13;

  private:
    // Eine Peering-Haelfte (SHORT oder LONG) im Original-HM-Format, 2-Byte-Zeiten.
    // modeByte: bit0 ACTION_TYPE(ACTIVE), bit2 LONG_MULTIEXECUTE,
    //           bit4-5 TOGGLE_USE, bit6 OFF_TIME_MODE, bit7 ON_TIME_MODE
    //           (TIME_MODE: 0=MINIMAL, 1=ABSOLUTE)
    struct s_peering_hm {
      uint8_t  modeByte;
      uint16_t onDelayTime;
      uint16_t onTime;
      uint16_t offDelayTime;
      uint16_t offTime;
      uint16_t jt;            // 4x 3-bit: jtOnDelay | jtOn | jtOffDelay | jtOff
    };
    s_peering_hm stateParamHM = {};   // gesichertes Peering fuer Timer-Schritte in loop()
    uint8_t  myPin;                   // eigene Pin-Kopie (Basis-`pin` ist private)

    uint32_t getDelayForStateHM(uint8_t state, const s_peering_hm* p) const;
    uint8_t  getJumpTargetHM(uint8_t state, uint16_t jt) const;
    void     jumpToTargetHM(HBWDevice* device, const s_peering_hm* p);
    void     setStateHM(HBWDevice* device, uint8_t next, uint32_t delay,
                        const s_peering_hm* p, uint8_t deep = 0);
};

#endif
