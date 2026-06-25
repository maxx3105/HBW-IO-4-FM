#include "HBWSwitchHM.h"
#include <Arduino.h>
#include <string.h>   // memcpy / memset


// Workaround fuer NOT_A_PIN == 0 (Arduino): HBWSwitchAdvanced::afterReadConfig()
// hat `if (pin == NOT_A_PIN) return;` -> ein OUTPUT-Kanal auf digitalem Pin 0
// (z.B. PB0) wuerde sonst NIE als Ausgang initialisiert und schaltet nicht. Hier
// holen wir die Init fuer Pin 0 nach (Basis-Logik repliziert); alle anderen Pins
// erledigt die Basis ganz normal.
void HBWSwitchHM::afterReadConfig()
{
  HBWSwitchAdvanced::afterReadConfig();   // macht alles AUSSER Pin 0
  if (myPin != NOT_A_PIN)  return;        // != Pin 0 -> Basis war zustaendig

  if (currentState == UNKNOWN_STATE) {
    digitalWrite(myPin, config->n_inverted ? LOW : HIGH);   // Ausgang, Zustand "aus"
    pinMode(myPin, OUTPUT);
    currentState = JT_OFF;
  }
  else {  // bei Re-Read Zustand nicht zuruecksetzen, nur Ausgang nachfuehren
    if (currentState == JT_ON || currentState == JT_OFFDELAY)
      digitalWrite(myPin, LOW ^ config->n_inverted);
    else if (currentState == JT_OFF || currentState == JT_ONDELAY)
      digitalWrite(myPin, HIGH ^ config->n_inverted);
  }
}


// Verzoegerung/Zeit fuer einen Zustand aus dem Peering holen -- 2-Byte HM-Zeit
uint32_t HBWSwitchHM::getDelayForStateHM(uint8_t state, const s_peering_hm* p) const
{
  if (p == NULL)  return getDefaultDelay(state);   // Basis: ON/OFF -> INFINITE, sonst NO

  uint16_t v = 0;
  switch (state) {
    case JT_ONDELAY:  v = p->onDelayTime;  break;
    case JT_ON:       v = p->onTime;       break;
    case JT_OFFDELAY: v = p->offDelayTime; break;
    case JT_OFF:      v = p->offTime;      break;
    default: break;
  }
  return convertTime(v);   // uint16_t -> 2-Byte-Ueberladung (Faktor in Bit 14-15)
}


// Sprungziel fuer den aktuellen Zustand aus der 16-Bit Jump-Table (4x 3-bit)
uint8_t HBWSwitchHM::getJumpTargetHM(uint8_t state, uint16_t jt) const
{
  switch (state) {
    case JT_ONDELAY:  return  jt        & 0x07;
    case JT_ON:       return (jt >> 3)  & 0x07;
    case JT_OFFDELAY: return (jt >> 6)  & 0x07;
    case JT_OFF:      return (jt >> 9)  & 0x07;
    default: break;
  }
  return JT_NO_JUMP_IGNORE_COMMAND;
}


// Sprung gemaess Jump-Table beim Tastendruck (entspricht jumpToTarget der Basis)
void HBWSwitchHM::jumpToTargetHM(HBWDevice* device, const s_peering_hm* p)
{
  uint8_t next = getJumpTargetHM(currentState, p->jt);
  if (next == JT_NO_JUMP_IGNORE_COMMAND)  return;

  uint32_t nextDelay = getDelayForStateHM(next, p);

  // ON/OFF im "minimal"-Modus: laufenden, kuerzeren Timer nicht verlaengern
  if (next == currentState && (next == JT_ON || next == JT_OFF) && nextDelay < DELAY_INFINITE) {
    bool minimal = (next == JT_ON) ? !(p->modeByte & 0x80) : !(p->modeByte & 0x40);
    if (minimal) {
      uint32_t currentDelay = getRemainingStateChangeTime();
      if (currentDelay == DELAY_INFINITE || currentDelay > nextDelay)  return;
    }
  }
  setStateHM(device, next, nextDelay, p);
}


// Zustandswechsel + ggf. Folgezustand/Timer (entspricht setState der Basis,
// aber mit 2-Byte-Zeiten und eigenem Peering-Speicher)
void HBWSwitchHM::setStateHM(HBWDevice* device, uint8_t next, uint32_t delay,
                             const s_peering_hm* p, uint8_t deep)
{
  if (next == JT_NO_JUMP_IGNORE_COMMAND || deep >= 4)  return;

  // gueltiges Peering sichern, damit loop() spaetere Timer-Schritte rechnen kann
  if (p != NULL && deep == 0 && p != &stateParamHM) {
    memcpy(&stateParamHM, p, sizeof(stateParamHM));
  }
  stopStateChangeTime();

  bool stateOk = true;
  if (currentState != next) {
    stateOk = switchState(device, next);   // Basis: setOutput + Feedback + currentState
  }
  if (stateOk) {
    if (delay == DELAY_NO) {
      // sofort weiter zum naechsten zyklischen Zustand (ONDELAY->ON->OFFDELAY->OFF)
      uint8_t  autoNext  = getNextState();
      uint32_t autoDelay = getDelayForStateHM(autoNext, &stateParamHM);
      setStateHM(device, autoNext, autoDelay, &stateParamHM, deep + 1);
    }
    else if (delay != DELAY_INFINITE) {
      startNewStateChangeTime(delay);
    }
    // Feedback ausloesen, wenn Timer gestartet/gestoppt wurde
    if (oldStateTimerRunningState != stateTimerRunning) {
      setFeedback(device, config->logging);
    }
  }
  oldStateTimerRunningState = stateTimerRunning;
}


// set(): Peering-Event (HM-Layout) oder Direktbefehl der CCU
void HBWSwitchHM::set(HBWDevice* device, uint8_t length, uint8_t const * const data)
{
  if (length == PEER_SET_LEN) {
    // Peering-Event vom HBWLinkSwitchHM (28-Byte HM-Layout, bereits zerlegt)
    uint8_t modeByte = data[0];
    if ((modeByte & 0x01) == 0)  return;            // ACTION_TYPE INACTIVE -> ignorieren

    uint8_t keyNum           = data[11];
    bool    sameLastSender   = data[12];
    bool    longMultiexecute = (modeByte & 0x04);
    if (lastKeyNum == keyNum && sameLastSender && !longMultiexecute)
      return;   // wiederholter Long-Press ohne LONG_MULTIEXECUTE

    s_peering_hm p;
    p.modeByte     = modeByte;
    p.onDelayTime  = (uint16_t)data[1] | ((uint16_t)data[2]  << 8);
    p.onTime       = (uint16_t)data[3] | ((uint16_t)data[4]  << 8);
    p.offDelayTime = (uint16_t)data[5] | ((uint16_t)data[6]  << 8);
    p.offTime      = (uint16_t)data[7] | ((uint16_t)data[8]  << 8);
    p.jt           = (uint16_t)data[9] | ((uint16_t)data[10] << 8);

    jumpToTargetHM(device, &p);
    lastKeyNum = keyNum;   // fuer Wiederholungserkennung merken
  }
  else {
    // Direktbefehl der CCU (STATE 0/200, INSTALL_TEST toggle >200):
    // bewaehrtes Basisverhalten nutzen, aber laufende HM-Timerkette verwerfen
    memset(&stateParamHM, 0, sizeof(stateParamHM));
    HBWSwitchAdvanced::set(device, length, data);
  }
}


// loop(): Timer-getriebene Zustandsfortschaltung + Feedback
void HBWSwitchHM::loop(HBWDevice* device, uint8_t channel)
{
  unsigned long now = millis();

  if (stateTimerRunning && (now - lastStateChangeTime > stateChangeWaitTime)) {
    stopStateChangeTime();
    uint8_t  nextState = getNextState();
    uint32_t delay     = getDelayForStateHM(nextState, &stateParamHM);
    setStateHM(device, nextState, delay, &stateParamHM);
  }

  checkFeedback(device, channel);
}
