//*******************************************************************
//
// HBW-IO-4-FM
//
// Homematic Wired Homebrew Hardware (Mini-Modul, 4 Kanaele).
// Jeder Kanal einzeln per CCU als INPUT (Taster/Schalter) oder
// OUTPUT (Schalter mit erweitertem Peering) konfigurierbar.
//
// Bewusst 1:1-Klon des Original eQ-3 HMW-IO-4-FM -> nutzt die ORIGINAL-XML
// hmw_io_4_fm.xml (HMW_DEVICETYPE = 0x10 / 16 dec), KEIN eigenes XML.
//
// Hardware: https://github.com/maxx3105/HBW-IO-4-FM  (Platine1 = ATmega328P @ 16 MHz,
//           Platine2 = 24V->5V Netzteil). RS485 (MAX487E) am Hardware-UART.
// Library:  https://github.com/maxx3105/HBWired (Fork)
//
// Das EEPROM-Layout dieses Sketches entspricht 1:1 der XML-Beschreibung:
//   0x01           logging_time
//   0x02 - 0x05    central_address
//   0x06 bit0      direct_link_deactivate
//   0x07           BEHAVIOUR-Bits (4 Bits, 1=OUTPUT, 0=INPUT; oberes Nibble ungenutzt)
//   0x08 - 0x0F    Input-Channel-Konfig (4 × 2 Byte)
//   0x10 - 0x17    Output-Channel-Konfig (4 × 2 Byte)
//   0x18 ...       Output-Peerings  (30 × 28 Byte) -> hmw_switch_ch_link
//                  (Original-HM-Format, 2-Byte-Zeiten -> HBWSwitchHM/HBWLinkSwitchHM)
//   0x360 ...      Input-Peerings   (26 × 6 Byte)  -> hmw_input_ch_link, endet 0x3FB
//                  -> KEIN Overlap mit der Busadresse (0x3FC..0x3FF), anders als beim 12er
//
//*******************************************************************

#define HARDWARE_VERSION 0x00   // muss = supported_types index1 im XML (=0)! Sonst HMW-Generic
#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION 0x0303 // XML verlangt index2 >= 0x0303 (GE) -> meldet 3.03.
                                // Per Build-Flag -DFIRMWARE_VERSION=0x0304 ueberschreibbar (Update-Test).
#endif
#define HMW_DEVICETYPE   0x10   // 16 dec = supported_types index0 im XML (hmw_io_4_fm.xml)

#define NUM_CHANNELS     4

// Output-Peerings: hmw_switch_ch_link, count=30, address_step=28, address_start=0x18
#define NUM_LINKS_OUT          30
#define LINKADDRESSSTART_OUT   0x0018

// Input-Peerings: hmw_input_ch_link, count=26, address_step=6, address_start=0x360
#define NUM_LINKS_IN           26
#define LINKADDRESSSTART_IN    0x0360


// HBWired Core + benötigte Channel-/Link-Klassen
#include <HBWired.h>
#include <HBWSwitchAdvanced.h>
#include <HBWLinkKey.h>
#include <HBW_eeprom.h>
#include <EEPROM.h>

#include "HBWChanIn.h"
#include "HBWSwitchHM.h"        // Output-Channel im Original-28-Byte-HM-Peering-Format
#include "HBWLinkSwitchHM.h"    // passender 28-Byte Link-Receiver

// Pin- und Hardware-Konfiguration (ATmega328P / Platine1)
#include "HBW-IO-4-FM_config.h"


// HBWired definiert POWERSAVE() fuer ATmega328P/PB und RP2040 bereits selbst.
// Fallback fuer andere AVRs (Sleep-Mode 0 = IDLE -- millis()/Timer laufen weiter).
#ifndef POWERSAVE
  #if defined(__AVR__)
    #include <avr/sleep.h>
    #define POWERSAVE() do { set_sleep_mode(SLEEP_MODE_IDLE); sleep_mode(); } while (0)
  #else
    #define POWERSAVE() do { } while (0)
  #endif
#endif


// ---- EEPROM-Strukturen exakt nach XML ----------------------------------------
// hbw_config_io_in (Input-Modus, 2 Byte/Kanal ab 0x08) ist in HBWChanIn.h definiert.
//
// 2 Byte pro Output-Kanal, address_step=2 ab 0x10:
//   byte 0 bit 0 : LOGGING (0 = OFF, 1 = ON)
//   byte 1       : reserviert
// Wir verwenden HBWSwitchAdvanced - dessen hbw_config_switch nutzt die ersten Bits
// identisch (logging:1, output_unlocked:1, n_inverted:1). Bits 1 und 2 sind in der
// XML nicht definiert und bleiben nach Reset=1 (= unlocked, not inverted) = Default.
//
// WICHTIG ggü. dem 12er: BEHAVIOUR ist hier nur 1 Byte (0x07, 4 Bits genutzt),
// daher beginnt die Input-Konfig schon bei 0x08 (nicht 0x09).

struct hbw_config {
  uint8_t  logging_time;                    // 0x01
  uint32_t central_address;                 // 0x02 - 0x05
  uint8_t  direct_link_deactivate : 1;      // 0x06 bit 0
  uint8_t                         : 7;
  uint8_t  behaviour;                       // 0x07, Bit pro Kanal (4 genutzt)
  hbw_config_io_in  inCfg [NUM_CHANNELS];   // 0x08 - 0x0F
  hbw_config_switch outCfg[NUM_CHANNELS];   // 0x10 - 0x17
} hbwconfig;


// ---- Channel-Array -----------------------------------------------------------

HBWChannel* channels[NUM_CHANNELS];


// ---- Pinout-Array (siehe HBW-IO-4-FM_config.h) ------------------------------
static const uint8_t ioPin[NUM_CHANNELS] = {
  IO1, IO2, IO3, IO4
};


// ---- BEHAVIOUR -> Kanal-Objekte (ohne Reboot) --------------------------------
// BEHAVIOUR-Bit je Kanal: 1 = OUTPUT (HBWSwitchHM), 0 = INPUT (HBWChanIn).
// Aendert die CCU die Bits, werden NUR die betroffenen Kanal-Objekte zur Laufzeit
// getauscht -- KEIN Reboot. (Ein Reboot macht das Geraet kurz unerreichbar ->
// "Geraetekommunikation gestoert" auf Kanal 0 nach JEDER Umstellung.) Aufruf aus
// setup() (Erstanlage) und HBWIODevice::afterReadConfig() (Aenderung); HBWired
// ruft direkt danach channels[i]->afterReadConfig() und initialisiert die neuen.
// Hinweis: HBWChanIn/HBWSwitchHM haben keine dynamischen Member -> delete ueber
// HBWChannel* ist hier unbedenklich (free() nutzt die Allokationsgroesse).
uint8_t g_currentBehaviour = 0xFF;   // Sentinel = noch nichts instanziiert

void applyBehaviour(uint8_t beh) {
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    bool wantOut = ((beh >> i) & 0x01) != 0;
    if (channels[i] != NULL) {
      bool haveOut = ((g_currentBehaviour >> i) & 0x01) != 0;
      if (wantOut == haveOut) continue;     // Kanal hat schon den richtigen Typ
      delete channels[i];
    }
    if (wantOut) channels[i] = new HBWSwitchHM(ioPin[i], &(hbwconfig.outCfg[i]));
    else         channels[i] = new HBWChanIn (ioPin[i], &(hbwconfig.inCfg[i]));
  }
  g_currentBehaviour = beh;
}


// ---- LinkSender-Pointer (fuer CCU-KEY-Sim) ----------------------------------
// Wir merken uns den Tasten-LinkSender, damit ein KEY-Sim der CCU ("Tastendruck
// simulieren"/Anlerntest) wie ein ECHTER lokaler Tastendruck die eigene
// Peering-Tabelle abarbeiten kann (siehe HBWIODevice::receiveKeyEvent unten).
typedef HBWLinkKey<NUM_LINKS_IN, LINKADDRESSSTART_IN> HBWLinkKeyType;
HBWLinkKeyType* g_linkKey = NULL;


// ---- Device-Subklasse mit afterReadConfig() ---------------------------------

class HBWIODevice : public HBWDevice {
  public:
    HBWIODevice(uint8_t _devicetype, uint8_t _hardware_version, uint16_t _firmware_version,
                Stream* _rs485, uint8_t _txen,
                uint8_t _configSize, void* _config,
                uint8_t _numChannels, HBWChannel** _channels,
                Stream* _debugstream,
                HBWLinkSender* _ls = NULL, HBWLinkReceiver* _lr = NULL)
      : HBWDevice(_devicetype, _hardware_version, _firmware_version,
                  _rs485, _txen, _configSize, _config, _numChannels, _channels,
                  _debugstream, _ls, _lr) { };

    virtual void afterReadConfig() {
      // gerätespezifische Defaults nach EEPROM-Read.
      // XML-Default LOGGING_TIME = 2.0 s (factor 10 -> Geräte-Wert 20).
      // WICHTIG: nicht nur im RAM defaulten! Nach einem Werksreset (factoryReset
      // loescht das Config-EEPROM auf 0xFF und ruft readConfig->afterReadConfig)
      // steht 0x01 = 0xFF, was die CCU als 25.5 s (= 0xFF/10) liest und anzeigt.
      // Daher den Default PERSISTENT ins EEPROM schreiben -- update() schreibt nur,
      // wenn nicht ohnehin 20, also genau einmal nach dem Reset. logging_time liegt
      // an EEPROM-Adresse 0x01 (HBWDevice::readConfig: readEEPROM(config, 0x01, ..)).
      if (hbwconfig.logging_time == 0xFF) {
        hbwconfig.logging_time = 20;
        EepromPtr->update(0x01, 20);
      }

      // BEHAVIOUR-Aenderung der CCU SOFORT umsetzen (Kanaltyp tauschen, kein
      // Reboot). behaviour==0xFF ist die GUELTIGE Konfig "alle 4 OUTPUT" und darf
      // NICHT als "frisch" verworfen werden -- frisch nur, wenn die CCU die
      // central_address noch nicht gesetzt hat (gleiche Logik wie in setup()).
      uint8_t beh;
      if (hbwconfig.central_address == 0xFFFFFFFF) {
        // ungepairt/nach Werksreset: XML-Default alle INPUT -- und zwar auch
        // PERSISTENT ins EEPROM (0x07), sonst liest die CCU das geloeschte 0xFF
        // und zeigt alle 4 Kanaele faelschlich als OUTPUT (XML: Bit 1 = OUTPUT).
        // update() schreibt nur einmal; sobald die CCU konfiguriert hat
        // (central != 0xFFFFFFFF), wird BEHAVIOUR hier NIE angefasst.
        beh = 0x00;
        EepromPtr->update(0x07, 0xF0);   // Kanaele 1-4 -> INPUT (oberes Nibble ungenutzt, wie CCU)
        hbwconfig.behaviour = 0xF0;      // RAM mitziehen
      }
      else {
        beh = hbwconfig.behaviour;       // CCU-Konfig (0xFF hier = gueltig "alle OUTPUT")
      }
      applyBehaviour(beh);
    };

    // KEY-Event-Empfang. Ein echtes Sensor-Ereignis eines ANDEREN Geraets traegt
    // dessen Adresse als Absender -> ganz normal an den LinkReceiver weiterreichen.
    // Ein KEY-Sim der CCU ("Tastendruck simulieren"/Anlerntest) traegt dagegen die
    // ZENTRALEN-Adresse als Absender; interne Direktverknuepfungen speichern aber
    // die EIGENE Geraeteadresse als Sensor -> der Sim wuerde nie matchen. Damit der
    // CCU-Testknopf genauso wirkt wie ein physischer Tastendruck, jagen wir den Sim
    // durch den eigenen LinkSender: der liest die eigene Peering-Tabelle (Eingangs-
    // Links) und loest interne Verknuepfungen LOKAL sowie externe ADRESSIERT aus --
    // exakt wie HBWChanIn::loop() -> device->sendKeyEvent() beim echten Taster.
    // (Kein Broadcast noetig - die CCU hat den Sim ja selbst ausgeloest.)
    // Keine Endlos-Rekursion: der LinkSender ruft fuer interne Links wieder
    // receiveKeyEvent() auf, dann aber mit der EIGENEN Adresse (!= Zentrale) -> else.
    virtual void receiveKeyEvent(uint32_t senderAddress, uint8_t srcChan,
                                 uint8_t dstChan, uint8_t keyPressNum, boolean longPress) {
      if (g_linkKey != NULL && senderAddress == getCentralAddress()) {
        g_linkKey->sendKeyEvent(this, srcChan, keyPressNum, longPress);
        return;
      }
      HBWDevice::receiveKeyEvent(senderAddress, srcChan, dstChan, keyPressNum, longPress);
    };
};

HBWIODevice* device = NULL;


void setup()
{
  // BEHAVIOUR-Bits aus dem internen EEPROM lesen (0x07, je 1 Bit/Kanal,
  // 1 = OUTPUT, 0 = INPUT) und die Kanal-Objekte passend anlegen.
  // WICHTIG: "frisch" NICHT an behaviour==0xFF erkennen -- das ist zugleich die
  // GUELTIGE Konfiguration "alle 4 Kanaele OUTPUT"! Ob die CCU ueberhaupt schon
  // konfiguriert hat, verraet die central_address (0x02..0x05): erst die CCU setzt
  // sie (ungepairt = 0xFFFFFFFF). So kollidiert "frisch" nicht mehr mit All-Output.
  uint8_t  behaviour = EEPROM.read(7);
  uint32_t central   = (uint32_t)EEPROM.read(2)        |  ((uint32_t)EEPROM.read(3) << 8)
                     | ((uint32_t)EEPROM.read(4) << 16) |  ((uint32_t)EEPROM.read(5) << 24);
  if (central == 0xFFFFFFFF) behaviour = 0x00;   // ungepairt -> XML-Default: alle INPUT
  applyBehaviour(behaviour);

  HBW_BEGIN_SERIALS();   // baud-init je MCU/Konfig (siehe Config-Header)

  g_linkKey = new HBWLinkKeyType();   // Pointer gemerkt (CCU-KEY-Sim, s. receiveKeyEvent)
  device = new HBWIODevice(HMW_DEVICETYPE, HARDWARE_VERSION, FIRMWARE_VERSION,
                           &HBW_RS485, RS485_TXEN,
                           sizeof(hbwconfig), &hbwconfig,
                           NUM_CHANNELS, channels,
                           HBW_DEBUGSTREAM,
                           g_linkKey,
                           new HBWLinkSwitchHM<NUM_LINKS_OUT, LINKADDRESSSTART_OUT>());

  device->setConfigPins(BUTTON, LED);

  // hbwdebug() ist intern NULL-safe -- HBW_DEBUGSTREAM == NULL (RS485 am HW-UART),
  // daher wird hier nichts ausgegeben.
  hbwdebug(F("HBW-IO-4-FM, behaviour=0x"));
  hbwdebug(behaviour, HEX);
  hbwdebug(F(", freeRam="));
  hbwdebug(freeRam());
  hbwdebug(F("\n"));
}


void loop()
{
  device->loop();
  POWERSAVE();
}
