/* HBWLinkSwitchHM -- Link-Receiver fuer das ORIGINAL eQ-3 28-Byte-Peering
 *  (hmw_switch_ch_link, address_step=28). Pendant zur loetmeister-
 *  HBWLinkSwitchAdvanced (20-Byte), aber im Original-HM-Format mit
 *  2-Byte-Zeiten. Zielkanaele muessen vom Typ HBWSwitchHM sein.
 *
 *  EEPROM-Layout pro Link (28 Byte, relativ zum Link-Start):
 *    +0..+3   SENSOR-Adresse (HM big-endian)
 *    +4       SENSOR-Channel  (Sender-Kanal)
 *    +5       CHANNEL         (Ziel-Kanal an diesem Geraet)
 *    +6       SHORT modeByte
 *    +7..+14  SHORT on_delay/on/off_delay/off (4x uint16 little-endian)
 *    +15..+16 SHORT jump-table (uint16 little-endian)
 *    +17      LONG  modeByte
 *    +18..+25 LONG  Zeiten
 *    +26..+27 LONG  jump-table
 */

#ifndef HBWLinkSwitchHM_h
#define HBWLinkSwitchHM_h

#include "HBWired.h"
#include "HBWSwitchHM.h"


template<uint8_t numLinks, uint16_t eepromStart>
class HBWLinkSwitchHM : public HBWLinkReceiver {
  public:
    HBWLinkSwitchHM() : lastSenderAddress(0), lastSenderChannel(0) {};
    void receiveKeyEvent(HBWDevice* device, uint32_t senderAddress, uint8_t senderChannel,
                         uint8_t targetChannel, uint8_t keyPressNum, boolean longPress);

  private:
    uint32_t lastSenderAddress;
    uint8_t  lastSenderChannel;

    static const uint8_t EEPROM_SIZE = 28;   // "address_step" im XML
};

#include "HBWLinkSwitchHM.hpp"
#endif
