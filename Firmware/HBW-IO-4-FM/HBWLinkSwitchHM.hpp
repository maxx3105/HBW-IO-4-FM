/* HBWLinkSwitchHM -- Implementierung (siehe HBWLinkSwitchHM.h) */


// receiveKeyEvent wird aufgerufen, wenn ein Tastendruck/Sensorereignis empfangen wurde.
// Durchsucht alle Links nach passender Sender-Adresse/-Kanal + Ziel-Kanal und ruft
// device->set() mit der zerlegten SHORT- bzw. LONG-Peering-Haelfte auf.
template<uint8_t numLinks, uint16_t eepromStart>
void HBWLinkSwitchHM<numLinks, eepromStart>::receiveKeyEvent(
        HBWDevice* device, uint32_t senderAddress, uint8_t senderChannel,
        uint8_t targetChannel, uint8_t keyPressNum, boolean longPress) {

  uint32_t sndAddrEEPROM;
  uint8_t  channelEEPROM;

  bool sameLastSender = (senderAddress == lastSenderAddress && senderChannel == lastSenderChannel);
  lastSenderAddress = senderAddress;
  lastSenderChannel = senderChannel;

  for (uint8_t i = 0; i < numLinks; i++) {
    uint16_t base = eepromStart + (uint16_t)EEPROM_SIZE * i;

    // Sender-Adresse (HM big-endian -> lowByteFirst) lesen und vergleichen
    device->readEEPROM(&sndAddrEEPROM, base, 4, true);
    if (sndAddrEEPROM == 0xFFFFFFFF)   continue;   // leerer Link
    if (sndAddrEEPROM != senderAddress) continue;
    // Sender-Kanal
    device->readEEPROM(&channelEEPROM, base + 4, 1);
    if (channelEEPROM != senderChannel) continue;
    // Ziel-Kanal an diesem Geraet
    device->readEEPROM(&channelEEPROM, base + 5, 1);
    if (channelEEPROM != targetChannel) continue;

    // Treffer: passende Haelfte auslesen (SHORT @+6, LONG @+17), je 11 Byte:
    // modeByte + 8 Zeit-Bytes (LE) + 2 jt-Bytes (LE)
    uint16_t half = base + (longPress ? 17 : 6);

    uint8_t buf[HBWSwitchHM::PEER_SET_LEN];
    device->readEEPROM(buf, half, 11);          // geradeaus -> little-endian erhalten
    if ((buf[0] & 0x01) == 0)  continue;        // ACTION_TYPE INACTIVE -> nichts tun

    buf[11] = keyPressNum;
    buf[12] = (uint8_t)sameLastSender;
    device->set(targetChannel, HBWSwitchHM::PEER_SET_LEN, buf);
  }
}
