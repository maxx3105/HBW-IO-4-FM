/* Pin- und Hardware-Konfiguration für HBW-IO-4-FM
 *
 * Ziel-Hardware = maxx3105 "HBW-IO-4-FM" Platine1:
 *   - ATmega328P-A @ 16 MHz, 5 V
 *   - RS485-Transceiver MAX487E am HARDWARE-UART (Serial):
 *       RO -> PD0 (RX0),  DI -> PD1 (TX0),  DE+/RE -> PD2 (D2) = TXEN
 *   - 4 IOs je über 330 Ω an die Klemmen ST7/ST5/ST4/ST8:
 *       IO1 = A0 (PC0),  IO2 = A1 (PC1),  IO3 = A2 (PC2),  IO4 = A3 (PC3)
 *   - LED = D7 (PD7),  Config-Button = D8 (PB0)
 *   - Stromversorgung über Platine2 (MC34063AD-Buck 24 V -> 5 V)
 *
 * Pinquelle: HBW-IO-4-FM_Platine1.kicad_sch / .pdf (Repo maxx3105/HBW-IO-4-FM).
 *
 * RS485 liegt fest auf dem Hardware-UART -> KEIN serielles Debug verfügbar
 * (HBW_DEBUGSTREAM = NULL). Flashen per ISP (ISP1-Header) oder UART-Bootloader
 * bei abgezogenem Bus. MightyCore/Arduino-Clock: "16 MHz external".
 */

#ifndef HBW_IO_4_FM_CONFIG_H
#define HBW_IO_4_FM_CONFIG_H

#include <Arduino.h>
#include <EEPROM.h>
#include "FreeRam.h"

EEPROMClass* EepromPtr = &EEPROM;   // internes EEPROM

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)

  // 4 IOs an PC0..PC3 (= A0..A3), je 330 Ω in Reihe (siehe Schaltplan Platine1)
  #define IO1   A0      // PC0  -> ST7
  #define IO2   A1      // PC1  -> ST5
  #define IO3   A2      // PC2  -> ST4
  #define IO4   A3      // PC3  -> ST8

  #define LED         7     // PD7
  #define BUTTON      8     // PB0 (D8) -> Config-Taster gegen GND

  // RS485 auf dem Hardware-UART (RO->PD0/RX0, DI->PD1/TX0)
  #define RS485_TXEN  2         // PD2 (D2) -> MAX487 DE + /RE
  #define HBW_RS485       (Serial)
  #define HBW_DEBUGSTREAM (NULL)
  #define HBW_BEGIN_SERIALS() do { Serial.begin(19200, SERIAL_8E1); } while (0)

#else
  #error "HBW-IO-4-FM: Nicht unterstuetzter Mikrocontroller. Ziel ist ATmega328P (Platine1)."
#endif

#endif  // HBW_IO_4_FM_CONFIG_H
