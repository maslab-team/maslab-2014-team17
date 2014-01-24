// Detect which type of boot loader is present, using a fixed built-in table
// 2012-03-06 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
 
#include <avr/pgmspace.h>
#include <util/crc16.h>
 
#define VERSION "2"
 
// list of all known boot loaders with their unique signatures
struct { word crc; const char* desc; } signatures[] = {
  0x489C, "Duemilanove",
  0xF1A0, "Nanode (Duemilanove mod)",
  0xFD70, "OptiBoot 4.4",
  0,      0
};
 
static word CalculateChecksum (word addr, word size) {
  word crc = ~0;
  prog_uint8_t* p = (prog_uint8_t*) addr;
  for (word i = 0; i < size; ++i)
    crc = _crc16_update(crc, pgm_read_byte(p++));
  Serial.print("  CRC ");
  Serial.print(size);
  Serial.print("b @ 0x");
  Serial.print(addr, HEX);
  Serial.print(" = ");
  Serial.println(crc, HEX);
  return crc;
}
 
static const char* IdentifyBootLoader (word addr, word size) {
  word crc = CalculateChecksum(addr, size);
  for (byte i = 0; signatures[i].desc != 0; ++i)
    if (signatures[i].crc == crc)
      return signatures[i].desc;
  return 0;
}
 
void setup () {
  Serial.begin(57600);
  Serial.println("\n[bootCheck." VERSION "]");
  
  const char* message = IdentifyBootLoader(0x7800, 2048);
  if (message == 0)
    message = IdentifyBootLoader(0x7E00, 512);
  if (message == 0)
    message = "(UNKNOWN)";
  
  Serial.print("Boot loader: ");
  Serial.println(message);
}
 
void loop () {}
