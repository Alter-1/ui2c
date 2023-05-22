#include <stdint.h>
#include <EEPROM.h>

const uint32_t crc_table[16] = {
  0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
  0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
  0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
  0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

uint32_t update_crc(uint32_t crc, uint8_t a)
{
  crc = crc_table[(crc ^ a) & 0x0f] ^ (crc >> 4);
  crc = crc_table[(crc ^ (a >> 4)) & 0x0f] ^ (crc >> 4);
  crc = ~crc;
  return crc;
}

uint32_t eeprom_crc(void) {
  uint32_t crc = ~0L;
  for (int index = 0 ; index < EEPROM.length()-sizeof(crc)  ; ++index) {
    crc = update_crc(crc, EEPROM[index]);
  }
  return crc;
}

uint32_t mem_crc(uint8_t* buf, int len) {
  uint32_t crc = ~0L;
  for (int index = 0 ; index < len  ; ++index) {
    crc = update_crc(crc, buf[index]);
  }
  return crc;
}
