
#include <stdint.h>
#include "util.h"

uint32_t swap_endian(uint32_t val) {
  return ((val << 24) & 0xff000000)
          | ((val << 8) & 0x00ff0000)
          | ((val >> 8) & 0x0000ff00)
          | ((val >> 24) & 0x000000ff);
}

