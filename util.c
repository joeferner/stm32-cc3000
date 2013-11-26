
#include <stdint.h>
#include <string.h>
#include "util.h"

uint32_t swap_endian(uint32_t val) {
  return ((val << 24) & 0xff000000)
          | ((val << 8) & 0x00ff0000)
          | ((val >> 8) & 0x0000ff00)
          | ((val >> 24) & 0x000000ff);
}

void trim_right(char* str) {
  char *p = str + strlen(str) - 1;
  while (is_whitespace(*p)) {
    *p-- = '\0';
  }
}

int is_whitespace(char ch) {
  switch (ch) {
    case '\n':
    case '\r':
    case '\t':
    case ' ':
      return 1;
  }
  return 0;
}
