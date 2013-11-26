
#ifndef UTIL_H
#define	UTIL_H

#ifdef	__cplusplus
extern "C" {
#endif

#define ABS(a) ( ((a) < 0) ? -(a) : (a) )  
#define MIN(a,b) ( ((a) < (b)) ? (a) : (b) )
#define MAX(a,b) ( ((a) > (b)) ? (a) : (b) )

  uint32_t swap_endian(uint32_t val);
  void trim_right(char* str);
  int is_whitespace(char ch);

#ifdef	__cplusplus
}
#endif

#endif	/* UTIL_H */

