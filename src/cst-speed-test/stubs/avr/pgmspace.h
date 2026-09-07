/*
 * Host-build shim for <avr/pgmspace.h>.
 *
 * The reference-trace test (test_speed.c) compiles cst-speed.c natively. cst-speed.c
 * itself uses nothing from pgmspace, but it includes src/lcd.h, which pulls this header
 * in unconditionally. On a normal AVR build the toolchain supplies the real one; on the
 * host it does not exist, so this no-op stand-in sits on the test's include path
 * (-Icst-speed-test/stubs) and lets src/lcd.h preprocess.
 *
 * The definitions below are the flat, no-op host equivalents - enough for any future
 * model code that reaches for a PROGMEM table to still compile and run under the test.
 */
#ifndef _AVR_PGMSPACE_H_
#define _AVR_PGMSPACE_H_

#include <stdint.h>
#include <string.h>

#define PROGMEM
#define PSTR(s)              (s)
#define PGM_P                const char *
#define PGM_VOID_P           const void *

#define pgm_read_byte(addr)   (*(const uint8_t  *)(addr))
#define pgm_read_word(addr)   (*(const uint16_t *)(addr))
#define pgm_read_dword(addr)  (*(const uint32_t *)(addr))
#define pgm_read_float(addr)  (*(const float    *)(addr))
#define pgm_read_ptr(addr)    (*(void * const   *)(addr))

#define pgm_read_byte_near(addr)  pgm_read_byte(addr)
#define pgm_read_word_near(addr)  pgm_read_word(addr)

#define memcpy_P   memcpy
#define strcpy_P   strcpy
#define strncpy_P  strncpy
#define strcat_P   strcat
#define strcmp_P   strcmp
#define strncmp_P  strncmp
#define strlen_P   strlen

#endif /* _AVR_PGMSPACE_H_ */
