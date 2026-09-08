/*
 * Host-build shim for <avr/eeprom.h>.
 *
 * The reference-trace test (test_eeprom.c) compiles cst-eeprom.c natively.
 * applyEepromMigrations() touches the EEPROM only through the byte-at-a-time
 * eeprom_read_byte() / eeprom_write_byte() API - no word/dword/block calls, no
 * EEMEM - so this shim backs just those two, over the g_eeprom[4096] array the
 * test defines. The pointer argument is a small integer address cast to a
 * pointer (CONFIG_OFFSET(s) + 0xNN), never dereferenced on the host, so
 * (uintptr_t) recovers the index; the & 0x0FFF mask keeps a stray address in
 * bounds of the 4096-byte model rather than corrupting the host heap.
 *
 * Sits on the test's include path (-Icst-eeprom-test/stubs), same arrangement
 * as cst-speed-test/stubs/avr/pgmspace.h.
 */
#ifndef _AVR_EEPROM_H_
#define _AVR_EEPROM_H_

#include <stdint.h>

extern uint8_t g_eeprom[4096];

static inline uint8_t eeprom_read_byte(const uint8_t *addr)
{
	return g_eeprom[(uintptr_t)addr & 0x0FFF];
}

static inline void eeprom_write_byte(uint8_t *addr, uint8_t value)
{
	g_eeprom[(uintptr_t)addr & 0x0FFF] = value;
}

#endif /* _AVR_EEPROM_H_ */
