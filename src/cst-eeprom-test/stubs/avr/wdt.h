/*
 * Host-build shim for <avr/wdt.h>.
 *
 * cst-eeprom.c calls wdt_reset() between per-slot migration passes to keep the
 * hardware watchdog fed during the long EEPROM rewrite. On the host there is no
 * watchdog, so this is a no-op. Sits on the test's include path
 * (-Icst-eeprom-test/stubs).
 */
#ifndef _AVR_WDT_H_
#define _AVR_WDT_H_

#define wdt_reset() ((void)0)

#endif /* _AVR_WDT_H_ */
