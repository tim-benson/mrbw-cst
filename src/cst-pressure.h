#ifndef _CST_PRESSURE_H_
#define _CST_PRESSURE_H_

#include <stdint.h>

// AIRBRAKE - a small air-brake system model (brake pipe + main reservoir) that runs permanently at
// 10 Hz and drives the ESU air sound functions. Enabled by the global PREFS AIRBRAKE bit; the model
// itself always ticks. See CLAUDE.md "AIRBRAKE".

// --- AIRBRAKE CFG per-profile config items, in AIRBRAKE_CONFIG_SCREEN menu order ---
// No item here is ADV-FUNC-gated (MR_LOAD was the last one; ungated 2026-09-05). VENT/EMERG vent
// rates used to be items here too (see AIRBRAKE_VENT_RATE_PSI_S/AIRBRAKE_EMERG_VENT_RATE_PSI_S in
// cst-pressure.c) - hardcoded and removed from this per-profile config since they were never
// usefully varied per-loco. FULLSVC (full-service reduction span, PSI) used to be a stored item too
// - it's now derived fresh from CHARGED (2/7, rounded up to the nearest PSI - see updateBrake10Hz())
// so it always tracks whatever CHARGED is currently set to, rather than risking the two drifting
// out of sync.
enum
{
	AIRBRAKE_CHARGED = 0,   // brake-pipe charged pressure, PSI - "BP CHARGE" on-screen, range 70-110
	AIRBRAKE_MR_LOAD,       // reservoir draw per PSI of pipe recharge, %, range 0-100
	AIRBRAKE_MR_CUTIN,      // reservoir governor cut-in, PSI - "MR LOW" on-screen
	AIRBRAKE_MR_CUTOUT,     // reservoir governor cut-out, PSI - "MR HIGH" on-screen
	AIRBRAKE_CHARGE_RATE,   // brake-pipe recharge rate (initial rate of the taper), PSI/min - "RECHARGE" on-screen
	AIRBRAKE_LEAK_RATE,     // reservoir base leak, PSI/min - "LEAKRATE" on-screen
	AIRBRAKE_PUMP_RATE,     // compressor fill rate, PSI/min (slow - governs the idle cycle on-time) - "PUMPRATE" on-screen
	AIRBRAKE_DISPLAY,       // DUAL/SINGLE - which AIRBRAKE_SCREEN rendering: BP:/MR: glyph view vs the analogue dial
	AIRBRAKE_COMP_MODE,     // NORMAL/CONSIST - gates the COMPRSR/COMPRSR2 split, see updateBrake10Hz()
	AIRBRAKE_COUNT
};

#define AIRBRAKE_CHARGED_DEFAULT       90
#define AIRBRAKE_CHARGED_MIN           70    // BP CHARGE on-device editing range
#define AIRBRAKE_CHARGED_MAX          110
#define AIRBRAKE_MR_CUTIN_DEFAULT     130
#define AIRBRAKE_MR_CUTOUT_DEFAULT    140
#define AIRBRAKE_CHARGE_RATE_DEFAULT  180    // PSI/min (= 3 PSI/s; initial rate of the recharge taper)
#define AIRBRAKE_LEAK_RATE_DEFAULT      5    // PSI/min  -> ~120 s idle off-phase over a 10 PSI band
#define AIRBRAKE_PUMP_RATE_DEFAULT     30    // PSI/min  -> ~24 s idle on-phase (net vs LEAK)
#define AIRBRAKE_MR_LOAD_DEFAULT       35    // %  -> ~one compressor cycle per full-service release
#define AIRBRAKE_MR_LOAD_MAX          100    // on-device editing range is 0-100
#define AIRBRAKE_COMP_MODE_NORMAL       0    // COMPRSR only, regardless of classification (pre-split behaviour)
#define AIRBRAKE_COMP_MODE_CONSIST      1    // split COMPRSR (deep recharge) / COMPRSR2 (routine)
#define AIRBRAKE_COMP_MODE_DEFAULT      AIRBRAKE_COMP_MODE_NORMAL
#define AIRBRAKE_DISPLAY_DUAL           0    // AIRBRAKE_SCREEN shows the BP:/MR: two-pressure glyph view
#define AIRBRAKE_DISPLAY_SINGLE         1    // AIRBRAKE_SCREEN shows the single analogue BP dial (LCD_AIRBRAKE_ALT)
#define AIRBRAKE_DISPLAY_DEFAULT        AIRBRAKE_DISPLAY_DUAL

uint8_t airbrakeGet(uint8_t item);
void    airbrakeSet(uint8_t item, uint8_t value);

// Simulation
void    initAirBrake(void);
// independentBrakeAtRest: mrbw-cst.c's own per-BRK-TYPE "is the independent brake genuinely at rest"
// check (see CLAUDE.md "Brake logic") - drives the automatic-brake pipe's own apply/release point
// instead of a separate, mode-unaware AIRBRAKE threshold.
// emergencyBrakeEnabled: the throttle's BRK ESTP option is on - only then does slamming the lever
// past AIRBRAKE_EMERG_PCNT dump the pipe to 0 (a real 26L has no emergency zone otherwise; the
// handle just laps at full service). A pure option flag, so it still models on the AIRBRAKE screen.
// See CLAUDE.md "AIRBRAKE".
void    updateBrake10Hz(uint8_t leverPcnt, uint8_t independentBrakeAtRest, uint8_t emergencyBrakeEnabled);

// Outputs
uint8_t airBrakeReleased(void);   // BRAKE_REL_FN: 1 = released (function asserted)
uint8_t airBrakeSetPulse(void);       // BRK SET: brake-pipe-reduction hiss pulse active
uint8_t airCompressorOn(void);    // COMPRESSOR_FN
uint8_t airCompressorReleaseRun(void); // 1 = current/most recent compressor run is a "deep" (COMPRSR) event
uint8_t airCompressorPendingRelease(void); // 1 = a run started right now would be classified deep (COMPRSR) - read-only, does not latch
uint8_t airEmergencyActive(void); // emergency brake-pipe dump in progress (BRK ESTP + lever slam)
uint8_t airBrakePipePsi(void);    // whole PSI, for the AIRBRAKE screen
uint8_t airMainResPsi(void);      // whole PSI, for the AIRBRAKE screen

#endif
