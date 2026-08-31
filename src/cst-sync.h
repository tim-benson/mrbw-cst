/*************************************************************************
Title:    MRBW-CST shared loco CNF sync - throttle side
Authors:  Tim Benson <blw@east-slope.com>
File:     cst-sync.h
License:  GNU General Public License v3
Copyright (C) 2026 Tim Benson

Talks to mrbw-cabbus's shared CNF table (cnf-store.c/.h in that repo) over the 'C' (push)/'D' (pull)
protocol. See mrbw-cst's own CLAUDE.md, "Shared network CNF store", for the full design. Packet type
letters and sub-type/status byte values here MUST match mrbw-cabbus's cnf-store.h exactly - there is
no shared protocol header between the two repos.
*************************************************************************/

#ifndef _CST_SYNC_H_
#define _CST_SYNC_H_

#include <stdint.h>

typedef enum
{
	SYNC_OK = 0,
	SYNC_BUSY,             // cabbus is mid-transfer with a different throttle
	SYNC_BAD_ENTRY,        // malformed request/reply (shouldn't happen against a matching protocol version)
	SYNC_EMPTY,             // pull only: the shared entry has never been saved to
	SYNC_CHECKSUM_FAIL,    // push only: cabbus's whole-payload CRC16 didn't match
	// Distinct per protocol step rather than one generic SYNC_TIMEOUT - which step got no reply is a real
	// diagnostic signal (e.g. BEGIN failing points at addressing/reachability; DATA/COMMIT failing after
	// BEGIN succeeded points at something step-specific), and cheap to preserve since each already fails
	// at its own distinct call site.
	SYNC_TIMEOUT_BEGIN,
	SYNC_TIMEOUT_DATA,
	SYNC_TIMEOUT_COMMIT,   // push only
	// push: cabbus's BEGIN reply reported CNF_STATUS_VERSION_MISMATCH (this throttle's EE_LAYOUT_VERSION
	// is strictly lower than the table's pinned version - it needs updating before it can push anywhere).
	// pull: this throttle's own comparison of its EE_LAYOUT_VERSION against the pulled entry's reported
	// layoutVersion failed (0xFF/unset is always trusted, never a refusal, on either side).
	SYNC_VERSION_MISMATCH,
} SyncResult;

// Pushes WORKING_CONFIG's current 128 bytes to mrbw-cabbus's shared CNF table, entry index 0-19.
SyncResult syncPushSharedCnf(uint8_t entry);

// Pulls mrbw-cabbus's shared CNF entry (0-19) into WORKING_CONFIG. Caller is responsible for calling
// readConfig() afterward on SYNC_OK, same as the existing local-slot LOAD path already does.
SyncResult syncPullSharedCnf(uint8_t entry);

// Lightweight read-only peek at just the given shared entry's (0-19) loco address (2 bytes), for previewing
// which loco is stored there before committing to a full LOAD/SAVE - does not touch WORKING_CONFIG. Only
// ever returns SYNC_OK/SYNC_BUSY/SYNC_EMPTY/SYNC_BAD_ENTRY/SYNC_TIMEOUT_BEGIN/SYNC_TIMEOUT_DATA (never
// SYNC_CHECKSUM_FAIL/SYNC_TIMEOUT_COMMIT/SYNC_VERSION_MISMATCH, which are push/full-payload-verify
// concepts this read-only path doesn't use).
SyncResult syncQuerySharedLocoAddress(uint8_t entry, uint16_t *outLocoAddress);

// Lightweight read-only peek at the table-wide pinned layoutVersion, via a PULL BEGIN+DONE against the
// given entry (0-19) - works even against an empty/never-written entry, since the pin is table-wide, not
// per-entry (see mrbw-cabbus's cnf-store.c). Used before a push to decide whether it would auto-advance
// the table (this throttle's EE_LAYOUT_VERSION > the peeked pin) and needs the two-stage on-device
// confirmation. Only ever returns SYNC_OK/SYNC_BUSY/SYNC_BAD_ENTRY/SYNC_TIMEOUT_BEGIN - status is
// otherwise irrelevant to this peek, since layoutVersion is reported regardless of entry occupancy.
SyncResult syncPeekSharedVersion(uint8_t entry, uint8_t *outLayoutVersion);

// Valid only immediately after a SYNC_TIMEOUT_DATA result - the byte offset (0-127) of the chunk that
// never got an acked reply, for diagnostic display.
uint8_t syncGetLastTimeoutOffset(void);

// Optional "still working" hook: while a blocking sync call runs, this is invoked roughly every
// CNF_POLL_INTERVAL_MS (~20ms) from the internal wait loop. Set it just before a sync call to drive an
// on-screen animation, clear it (NULL) immediately after. Must be quick and non-blocking - it runs in
// place of one poll tick. cst-sync.c never touches the LCD itself.
void syncSetProgressCallback(void (*cb)(void));

#endif
