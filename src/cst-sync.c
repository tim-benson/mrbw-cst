/*************************************************************************
Title:    MRBW-CST shared loco CNF sync - throttle side
Authors:  Tim Benson <blw@east-slope.com>
File:     cst-sync.c
License:  GNU General Public License v3
Copyright (C) 2026 Tim Benson

See cst-sync.h for the API and mrbw-cst's CLAUDE.md ("Shared network CNF store") for the full design.
*************************************************************************/

#include <string.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/atomic.h>
#include <util/delay.h>
#include "mrbee.h"
#include "cst-eeprom.h"
#include "cst-sync.h"

extern uint8_t mrbus_dev_addr;
extern uint8_t mrbus_base_addr;
extern volatile uint8_t txHoldoff;
extern uint8_t txHoldoff_centisecs;

// Must match mrbw-cabbus's cnf-store.h exactly - see that file's header comment.
#define CNF_PKT_TYPE_PUSH              'C'
#define CNF_PKT_TYPE_PUSH_ACK          'c'
#define CNF_PKT_TYPE_PULL              'D'
#define CNF_PKT_TYPE_PULL_ACK          'd'

#define CNF_SUBTYPE_BEGIN              0x00
#define CNF_SUBTYPE_DATA               0x01
#define CNF_SUBTYPE_COMMIT             0x02
#define CNF_SUBTYPE_DONE               0x03
// CNF_SUBTYPE_RESET (0x04) is a PC-tool-only operation (cst_cfgnetwork.py's `reset-cabbus`) - this
// throttle firmware never sends it, so it's not defined here; see mrbw-cabbus's cnf-store.h.

#define CNF_STATUS_OK                  0x00
#define CNF_STATUS_BUSY                0x01
#define CNF_STATUS_BAD_ENTRY           0x02
#define CNF_STATUS_EMPTY               0x03
#define CNF_STATUS_CHECKSUM_FAIL       0x04
#define CNF_STATUS_VERSION_MISMATCH    0x06

#define CNF_VERSION_UNSET              0xFF

#define CNF_CHUNK_MAX_BYTES            10
#define CNF_PAYLOAD_SIZE               128

#define CNF_POLL_INTERVAL_MS           20
#define CNF_CHUNK_TIMEOUT_MS           300
// COMMIT is the one step where cabbus does real EEPROM writes (128-byte payload + 4 metadata bytes)
// before it can even queue its reply - AVR EEPROM writes run ~3.3ms/byte, so ~132 bytes is ~440ms of
// genuine hardware write time alone, comfortably longer than CNF_CHUNK_TIMEOUT_MS. Confirmed on real
// hardware: BEGIN/DATA (RAM-only on cabbus's side) reliably succeed within 300ms; COMMIT reliably timed
// out until given this larger budget.
#define CNF_COMMIT_TIMEOUT_MS          800
#define CNF_CHUNK_MAX_RETRIES          3
#define CNF_OVERALL_TIMEOUT_MS         15000

static uint8_t cnfSyncScratch[CNF_PAYLOAD_SIZE];
static uint16_t cnfOverallElapsedMs;
static uint8_t cnfLastTimeoutOffset;

// Optional "still working" hook, invoked ~every CNF_POLL_INTERVAL_MS from cnfSendAndWait()'s wait
// loop. NULL = no hook. See syncSetProgressCallback() / cst-sync.h.
static void (*cnfProgressCallback)(void) = NULL;

uint8_t syncGetLastTimeoutOffset(void)
{
	return cnfLastTimeoutOffset;
}

void syncSetProgressCallback(void (*cb)(void))
{
	cnfProgressCallback = cb;
}

static uint16_t cnfCrc16(const uint8_t *data, uint8_t len)
{
	uint16_t crc = 0;
	uint8_t i;
	for(i = 0; i < len; i++)
		crc = mrbusCRC16Update(crc, data[i]);
	return crc;
}

// Actually transmitting a queued packet normally happens from the main loop (mrbw-cst.c, gated on
// "queue non-empty AND !txHoldoff", calling mrbeeTransmit() then re-arming txHoldoff) - but this code
// runs entirely inside a blocking button-handler call, so the main loop never gets a turn while a sync
// is in progress. Without this, mrbusPktQueuePush() only ever queues a packet; nothing would actually
// put it on the air until the sync gives up and control finally returns to the main loop - which exactly
// matches the bug this fixes (confirmed on real hardware: cabbus only ever received the queued packet the
// instant the throttle's timeout screen was dismissed). txHoldoff itself decrements via ISR(TIMER0_COMPA_vect),
// so it counts down on its own regardless of blocking - safe to just poll it here.
static void cnfKickTransmit(void)
{
	uint8_t waited = 0;
	while(txHoldoff && waited < 50)  // bounded wait, ~500ms worst case; txHoldoff always self-clears via ISR
	{
		wdt_reset();
		_delay_ms(10);
		waited++;
	}
	wdt_reset();
	mrbeeTransmit();
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		txHoldoff = txHoldoff_centisecs;
	}
}

static void cnfSendPacket(uint8_t type, uint8_t subtype, uint8_t entry, const uint8_t *extra, uint8_t extraLen)
{
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];
	txBuffer[MRBUS_PKT_DEST] = mrbus_base_addr;
	txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
	txBuffer[MRBUS_PKT_TYPE] = type;
	txBuffer[MRBUS_PKT_SUBTYPE] = subtype;
	txBuffer[7] = entry;
	if(extraLen)
		memcpy(&txBuffer[8], extra, extraLen);
	txBuffer[MRBUS_PKT_LEN] = 8 + extraLen;
	mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
	cnfKickTransmit();
}

static uint8_t cnfPacketIsValid(const uint8_t *rxBuffer)
{
	if(rxBuffer[MRBUS_PKT_SRC] != mrbus_base_addr)
		return 0;
	if(0xFF != rxBuffer[MRBUS_PKT_DEST] && mrbus_dev_addr != rxBuffer[MRBUS_PKT_DEST])
		return 0;

	uint16_t crc = 0;
	uint8_t i;
	for(i = 0; i < rxBuffer[MRBUS_PKT_LEN]; i++)
		if(i != MRBUS_PKT_CRC_H && i != MRBUS_PKT_CRC_L)
			crc = mrbusCRC16Update(crc, rxBuffer[i]);
	return (UINT16_HIGH_BYTE(crc) == rxBuffer[MRBUS_PKT_CRC_H]) && (UINT16_LOW_BYTE(crc) == rxBuffer[MRBUS_PKT_CRC_L]);
}

// Sends a request, waits for the matching reply (right type/subtype/entry), retrying (resending) up to
// CNF_CHUNK_MAX_RETRIES times on a per-attempt `timeoutMs` budget (CNF_CHUNK_TIMEOUT_MS for BEGIN/DATA,
// the larger CNF_COMMIT_TIMEOUT_MS for COMMIT specifically - see that constant's comment) - bounded
// overall by CNF_OVERALL_TIMEOUT_MS across the whole push/pull, tracked in cnfOverallElapsedMs. Any
// packet that arrives but doesn't match (stray traffic, a reply to some earlier retried attempt, etc.)
// is discarded and polling continues within the same attempt's remaining time. Returns 1 (rxBuffer
// filled with the matching reply) on success, 0 on exhausted retries/overall timeout.
//
// This device isn't running its own PktHandler() dispatch while blocked in here, so any *other* incoming
// traffic during a sync is silently dropped - matching how the existing local-slot LOAD/SAVE dot-animation
// already doesn't process incoming packets during its own (much shorter) blocking window.
static uint8_t cnfSendAndWait(uint8_t reqType, uint8_t replyType, uint8_t subtype, uint8_t entry,
                               const uint8_t *extra, uint8_t extraLen, uint8_t *rxBuffer, uint16_t timeoutMs)
{
	uint8_t attempt;
	for(attempt = 0; attempt < CNF_CHUNK_MAX_RETRIES; attempt++)
	{
		cnfSendPacket(reqType, subtype, entry, extra, extraLen);

		uint16_t waitedThisAttempt = 0;
		while(waitedThisAttempt < timeoutMs)
		{
			if(cnfOverallElapsedMs >= CNF_OVERALL_TIMEOUT_MS)
				return 0;

			wdt_reset();
			uint8_t rssi;
			if(mrbeePktQueuePop(&mrbeeRxQueue, rxBuffer, MRBUS_BUFFER_SIZE, &rssi))
			{
				if(cnfPacketIsValid(rxBuffer) && rxBuffer[MRBUS_PKT_TYPE] == replyType &&
				   rxBuffer[MRBUS_PKT_SUBTYPE] == subtype && rxBuffer[7] == entry)
					return 1;
				continue;  // not our reply - keep polling within this attempt's remaining time
			}

			if(cnfProgressCallback)
				cnfProgressCallback();   // ~20ms cadence (no-packet path only) - drives the caller's animation
			_delay_ms(CNF_POLL_INTERVAL_MS);
			waitedThisAttempt += CNF_POLL_INTERVAL_MS;
			cnfOverallElapsedMs += CNF_POLL_INTERVAL_MS;
		}
		// This attempt's timeout expired with no matching reply - loop around and resend.
	}
	return 0;
}

SyncResult syncPushSharedCnf(uint8_t entry)
{
	uint8_t rxBuffer[MRBUS_BUFFER_SIZE];
	uint8_t extra[2 + CNF_CHUNK_MAX_BYTES];
	cnfOverallElapsedMs = 0;

	eeprom_read_block(cnfSyncScratch, (const void*)CONFIG_OFFSET(WORKING_CONFIG), CNF_PAYLOAD_SIZE);

	extra[0] = CNF_PAYLOAD_SIZE;
	extra[1] = EEPROM_LAYOUT_VERSION;
	if(!cnfSendAndWait(CNF_PKT_TYPE_PUSH, CNF_PKT_TYPE_PUSH_ACK, CNF_SUBTYPE_BEGIN, entry,
	                    extra, 2, rxBuffer, CNF_CHUNK_TIMEOUT_MS))
		return SYNC_TIMEOUT_BEGIN;
	if(rxBuffer[8] != CNF_STATUS_OK)
	{
		if(CNF_STATUS_BUSY == rxBuffer[8])
			return SYNC_BUSY;
		if(CNF_STATUS_VERSION_MISMATCH == rxBuffer[8])
			return SYNC_VERSION_MISMATCH;
		return SYNC_BAD_ENTRY;
	}

	uint8_t offset = 0;
	while(offset < CNF_PAYLOAD_SIZE)
	{
		uint8_t length = CNF_PAYLOAD_SIZE - offset;
		if(length > CNF_CHUNK_MAX_BYTES)
			length = CNF_CHUNK_MAX_BYTES;

		extra[0] = offset;
		extra[1] = length;
		memcpy(&extra[2], &cnfSyncScratch[offset], length);

		if(!cnfSendAndWait(CNF_PKT_TYPE_PUSH, CNF_PKT_TYPE_PUSH_ACK, CNF_SUBTYPE_DATA, entry,
		                    extra, 2 + length, rxBuffer, CNF_CHUNK_TIMEOUT_MS))
		{
			cnfLastTimeoutOffset = offset;
			return SYNC_TIMEOUT_DATA;
		}
		if(rxBuffer[9] != CNF_STATUS_OK)   // DATA reply: [8]=offset [9]=status
			return SYNC_BAD_ENTRY;

		offset += length;
	}

	uint16_t crc = cnfCrc16(cnfSyncScratch, CNF_PAYLOAD_SIZE);
	extra[0] = crc & 0xFF;
	extra[1] = (crc >> 8) & 0xFF;
	if(!cnfSendAndWait(CNF_PKT_TYPE_PUSH, CNF_PKT_TYPE_PUSH_ACK, CNF_SUBTYPE_COMMIT, entry,
	                    extra, 2, rxBuffer, CNF_COMMIT_TIMEOUT_MS))
		return SYNC_TIMEOUT_COMMIT;
	if(rxBuffer[8] != CNF_STATUS_OK)
		return SYNC_CHECKSUM_FAIL;

	return SYNC_OK;
}

SyncResult syncPullSharedCnf(uint8_t entry)
{
	uint8_t rxBuffer[MRBUS_BUFFER_SIZE];
	cnfOverallElapsedMs = 0;

	if(!cnfSendAndWait(CNF_PKT_TYPE_PULL, CNF_PKT_TYPE_PULL_ACK, CNF_SUBTYPE_BEGIN, entry,
	                    NULL, 0, rxBuffer, CNF_CHUNK_TIMEOUT_MS))
		return SYNC_TIMEOUT_BEGIN;
	uint8_t status = rxBuffer[8];
	if(status != CNF_STATUS_OK)
	{
		if(CNF_STATUS_BUSY == status)
			return SYNC_BUSY;
		if(CNF_STATUS_EMPTY == status)
			return SYNC_EMPTY;
		return SYNC_BAD_ENTRY;
	}
	// rxBuffer[12] = the table-wide pinned layoutVersion this entry was written under. Checked (and, on a
	// mismatch, refused) BEFORE trusting rxBuffer[9]'s reported length for anything - an equal-version
	// pull is guaranteed by construction to report exactly CNF_PAYLOAD_SIZE (same firmware, same
	// CONFIG_SIZE), so the DATA loop below still walks the fixed local CNF_PAYLOAD_SIZE rather than a
	// server-reported length, which would otherwise risk overflowing cnfSyncScratch against a stale/
	// mismatched tool or a future larger-format entry this firmware doesn't understand.
	uint8_t entryLayoutVersion = rxBuffer[12];
	if(CNF_VERSION_UNSET != entryLayoutVersion && entryLayoutVersion != EEPROM_LAYOUT_VERSION)
	{
		cnfSendPacket(CNF_PKT_TYPE_PULL, CNF_SUBTYPE_DONE, entry, NULL, 0);
		return SYNC_VERSION_MISMATCH;
	}
	uint16_t expectedCrc = rxBuffer[10] | ((uint16_t)rxBuffer[11] << 8);

	uint8_t offset = 0;
	while(offset < CNF_PAYLOAD_SIZE)
	{
		uint8_t length = CNF_PAYLOAD_SIZE - offset;
		if(length > CNF_CHUNK_MAX_BYTES)
			length = CNF_CHUNK_MAX_BYTES;

		uint8_t reqExtra[2];
		reqExtra[0] = offset;
		reqExtra[1] = length;
		if(!cnfSendAndWait(CNF_PKT_TYPE_PULL, CNF_PKT_TYPE_PULL_ACK, CNF_SUBTYPE_DATA, entry,
		                    reqExtra, 2, rxBuffer, CNF_CHUNK_TIMEOUT_MS))
		{
			cnfLastTimeoutOffset = offset;
			return SYNC_TIMEOUT_DATA;
		}
		if(rxBuffer[9] != CNF_STATUS_OK)   // DATA reply: [8]=offset [9]=status [10..]=data
			return SYNC_BAD_ENTRY;
		memcpy(&cnfSyncScratch[offset], &rxBuffer[10], length);

		offset += length;
	}

	uint16_t actualCrc = cnfCrc16(cnfSyncScratch, CNF_PAYLOAD_SIZE);

	// Sent regardless of verify outcome below, so cabbus releases its read snapshot promptly either way -
	// not required for correctness (cabbus's own busy-timeout is the real safety net), just avoids leaving
	// it falsely busy for the full timeout window on every ordinary successful pull.
	cnfSendPacket(CNF_PKT_TYPE_PULL, CNF_SUBTYPE_DONE, entry, NULL, 0);

	if(actualCrc != expectedCrc)
		return SYNC_CHECKSUM_FAIL;

	eeprom_write_block(cnfSyncScratch, (void*)CONFIG_OFFSET(WORKING_CONFIG), CNF_PAYLOAD_SIZE);
	return SYNC_OK;
}

SyncResult syncQuerySharedLocoAddress(uint8_t entry, uint16_t *outLocoAddress)
{
	uint8_t rxBuffer[MRBUS_BUFFER_SIZE];
	cnfOverallElapsedMs = 0;

	if(!cnfSendAndWait(CNF_PKT_TYPE_PULL, CNF_PKT_TYPE_PULL_ACK, CNF_SUBTYPE_BEGIN, entry,
	                    NULL, 0, rxBuffer, CNF_CHUNK_TIMEOUT_MS))
		return SYNC_TIMEOUT_BEGIN;
	uint8_t status = rxBuffer[8];
	if(status != CNF_STATUS_OK)
	{
		if(CNF_STATUS_BUSY == status)
			return SYNC_BUSY;
		if(CNF_STATUS_EMPTY == status)
			return SYNC_EMPTY;
		return SYNC_BAD_ENTRY;
	}

	// Loco address is the first 2 bytes of the payload (EE_LOCO_ADDRESS's position within a slot) - a
	// single DATA chunk covers it, no need to walk the whole 128-byte payload like a real pull does.
	uint8_t reqExtra[2] = {0, 2};
	uint8_t result = cnfSendAndWait(CNF_PKT_TYPE_PULL, CNF_PKT_TYPE_PULL_ACK, CNF_SUBTYPE_DATA, entry,
	                                 reqExtra, 2, rxBuffer, CNF_CHUNK_TIMEOUT_MS);

	// BEGIN succeeded, so cabbus is holding a snapshot/busy lock for us regardless of how DATA went -
	// release it promptly rather than leaving it for the 3s busy-timeout to clear.
	cnfSendPacket(CNF_PKT_TYPE_PULL, CNF_SUBTYPE_DONE, entry, NULL, 0);

	if(!result)
	{
		cnfLastTimeoutOffset = 0;
		return SYNC_TIMEOUT_DATA;
	}
	if(rxBuffer[9] != CNF_STATUS_OK)   // DATA reply: [8]=offset [9]=status [10..]=data
		return SYNC_BAD_ENTRY;

	*outLocoAddress = rxBuffer[10] | ((uint16_t)rxBuffer[11] << 8);
	return SYNC_OK;
}

SyncResult syncPeekSharedVersion(uint8_t entry, uint8_t *outLayoutVersion)
{
	uint8_t rxBuffer[MRBUS_BUFFER_SIZE];
	cnfOverallElapsedMs = 0;

	if(!cnfSendAndWait(CNF_PKT_TYPE_PULL, CNF_PKT_TYPE_PULL_ACK, CNF_SUBTYPE_BEGIN, entry,
	                    NULL, 0, rxBuffer, CNF_CHUNK_TIMEOUT_MS))
		return SYNC_TIMEOUT_BEGIN;

	uint8_t status = rxBuffer[8];
	*outLayoutVersion = rxBuffer[12];

	// BEGIN's reply always carries the table-wide pinned layoutVersion regardless of status - only OK/
	// EMPTY against a real entry actually arm a busy lock on cabbus's side (BUSY/BAD_ENTRY are immediate
	// replies with nothing to release), but sending DONE unconditionally is harmless either way (cabbus's
	// DONE handler is a no-op unless it matches an in-progress transfer this exact request armed).
	cnfSendPacket(CNF_PKT_TYPE_PULL, CNF_SUBTYPE_DONE, entry, NULL, 0);

	if(CNF_STATUS_BUSY == status)
		return SYNC_BUSY;
	if(CNF_STATUS_BAD_ENTRY == status)
		return SYNC_BAD_ENTRY;
	return SYNC_OK;  // OK or EMPTY - either way *outLayoutVersion is valid
}
