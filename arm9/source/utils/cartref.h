#pragma once

// Reference-guided cartridge repair engine.
//
// Instead of re-reading whole hash units (an ExeFS file can be several MB with
// a single hash), the cartridge is compared sector by sector (0x200 bytes)
// against a known-good image of the same title. Only the sectors that differ
// are then re-read together with the cartridge refresh command, until the
// cartridge controller has restored them.
//
// This file is hardware independent (it is also built by the host unit tests
// in tests/cartref). All I/O goes through CartRefIo.

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define CARTREF_SECTOR      0x200u
#define CARTREF_SCAN_CHUNK  0x800u   // sectors per scan read (1 MB)
#define CARTREF_RUN_MAX     4u       // neighbouring bad sectors handled by one refresh + read
#define CARTREF_COLD_EVERY  8u       // default: runs without near misses are visited every Nth pass
#define CARTREF_NEAR_BURST  8u       // default: extra attempts right after a near miss ...
#define CARTREF_BURST_MAX   64u      // ... up to this many attempts in one visit
#define CARTREF_WAIT_MS     5u       // "wait" strategy: pause between refresh and read
#define CARTREF_WIDE        8u       // "wide" strategy: read the aligned 4 KB block around the run

// How much a sector matters, set by the caller from where it lies.
enum {
    CARTREF_PRIO_CRITICAL = 0, // headers, exheader, ExeFS: the game does not start without them
    CARTREF_PRIO_NORMAL,       // RomFS and anything else
    CARTREF_PRIO_LOW           // partitions not needed to play (manual, update data)
};

// Read strategies. Each attempt uses one of them; their time shares adapt to
// how well each works on the cartridge (see CartRef_StrategyShares).
enum {
    CARTREF_STRAT_NORMAL = 0, // refresh + read of the run
    CARTREF_STRAT_WIDE,       // refresh + read of the whole aligned 4 KB block
    CARTREF_STRAT_WAIT,       // refresh, wait CARTREF_WAIT_MS, plain read
    CARTREF_STRAT_DOUBLE,     // extra refresh, then refresh + read
    CARTREF_STRAT_SINGLE,     // refresh + read of one sector of the run alone
    CARTREF_STRAT_STREAM,     // one refresh, then up to 8 plain reads
    CARTREF_STRATS
};
#define CARTREF_STRATS_ALL ((1u << CARTREF_STRATS) - 1)

typedef struct {
    uint64_t attempts;    // refreshed reads with this strategy
    uint64_t near_misses; // ... that matched but failed confirmation (per sector)
    uint32_t fixed;       // sectors fixed by it
    uint64_t msec;        // time spent (attempt + confirmation), if io->msec is set
    uint32_t reserved;
} CartRefStratStats;

typedef struct {         // the same, fading with time (what the choice is based on)
    double fixed, near, cost;
} CartRefStratDecay;

typedef enum {
    CARTREF_PENDING = 0,
    CARTREF_FIXED,
    CARTREF_UNFIXABLE,   // data stopped changing (stuck limit reached)
    CARTREF_SKIPPED,     // retry limit reached and skipped (autoskip / Y)
} CartRefState;

typedef struct {
    uint32_t sector;      // absolute cartridge sector
    uint32_t attempts;    // refresh+read attempts so far
    uint32_t last_sum;    // checksum of the last mismatching read
    uint16_t stuck;       // consecutive identical mismatching reads
    uint8_t  confirm;     // consecutive matching confirmation reads
    uint8_t  state;       // CartRefState
    uint8_t  prio;        // CARTREF_PRIO_* (set by the caller; not saved)
    uint32_t near_miss;   // refreshed reads that matched but failed confirmation
} CartRefSector;

typedef struct {
    CartRefSector* s;
    uint32_t count;
    uint32_t capacity;
    bool overflow;        // more bad sectors existed than fit in the list
    CartRefStratStats strat[CARTREF_STRATS]; // accumulated over every repair round
    CartRefStratDecay dec[CARTREF_STRATS];   // recent results (half-life one hour)
    bool dec_ready;
    double dec_clock;
} CartRefList;

typedef struct {
    uint32_t start;       // first sector
    uint32_t count;       // number of sectors
} CartRefRange;

typedef struct {
    uint32_t stuck_limit;         // identical bad reads before giving up on a sector
    uint32_t retries_before_skip; // attempts before a sector may be skipped
    bool autoskip;                // skip automatically at retries_before_skip
    uint32_t attempts_per_visit;  // attempts on one sector before moving to the next
    uint32_t confirm_reads;       // matching plain reads required to call it fixed
    uint32_t cold_every;          // visit runs without near misses every Nth pass (0/1: every pass)
    uint32_t near_burst;          // extra attempts after a near miss (0: none)
    uint32_t strategies;          // bit mask of CARTREF_STRAT_* to use (bit 0 alone: normal only)
    uint32_t low_every;           // visit CARTREF_PRIO_LOW runs every Nth pass (0/1: every pass)
    uint32_t critical_boost;      // CARTREF_PRIO_CRITICAL runs get this many times the attempts per visit
    uint32_t min_round_ms;        // with io->msec: a round lasts at least this long before skipping
} CartRefParams;

typedef enum {
    CARTREF_PHASE_SCAN = 0,
    CARTREF_PHASE_REPAIR,
} CartRefPhase;

typedef struct {
    CartRefPhase phase;
    uint64_t done, total;          // scan: sectors scanned / total
    uint32_t pass;                 // repair: current pass number (1-based)
    uint32_t pending, fixed, unfixable, skipped;
    uint32_t current_sector;       // repair: sector being worked on
    uint32_t current_attempts;
    uint64_t total_attempts;
    uint64_t near_misses;          // repair: refreshed matches that did not hold
    uint32_t hot;                  // repair: pending sectors that had a near miss
} CartRefStatus;

typedef struct {
    void* ctx;
    // Read `count` sectors from the cartridge. With `refresh` set, the refresh
    // command is sent before the read. Returns 0 on success.
    int  (*cart_read)(void* ctx, uint32_t sector, uint32_t count, void* buf, bool refresh);
    // Check `count` sectors of cartridge data (starting at `sector`) against
    // the reference (a full image or a per-sector hash list). Sets match[i] to
    // 1 for every sector that is correct, 0 otherwise. Returns 0 on success.
    int  (*ref_compare)(void* ctx, uint32_t sector, uint32_t count, const void* cart_buf, uint8_t* match);
    // Called once per repair pass (lets the fault simulator advance).
    void (*begin_pass)(void* ctx);
    // Return true to stop (user pressed B).
    bool (*poll_abort)(void* ctx);
    // Return true if the user asks to skip the current sector (held Y).
    bool (*poll_skip)(void* ctx);
    // Progress display. May be NULL.
    void (*status)(void* ctx, const CartRefStatus* st);
    // Called after every repair pass (save progress, write log). May be NULL.
    void (*end_pass)(void* ctx, const CartRefStatus* st);
    // Optional (strategies): send the refresh command alone (0 = ok), wait,
    // and a millisecond clock for the time spent per strategy.
    int  (*refresh_only)(void* ctx);
    void (*wait_ms)(void* ctx, uint32_t ms);
    uint32_t (*msec)(void* ctx);
} CartRefIo;

typedef enum {
    CARTREF_OK = 0,        // finished (every sector fixed, unfixable or skipped)
    CARTREF_ABORTED,       // user stopped; list holds the remaining work
    CARTREF_CART_ERROR,    // cartridge stopped responding
    CARTREF_REF_ERROR,     // reference image could not be read
    CARTREF_NO_MEMORY,
} CartRefResult;

void CartRef_DefaultParams(CartRefParams* p);

bool CartRef_ListInit(CartRefList* l, uint32_t capacity);
void CartRef_ListFree(CartRefList* l);
bool CartRef_ListAdd(CartRefList* l, uint32_t sector);
void CartRef_Count(const CartRefList* l, CartRefStatus* st);
// Put every UNFIXABLE / SKIPPED sector back to PENDING with fresh counters
// (for another repair round). Returns how many were reset. near_miss is kept.
uint32_t CartRef_ResetUnresolved(CartRefList* l);

// Compare the given ranges of cartridge and reference, adding every differing
// sector to `list`. `buf` must hold CARTREF_SCAN_CHUNK sectors plus
// CARTREF_SCAN_CHUNK bytes (match flags).
CartRefResult CartRef_Scan(const CartRefIo* io, const CartRefRange* ranges, uint32_t n_ranges,
                           CartRefList* list, uint8_t* buf);

// Work on every PENDING sector of `list` until all are resolved or the user
// stops. Sectors are visited round-robin, a few attempts at a time. Runs that
// have had a near miss (the only ones real logs show getting fixed) are
// visited every pass, the others every `cold_every` passes; a near miss also
// extends the current visit by `near_burst` attempts. The skip
// limit counts passes (retries_before_skip / attempts_per_visit), so every
// sector of a round reaches it at the same time.
CartRefResult CartRef_Repair(const CartRefIo* io, const CartRefParams* p, CartRefList* list);

// Current time share (0..1) of every read strategy, from the statistics in
// list->dec (unusable or disabled strategies get 0).
void CartRef_StrategyShares(const CartRefIo* io, const CartRefParams* p, const CartRefList* l, double* share);

// Read every FIXED sector again (plain reads, no refresh) and put those that
// no longer match back to PENDING. Returns how many relapsed; `checked` (may be
// NULL) gets how many were read successfully.
#define CARTREF_RECHECK_RUN 64u
uint32_t CartRef_Recheck(const CartRefIo* io, CartRefList* l, uint32_t* checked);

// Let the recent figures fade by `cost_units` (seconds, or k-attempts without a clock).
void CartRef_DecayStrategies(CartRefList* l, double cost_units);

// Resume state (serialised list of unresolved sectors).
typedef struct {
    uint32_t magic;        // CARTREF_MAGIC
    uint32_t version;
    uint64_t title_id;
    uint64_t cart_size;    // bytes
    uint64_t ref_size;     // bytes
    uint32_t count;        // number of entries that follow (v3: u32 sector + u32 near misses; v2: u32 sector)
    uint32_t reserved;
} CartRefResumeHeader;

#define CARTREF_MAGIC   0x31464352u  // "CRF1"
#define CARTREF_VERSION 3u
#define CARTREF_STATE_ENTRY 8u    // bytes per saved sector (v3)

// Serialise unresolved (PENDING / UNFIXABLE / SKIPPED) sectors into `out`
// (header + entries). Returns the byte size, or 0 if `out_size` is too small.
size_t CartRef_SaveState(const CartRefList* l, const CartRefResumeHeader* hdr, void* out, size_t out_size);
// Restore a list from saved state (v3, or v2 without near misses). Restored
// sectors start as PENDING.
// Returns false if the data is invalid or does not match `expect`.
bool CartRef_LoadState(CartRefList* l, const CartRefResumeHeader* expect, const void* data, size_t size);
