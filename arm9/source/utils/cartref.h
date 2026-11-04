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
} CartRefSector;

typedef struct {
    CartRefSector* s;
    uint32_t count;
    uint32_t capacity;
    bool overflow;        // more bad sectors existed than fit in the list
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

// Compare the given ranges of cartridge and reference, adding every differing
// sector to `list`. `buf` must hold CARTREF_SCAN_CHUNK sectors plus
// CARTREF_SCAN_CHUNK bytes (match flags).
CartRefResult CartRef_Scan(const CartRefIo* io, const CartRefRange* ranges, uint32_t n_ranges,
                           CartRefList* list, uint8_t* buf);

// Work on every PENDING sector of `list` until all are resolved or the user
// stops. Sectors are visited round-robin, a few attempts at a time.
CartRefResult CartRef_Repair(const CartRefIo* io, const CartRefParams* p, CartRefList* list);

// Resume state (serialised list of unresolved sectors).
typedef struct {
    uint32_t magic;        // CARTREF_MAGIC
    uint32_t version;
    uint64_t title_id;
    uint64_t cart_size;    // bytes
    uint64_t ref_size;     // bytes
    uint32_t count;        // number of u32 sector entries that follow
    uint32_t reserved;
} CartRefResumeHeader;

#define CARTREF_MAGIC   0x31464352u  // "CRF1"
#define CARTREF_VERSION 2u

// Serialise unresolved (PENDING / UNFIXABLE / SKIPPED) sectors into `out`
// (header + u32 list). Returns the byte size, or 0 if `out_size` is too small.
size_t CartRef_SaveState(const CartRefList* l, const CartRefResumeHeader* hdr, void* out, size_t out_size);
// Restore a list from saved state. Restored sectors start as PENDING.
// Returns false if the data is invalid or does not match `expect`.
bool CartRef_LoadState(CartRefList* l, const CartRefResumeHeader* expect, const void* data, size_t size);
