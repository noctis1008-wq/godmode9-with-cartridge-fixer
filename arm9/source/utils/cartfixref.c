// "Fix with reference image": GodMode9 front end for the reference-guided
// cartridge repair engine (cartref.c).

#include "cartfixref.h"
#include "cartref.h"
#include "cartrefhash.h"
#include "sha.h"
#include "gameutil.h"
#include "fsutil.h"
#include "game.h"
#include "hid.h"
#include "ui.h"
#include "fs.h"
#include "timer.h"
#include "fixersim.h"
#include "command_ctr.h"
#include "fsinit.h"
#include "power.h"
#include "rtc.h"

#include <stdarg.h>

#define REF_LIST_CAPACITY   (256u * 1024u)       // max. bad sectors tracked per run (128 MB of data)
#define REF_HDR_TOLERANCE   32                   // differing bytes allowed in a damaged NCCH header
#define REF_STATUS_MS       150
#define REF_HASH_DIR        "0:/luma/payloads/hash"
#define REF_LOG_DIR         "0:/luma/payloads/log"
#define REF_LOG_EVENTS_SIZE (1024u * 1024u)      // event part of the log
#define REF_LOG_EVERY_PASSES 100                  // a quiet pass is logged only this often
#define REF_SAVE_MS          60000                // progress/log saved at least this often
#define REF_LOG_TABLE_SIZE  STD_BUFFER_SIZE      // per-sector table (rebuilt on every save)
#define REF_LOG_MAX_LINES   20000
#define REF_COUNTDOWN_SEC   10
#define REF_SHUTDOWN_SEC    30                   // power off this long after everything is fixed
#define REF_HISTORY         10                   // rounds shown in the "fixed per round" rows
#define REF_CFG_PATH        OUTPUT_PATH "/reffix.cfg"
#define REF_HASH_SECTORS    0x800u               // sectors hashed per read when creating a .crh

extern bool force_refresh;                       // command_ctr.c: refresh before every read
u32 LoadNcsdHeader(NcsdHeader* ncsd, const char* path); // gameutil.c

// The reference: either a full .3ds image or a .crh hash list.
typedef struct {
    bool is_hash;
    FIL file;
    NcsdHeader ncsd;
    u8 ncch[8][0x200];        // each partition's NCCH header
    u8 exefs[8][0x200];       // each partition's ExeFS header (decrypted refs only)
    u64 image_size;
    u32 table_offset;         // hash list only
    u8* chunk;                // reference data / hashes for one scan chunk
    u32 cache_sector;         // small cache for the repair phase (one run)
    u32 cache_count;
    bool cache_valid;
    u8 cache[CARTREF_RUN_MAX * CARTREF_SECTOR];
} RefSource;

// One cartridge partition that is compared against the reference.
typedef struct {
    u32 start, count;         // sectors
    bool decrypt;             // reference is decrypted: decrypt cart data first
    NcchHeader ncch;          // cartridge NCCH header (crypto flags of the cart)
    ExeFsHeader exefs;        // decrypted ExeFS header (from the reference)
    bool has_exefs;
} RefPart;

typedef struct {
    FIL cart;
    RefSource* ref;
    RefPart parts[8];
    u32 part_index[8];        // NCSD partition number of parts[i]
    u32 n_parts;
    u8* scratch;              // CARTREF_SCAN_CHUNK sectors, for decrypting cart data
    // log + progress saving
    char* log;                // event log text
    size_t log_len;
    bool log_full;
    char log_path[96];
    CartRefList* list;
    const CartRefResumeHeader* rhdr;
    const CartRefIo* io;          // for the strategy shares on screen and in the log
    const CartRefParams* params;
    const char* state_file;
    u64 pass_timer;
    u64 attempts_before_pass;
    u32 round;                // repair round (auto retry)
    u32 last_resolved;        // fixed+unfixable+skipped at the last logged pass
    u32 quiet_passes;         // passes since the last logged one
    u64 save_timer;           // since the last progress/log save
    u32 fixed_at_round_start; // to count the sectors fixed in the current round
    u32 round_fixed[REF_HISTORY]; // sectors fixed in the last finished rounds (ring buffer)
    u32 rounds_done;
    u64 status_timer;
    u64 run_timer;
} RefCtx;

static void log_printf(RefCtx* ctx, const char* fmt, ...) __attribute__((format(printf, 2, 3)));
static void log_printf(RefCtx* ctx, const char* fmt, ...) {
    if (!ctx->log || ctx->log_full) return;
    size_t avail = REF_LOG_EVENTS_SIZE - ctx->log_len;
    if (avail < 64) {
        ctx->log_len += snprintf(ctx->log + ctx->log_len, avail, "[event log full]\n");
        ctx->log_full = true;
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(ctx->log + ctx->log_len, avail, fmt, ap);
    va_end(ap);
    if (n > 0) ctx->log_len += ((size_t) n < avail) ? (size_t) n : avail - 1;
}

static void log_time(RefCtx* ctx) {
    DsTime t;
    get_dstime(&t);
    log_printf(ctx, "[20%02lX-%02lX-%02lX %02lX:%02lX:%02lX] ", (u32) t.bcd_Y, (u32) t.bcd_M, (u32) t.bcd_D,
        (u32) t.bcd_h, (u32) t.bcd_m, (u32) t.bcd_s);
}

// Which part of the NCCH a sector belongs to (for the log).
enum { RG_HEADER = 0, RG_EXHEADER, RG_LOGO, RG_PLAIN, RG_EXEFS, RG_ROMFS, RG_OTHER, RG_COUNT };
static const char* rg_names[RG_COUNT] = { "header", "exheader", "logo", "plain", "exefs", "romfs", "other" };

static u32 sector_region(const RefCtx* ctx, u32 sector, u32* part_out) {
    for (u32 p = 0; p < ctx->n_parts; p++) {
        const RefPart* part = ctx->parts + p;
        if ((sector < part->start) || (sector >= part->start + part->count)) continue;
        if (part_out) *part_out = ctx->part_index[p];
        const NcchHeader* h = &part->ncch;
        u32 u = sector - part->start; // in 0x200 units == NCCH media units
        if (u == 0) return RG_HEADER;
        if (h->size_exthdr && (u < 1 + 4)) return RG_EXHEADER;   // 0x200..0xA00 (exheader + AccessDesc)
        if (h->size_logo && (u >= h->offset_logo) && (u < h->offset_logo + h->size_logo)) return RG_LOGO;
        if (h->size_plain && (u >= h->offset_plain) && (u < h->offset_plain + h->size_plain)) return RG_PLAIN;
        if (h->size_exefs && (u >= h->offset_exefs) && (u < h->offset_exefs + h->size_exefs)) return RG_EXEFS;
        if (h->size_romfs && (u >= h->offset_romfs) && (u < h->offset_romfs + h->size_romfs)) return RG_ROMFS;
        return RG_OTHER;
    }
    if (part_out) *part_out = 0xFF;
    return RG_OTHER;
}

static void log_distribution(RefCtx* ctx, const CartRefList* list, bool only_unresolved) {
    u32 counts[8][RG_COUNT];
    memset(counts, 0, sizeof(counts));
    for (u32 i = 0; i < list->count; i++) {
        if (only_unresolved && (list->s[i].state == CARTREF_FIXED)) continue;
        u32 part = 0;
        u32 rg = sector_region(ctx, list->s[i].sector, &part);
        if (part < 8) counts[part][rg]++;
    }
    for (u32 p = 0; p < 8; p++) {
        u32 total = 0;
        for (u32 r = 0; r < RG_COUNT; r++) total += counts[p][r];
        if (!total) continue;
        log_printf(ctx, "  content%lu: %lu sectors (", p, total);
        for (u32 r = 0; r < RG_COUNT; r++)
            if (counts[p][r]) log_printf(ctx, " %s=%lu", rg_names[r], counts[p][r]);
        log_printf(ctx, " )\n");
    }
}

// Write the log file: event log + current per-sector table.
static void log_save(RefCtx* ctx) {
    if (!ctx->log || !*ctx->log_path) return;
    fvx_rmkdir(REF_LOG_DIR);
    if (!FileSetData(ctx->log_path, ctx->log, ctx->log_len, 0, true)) return;
    if (!ctx->list || !ctx->list->count) return;

    char* buf = (char*) malloc(REF_LOG_TABLE_SIZE);
    if (!buf) return;
    static const char* names[] = { "PENDING", "FIXED", "UNFIXABLE", "SKIPPED" };
    char* w = buf;
    char* end = buf + REF_LOG_TABLE_SIZE - 64;
    w += snprintf(w, end - w, "\n--- sectors (cart offset, state, region, attempts, stuck, near misses) ---\n");
    u32 i = 0;
    for (; (i < ctx->list->count) && (i < REF_LOG_MAX_LINES) && (w < end); i++) {
        const CartRefSector* e = &ctx->list->s[i];
        u32 part = 0;
        u32 rg = sector_region(ctx, e->sector, &part);
        w += snprintf(w, end - w, "%09llX %-9s c%lu/%-8s %6lu %4u %5lu\n",
            (unsigned long long) e->sector * CARTREF_SECTOR, names[e->state & 3], part, rg_names[rg],
            (unsigned long) e->attempts, (unsigned) e->stuck, (unsigned long) e->near_miss);
    }
    if (i < ctx->list->count) w += snprintf(w, buf + REF_LOG_TABLE_SIZE - w, "[%lu more not listed]\n", ctx->list->count - i);
    FileSetData(ctx->log_path, buf, w - buf, ctx->log_len, false);
    free(buf);
}

// What the read strategies achieved, carried over to the next cartridge as a
// starting point (scaled down so the new cartridge's own results soon rule).
#define REF_LEARN_FILE   OUTPUT_PATH "/reffix_learn.dat"
#define REF_LEARN_MAGIC  0x4E524C52u  // "RLRN"
#define REF_LEARN_MAX_MS (10ull * 60 * 1000) // at most 10 minutes' worth of old results

typedef struct {
    u32 magic, count;
    CartRefStratStats s[CARTREF_STRATS];
} RefLearn;

static void learn_load(CartRefList* list) {
    RefLearn lr;
    if ((FileGetData(REF_LEARN_FILE, &lr, sizeof(lr), 0) != sizeof(lr)) ||
        (lr.magic != REF_LEARN_MAGIC) || (lr.count != CARTREF_STRATS)) return;
    u64 total = 0;
    for (u32 i = 0; i < CARTREF_STRATS; i++) total += lr.s[i].msec;
    for (u32 i = 0; i < CARTREF_STRATS; i++) {
        CartRefStratStats* d = &list->strat[i];
        *d = lr.s[i];
        if (total > REF_LEARN_MAX_MS) { // keep the ratios, shrink the weight
            d->msec        = d->msec * REF_LEARN_MAX_MS / total;
            d->attempts    = d->attempts * REF_LEARN_MAX_MS / total;
            d->near_misses = d->near_misses * REF_LEARN_MAX_MS / total;
            d->fixed       = (u32) ((u64) d->fixed * REF_LEARN_MAX_MS / total);
        }
    }
}

static void learn_save(const CartRefList* list) {
    RefLearn lr;
    lr.magic = REF_LEARN_MAGIC;
    lr.count = CARTREF_STRATS;
    memcpy(lr.s, list->strat, sizeof(lr.s));
    fvx_rmkdir(OUTPUT_PATH);
    FileSetData(REF_LEARN_FILE, &lr, sizeof(lr), 0, true);
}

static bool save_state(RefCtx* ctx) {
    if (!ctx->list || !ctx->rhdr || !ctx->state_file) return false;
    if (ctx->io) learn_save(ctx->list); // only once repairing (io set)
    CartRefStatus st;
    CartRef_Count(ctx->list, &st);
    if (!st.pending && !st.unfixable && !st.skipped) {
        fvx_unlink(ctx->state_file);
        return false;
    }
    size_t max = sizeof(CartRefResumeHeader) + (size_t) ctx->list->count * CARTREF_STATE_ENTRY;
    void* sdata = malloc(max);
    size_t sz = sdata ? CartRef_SaveState(ctx->list, ctx->rhdr, sdata, max) : 0;
    bool ok = false;
    if (sz) {
        fvx_rmkdir(OUTPUT_PATH);
        ok = FileSetData(ctx->state_file, sdata, sz, 0, true);
    }
    free(sdata);
    return ok;
}

static void io_end_pass(void* vctx, const CartRefStatus* st) {
    RefCtx* ctx = (RefCtx*) vctx;
    // A pass takes about a second, so logging and saving every one of them
    // filled the log within the first hours and wrote the SD card nonstop.
    // Log a pass when a sector got resolved (or every REF_LOG_EVERY_PASSES),
    // save when something changed or once a minute.
    u32 resolved = st->fixed + st->unfixable + st->skipped;
    bool changed = resolved != ctx->last_resolved;
    if (changed || (++ctx->quiet_passes >= REF_LOG_EVERY_PASSES)) {
        log_time(ctx);
        log_printf(ctx, "round %lu pass %lu: remaining %lu, fixed %lu, unfixable %lu, skipped %lu, refresh reads %llu since last line (total %llu), near misses %llu, near-miss sectors %lu\n",
            ctx->round, st->pass, st->pending, st->fixed, st->unfixable, st->skipped,
            (unsigned long long) (st->total_attempts - ctx->attempts_before_pass), (unsigned long long) st->total_attempts,
            (unsigned long long) st->near_misses, (unsigned long) st->hot);
        ctx->attempts_before_pass = st->total_attempts;
        ctx->quiet_passes         = 0;
    }
    ctx->pass_timer = timer_start();
    if (changed || (timer_msec(ctx->save_timer) >= REF_SAVE_MS)) {
        ctx->last_resolved = resolved;
        ctx->save_timer    = timer_start();
        save_state(ctx);
        log_save(ctx);
    }
}

// Wait `secs` seconds. Returns false if the user cancelled with B.
static bool countdown_secs(const char* msg, const char* what, u32 secs) {
    for (u32 t = secs; t > 0; t--) {
        ShowString("%s\n \n%s in %lu s.\n \n(A) now   (B) cancel", msg, what, t);
        u64 t0 = timer_start();
        while (timer_msec(t0) < 1000) {
            if (CheckButton(BUTTON_A)) return true;
            if (CheckButton(BUTTON_B)) return false;
        }
    }
    return true;
}

static bool load_autoretry(void) {
    char buf[32] = { 0 };
    size_t n = FileGetData(REF_CFG_PATH, buf, sizeof(buf) - 1, 0);
    if (!n) return true; // default: on
    return strstr(buf, "autoretry=0") == NULL;
}

static void save_autoretry(bool on) {
    const char* txt = on ? "autoretry=1\n" : "autoretry=0\n";
    fvx_rmkdir(OUTPUT_PATH);
    FileSetData(REF_CFG_PATH, txt, strlen(txt), 0, true);
}

// Ask for the auto retry setting (remembered). Returns false if cancelled.
static bool choose_autoretry(bool* autoretry) {
    bool last = load_autoretry();
    const char* opts[2];
    opts[0] = last ? "Auto retry: ON (last used)" : "Auto retry: ON";
    opts[1] = last ? "Auto retry: OFF" : "Auto retry: OFF (last used)";
    u32 sel = ShowSelectPrompt(2, opts,
        "Auto retry\n \nWhen a round ends with sectors still\nnot fixed, start the next round\nright away (B pauses at any time)?");
    if (!sel) return false;
    *autoretry = (sel == 1);
    if (*autoretry != last) save_autoretry(*autoretry);
    return true;
}

static bool countdown(const char* msg, const char* what) {
    char w[48];
    snprintf(w, sizeof(w), "%s starts", what);
    return countdown_secs(msg, w, REF_COUNTDOWN_SEC);
}

static int io_cart_read(void* vctx, u32 sector, u32 count, void* buf, bool refresh) {
    RefCtx* ctx = (RefCtx*) vctx;
    u64 offset = (u64) sector * CARTREF_SECTOR;
    UINT size = count * CARTREF_SECTOR;
    UINT br = 0;
    if (fvx_lseek(&ctx->cart, offset) != FR_OK) return 1;
    force_refresh = refresh;
    FRESULT fr = fvx_read(&ctx->cart, buf, size, &br);
    force_refresh = false;
    fr = (FRESULT) FixerSim_FilterRead((int) fr, buf, size, offset);
    return ((fr != FR_OK) || (br != size)) ? 1 : 0;
}

// optional engine calls for the read strategies
static int io_refresh_only(void* vctx) {
    (void) vctx;
    return CTR_Refresh() ? 0 : 1;
}

static void io_wait_ms(void* vctx, u32 ms) {
    (void) vctx;
    wait_msec(ms);
}

static u64 io_clock = 0;
static u32 io_msec(void* vctx) {
    (void) vctx;
    return (u32) timer_msec(io_clock);
}

static const char* strat_name[CARTREF_STRATS] = { "normal", "wide 4KB", "wait 5ms", "refresh x2", "single", "stream" };

// one log line per read strategy: how often it was used, what it achieved, its time share now
static void log_strats(RefCtx* ctx) {
    if (!ctx->list || !ctx->io || !ctx->params) return;
    double share[CARTREF_STRATS];
    CartRef_StrategyShares(ctx->io, ctx->params, ctx->list, share);
    for (u32 i = 0; i < CARTREF_STRATS; i++) {
        const CartRefStratStats* s = &ctx->list->strat[i];
        if (!s->attempts) continue;
        log_printf(ctx, "  strategy %-10s: %llu attempts, %llu s, near misses %llu (%llu per 10k attempts), fixed %lu, share now %lu%%\n",
            strat_name[i], (unsigned long long) s->attempts, (unsigned long long) (s->msec / 1000),
            (unsigned long long) s->near_misses, (unsigned long long) (s->near_misses * 10000 / s->attempts),
            (unsigned long) s->fixed, (unsigned long) (share[i] * 100 + 0.5));
    }
}

static int ref_read_at(RefSource* ref, u64 offset, void* buf, UINT size) {
    UINT br = 0;
    if (fvx_lseek(&ref->file, offset) != FR_OK) return 1;
    if ((fvx_read(&ref->file, buf, size, &br) != FR_OK) || (br != size)) return 1;
    return 0;
}

// Reference bytes for `count` sectors: image data, or `CRH_HASH_SIZE` bytes per sector.
static int ref_load(RefSource* ref, u32 sector, u32 count, u8* out) {
    if (ref->is_hash)
        return ref_read_at(ref, ref->table_offset + (u64) sector * CRH_HASH_SIZE, out, count * CRH_HASH_SIZE);
    return ref_read_at(ref, (u64) sector * CARTREF_SECTOR, out, count * CARTREF_SECTOR);
}

static bool ref_match_one(RefSource* ref, const u8* cart, const u8* refdata) {
    if (ref->is_hash) {
        u8 sha[32];
        sha_quick(sha, cart, CARTREF_SECTOR, SHA256_MODE);
        return memcmp(sha, refdata, CRH_HASH_SIZE) == 0;
    }
    return memcmp(cart, refdata, CARTREF_SECTOR) == 0;
}

static int io_ref_compare(void* vctx, u32 sector, u32 count, const void* cart_buf, u8* match) {
    RefCtx* ctx = (RefCtx*) vctx;
    RefSource* ref = ctx->ref;
    const u8* cart = (const u8*) cart_buf;

    if (count > CARTREF_SCAN_CHUNK) return 1;

    // decrypted reference: bring the cartridge data into the same (decrypted) form
    for (u32 p = 0; p < ctx->n_parts; p++) {
        RefPart* part = ctx->parts + p;
        if ((sector < part->start) || (sector >= part->start + part->count)) continue;
        if (sector + count > part->start + part->count) {
            // a repair run that crosses into the next partition: one sector at a time
            for (u32 i = 0; i < count; i++)
                if (io_ref_compare(vctx, sector + i, 1, cart + (size_t) i * CARTREF_SECTOR, match + i) != 0)
                    return 1;
            return 0;
        }
        if (!part->decrypt) break;
        memcpy(ctx->scratch, cart, (size_t) count * CARTREF_SECTOR);
        if (CryptNcch(ctx->scratch, (sector - part->start) * CARTREF_SECTOR, count * CARTREF_SECTOR,
                &part->ncch, part->has_exefs ? &part->exefs : NULL, NCCH_NOCRYPTO) != 0)
            return 1;
        cart = ctx->scratch;
        break;
    }
    u32 unit = ref->is_hash ? CRH_HASH_SIZE : CARTREF_SECTOR;
    const u8* refdata;

    if (count <= CARTREF_RUN_MAX) { // repair phase: the same run is checked many times in a row
        if (!ref->cache_valid || (ref->cache_sector != sector) || (ref->cache_count != count)) {
            if (ref_load(ref, sector, count, ref->cache) != 0) return 1;
            ref->cache_sector = sector;
            ref->cache_count  = count;
            ref->cache_valid  = true;
        }
        refdata = ref->cache;
    } else {
        if (ref_load(ref, sector, count, ref->chunk) != 0) return 1;
        refdata = ref->chunk;
    }

    for (u32 i = 0; i < count; i++)
        match[i] = ref_match_one(ref, cart + (size_t) i * CARTREF_SECTOR, refdata + (size_t) i * unit) ? 1 : 0;
    return 0;
}

// Open a reference (.3ds or .crh). Returns NULL and shows a message on failure.
static RefSource* ref_open(const char* path) {
    char pathstr[UTF_BUFFER_BYTESIZE(32)];
    TruncateString(pathstr, path, 32, 8);

    RefSource* ref = (RefSource*) malloc(sizeof(RefSource));
    if (!ref) return NULL;
    memset(ref, 0, sizeof(RefSource));
    if (fvx_open(&ref->file, path, FA_READ | FA_OPEN_EXISTING) != FR_OK) {
        free(ref);
        ShowPrompt(false, "%s\nCannot open file.", pathstr);
        return NULL;
    }

    u64 fsize = fvx_size(&ref->file);
    CrhHeader crh;
    bool ok = false;
    if ((fsize >= CRH_TABLE_OFFSET) && (ref_read_at(ref, 0, &crh, sizeof(crh)) == 0) &&
        (memcmp(crh.magic, CRH_MAGIC, 4) == 0)) {
        ref->is_hash = true;
        ok = (crh.version == CRH_VERSION) && (crh.hash_size == CRH_HASH_SIZE) &&
            (crh.table_offset >= CRH_TABLE_OFFSET) &&
            (crh.sector_count == crh.image_size / CARTREF_SECTOR) &&
            (fsize == crh.table_offset + (u64) crh.sector_count * CRH_HASH_SIZE) &&
            (ref_read_at(ref, CRH_NCSD_OFFSET, &ref->ncsd, sizeof(NcsdHeader)) == 0) &&
            (ref_read_at(ref, CRH_NCCH_OFFSET, ref->ncch, sizeof(ref->ncch)) == 0) &&
            (ref_read_at(ref, CRH_EXEFS_OFFSET, ref->exefs, sizeof(ref->exefs)) == 0) &&
            (ValidateNcsdHeader(&ref->ncsd) == 0) && (ref->ncsd.mediaId == crh.title_id);
        ref->image_size = crh.image_size;
        ref->table_offset = crh.table_offset;
        if (!ok) ShowPrompt(false, "%s\nInvalid or outdated .crh hash file.\nCreate it again with make_crh.py.", pathstr);
    } else {
        ref->image_size = fsize;
        ok = (ref_read_at(ref, 0, &ref->ncsd, sizeof(NcsdHeader)) == 0) &&
            (ValidateNcsdHeader(&ref->ncsd) == 0);
        for (u32 i = 0; ok && (i < 8); i++) {
            NcchPartition* p = ref->ncsd.partitions + i;
            u64 off = (u64) p->offset * NCSD_MEDIA_UNIT;
            if (p->size && (off + 0x200 <= fsize)) {
                NcchHeader* h = (NcchHeader*) (void*) ref->ncch[i];
                ok = (ref_read_at(ref, off, h, 0x200) == 0);
                u64 exefs_off = off + (u64) h->offset_exefs * NCCH_MEDIA_UNIT;
                if (ok && (h->flags[7] & 0x04) && h->size_exefs && (exefs_off + 0x200 <= fsize))
                    ok = (ref_read_at(ref, exefs_off, ref->exefs[i], 0x200) == 0);
            }
        }
        if (!ok) ShowPrompt(false, "%s\nNot a valid .3ds or .crh file.", pathstr);
    }

    if (ok) {
        ref->chunk = (u8*) malloc((size_t) CARTREF_SCAN_CHUNK * (ref->is_hash ? CRH_HASH_SIZE : CARTREF_SECTOR));
        if (!ref->chunk) {
            ShowPrompt(false, "Out of memory.");
            ok = false;
        }
    }
    if (!ok) {
        fvx_close(&ref->file);
        free(ref->chunk);
        free(ref);
        return NULL;
    }
    return ref;
}

static void ref_close(RefSource* ref) {
    if (!ref) return;
    fvx_close(&ref->file);
    free(ref->chunk);
    free(ref);
}

static void io_begin_pass(void* vctx) {
    (void) vctx;
    FixerSim_BeginAttempt();
}

static bool io_poll_abort(void* vctx) {
    (void) vctx;
    return CheckButton(BUTTON_B);
}

static bool io_poll_skip(void* vctx) {
    (void) vctx;
    return CheckButton(BUTTON_Y);
}

static void io_status(void* vctx, const CartRefStatus* st) {
    RefCtx* ctx = (RefCtx*) vctx;

    if (st->phase == CARTREF_PHASE_SCAN) {
        char opstr[64];
        snprintf(opstr, sizeof(opstr), "Scanning... bad sectors: %lu", (unsigned long) st->pending);
        ShowProgress(st->done, st->total, opstr);
        return;
    }

    if (timer_msec(ctx->status_timer) < REF_STATUS_MS) return;
    ctx->status_timer = timer_start();

    char line[64];
    const int x = 8;
    int y = 40;
    const int lh = 12;
    u64 sec = timer_sec(ctx->run_timer);

    // version in the top right corner (the line "Refresh count:" holds the left side)
    DrawString(MAIN_SCREEN, VERSION, SCREEN_WIDTH_MAIN - (int) strlen(VERSION) * FONT_WIDTH_EXT - 4, 0,
        COLOR_GREY, COLOR_STD_BG);
    DrawString(MAIN_SCREEN, "Fix with reference image", x, y, COLOR_STD_FONT, COLOR_STD_BG); y += 2 * lh;
    snprintf(line, sizeof(line), "Round %-3lu Pass %-6lu Time %02lu:%02lu:%02lu   ", (unsigned long) ctx->round, (unsigned long) st->pass,
        (unsigned long) (sec / 3600), (unsigned long) ((sec / 60) % 60), (unsigned long) (sec % 60));
    DrawString(MAIN_SCREEN, line, x, y, COLOR_STD_FONT, COLOR_STD_BG); y += lh;
    snprintf(line, sizeof(line), "Remaining  %-6lu (near miss: %lu)        ", (unsigned long) st->pending,
        (unsigned long) st->hot);
    DrawString(MAIN_SCREEN, line, x, y, COLOR_STD_FONT, COLOR_STD_BG); y += lh;
    // estimate from the rate at which sectors got resolved so far in this run
    u32 resolved = st->fixed + st->unfixable + st->skipped;
    if (resolved && (sec >= 30)) {
        u64 eta = (u64) sec * st->pending / resolved;
        if (eta > 99 * 3600 + 3599) eta = 99 * 3600 + 3599;
        snprintf(line, sizeof(line), "Est. left  %02lu:%02lu:%02lu (rough)       ", (unsigned long) (eta / 3600),
            (unsigned long) ((eta / 60) % 60), (unsigned long) (eta % 60));
    } else snprintf(line, sizeof(line), "Est. left  estimating...          ");
    DrawString(MAIN_SCREEN, line, x, y, COLOR_STD_FONT, COLOR_STD_BG); y += lh;
    snprintf(line, sizeof(line), "Fixed      %-6lu (this round %lu)        ", (unsigned long) st->fixed,
        (unsigned long) (st->fixed - ctx->fixed_at_round_start));
    DrawString(MAIN_SCREEN, line, x, y, COLOR_GREEN, COLOR_STD_BG); y += lh;
    snprintf(line, sizeof(line), "Unfixable  %-10lu              ", (unsigned long) st->unfixable);
    DrawString(MAIN_SCREEN, line, x, y, COLOR_RED, COLOR_STD_BG); y += lh;
    snprintf(line, sizeof(line), "Skipped    %-10lu              ", (unsigned long) st->skipped);
    DrawString(MAIN_SCREEN, line, x, y, COLOR_STD_FONT, COLOR_STD_BG); y += lh;
    if (ctx->io && ctx->params && ctx->list) { // the read strategy getting the most time now
        double share[CARTREF_STRATS];
        CartRef_StrategyShares(ctx->io, ctx->params, ctx->list, share);
        u32 b = 0;
        for (u32 i = 1; i < CARTREF_STRATS; i++) if (share[i] > share[b]) b = i;
        snprintf(line, sizeof(line), "Method     %-10s %3lu%%           ", strat_name[b], (unsigned long) (share[b] * 100 + 0.5));
        DrawString(MAIN_SCREEN, line, x, y, COLOR_STD_FONT, COLOR_STD_BG);
    }
    y += lh;
    snprintf(line, sizeof(line), "Sector %08lX  attempts %-8lu   ", (unsigned long) st->current_sector,
        (unsigned long) st->current_attempts);
    DrawString(MAIN_SCREEN, line, x, y, COLOR_STD_FONT, COLOR_STD_BG); y += lh;
    snprintf(line, sizeof(line), "Refresh reads %-10llu Near miss %-6llu ", (unsigned long long) st->total_attempts,
        (unsigned long long) st->near_misses);
    DrawString(MAIN_SCREEN, line, x, y, COLOR_STD_FONT, COLOR_STD_BG); y += 2 * lh;
    DrawString(MAIN_SCREEN, "B: pause (progress is saved)", x, y, COLOR_STD_FONT, COLOR_STD_BG); y += lh;
    DrawString(MAIN_SCREEN, "Hold Y: skip sector at retry limit", x, y, COLOR_STD_FONT, COLOR_STD_BG);

    // bottom two rows: sectors fixed per round, newest 10 (the running round last, marked *)
    u32 vals[REF_HISTORY], rnds[REF_HISTORY];
    u32 n = 0;
    u32 first = (ctx->rounds_done >= REF_HISTORY) ? ctx->rounds_done - (REF_HISTORY - 1) : 0;
    for (u32 r = first; r < ctx->rounds_done; r++, n++) {
        vals[n] = ctx->round_fixed[r % REF_HISTORY];
        rnds[n] = r + 1;
    }
    vals[n] = st->fixed - ctx->fixed_at_round_start; // running round
    rnds[n] = ctx->round;
    n++;
    // wrap the newest entries into two rows that fit the screen; drop the oldest if needed
    const u32 maxw = (SCREEN_WIDTH_MAIN - x) / FONT_WIDTH_EXT - 1;
    char tok[REF_HISTORY][16];
    u32 tlen[REF_HISTORY];
    for (u32 i = 0; i < n; i++)
        tlen[i] = snprintf(tok[i], sizeof(tok[i]), "R%lu:%lu%s", (unsigned long) rnds[i], (unsigned long) vals[i],
            (i == n - 1) ? "*" : "");
    u32 start = 0;
    for (; start < n; start++) { // first start index whose tokens fit in two rows
        u32 rows = 1, w = 0;
        for (u32 i = start; i < n; i++) {
            u32 need = tlen[i] + (w ? 1 : 0);
            if (w + need > maxw) { rows++; w = tlen[i]; }
            else w += need;
        }
        if (rows <= 2) break;
    }
    char row[2][64];
    u32 k = 0, w = 0;
    row[0][0] = row[1][0] = '\0';
    for (u32 i = start; (i < n) && (k < 2); i++) {
        u32 need = tlen[i] + (w ? 1 : 0);
        if (w + need > maxw) { k++; w = 0; need = tlen[i]; if (k >= 2) break; }
        snprintf(row[k] + w, sizeof(row[k]) - w, "%s%s", w ? " " : "", tok[i]);
        w += need;
    }
    for (u32 r = 0; r < 2; r++) { // pad to clear leftovers from the previous draw
        u32 l = strlen(row[r]);
        while ((l < maxw) && (l < sizeof(row[r]) - 1)) row[r][l++] = ' ';
        row[r][l] = '\0';
    }
    DrawString(MAIN_SCREEN, row[0], x, SCREEN_HEIGHT - 2 * lh - 2, COLOR_GREEN, COLOR_STD_BG);
    DrawString(MAIN_SCREEN, row[1], x, SCREEN_HEIGHT - lh - 2, COLOR_GREEN, COLOR_STD_BG);
}

static bool is_gamecart_path(const char* path) {
    return ((path[0] == 'C') || (path[0] == 'c')) && (path[1] == ':');
}

static u32 count_diff(const u8* a, const u8* b, u32 size) {
    u32 n = 0;
    for (u32 i = 0; i < size; i++) n += (a[i] != b[i]);
    return n;
}

static void state_path(char* out, size_t size, u64 title_id) {
    snprintf(out, size, "%s/cartref_%016llX.dat", OUTPUT_PATH, (unsigned long long) title_id);
}

u32 FixCartWithReference(const char* cart_path) {
    char pathstr[UTF_BUFFER_BYTESIZE(32)];
    TruncateString(pathstr, cart_path, 32, 8);

    if (!is_gamecart_path(cart_path)) {
        ShowPrompt(false, "%s\nThis only works on the cartridge\nitself ([C:] GAMECART).", pathstr);
        return 1;
    }

    // pick the reference (.3ds image or .crh hash file)
    char ref_path[256];
    // start in luma/payloads/hash/ if it exists, otherwise at the SD root
    FILINFO fno;
    const char* ref_dir = ((fvx_stat(REF_HASH_DIR, &fno) == FR_OK) && (fno.fattrib & AM_DIR)) ? REF_HASH_DIR : "0:/";
    if (!FileSelector(ref_path, "Select reference: verified .3ds\nor .crh hash file (same game/version)", ref_dir, "*", 0, false))
        return 1;
    char refstr[UTF_BUFFER_BYTESIZE(32)];
    TruncateString(refstr, ref_path, 32, 8);

    NcsdHeader ncsd_cart;
    if (LoadNcsdHeader(&ncsd_cart, cart_path) != 0) {
        ShowPrompt(false, "%s\nCannot read the cartridge header.", pathstr);
        return 1;
    }
    RefSource* ref = ref_open(ref_path);
    if (!ref) return 1;

    u32 ret = 1;
    CartRefList list;
    list.s = NULL;
    RefCtx ctx;
    memset(&ctx, 0, sizeof(ctx));
    bool cart_open = false;

    if (ncsd_cart.mediaId != ref->ncsd.mediaId) {
        ShowPrompt(false, "Different game.\n \nCart: %016llX\nRef:  %016llX",
            (unsigned long long) ncsd_cart.mediaId, (unsigned long long) ref->ncsd.mediaId);
        goto done;
    }

    u64 cart_size = fvx_qsize(cart_path);
    u64 ref_size = ref->image_size;

    // match partitions: same place, same size, same NCCH header
    CartRefRange ranges[8];
    u32 n_ranges = 0;
    u32 n_unmatched = 0;
    char partinfo[512];
    char* pw = partinfo;
    *pw = '\0';
    u8 hdr_cart[0x200] __attribute__((aligned(16)));
    u8 hdr_norm[0x200];
    u32 n_decrypt = 0;
    for (u32 i = 0; i < 8; i++) {
        NcchPartition* pc = ncsd_cart.partitions + i;
        NcchPartition* pr = ref->ncsd.partitions + i;
        if (!pc->size) continue;
        u64 offset = (u64) pc->offset * NCSD_MEDIA_UNIT;
        u64 size = (u64) pc->size * NCSD_MEDIA_UNIT;
        bool match = (pc->offset == pr->offset) && (pc->size == pr->size) &&
            (offset + size <= ref_size) && (offset + size <= cart_size);
        u32 diff = 0;
        bool decrypt = false;
        const NcchHeader* rh = (const NcchHeader*) (const void*) ref->ncch[i];
        const NcchHeader* ch = (const NcchHeader*) (const void*) hdr_cart;
        if (match) {
            if (FileGetData(cart_path, hdr_cart, 0x200, offset) != 0x200) {
                ShowPrompt(false, "Cannot read partition %lu header.", i);
                goto done;
            }
            // compare headers ignoring the crypto flags (a decrypted dump only differs there)
            memcpy(hdr_norm, hdr_cart, 0x200);
            hdr_norm[0x188 + 3] = ref->ncch[i][0x188 + 3];
            hdr_norm[0x188 + 7] = ref->ncch[i][0x188 + 7];
            diff = count_diff(hdr_norm, ref->ncch[i], 0x200);
            match = (diff <= REF_HDR_TOLERANCE);
            bool ref_plain = (rh->flags[7] & 0x04) != 0;
            bool cart_plain = (ch->flags[7] & 0x04) != 0;
            if (match && ref_plain && !cart_plain) decrypt = true;
            else if (match && !ref_plain && cart_plain) match = false; // unexpected: decrypted cart
        }
        if (match) {
            ctx.part_index[ctx.n_parts] = i;
            RefPart* part = ctx.parts + ctx.n_parts++;
            part->start = (u32) (offset / CARTREF_SECTOR);
            part->count = (u32) (size / CARTREF_SECTOR);
            part->decrypt = decrypt;
            // reference header (undamaged) with the cartridge's crypto flags:
            // used for decryption and to name regions in the log
            memcpy(&part->ncch, ref->ncch[i], 0x200);
            part->ncch.flags[3] = ch->flags[3];
            part->ncch.flags[7] = ch->flags[7];
            if (decrypt) {
                part->has_exefs = (rh->size_exefs != 0);
                memcpy(&part->exefs, ref->exefs[i], 0x200);
                n_decrypt++;
            }
            ranges[n_ranges].start = part->start;
            ranges[n_ranges].count = part->count;
            n_ranges++;
        } else n_unmatched++;
        pw += snprintf(pw, partinfo + sizeof(partinfo) - pw, "Content%lu: %s%s%s\n", i,
            match ? "match" : "DIFFERENT (skipped)", decrypt ? " (decrypted ref)" : "",
            (match && diff) ? " (header damaged)" : "");
    }

    if (!n_ranges) {
        ShowPrompt(false, "%s\nNo partition matches the cartridge.\nIs it the same version/revision?", refstr);
        goto done;
    }

    // resume?
    char spath[64];
    state_path(spath, sizeof(spath), ncsd_cart.mediaId);
    CartRefResumeHeader rhdr;
    memset(&rhdr, 0, sizeof(rhdr));
    rhdr.title_id = ncsd_cart.mediaId;
    rhdr.cart_size = cart_size;
    rhdr.ref_size = ref_size;

    if (!CartRef_ListInit(&list, REF_LIST_CAPACITY)) {
        ShowPrompt(false, "Out of memory.");
        goto done;
    }

    bool resumed = false;
    u64 ssize = fvx_qsize(spath);
    if (ssize && (ssize <= sizeof(CartRefResumeHeader) + (u64) REF_LIST_CAPACITY * CARTREF_STATE_ENTRY)) {
        void* sdata = malloc(ssize);
        if (sdata && (FileGetData(spath, sdata, ssize, 0) == ssize) &&
            CartRef_LoadState(&list, &rhdr, sdata, ssize)) {
            if (ShowPrompt(true, "Saved progress found:\n%lu sectors still to fix.\n \nResume? (No = scan again)", list.count))
                resumed = true;
            else list.count = 0;
        } else list.count = 0;
        free(sdata);
    }

    if (!resumed) {
        if (!ShowPrompt(true, "%s\nReference: %s\n(%s)\n \n%s\nStart scan?", pathstr, refstr,
                ref->is_hash ? "hash file" : "full image", partinfo))
            goto done;
        if (!ref->is_hash &&
            ShowPrompt(true, "Verify the reference image first?\n(recommended, takes a few minutes)")) {
            if (VerifyGameFile(ref_path, false) != 0) {
                ShowPrompt(false, "%s\nReference image failed verification.\nUse a good dump.", refstr);
                goto done;
            }
        }
    }

    bool autoretry = true;
    if (!choose_autoretry(&autoretry)) goto done;

    FixerSim_LoadConfig();
    {
        const char* simstatus = FixerSim_Status();
        if (*simstatus) ShowPrompt(false, "%s\nSIM BUILD: %s", pathstr, simstatus);
    }
    FixerSim_BeginUnit(0, 0);

    if (fvx_open(&ctx.cart, cart_path, FA_READ | FA_OPEN_EXISTING) != FR_OK) {
        ShowPrompt(false, "%s\nCannot open the cartridge.", pathstr);
        goto done;
    }
    cart_open = true;
    ctx.ref = ref;
    ctx.scratch = (u8*) malloc((size_t) CARTREF_SCAN_CHUNK * CARTREF_SECTOR);
    if (!ctx.scratch) {
        ShowPrompt(false, "Out of memory.");
        goto done;
    }
    if (n_decrypt) {
        // make sure the cartridge can actually be decrypted here (keys available)
        u8 test[CARTREF_SECTOR];
        u8 m = 0;
        u32 tsec = 0;
        for (u32 p = 0; p < ctx.n_parts; p++)
            if (ctx.parts[p].decrypt) { tsec = ctx.parts[p].start + 1; break; }
        if ((io_cart_read(&ctx, tsec, 1, test, false) != 0) ||
            (io_ref_compare(&ctx, tsec, 1, test, &m) != 0)) {
            ShowPrompt(false, "Cannot decrypt the cartridge data\nfor comparison with a decrypted\nreference. Use an encrypted dump.");
            goto done;
        }
    }
    // log: SD:/luma/payloads/log/reffix_<titleid>_<date>.log
    ctx.log = (char*) malloc(REF_LOG_EVENTS_SIZE);
    if (ctx.log) {
        DsTime t;
        get_dstime(&t);
        snprintf(ctx.log_path, sizeof(ctx.log_path), "%s/reffix_%016llX_%02lX%02lX%02lX_%02lX%02lX%02lX.log", REF_LOG_DIR,
            (unsigned long long) ncsd_cart.mediaId, (u32) t.bcd_Y, (u32) t.bcd_M, (u32) t.bcd_D,
            (u32) t.bcd_h, (u32) t.bcd_m, (u32) t.bcd_s);
        log_time(&ctx);
        log_printf(&ctx, "Fix with reference image - GodMode9 %s (%s)\n", VERSION, DBUILTL);
        log_printf(&ctx, "cart: %s  title %016llX  size %llu\n", cart_path, (unsigned long long) ncsd_cart.mediaId,
            (unsigned long long) cart_size);
        log_printf(&ctx, "ref:  %s  (%s, image size %llu)\n", ref_path, ref->is_hash ? "hash file" : "full image",
            (unsigned long long) ref_size);
        log_printf(&ctx, "%s", partinfo);
        FixerCfg_Load(&fixer_cfg);
        log_printf(&ctx, "settings: stuck_limit=%lu retries_before_skip=%lu autoskip=%d attempts_per_visit=4 confirm_reads=5 cold_every=%u near_burst=%u strategies=normal/wide4KB/wait%ums/refreshx2/single/stream (adaptive)\n",
            fixer_cfg.stuck_limit, fixer_cfg.retries_before_skip, (fixer_cfg.autoskip || autoretry) ? 1 : 0,
            (unsigned) CARTREF_COLD_EVERY, (unsigned) CARTREF_NEAR_BURST, (unsigned) CARTREF_WAIT_MS); /* = the CartRef_DefaultParams values */
        if (*FixerSim_Status()) log_printf(&ctx, "SIM BUILD: %s\n", FixerSim_Status());
        log_printf(&ctx, "auto retry: %s\n", autoretry ? "on" : "off");
        log_printf(&ctx, "%s\n", resumed ? "resumed from saved progress" : "new scan");
    }
    ctx.list = &list;
    ctx.rhdr = &rhdr;
    ctx.state_file = spath;

    ctx.status_timer = ctx.run_timer = timer_start();

    CartRefIo io = { &ctx, io_cart_read, io_ref_compare, io_begin_pass, io_poll_abort, io_poll_skip, io_status, io_end_pass,
                     io_refresh_only, io_wait_ms, io_msec };
    io_clock = timer_start();
    CartRefResult res = CARTREF_OK;

    if (!resumed) {
        u8* buf = (u8*) malloc((size_t) CARTREF_SCAN_CHUNK * (CARTREF_SECTOR + 1));
        if (!buf) res = CARTREF_NO_MEMORY;
        else {
            ShowProgress(0, 0, "Scanning...");
            u64 scan_timer = timer_start();
            res = CartRef_Scan(&io, ranges, n_ranges, &list, buf);
            u64 scan_ms = timer_msec(scan_timer);
            free(buf);
            u64 scan_bytes = 0;
            for (u32 r = 0; r < n_ranges; r++) scan_bytes += (u64) ranges[r].count * CARTREF_SECTOR;
            log_time(&ctx);
            log_printf(&ctx, "scan: result %d, %llu MB in %llu.%01llu s (%llu KB/s), %lu%s bad sectors\n", (int) res,
                (unsigned long long) (scan_bytes >> 20), (unsigned long long) (scan_ms / 1000),
                (unsigned long long) ((scan_ms % 1000) / 100),
                (unsigned long long) (scan_ms ? (scan_bytes / 1024) * 1000 / scan_ms : 0),
                list.count, list.overflow ? "+" : "");
            log_distribution(&ctx, &list, false);
            log_save(&ctx);
        }
        if ((res == CARTREF_OK) && !list.count) {
            fvx_unlink(spath);
            log_printf(&ctx, "no differences found\n");
            log_save(&ctx);
            ShowPrompt(false, "%s\nNo differences found in the\nmatching partitions.%s", pathstr,
                n_unmatched ? "\n \nOther partitions were not checked;\nuse \"Fix cartridge corruption\"." : "");
            ret = 0;
            goto done;
        }
        u64 total_sectors = 0;
        for (u32 r = 0; r < n_ranges; r++) total_sectors += ranges[r].count;
        if ((res == CARTREF_OK) && (list.overflow || ((u64) list.count * 4 > total_sectors))) {
            // a quarter or more of everything differs: almost certainly a wrong reference
            if (!ShowPrompt(true, "%lu%s of %llu sectors differ.\n \nThis looks like a reference that does\nnot match the cartridge (other version\nor region, or a bad dump), not like\ncorruption.\n \nRepair anyway?",
                    list.count, list.overflow ? "+" : "", (unsigned long long) total_sectors)) {
                res = CARTREF_ABORTED;
                list.count = 0; // nothing worth saving
                log_printf(&ctx, "user declined: too many differences (wrong reference?)\n");
            }
        }
        if (res == CARTREF_OK) {
            char msg[160];
            if (list.overflow)
                snprintf(msg, sizeof(msg), "More than %lu bad sectors.\nThe first %lu will be fixed now;\nrun this again afterwards.",
                    (unsigned long) REF_LIST_CAPACITY, (unsigned long) REF_LIST_CAPACITY);
            else snprintf(msg, sizeof(msg), "%lu bad sectors found (%lu KB).", list.count, list.count / 2);
            save_state(&ctx); // the scan result survives even if the repair never starts
            if (!countdown(msg, "Repair")) {
                res = CARTREF_ABORTED;
                log_printf(&ctx, "repair cancelled at the countdown\n");
            }
        }
    }

    if (res == CARTREF_OK) {
        static CartRefParams params; // static: the log summary after this block still reads it
        CartRef_DefaultParams(&params);
        FixerCfg_Load(&fixer_cfg);  // share retry/stuck/autoskip with the classic fixer
        params.stuck_limit = fixer_cfg.stuck_limit;
        params.retries_before_skip = fixer_cfg.retries_before_skip;
        // Auto retry works by ending a round once every sector has had its share
        // of attempts; without autoskip a round would never end (the red
        // console's 11.7 h round 1 with autoskip=0 in the classic fixer's
        // settings). So auto retry implies autoskip.
        params.autoskip = fixer_cfg.autoskip || autoretry;
        learn_load(&list);   // start from what worked on earlier cartridges
        ctx.io = &io;
        ctx.params = &params;

        ClearScreenF(true, false, COLOR_STD_BG);
        CTR_ResetRefreshCount();
        u64 total_timer = timer_start();
        for (ctx.round = 1;; ctx.round++) {
            CartRefStatus rs;
            CartRef_Count(&list, &rs);
            log_time(&ctx);
            log_printf(&ctx, "round %lu start: %lu sectors to work on\n", ctx.round, rs.pending);
            ctx.run_timer = ctx.pass_timer = ctx.save_timer = timer_start();
            ctx.attempts_before_pass = 0; /* the engine counts attempts per round */
            ctx.last_resolved        = rs.fixed + rs.unfixable + rs.skipped;
            ctx.quiet_passes         = 0;
            ClearScreenF(true, false, COLOR_STD_BG);
            ctx.fixed_at_round_start = rs.fixed;
            res = CartRef_Repair(&io, &params, &list);
            ClearScreenF(true, false, COLOR_STD_BG);

            CartRef_Count(&list, &rs);
            ctx.round_fixed[ctx.rounds_done % REF_HISTORY] = rs.fixed - ctx.fixed_at_round_start;
            ctx.rounds_done++;
            log_time(&ctx);
            log_printf(&ctx, "round %lu end: result %d, fixed %lu, unfixable %lu, skipped %lu, remaining %lu (%llu s)\n",
                ctx.round, (int) res, rs.fixed, rs.unfixable, rs.skipped, rs.pending,
                (unsigned long long) timer_sec(ctx.run_timer));
            log_strats(&ctx);
            if ((res != CARTREF_OK) || !autoretry || !(rs.unfixable + rs.skipped)) break;

            // next round straight away: a 10 s countdown every ~3 minutes only
            // cost time, and B still pauses the repair at any moment
            save_state(&ctx);
            log_save(&ctx);
            CartRef_ResetUnresolved(&list);
        }
        ctx.run_timer = total_timer; // the final summary reports the total time
    }

    fvx_close(&ctx.cart);
    cart_open = false;

    // make sure the SD card is usable (the user may have swapped it while paused)
    while (!InitSDCardFS()) {
        if (InputWait(1) & BUTTON_POWER) PowerOff();
        DeinitSDCardFS();
    }

    CartRefStatus st;
    CartRef_Count(&list, &st);
    bool saved = save_state(&ctx);
    log_time(&ctx);
    log_printf(&ctx, "end: result %d after %llu s: fixed %lu, unfixable %lu, skipped %lu, remaining %lu, refresh count %lu\n",
        (int) res, (unsigned long long) timer_sec(ctx.run_timer), st.fixed, st.unfixable, st.skipped, st.pending,
        (unsigned long) CTR_GetRefreshCount());
    log_strats(&ctx);
    if (st.pending || st.unfixable || st.skipped) {
        log_printf(&ctx, "unresolved by region:\n");
        log_distribution(&ctx, &list, true);
    }
    // everything fixed: verify the cartridge, log the result, power off after a countdown
    if ((res == CARTREF_OK) && !st.pending && !st.unfixable && !st.skipped) {
        log_time(&ctx);
        log_printf(&ctx, "all sectors fixed, verifying the cartridge\n");
        log_save(&ctx);
        char vmsg[256];
        u64 vtimer = timer_start();
        u32 vres   = VerifyCartQuiet(cart_path, vmsg, sizeof(vmsg));
        ClearScreenF(true, false, COLOR_STD_BG);
        for (char* c = vmsg; *c; c++) if (*c == '\n') *c = ' '; // one log line
        log_time(&ctx);
        log_printf(&ctx, "verify: %s (%s, %llu s)\n", vres ? "FAILED" : "OK", vmsg,
            (unsigned long long) timer_sec(vtimer));
        char msg[160];
        snprintf(msg, sizeof(msg), "All %lu sectors fixed.\n \nVerify: %s", st.fixed,
            vres ? "FAILED (see the log)" : "successful");
        log_printf(&ctx, "power off countdown\n");
        log_save(&ctx);
        if (countdown_secs(msg, "Power off", REF_SHUTDOWN_SEC)) {
            ref_close(ref);
            DeinitExtFS();
            DeinitSDCardFS();
            PowerOff();
        }
        log_printf(&ctx, "power off cancelled by user\n");
    }
    log_save(&ctx);

    const char* why =
        (res == CARTREF_ABORTED)    ? "Stopped." :
        (res == CARTREF_CART_ERROR) ? "Cartridge stopped responding.\nReseat it and run this again." :
        (res == CARTREF_REF_ERROR)  ? "Cannot read the reference file\nor decrypt the cartridge data." :
        (res == CARTREF_NO_MEMORY)  ? "Out of memory." : "Finished.";
    ShowPrompt(false, "%s\n \nFixed: %lu\nUnfixable: %lu\nSkipped: %lu\nRemaining: %lu\n \n%s%s",
        why, st.fixed, st.unfixable, st.skipped, st.pending,
        saved ? "Progress saved - run again to continue.\nLog: luma/payloads/log\n" : "Log: luma/payloads/log\n",
        (res == CARTREF_OK && !st.unfixable && !st.skipped) ? "Run Verify to confirm." : "");
    ret = (res == CARTREF_OK) ? 0 : 1;

done:
    if (cart_open) fvx_close(&ctx.cart);
    free(ctx.scratch);
    free(ctx.log);
    if (list.s) CartRef_ListFree(&list);
    ref_close(ref);
    return ret;
}

// ---------------------------------------------------------------------------
// Create a .crh reference hash file from a known-good cartridge.
// The hashes are of the raw (encrypted) cartridge data, so a cartridge fixed
// with it is compared directly, without any decryption step.

u32 CreateCartRefHash(const char* cart_path) {
    char pathstr[UTF_BUFFER_BYTESIZE(32)];
    TruncateString(pathstr, cart_path, 32, 8);

    if (!is_gamecart_path(cart_path)) {
        ShowPrompt(false, "%s\nThis only works on the cartridge\nitself ([C:] GAMECART).", pathstr);
        return 1;
    }

    NcsdHeader ncsd;
    if (LoadNcsdHeader(&ncsd, cart_path) != 0) {
        ShowPrompt(false, "%s\nCannot read the cartridge header.", pathstr);
        return 1;
    }
    u64 cart_size = fvx_qsize(cart_path);
    u64 image_size = GetNcsdTrimmedSize(&ncsd); // padding after the last partition is not needed
    if (!image_size || (image_size > cart_size)) image_size = cart_size;
    image_size &= ~((u64) CARTREF_SECTOR - 1);

    // per-partition NCCH headers
    u8* head = (u8*) malloc(CRH_TABLE_OFFSET);
    if (!head) return 1;
    memset(head, 0, CRH_TABLE_OFFSET);
    memcpy(head + CRH_NCSD_OFFSET, &ncsd, sizeof(NcsdHeader));
    char product[0x11] = { 0 };
    for (u32 i = 0; i < 8; i++) {
        NcchPartition* p = ncsd.partitions + i;
        if (!p->size) continue;
        u8* h = head + CRH_NCCH_OFFSET + i * 0x200;
        if ((FileGetData(cart_path, h, 0x200, (u64) p->offset * NCSD_MEDIA_UNIT) != 0x200) ||
            (memcmp(h + 0x100, "NCCH", 4) != 0)) {
            ShowPrompt(false, "%s\nContent%lu has no valid NCCH header.\nIs this cartridge really healthy?", pathstr, i);
            free(head);
            return 1;
        }
        if (i == 0) memcpy(product, ((NcchHeader*) (void*) h)->productcode, 0x10);
    }
    // ExeFS headers stay zero: the hashes are of encrypted data, no decryption needed

    for (char* c = product; *c; c++) // keep the file name FAT-safe
        if (!isalnum((unsigned char) *c) && (*c != '-') && (*c != '_')) *c = '_';
    char out_path[128];
    snprintf(out_path, sizeof(out_path), "%s/%s_%016llX.crh", REF_HASH_DIR, *product ? product : "CART",
        (unsigned long long) ncsd.mediaId);

    if (!ShowPrompt(true, "%s\nCreate a reference hash file\nfrom this cartridge?\n \nOutput:\n%s\n(%llu MB of cartridge data)\n \nOnly do this with a HEALTHY cartridge.",
            pathstr, out_path, (unsigned long long) (image_size >> 20))) {
        free(head);
        return 1;
    }
    if (ShowPrompt(true, "Verify the cartridge first?\n(recommended: bad data would make\na bad reference)")) {
        if (VerifyGameFile(cart_path, false) != 0) {
            ShowPrompt(false, "%s\nCartridge failed verification.\nIt cannot be used as a reference.", pathstr);
            free(head);
            return 1;
        }
    }

    CrhHeader* crh = (CrhHeader*) (void*) head;
    memcpy(crh->magic, CRH_MAGIC, 4);
    crh->version = CRH_VERSION;
    crh->hash_size = CRH_HASH_SIZE;
    crh->table_offset = CRH_TABLE_OFFSET;
    crh->title_id = ncsd.mediaId;
    crh->image_size = image_size;
    crh->sector_count = (u32) (image_size / CARTREF_SECTOR);

    u8* buf = (u8*) malloc((size_t) REF_HASH_SECTORS * CARTREF_SECTOR);
    u8* hashes = (u8*) malloc((size_t) REF_HASH_SECTORS * CRH_HASH_SIZE);
    FIL cart, out;
    bool cart_open = false, out_open = false, ok = false;
    if (!buf || !hashes) goto fail;
    if (fvx_open(&cart, cart_path, FA_READ | FA_OPEN_EXISTING) != FR_OK) goto fail;
    cart_open = true;
    fvx_rmkdir(REF_HASH_DIR);
    if (fvx_open(&out, out_path, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
        ShowPrompt(false, "Cannot create\n%s", out_path);
        goto fail;
    }
    out_open = true;

    UINT bw = 0;
    if ((fvx_write(&out, head, CRH_TABLE_OFFSET, &bw) != FR_OK) || (bw != CRH_TABLE_OFFSET)) goto fail;

    u32 total = crh->sector_count;
    ShowProgress(0, 0, "Hashing cartridge...");
    u64 t0 = timer_start();
    for (u32 sector = 0; sector < total; sector += REF_HASH_SECTORS) {
        u32 n = min(REF_HASH_SECTORS, total - sector);
        UINT br = 0;
        if ((fvx_lseek(&cart, (u64) sector * CARTREF_SECTOR) != FR_OK) ||
            (fvx_read(&cart, buf, n * CARTREF_SECTOR, &br) != FR_OK) || (br != n * CARTREF_SECTOR)) {
            ShowPrompt(false, "Cartridge read error at %08llX.", (unsigned long long) sector * CARTREF_SECTOR);
            goto fail;
        }
        for (u32 i = 0; i < n; i++) {
            u8 sha[32];
            sha_quick(sha, buf + (size_t) i * CARTREF_SECTOR, CARTREF_SECTOR, SHA256_MODE);
            memcpy(hashes + (size_t) i * CRH_HASH_SIZE, sha, CRH_HASH_SIZE);
        }
        if ((fvx_write(&out, hashes, n * CRH_HASH_SIZE, &bw) != FR_OK) || (bw != n * CRH_HASH_SIZE)) {
            ShowPrompt(false, "SD card write error.");
            goto fail;
        }
        if (!ShowProgress((u64) sector + n, total, "Hashing cartridge...")) {
            ShowPrompt(false, "Cancelled.");
            goto fail;
        }
    }
    ok = true;
    ShowPrompt(false, "Reference hash created (%llu s):\n%s\n \nUse it with \"Fix with reference image\".",
        (unsigned long long) timer_sec(t0), out_path);

fail:
    if (out_open) fvx_close(&out);
    if (cart_open) fvx_close(&cart);
    if (!ok && out_open) fvx_unlink(out_path); // never leave a half-written reference
    free(buf);
    free(hashes);
    free(head);
    return ok ? 0 : 1;
}
