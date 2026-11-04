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

#define REF_LIST_CAPACITY   (256u * 1024u)       // max. bad sectors tracked per run (128 MB of data)
#define REF_HDR_TOLERANCE   32                   // differing bytes allowed in a damaged NCCH header
#define REF_STATUS_MS       150
#define REF_HASH_DIR        "0:/luma/payloads/hash"
#define REF_REPORT_SIZE     STD_BUFFER_SIZE
#define REF_REPORT_MAX_LINES 20000

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
    u32 cache_sector;         // single-sector cache for the repair phase
    bool cache_valid;
    u8 cache[CARTREF_SECTOR];
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
    u32 n_parts;
    u8* scratch;              // CARTREF_SCAN_CHUNK sectors, for decrypting cart data
    u64 status_timer;
    u64 run_timer;
} RefCtx;

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
        if (!part->decrypt) break;
        if (sector + count > part->start + part->count) return 1; // never spans partitions
        memcpy(ctx->scratch, cart, (size_t) count * CARTREF_SECTOR);
        if (CryptNcch(ctx->scratch, (sector - part->start) * CARTREF_SECTOR, count * CARTREF_SECTOR,
                &part->ncch, part->has_exefs ? &part->exefs : NULL, NCCH_NOCRYPTO) != 0)
            return 1;
        cart = ctx->scratch;
        break;
    }
    u32 unit = ref->is_hash ? CRH_HASH_SIZE : CARTREF_SECTOR;
    const u8* refdata;

    if (count == 1) { // repair phase: same sector is checked many times in a row
        if (!ref->cache_valid || (ref->cache_sector != sector)) {
            if (ref_load(ref, sector, 1, ref->cache) != 0) return 1;
            ref->cache_sector = sector;
            ref->cache_valid = true;
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

    DrawString(MAIN_SCREEN, "Fix with reference image", x, y, COLOR_STD_FONT, COLOR_STD_BG); y += 2 * lh;
    snprintf(line, sizeof(line), "Pass %-8lu   Time %02lu:%02lu:%02lu        ", (unsigned long) st->pass,
        (unsigned long) (sec / 3600), (unsigned long) ((sec / 60) % 60), (unsigned long) (sec % 60));
    DrawString(MAIN_SCREEN, line, x, y, COLOR_STD_FONT, COLOR_STD_BG); y += lh;
    snprintf(line, sizeof(line), "Remaining  %-10lu              ", (unsigned long) st->pending);
    DrawString(MAIN_SCREEN, line, x, y, COLOR_STD_FONT, COLOR_STD_BG); y += lh;
    snprintf(line, sizeof(line), "Fixed      %-10lu              ", (unsigned long) st->fixed);
    DrawString(MAIN_SCREEN, line, x, y, COLOR_GREEN, COLOR_STD_BG); y += lh;
    snprintf(line, sizeof(line), "Unfixable  %-10lu              ", (unsigned long) st->unfixable);
    DrawString(MAIN_SCREEN, line, x, y, COLOR_RED, COLOR_STD_BG); y += lh;
    snprintf(line, sizeof(line), "Skipped    %-10lu              ", (unsigned long) st->skipped);
    DrawString(MAIN_SCREEN, line, x, y, COLOR_STD_FONT, COLOR_STD_BG); y += 2 * lh;
    snprintf(line, sizeof(line), "Sector %08lX  attempts %-8lu   ", (unsigned long) st->current_sector,
        (unsigned long) st->current_attempts);
    DrawString(MAIN_SCREEN, line, x, y, COLOR_STD_FONT, COLOR_STD_BG); y += lh;
    snprintf(line, sizeof(line), "Total refresh reads %-12llu    ", (unsigned long long) st->total_attempts);
    DrawString(MAIN_SCREEN, line, x, y, COLOR_STD_FONT, COLOR_STD_BG); y += 2 * lh;
    DrawString(MAIN_SCREEN, "B: pause (progress is saved)", x, y, COLOR_STD_FONT, COLOR_STD_BG); y += lh;
    DrawString(MAIN_SCREEN, "Hold Y: skip sector at retry limit", x, y, COLOR_STD_FONT, COLOR_STD_BG);
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

static void write_report(const char* cart_path, const char* ref_path, const CartRefList* list,
                         const char* partinfo, CartRefResult res) {
    char* buf = (char*) malloc(REF_REPORT_SIZE);
    if (!buf) return;
    char* w = buf;
    char* end = buf + REF_REPORT_SIZE - 64;
    static const char* names[] = { "PENDING", "FIXED", "UNFIXABLE", "SKIPPED" };

    w += snprintf(w, end - w, "REFERENCE FIX REPORT\ncart: %s\nref:  %s\nresult: %d\n%s\n",
        cart_path, ref_path, (int) res, partinfo);
    u32 lines = 0;
    for (u32 i = 0; (i < list->count) && (w < end) && (lines < REF_REPORT_MAX_LINES); i++, lines++) {
        const CartRefSector* s = &list->s[i];
        w += snprintf(w, end - w, "%08lX %-9s attempts=%lu\n", (unsigned long) (s->sector * CARTREF_SECTOR),
            names[s->state & 3], (unsigned long) s->attempts);
    }
    if (lines < list->count) w += snprintf(w, buf + REF_REPORT_SIZE - w, "[truncated]\n");

    DsTime dstime;
    get_dstime(&dstime);
    char fileout[64];
    snprintf(fileout, sizeof(fileout), "%s/reffix_report_%02lX%02lX%02lX%02lX%02lX%02lX.txt", OUTPUT_PATH,
        (u32) dstime.bcd_Y, (u32) dstime.bcd_M, (u32) dstime.bcd_D,
        (u32) dstime.bcd_h, (u32) dstime.bcd_m, (u32) dstime.bcd_s);
    FileSetData(fileout, buf, w - buf, 0, true);
    free(buf);
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
            RefPart* part = ctx.parts + ctx.n_parts++;
            part->start = (u32) (offset / CARTREF_SECTOR);
            part->count = (u32) (size / CARTREF_SECTOR);
            part->decrypt = decrypt;
            if (decrypt) {
                // reference header (undamaged) with the cartridge's crypto flags
                memcpy(&part->ncch, ref->ncch[i], 0x200);
                part->ncch.flags[3] = ch->flags[3];
                part->ncch.flags[7] = ch->flags[7];
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
    if (ssize && (ssize <= sizeof(CartRefResumeHeader) + (u64) REF_LIST_CAPACITY * 4)) {
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
    ctx.status_timer = ctx.run_timer = timer_start();

    CartRefIo io = { &ctx, io_cart_read, io_ref_compare, io_begin_pass, io_poll_abort, io_poll_skip, io_status };
    CartRefResult res = CARTREF_OK;

    if (!resumed) {
        u8* buf = (u8*) malloc((size_t) CARTREF_SCAN_CHUNK * (CARTREF_SECTOR + 1));
        if (!buf) res = CARTREF_NO_MEMORY;
        else {
            ShowProgress(0, 0, "Scanning...");
            res = CartRef_Scan(&io, ranges, n_ranges, &list, buf);
            free(buf);
        }
        if ((res == CARTREF_OK) && !list.count) {
            fvx_unlink(spath);
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
            }
        }
        if ((res == CARTREF_OK) && list.overflow)
            ShowPrompt(false, "More than %lu bad sectors.\nThe first %lu will be fixed now;\nrun this again afterwards.",
                (unsigned long) REF_LIST_CAPACITY, (unsigned long) REF_LIST_CAPACITY);
        else if (res == CARTREF_OK)
            ShowPrompt(false, "%lu bad sectors found (%lu KB).\nStarting repair.", list.count, list.count / 2);
    }

    if (res == CARTREF_OK) {
        CartRefParams params;
        CartRef_DefaultParams(&params);
        FixerCfg_Load(&fixer_cfg);  // share retry/stuck/autoskip with the classic fixer
        params.stuck_limit = fixer_cfg.stuck_limit;
        params.retries_before_skip = fixer_cfg.retries_before_skip;
        params.autoskip = fixer_cfg.autoskip;

        ClearScreenF(true, false, COLOR_STD_BG);
        CTR_ResetRefreshCount();
        res = CartRef_Repair(&io, &params, &list);
        ClearScreenF(true, false, COLOR_STD_BG);
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
    bool saved = false;
    if (st.pending || st.unfixable || st.skipped) {
        size_t max = sizeof(CartRefResumeHeader) + (size_t) list.count * 4;
        void* sdata = malloc(max);
        size_t sz = sdata ? CartRef_SaveState(&list, &rhdr, sdata, max) : 0;
        if (sz) {
            fvx_rmkdir(OUTPUT_PATH);
            saved = FileSetData(spath, sdata, sz, 0, true);
        }
        free(sdata);
    } else fvx_unlink(spath);
    write_report(cart_path, ref_path, &list, partinfo, res);

    const char* why =
        (res == CARTREF_ABORTED)    ? "Stopped." :
        (res == CARTREF_CART_ERROR) ? "Cartridge stopped responding.\nReseat it and run this again." :
        (res == CARTREF_REF_ERROR)  ? "Cannot read the reference file\nor decrypt the cartridge data." :
        (res == CARTREF_NO_MEMORY)  ? "Out of memory." : "Finished.";
    ShowPrompt(false, "%s\n \nFixed: %lu\nUnfixable: %lu\nSkipped: %lu\nRemaining: %lu\n \n%s%s",
        why, st.fixed, st.unfixable, st.skipped, st.pending,
        saved ? "Progress saved - run again to continue.\n" : "",
        (res == CARTREF_OK && !st.unfixable && !st.skipped) ? "Run Verify to confirm." : "");
    ret = (res == CARTREF_OK) ? 0 : 1;

done:
    if (cart_open) fvx_close(&ctx.cart);
    free(ctx.scratch);
    if (list.s) CartRef_ListFree(&list);
    ref_close(ref);
    return ret;
}
