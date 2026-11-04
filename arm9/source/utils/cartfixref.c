// "Fix with reference image": GodMode9 front end for the reference-guided
// cartridge repair engine (cartref.c).

#include "cartfixref.h"
#include "cartref.h"
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
#define REF_REPORT_SIZE     STD_BUFFER_SIZE
#define REF_REPORT_MAX_LINES 20000

extern bool force_refresh;                       // command_ctr.c: refresh before every read
u32 LoadNcsdHeader(NcsdHeader* ncsd, const char* path); // gameutil.c

typedef struct {
    FIL cart;
    FIL ref;
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

static int io_ref_read(void* vctx, u32 sector, u32 count, void* buf) {
    RefCtx* ctx = (RefCtx*) vctx;
    UINT size = count * CARTREF_SECTOR;
    UINT br = 0;
    if (fvx_lseek(&ctx->ref, (u64) sector * CARTREF_SECTOR) != FR_OK) return 1;
    if ((fvx_read(&ctx->ref, buf, size, &br) != FR_OK) || (br != size)) return 1;
    return 0;
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

    // pick the reference image
    char ref_path[256];
    if (!FileSelector(ref_path, "Select a verified reference .3ds\n(same game, same version)", "0:/", "*.3ds", 0, false))
        return 1;
    char refstr[UTF_BUFFER_BYTESIZE(32)];
    TruncateString(refstr, ref_path, 32, 8);

    NcsdHeader ncsd_cart, ncsd_ref;
    if (LoadNcsdHeader(&ncsd_cart, cart_path) != 0) {
        ShowPrompt(false, "%s\nCannot read the cartridge header.", pathstr);
        return 1;
    }
    if (LoadNcsdHeader(&ncsd_ref, ref_path) != 0) {
        ShowPrompt(false, "%s\nNot a valid .3ds (NCSD) file.", refstr);
        return 1;
    }
    if (ncsd_cart.mediaId != ncsd_ref.mediaId) {
        ShowPrompt(false, "Different game.\n \nCart: %016llX\nRef:  %016llX",
            (unsigned long long) ncsd_cart.mediaId, (unsigned long long) ncsd_ref.mediaId);
        return 1;
    }

    u64 cart_size = fvx_qsize(cart_path);
    u64 ref_size = fvx_qsize(ref_path);

    // match partitions: same place, same size, same NCCH header
    CartRefRange ranges[8];
    u32 n_ranges = 0;
    u32 n_unmatched = 0;
    char partinfo[512];
    char* pw = partinfo;
    u8 hdr_cart[0x200], hdr_ref[0x200];
    for (u32 i = 0; i < 8; i++) {
        NcchPartition* pc = ncsd_cart.partitions + i;
        NcchPartition* pr = ncsd_ref.partitions + i;
        if (!pc->size) continue;
        u64 offset = (u64) pc->offset * NCSD_MEDIA_UNIT;
        u64 size = (u64) pc->size * NCSD_MEDIA_UNIT;
        bool match = (pc->offset == pr->offset) && (pc->size == pr->size) &&
            (offset + size <= ref_size) && (offset + size <= cart_size);
        u32 diff = 0;
        if (match) {
            if ((FileGetData(cart_path, hdr_cart, 0x200, offset) != 0x200) ||
                (FileGetData(ref_path, hdr_ref, 0x200, offset) != 0x200)) {
                ShowPrompt(false, "Cannot read partition %lu header.", i);
                return 1;
            }
            diff = count_diff(hdr_cart, hdr_ref, 0x200);
            match = (diff <= REF_HDR_TOLERANCE);
        }
        if (match) {
            ranges[n_ranges].start = (u32) (offset / CARTREF_SECTOR);
            ranges[n_ranges].count = (u32) (size / CARTREF_SECTOR);
            n_ranges++;
        } else n_unmatched++;
        pw += snprintf(pw, partinfo + sizeof(partinfo) - pw, "Content%lu: %s%s\n", i,
            match ? "match" : "DIFFERENT (skipped)", (match && diff) ? " (header damaged)" : "");
    }

    if (!n_ranges) {
        ShowPrompt(false, "%s\nNo partition matches the cartridge.\nIs it the same version/revision?", refstr);
        return 1;
    }

    // resume?
    char spath[64];
    state_path(spath, sizeof(spath), ncsd_cart.mediaId);
    CartRefResumeHeader rhdr;
    memset(&rhdr, 0, sizeof(rhdr));
    rhdr.title_id = ncsd_cart.mediaId;
    rhdr.cart_size = cart_size;
    rhdr.ref_size = ref_size;

    CartRefList list;
    if (!CartRef_ListInit(&list, REF_LIST_CAPACITY)) {
        ShowPrompt(false, "Out of memory.");
        return 1;
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
        if (!ShowPrompt(true, "%s\nReference: %s\n \n%s\nStart scan?", pathstr, refstr, partinfo)) {
            CartRef_ListFree(&list);
            return 1;
        }
        if (ShowPrompt(true, "Verify the reference image first?\n(recommended, takes a few minutes)")) {
            if (VerifyGameFile(ref_path, false) != 0) {
                ShowPrompt(false, "%s\nReference image failed verification.\nUse a good dump.", refstr);
                CartRef_ListFree(&list);
                return 1;
            }
        }
    }

    FixerSim_LoadConfig();
    {
        const char* simstatus = FixerSim_Status();
        if (*simstatus) ShowPrompt(false, "%s\nSIM BUILD: %s", pathstr, simstatus);
    }
    FixerSim_BeginUnit(0, 0);

    RefCtx ctx;
    if (fvx_open(&ctx.cart, cart_path, FA_READ | FA_OPEN_EXISTING) != FR_OK) {
        CartRef_ListFree(&list);
        return 1;
    }
    if (fvx_open(&ctx.ref, ref_path, FA_READ | FA_OPEN_EXISTING) != FR_OK) {
        fvx_close(&ctx.cart);
        CartRef_ListFree(&list);
        return 1;
    }
    ctx.status_timer = ctx.run_timer = timer_start();

    CartRefIo io = { &ctx, io_cart_read, io_ref_read, io_begin_pass, io_poll_abort, io_poll_skip, io_status };
    CartRefResult res = CARTREF_OK;

    if (!resumed) {
        u8* buf = (u8*) malloc(2 * CARTREF_SCAN_CHUNK * CARTREF_SECTOR);
        if (!buf) res = CARTREF_NO_MEMORY;
        else {
            ShowProgress(0, 0, "Scanning...");
            res = CartRef_Scan(&io, ranges, n_ranges, &list, buf);
            free(buf);
        }
        if ((res == CARTREF_OK) && !list.count) {
            fvx_close(&ctx.cart);
            fvx_close(&ctx.ref);
            CartRef_ListFree(&list);
            fvx_unlink(spath);
            ShowPrompt(false, "%s\nNo differences found in the\nmatching partitions.%s", pathstr,
                n_unmatched ? "\n \nOther partitions were not checked;\nuse \"Fix cartridge corruption\"." : "");
            return 0;
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
    fvx_close(&ctx.ref);

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
        (res == CARTREF_ABORTED)    ? "Paused." :
        (res == CARTREF_CART_ERROR) ? "Cartridge stopped responding.\nReseat it and run this again." :
        (res == CARTREF_REF_ERROR)  ? "Cannot read the reference image." :
        (res == CARTREF_NO_MEMORY)  ? "Out of memory." : "Finished.";
    ShowPrompt(false, "%s\n \nFixed: %lu\nUnfixable: %lu\nSkipped: %lu\nRemaining: %lu\n \n%s%s",
        why, st.fixed, st.unfixable, st.skipped, st.pending,
        saved ? "Progress saved - run again to continue.\n" : "",
        (res == CARTREF_OK && !st.unfixable && !st.skipped) ? "Run Verify to confirm." : "");

    CartRef_ListFree(&list);
    return (res == CARTREF_OK) ? 0 : 1;
}
