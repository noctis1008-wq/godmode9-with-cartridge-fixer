// Host unit tests for the reference-guided repair engine (arm9/source/utils/cartref.c).
// Build and run:  make -C tests/cartref
#include "cartref.h"

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define NSECT 6000u

typedef enum { OK = 0, RECOVER, STUCK, FLAKY, NOISE, WIDEFIX, SINGLENEAR } Fault;

typedef struct {
    uint8_t* ref;
    Fault fault[NSECT];
    uint32_t need[NSECT];      // RECOVER: refreshed reads needed
    uint32_t refreshed[NSECT];
    uint32_t reads, refreshes, passes;
    int fail_next;             // fail this many reads
    int abort_after;           // poll_abort returns true after N polls (-1 = never)
    int polls;
} Mock;

static uint32_t rng = 12345;
static uint32_t rnd(void) { rng = rng * 1103515245u + 12345u; return rng >> 8; }

static void fill_bad(uint8_t* b, uint32_t seed) {
    for (uint32_t i = 0; i < CARTREF_SECTOR; i++) b[i] = (uint8_t) (seed * 31u + i * 7u + 1u);
}

static int m_cart_read(void* ctx, uint32_t sector, uint32_t count, void* buf, bool refresh) {
    Mock* m = ctx;
    if (m->fail_next > 0) { m->fail_next--; return 1; }
    if (sector + count > NSECT) return 1; // past the end of the card
    m->reads++;
    if (refresh) m->refreshes++;
    uint8_t* out = buf;
    for (uint32_t i = 0; i < count; i++) {
        uint32_t s = sector + i;
        uint8_t* o = out + (size_t) i * CARTREF_SECTOR;
        memcpy(o, m->ref + (size_t) s * CARTREF_SECTOR, CARTREF_SECTOR);
        switch (m->fault[s]) {
            case RECOVER:
                if (refresh) m->refreshed[s]++;
                if (m->refreshed[s] < m->need[s]) fill_bad(o, m->refreshed[s] + 1); // changing
                break;
            case STUCK: fill_bad(o, 999); break;
            case FLAKY: if (!refresh) fill_bad(o, rnd()); break; // correct only right after refresh
            case NOISE: fill_bad(o, rnd() | 1); break;
            case SINGLENEAR: // right only on a refreshed read of this sector alone, never sticks
                if (!(refresh && (count == 1))) fill_bad(o, rnd() | 1);
                break;
            case WIDEFIX: // restored for good by a refreshed read of a whole 4 KB block
                if (refresh && (count >= 8)) m->fault[s] = OK;
                else fill_bad(o, rnd() | 1);
                break;
            default: break;
        }
    }
    return 0;
}

static int m_ref_compare(void* ctx, uint32_t sector, uint32_t count, const void* cart_buf, uint8_t* match) {
    Mock* m = ctx;
    const uint8_t* c = cart_buf;
    for (uint32_t i = 0; i < count; i++)
        match[i] = memcmp(c + (size_t) i * CARTREF_SECTOR, m->ref + (size_t) (sector + i) * CARTREF_SECTOR,
                          CARTREF_SECTOR) == 0;
    return 0;
}

static void m_begin_pass(void* ctx) { ((Mock*) ctx)->passes++; }
static uint32_t end_passes = 0;
static void m_end_pass(void* ctx, const CartRefStatus* st) { (void) ctx; (void) st; end_passes++; }
static bool m_abort(void* ctx) {
    Mock* m = ctx;
    m->polls++;
    return (m->abort_after >= 0) && (m->polls > m->abort_after);
}

static int failures = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

static Mock* new_mock(void) {
    Mock* m = calloc(1, sizeof(Mock));
    m->ref = malloc((size_t) NSECT * CARTREF_SECTOR);
    for (size_t i = 0; i < (size_t) NSECT * CARTREF_SECTOR; i++) m->ref[i] = (uint8_t) rnd();
    m->abort_after = -1;
    return m;
}
static void free_mock(Mock* m) { free(m->ref); free(m); }

static CartRefIo io_for(Mock* m) {
    CartRefIo io = { m, m_cart_read, m_ref_compare, m_begin_pass, m_abort, NULL, NULL, m_end_pass, NULL, NULL, NULL };
    return io;
}

static CartRefSector* find(CartRefList* l, uint32_t sector) {
    for (uint32_t i = 0; i < l->count; i++) if (l->s[i].sector == sector) return &l->s[i];
    return NULL;
}

static void test_scan_and_repair(void) {
    Mock* m = new_mock();
    m->fault[100] = RECOVER; m->need[100] = 3;
    m->fault[2047] = RECOVER; m->need[2047] = 40;     // chunk boundary
    m->fault[2048] = RECOVER; m->need[2048] = 1;
    m->fault[3000] = STUCK;
    m->fault[4000] = FLAKY;
    m->fault[5999] = RECOVER; m->need[5999] = 7;      // last sector
    m->fault[50] = RECOVER; m->need[50] = 2;          // outside scanned ranges

    CartRefIo io = io_for(m);
    CartRefRange ranges[2] = { { 64, 3000 }, { 3064, NSECT - 3064 } }; // 3000 is in range 1? 64+3000=3064 -> yes, 3000 < 3064
    CartRefList l;
    CHECK(CartRef_ListInit(&l, 1000));
    uint8_t* buf = malloc((size_t) CARTREF_SCAN_CHUNK * (CARTREF_SECTOR + 1));
    CHECK(CartRef_Scan(&io, ranges, 2, &l, buf) == CARTREF_OK);
    CHECK(l.count == 6);
    CHECK(find(&l, 50) == NULL);
    CHECK(find(&l, 2047) && find(&l, 2048) && find(&l, 5999) && find(&l, 4000));

    CartRefParams p;
    CartRef_DefaultParams(&p);
    p.cold_every = 1; p.near_burst = 0; p.strategies = 1; // uniform scheduling for these checks
    p.stuck_limit = 10;
    p.retries_before_skip = 200;
    p.autoskip = true;
    CHECK(CartRef_Repair(&io, &p, &l) == CARTREF_OK);
    CHECK(find(&l, 100)->state == CARTREF_FIXED);
    CHECK(find(&l, 2047)->state == CARTREF_FIXED);
    CHECK(find(&l, 2048)->state == CARTREF_FIXED);
    CHECK(find(&l, 5999)->state == CARTREF_FIXED);
    CHECK(find(&l, 3000)->state == CARTREF_UNFIXABLE);
    CHECK(find(&l, 4000)->state == CARTREF_SKIPPED);   // never stable on plain reads
    // every refreshed read of the flaky sector matched, then failed confirmation
    CHECK(find(&l, 4000)->near_miss == find(&l, 4000)->attempts);
    CHECK(find(&l, 3000)->near_miss == 0 && find(&l, 2047)->near_miss == 0);
    uint32_t flaky_near = find(&l, 4000)->near_miss;
    // stuck sector gave up after stuck_limit identical reads, not 200
    CHECK(find(&l, 3000)->attempts <= 12);
    // RECOVER needing 40 refreshes: fixed after ~40 attempts across passes
    CHECK(find(&l, 2047)->attempts >= 40 && find(&l, 2047)->attempts <= 41);
    CHECK(end_passes == m->passes);
    // another round: unresolved sectors go back to PENDING with fresh counters
    CHECK(CartRef_ResetUnresolved(&l) == 2);
    CHECK(find(&l, 4000)->near_miss == flaky_near);    // kept across rounds
    CHECK(find(&l, 3000)->state == CARTREF_PENDING && find(&l, 3000)->attempts == 0 && find(&l, 3000)->stuck == 0);
    CHECK(find(&l, 4000)->state == CARTREF_PENDING);
    CHECK(find(&l, 100)->state == CARTREF_FIXED);
    m->fault[3000] = OK; // e.g. the controller finally restored it
    CHECK(CartRef_Repair(&io, &p, &l) == CARTREF_OK);
    CHECK(find(&l, 3000)->state == CARTREF_FIXED);
    CHECK(find(&l, 4000)->state == CARTREF_SKIPPED);
    printf("scan+repair: reads=%u refreshes=%u passes=%u\n", m->reads, m->refreshes, m->passes);

    free(buf);
    CartRef_ListFree(&l);
    free_mock(m);
}

static void test_runs_share_commands(void)
{
    /* four adjacent sectors that each need 10 refreshes: one refresh + read
     * covers all four, so about 10 refreshing commands instead of 40 */
    Mock* m = new_mock();
    uint32_t s;
    for (s = 500; s < 504; s++) { m->fault[s] = RECOVER; m->need[s] = 10; }
    m->fault[600] = RECOVER; m->need[600] = 3; /* a separate run */
    CartRefIo io = io_for(m);
    CartRefList l;
    CartRef_ListInit(&l, 10);
    for (s = 500; s < 504; s++) CartRef_ListAdd(&l, s);
    CartRef_ListAdd(&l, 600);
    CartRefParams p;
    CartRef_DefaultParams(&p);
    p.cold_every = 1; p.near_burst = 0; p.strategies = 1; // uniform scheduling for these checks
    CHECK(CartRef_Repair(&io, &p, &l) == CARTREF_OK);
    for (s = 0; s < 5; s++) CHECK(l.s[s].state == CARTREF_FIXED);
    printf("runs: %u refreshing commands for 5 sectors\n", m->refreshes);
    CHECK(m->refreshes <= 14);
    CartRef_ListFree(&l);
    free_mock(m);
}

static void test_near_miss_priority(void) {
    Mock* m = new_mock();
    m->fault[100] = FLAKY;   // near miss on every refreshed read
    m->fault[500] = NOISE;   // never right, never stuck: no near misses
    CartRefIo io = io_for(m);
    CartRefList l;
    CartRef_ListInit(&l, 10);
    CartRef_ListAdd(&l, 100);
    CartRef_ListAdd(&l, 500);
    CartRefParams p;
    CartRef_DefaultParams(&p);
    p.retries_before_skip = 400; // 100 passes of 4
    p.strategies = 1;
    p.autoskip = true;
    CHECK(CartRef_Repair(&io, &p, &l) == CARTREF_OK);
    CHECK(find(&l, 100)->state == CARTREF_SKIPPED && find(&l, 500)->state == CARTREF_SKIPPED);
    // the cold sector got about one visit in CARTREF_COLD_EVERY passes, the hot one bursts
    CHECK(find(&l, 500)->attempts <= 4 * (100 / CARTREF_COLD_EVERY + 2));
    CHECK(find(&l, 100)->attempts > 4 * 100);
    CHECK(find(&l, 500)->near_miss == 0 && find(&l, 100)->near_miss == find(&l, 100)->attempts);
    printf("priority: hot %u attempts, cold %u attempts\n", find(&l, 100)->attempts, find(&l, 500)->attempts);
    CartRef_ListFree(&l);
    free_mock(m);
}

static int s_refresh_only(void* ctx) { ((Mock*) ctx)->refreshes++; return 0; }
static uint32_t s_waited = 0;
static void s_wait(void* ctx, uint32_t ms) { (void) ctx; s_waited += ms; }
static uint32_t s_clock = 0;
static uint32_t s_msec(void* ctx) { (void) ctx; return s_clock += 40; } /* ~ hours of repair in the test */

static void test_strategies(void) {
    Mock* m = new_mock();
    m->fault[1001] = WIDEFIX;         // only the wide read helps
    m->fault[2000] = NOISE;           // nothing helps
    m->fault[NSECT - 1] = WIDEFIX;    // last sector of the card (block ends exactly there)
    CartRefIo io = io_for(m);
    io.refresh_only = s_refresh_only; io.wait_ms = s_wait; io.msec = s_msec;
    CartRefList l;
    CartRef_ListInit(&l, 10);
    CartRef_ListAdd(&l, 1001);
    CartRef_ListAdd(&l, 2000);
    CartRef_ListAdd(&l, NSECT - 1);
    CartRefParams p;
    CartRef_DefaultParams(&p);
    p.retries_before_skip = 80; p.autoskip = true;
    CHECK(CartRef_Repair(&io, &p, &l) == CARTREF_OK);
    CHECK(find(&l, 1001)->state == CARTREF_FIXED);
    CHECK(find(&l, 2000)->state == CARTREF_SKIPPED);
    CHECK(l.strat[CARTREF_STRAT_WIDE].fixed >= 1 && l.strat[CARTREF_STRAT_NORMAL].fixed == 0);
    for (int i = 0; i < CARTREF_STRATS; i++) CHECK(l.strat[i].attempts > 0 && l.strat[i].msec > 0);
    CHECK(s_waited == l.strat[CARTREF_STRAT_WAIT].attempts * CARTREF_WAIT_MS);
    // with no clock the time unit is attempts; here a clock is set, so shares follow msec
    printf("strategies: attempts %llu/%llu/%llu/%llu, wide fixed %u, last sector %d\n",
        (unsigned long long) l.strat[0].attempts, (unsigned long long) l.strat[1].attempts,
        (unsigned long long) l.strat[2].attempts, (unsigned long long) l.strat[3].attempts,
        l.strat[CARTREF_STRAT_WIDE].fixed, find(&l, NSECT - 1)->state);
    CartRef_ListFree(&l);
    free_mock(m);
}

static void test_adaptive(void) {
    // only one way of reading ever gets close: it should end up with most of the time
    Mock* m = new_mock();
    for (uint32_t k = 0; k < 6; k++) m->fault[3000 + 10 * (k / 2) + (k % 2)] = SINGLENEAR; /* 3 pairs */
    CartRefIo io = io_for(m);
    io.refresh_only = s_refresh_only; io.wait_ms = s_wait; io.msec = s_msec;
    CartRefList l;
    CartRef_ListInit(&l, 10);
    for (uint32_t k = 0; k < 6; k++) CartRef_ListAdd(&l, 3000 + 10 * (k / 2) + (k % 2));
    CartRefParams p;
    CartRef_DefaultParams(&p);
    p.retries_before_skip = 20000; p.autoskip = true; p.near_burst = 0;
    CHECK(CartRef_Repair(&io, &p, &l) == CARTREF_OK);
    double sh[CARTREF_STRATS];
    CartRef_StrategyShares(&io, &p, &l, sh);
    uint64_t tot = 0;
    for (int i = 0; i < CARTREF_STRATS; i++) tot += l.strat[i].msec;
    CHECK(sh[CARTREF_STRAT_SINGLE] > 0.7);
    CHECK(l.strat[CARTREF_STRAT_SINGLE].msec * 2 > tot);       // most of the time went to it
    for (int i = 0; i < CARTREF_STRATS; i++) CHECK(sh[i] >= 0.029); // the others keep a little
    printf("adaptive: single share %.2f, time %.0f%%\n", sh[CARTREF_STRAT_SINGLE],
        100.0 * l.strat[CARTREF_STRAT_SINGLE].msec / tot);
    // only normal allowed: the shares say so
    p.strategies = 1u << CARTREF_STRAT_NORMAL;
    CartRef_StrategyShares(&io, &p, &l, sh);
    CHECK(sh[CARTREF_STRAT_NORMAL] == 1.0 && sh[CARTREF_STRAT_SINGLE] == 0);
    CartRef_ListFree(&l);
    free_mock(m);
}

static void test_red_console_shares(void) {
    // figures from the red console's night: stream had by far the most near
    // misses but fewer fixes per hour than wide/normal; the fixes must decide
    static const double F[CARTREF_STRATS] = { 22, 27, 4, 10, 3, 31 };
    static const double N[CARTREF_STRATS] = { 13458, 12325, 2488, 6940, 2914, 87775 };
    static const double T[CARTREF_STRATS] = { 6932, 7458, 2784, 4481, 2648, 14030 };
    Mock* m = new_mock();
    CartRefIo io = io_for(m);
    io.refresh_only = s_refresh_only; io.wait_ms = s_wait; io.msec = s_msec;
    CartRefList l;
    CartRef_ListInit(&l, 4);
    for (int i = 0; i < CARTREF_STRATS; i++) { l.dec[i].fixed = F[i]; l.dec[i].near = N[i]; l.dec[i].cost = T[i]; }
    l.dec_ready = true;
    CartRefParams p;
    CartRef_DefaultParams(&p);
    double sh[CARTREF_STRATS];
    CartRef_StrategyShares(&io, &p, &l, sh);
    printf("red shares: normal %.2f wide %.2f wait %.2f x2 %.2f single %.2f stream %.2f\n",
        sh[0], sh[1], sh[2], sh[3], sh[4], sh[5]);
    CHECK(sh[CARTREF_STRAT_WIDE] > sh[CARTREF_STRAT_STREAM]);
    CHECK(sh[CARTREF_STRAT_NORMAL] > sh[CARTREF_STRAT_STREAM]);
    CHECK(sh[CARTREF_STRAT_WIDE] > sh[CARTREF_STRAT_SINGLE]);
    // an hour of repair halves the old figures
    CartRef_DecayStrategies(&l, 3600.0);
    CHECK(fabs(l.dec[1].fixed - 13.5) < 1e-9 && fabs(l.dec[5].cost - 7015) < 1e-6);
    CartRef_ListFree(&l);
    free_mock(m);
}

static void test_noise_not_stuck(void) {
    // Data that keeps changing is never "stuck": only the retry limit ends it.
    Mock* m = new_mock();
    m->fault[10] = NOISE;
    CartRefIo io = io_for(m);
    CartRefList l;
    CartRef_ListInit(&l, 10);
    CartRef_ListAdd(&l, 10);
    CartRefParams p;
    CartRef_DefaultParams(&p);
    p.cold_every = 1; p.near_burst = 0; p.strategies = 1;
    p.stuck_limit = 5; p.retries_before_skip = 30; p.autoskip = true;
    CHECK(CartRef_Repair(&io, &p, &l) == CARTREF_OK);
    CHECK(l.s[0].state == CARTREF_SKIPPED);
    CHECK(l.s[0].attempts >= 28 && l.s[0].attempts <= 32); // the limit is counted in passes of 4 attempts
    CartRef_ListFree(&l);
    free_mock(m);
}

static void test_read_errors(void) {
    Mock* m = new_mock();
    m->fault[10] = RECOVER; m->need[10] = 2;
    CartRefIo io = io_for(m);
    CartRefList l;
    CartRef_ListInit(&l, 10);
    CartRef_ListAdd(&l, 10);
    CartRefParams p;
    CartRef_DefaultParams(&p);
    p.cold_every = 1; p.near_burst = 0; p.strategies = 1;
    m->fail_next = 2; // transient: recovers
    CHECK(CartRef_Repair(&io, &p, &l) == CARTREF_OK);
    CHECK(l.s[0].state == CARTREF_FIXED);

    l.s[0].state = CARTREF_PENDING;
    m->fail_next = 1000; // dead cart
    CHECK(CartRef_Repair(&io, &p, &l) == CARTREF_CART_ERROR);

    uint8_t* buf = malloc((size_t) CARTREF_SCAN_CHUNK * (CARTREF_SECTOR + 1));
    CartRefRange r = { 0, 100 };
    CartRefList l2;
    CartRef_ListInit(&l2, 10);
    m->fail_next = 1000;
    CHECK(CartRef_Scan(&io, &r, 1, &l2, buf) == CARTREF_CART_ERROR);
    free(buf);
    CartRef_ListFree(&l2);
    CartRef_ListFree(&l);
    free_mock(m);
}

static void test_abort_and_resume(void) {
    Mock* m = new_mock();
    for (uint32_t s = 200; s < 230; s++) { m->fault[s] = RECOVER; m->need[s] = 1000; }
    m->fault[300] = RECOVER; m->need[300] = 1;
    CartRefIo io = io_for(m);
    CartRefList l;
    CartRef_ListInit(&l, 100);
    for (uint32_t s = 200; s < 230; s++) CartRef_ListAdd(&l, s);
    CartRef_ListAdd(&l, 300);
    CartRefParams p;
    CartRef_DefaultParams(&p);
    p.cold_every = 1; p.near_burst = 0; p.strategies = 1;
    m->abort_after = 100;
    CHECK(CartRef_Repair(&io, &p, &l) == CARTREF_ABORTED);
    CHECK(find(&l, 300)->state == CARTREF_FIXED);

    CartRefResumeHeader h = { 0, 0, 0x0004000000123400ull, (uint64_t) NSECT * CARTREF_SECTOR, 777, 0, 0 };
    uint8_t out[4096];
    size_t sz = CartRef_SaveState(&l, &h, out, sizeof(out));
    CHECK(sz == sizeof(CartRefResumeHeader) + 30 * CARTREF_STATE_ENTRY);
    CHECK(CartRef_SaveState(&l, &h, out, 16) == 0);

    CartRefList l2;
    CartRef_ListInit(&l2, 100);
    CHECK(CartRef_LoadState(&l2, &h, out, sz));
    CHECK(l2.count == 30 && l2.s[0].sector == 200 && l2.s[29].sector == 229);
    CHECK(l2.s[5].state == CARTREF_PENDING);
    // near misses survive a save/load
    {
        CartRefList a; CartRef_ListInit(&a, 4);
        CartRef_ListAdd(&a, 10); CartRef_ListAdd(&a, 11);
        a.s[1].near_miss = 1234;
        uint8_t o[256];
        size_t z = CartRef_SaveState(&a, &h, o, sizeof(o));
        CartRefList b; CartRef_ListInit(&b, 4);
        CHECK(z && CartRef_LoadState(&b, &h, o, z));
        CHECK(b.count == 2 && b.s[0].near_miss == 0 && b.s[1].near_miss == 1234);
        // old v2 state (u32 sectors only) still loads
        CartRefResumeHeader h2 = h; h2.magic = CARTREF_MAGIC; h2.version = 2; h2.count = 2;
        uint8_t o2[sizeof(h2) + 8];
        uint32_t secs[2] = { 20, 21 };
        memcpy(o2, &h2, sizeof(h2)); memcpy(o2 + sizeof(h2), secs, 8);
        CartRefList c; CartRef_ListInit(&c, 4);
        CHECK(CartRef_LoadState(&c, &h, o2, sizeof(o2)));
        CHECK(c.count == 2 && c.s[1].sector == 21 && c.s[1].near_miss == 0);
        CartRef_ListFree(&a); CartRef_ListFree(&b); CartRef_ListFree(&c);
    }

    CartRefResumeHeader other = h;
    other.ref_size = 778;
    CartRefList l3;
    CartRef_ListInit(&l3, 100);
    CHECK(!CartRef_LoadState(&l3, &other, out, sz));
    CHECK(!CartRef_LoadState(&l3, &h, out, sz - 1));
    out[sizeof(CartRefResumeHeader)] = 0xFF; out[sizeof(CartRefResumeHeader) + 3] = 0xFF; // out-of-range sector
    CHECK(!CartRef_LoadState(&l3, &h, out, sz));

    CartRef_ListFree(&l);
    CartRef_ListFree(&l2);
    CartRef_ListFree(&l3);
    free_mock(m);
}

static void test_overflow(void) {
    Mock* m = new_mock();
    for (uint32_t s = 0; s < 20; s++) { m->fault[s] = STUCK; }
    CartRefIo io = io_for(m);
    CartRefList l;
    CartRef_ListInit(&l, 5);
    uint8_t* buf = malloc((size_t) CARTREF_SCAN_CHUNK * (CARTREF_SECTOR + 1));
    CartRefRange r = { 0, 100 };
    CHECK(CartRef_Scan(&io, &r, 1, &l, buf) == CARTREF_OK);
    CHECK(l.count == 5 && l.overflow);
    free(buf);
    CartRef_ListFree(&l);
    free_mock(m);
}

int main(void) {
    test_scan_and_repair();
    test_runs_share_commands();
    test_near_miss_priority();
    test_strategies();
    test_adaptive();
    test_red_console_shares();
    test_noise_not_stuck();
    test_read_errors();
    test_abort_and_resume();
    test_overflow();
    if (failures) { printf("%d check(s) failed\n", failures); return 1; }
    printf("all tests passed\n");
    return 0;
}
