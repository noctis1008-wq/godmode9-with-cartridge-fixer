// Host unit tests for the reference-guided repair engine (arm9/source/utils/cartref.c).
// Build and run:  make -C tests/cartref
#include "cartref.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NSECT 6000u

typedef enum { OK = 0, RECOVER, STUCK, FLAKY, NOISE } Fault;

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
    CartRefIo io = { m, m_cart_read, m_ref_compare, m_begin_pass, m_abort, NULL, NULL, m_end_pass };
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
    // stuck sector gave up after stuck_limit identical reads, not 200
    CHECK(find(&l, 3000)->attempts <= 12);
    // RECOVER needing 40 refreshes: fixed after ~40 attempts across passes
    CHECK(find(&l, 2047)->attempts >= 40 && find(&l, 2047)->attempts <= 41);
    CHECK(end_passes == m->passes);
    printf("scan+repair: reads=%u refreshes=%u passes=%u\n", m->reads, m->refreshes, m->passes);

    free(buf);
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
    p.stuck_limit = 5; p.retries_before_skip = 30; p.autoskip = true;
    CHECK(CartRef_Repair(&io, &p, &l) == CARTREF_OK);
    CHECK(l.s[0].state == CARTREF_SKIPPED);
    CHECK(l.s[0].attempts == 30);
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
    m->abort_after = 100;
    CHECK(CartRef_Repair(&io, &p, &l) == CARTREF_ABORTED);
    CHECK(find(&l, 300)->state == CARTREF_FIXED);

    CartRefResumeHeader h = { 0, 0, 0x0004000000123400ull, (uint64_t) NSECT * CARTREF_SECTOR, 777, 0, 0 };
    uint8_t out[4096];
    size_t sz = CartRef_SaveState(&l, &h, out, sizeof(out));
    CHECK(sz == sizeof(CartRefResumeHeader) + 30 * 4);
    CHECK(CartRef_SaveState(&l, &h, out, 16) == 0);

    CartRefList l2;
    CartRef_ListInit(&l2, 100);
    CHECK(CartRef_LoadState(&l2, &h, out, sz));
    CHECK(l2.count == 30 && l2.s[0].sector == 200 && l2.s[29].sector == 229);
    CHECK(l2.s[5].state == CARTREF_PENDING);

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
    test_noise_not_stuck();
    test_read_errors();
    test_abort_and_resume();
    test_overflow();
    if (failures) { printf("%d check(s) failed\n", failures); return 1; }
    printf("all tests passed\n");
    return 0;
}
