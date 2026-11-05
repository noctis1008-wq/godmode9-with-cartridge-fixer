#include "cartref.h"

#include <string.h>
#include <stdlib.h>

// Consecutive failed cartridge reads before giving up (each one already waited
// for the command timeout).
#define CARTREF_MAX_READ_FAILS 3

void CartRef_DefaultParams(CartRefParams* p) {
    p->stuck_limit = 50;
    p->retries_before_skip = 500;
    p->autoskip = false;
    p->attempts_per_visit = 4;
    p->confirm_reads = 5;
    p->cold_every = CARTREF_COLD_EVERY;
    p->near_burst = CARTREF_NEAR_BURST;
    p->experiment = true;
}

bool CartRef_ListInit(CartRefList* l, uint32_t capacity) {
    l->count = 0;
    l->overflow = false;
    memset(l->strat, 0, sizeof(l->strat));
    l->capacity = capacity;
    l->s = (CartRefSector*) malloc((size_t) capacity * sizeof(CartRefSector));
    if (!l->s) l->capacity = 0;
    return l->s != NULL;
}

void CartRef_ListFree(CartRefList* l) {
    free(l->s);
    l->s = NULL;
    l->count = l->capacity = 0;
}

bool CartRef_ListAdd(CartRefList* l, uint32_t sector) {
    if (l->count >= l->capacity) {
        l->overflow = true;
        return false;
    }
    CartRefSector* e = &l->s[l->count++];
    memset(e, 0, sizeof(*e));
    e->sector = sector;
    e->state = CARTREF_PENDING;
    return true;
}

void CartRef_Count(const CartRefList* l, CartRefStatus* st) {
    st->pending = st->fixed = st->unfixable = st->skipped = st->hot = 0;
    for (uint32_t i = 0; i < l->count; i++) {
        switch (l->s[i].state) {
            case CARTREF_PENDING:   st->pending++; if (l->s[i].near_miss) st->hot++; break;
            case CARTREF_FIXED:     st->fixed++; break;
            case CARTREF_UNFIXABLE: st->unfixable++; break;
            default:                st->skipped++; break;
        }
    }
}

uint32_t CartRef_ResetUnresolved(CartRefList* l) {
    uint32_t n = 0;
    for (uint32_t i = 0; i < l->count; i++) {
        CartRefSector* e = &l->s[i];
        if ((e->state != CARTREF_UNFIXABLE) && (e->state != CARTREF_SKIPPED)) continue;
        uint32_t sector = e->sector, near = e->near_miss;
        memset(e, 0, sizeof(*e));
        e->sector = sector;
        e->near_miss = near;
        e->state = CARTREF_PENDING;
        n++;
    }
    return n;
}

static uint32_t sector_sum(const uint8_t* b) {
    uint32_t h = 2166136261u; // FNV-1a
    for (uint32_t i = 0; i < CARTREF_SECTOR; i++) {
        h ^= b[i];
        h *= 16777619u;
    }
    return h;
}

static void report(const CartRefIo* io, const CartRefStatus* st) {
    if (io->status) io->status(io->ctx, st);
}

CartRefResult CartRef_Scan(const CartRefIo* io, const CartRefRange* ranges, uint32_t n_ranges,
                           CartRefList* list, uint8_t* buf) {
    uint8_t* cbuf = buf;
    uint8_t* match = buf + (size_t) CARTREF_SCAN_CHUNK * CARTREF_SECTOR;

    CartRefStatus st;
    memset(&st, 0, sizeof(st));
    st.phase = CARTREF_PHASE_SCAN;
    for (uint32_t r = 0; r < n_ranges; r++)
        st.total += ranges[r].count;

    for (uint32_t r = 0; r < n_ranges; r++) {
        for (uint32_t off = 0; off < ranges[r].count; off += CARTREF_SCAN_CHUNK) {
            uint32_t n = ranges[r].count - off;
            if (n > CARTREF_SCAN_CHUNK) n = CARTREF_SCAN_CHUNK;
            uint32_t sector = ranges[r].start + off;

            if (io->poll_abort && io->poll_abort(io->ctx))
                return CARTREF_ABORTED;

            int fails = 0;
            while (io->cart_read(io->ctx, sector, n, cbuf, false) != 0) {
                if (++fails >= CARTREF_MAX_READ_FAILS)
                    return CARTREF_CART_ERROR;
            }
            if (io->ref_compare(io->ctx, sector, n, cbuf, match) != 0)
                return CARTREF_REF_ERROR;

            for (uint32_t i = 0; i < n; i++) {
                if (!match[i]) {
                    if (!CartRef_ListAdd(list, sector + i))
                        return CARTREF_OK; // list full: work on what we have
                }
            }

            st.done += n;
            st.pending = list->count;
            report(io, &st);
        }
    }
    return CARTREF_OK;
}

/* One repair attempt on `n` neighbouring sectors list->s[first..first+n): a
 * single refresh + read covers them all. The card's error correction works on
 * blocks of about 1 KB, and real logs show bad sectors in adjacent pairs that
 * recover together, so one command for the run gives each sector the same
 * chance at a fraction of the commands. */
CartRefResult CartRef_Repair(const CartRefIo* io, const CartRefParams* p, CartRefList* list) {
    static uint8_t cbuf[CARTREF_RUN_MAX * CARTREF_SECTOR]; /* the refreshed read (static: keep it off the small ARM9 stack) */
    static uint8_t vbuf[CARTREF_RUN_MAX * CARTREF_SECTOR]; /* confirmation reads */
    static uint8_t wbuf[2 * CARTREF_WIDE * CARTREF_SECTOR]; /* "wide" reads (a run may cross a block) */
    static uint32_t strat_next = 0;
    uint8_t m[CARTREF_RUN_MAX];
    uint8_t cand[CARTREF_RUN_MAX];
    uint32_t apv = p->attempts_per_visit ? p->attempts_per_visit : 1;
    int read_fails = 0;

    CartRefStatus st;
    memset(&st, 0, sizeof(st));
    st.phase = CARTREF_PHASE_REPAIR;

    for (;;) {
        CartRef_Count(list, &st);
        if (!st.pending) {
            report(io, &st);
            return CARTREF_OK;
        }

        st.pass++;
        if (io->begin_pass) io->begin_pass(io->ctx);
        /* cold runs (no near miss yet) only get every Nth pass, unless nothing is hot */
        bool any_hot = false;
        for (uint32_t j = 0; j < list->count; j++)
            if ((list->s[j].state == CARTREF_PENDING) && list->s[j].near_miss) { any_hot = true; break; }
        bool cold_pass = !any_hot || (p->cold_every <= 1) || ((st.pass % p->cold_every) == 1);
        /* round-relative attempt budget: the skip limit is reached by every sector at the same pass */
        bool at_limit = (uint64_t) st.pass * apv >= p->retries_before_skip;

        uint32_t i = 0;
        while (i < list->count) {
            if (list->s[i].state != CARTREF_PENDING) {
                i++;
                continue;
            }
            /* the run: following PENDING entries with consecutive sector numbers */
            uint32_t n = 1;
            while ((n < CARTREF_RUN_MAX) && (i + n < list->count) &&
                   (list->s[i + n].state == CARTREF_PENDING) &&
                   (list->s[i + n].sector == list->s[i].sector + n))
                n++;
            CartRefSector* run = &list->s[i];
            uint32_t first = run[0].sector;
            bool hot = false;
            for (uint32_t k = 0; k < n; k++) hot |= (run[k].near_miss != 0);
            if (!hot && !cold_pass && !at_limit) {
                i += n;
                continue;
            }

            if (io->poll_abort && io->poll_abort(io->ctx))
                return CARTREF_ABORTED;

            uint32_t limit = apv;
            for (uint32_t a = 0; a < limit; a++) {
                uint32_t open = 0;
                for (uint32_t k = 0; k < n; k++)
                    open += (run[k].state == CARTREF_PENDING);
                if (!open) break;

                /* pick the read strategy (rotating, skipping those the io cannot do) */
                uint32_t sidx = CARTREF_STRAT_NORMAL;
                if (p->experiment) {
                    for (uint32_t t = 0; t < CARTREF_STRATS; t++) {
                        uint32_t c = strat_next++ % CARTREF_STRATS;
                        if (((c == CARTREF_STRAT_WAIT) && (!io->refresh_only || !io->wait_ms)) ||
                            ((c == CARTREF_STRAT_DOUBLE) && !io->refresh_only))
                            continue;
                        sidx = c;
                        break;
                    }
                }
                uint32_t t0 = io->msec ? io->msec(io->ctx) : 0;
                const uint8_t* data = cbuf;
                int rr = 1;
                if (sidx == CARTREF_STRAT_WIDE) {
                    uint32_t ws = first & ~(CARTREF_WIDE - 1);
                    uint32_t we = ((first + n - 1) | (CARTREF_WIDE - 1)) + 1;
                    rr = io->cart_read(io->ctx, ws, we - ws, wbuf, true);
                    if (!rr) data = wbuf + (size_t) (first - ws) * CARTREF_SECTOR;
                    else sidx = CARTREF_STRAT_NORMAL; /* e.g. past the end of the card: plain attempt instead */
                }
                if (sidx == CARTREF_STRAT_WAIT) {
                    rr = io->refresh_only(io->ctx);
                    if (!rr) {
                        io->wait_ms(io->ctx, CARTREF_WAIT_MS);
                        rr = io->cart_read(io->ctx, first, n, cbuf, false);
                    }
                } else if (sidx == CARTREF_STRAT_DOUBLE) {
                    rr = io->refresh_only(io->ctx);
                    if (!rr) rr = io->cart_read(io->ctx, first, n, cbuf, true);
                } else if (sidx == CARTREF_STRAT_NORMAL) {
                    rr = io->cart_read(io->ctx, first, n, cbuf, true);
                }
                if (rr != 0) {
                    if (++read_fails >= CARTREF_MAX_READ_FAILS) return CARTREF_CART_ERROR;
                    continue;
                }
                read_fails = 0;
                st.total_attempts++;
                CartRefStratStats* ss = &list->strat[sidx];
                ss->attempts++;
                if (io->ref_compare(io->ctx, first, n, data, m) != 0)
                    return CARTREF_REF_ERROR;

                /* matches get confirmed with plain reads (no refresh) */
                uint32_t ncand = 0;
                for (uint32_t k = 0; k < n; k++) {
                    cand[k] = (run[k].state == CARTREF_PENDING) && m[k];
                    ncand += cand[k];
                }
                bool readerr = false;
                for (uint32_t c = 0; ncand && (c < p->confirm_reads); c++) {
                    if (io->cart_read(io->ctx, first, n, vbuf, false) != 0) {
                        if (++read_fails >= CARTREF_MAX_READ_FAILS) return CARTREF_CART_ERROR;
                        readerr = true;
                        break;
                    }
                    read_fails = 0;
                    uint8_t mv[CARTREF_RUN_MAX];
                    if (io->ref_compare(io->ctx, first, n, vbuf, mv) != 0)
                        return CARTREF_REF_ERROR;
                    for (uint32_t k = 0; k < n; k++) {
                        if (cand[k] && !mv[k]) {
                            cand[k] = 0;
                            ncand--;
                        }
                    }
                }

                for (uint32_t k = 0; k < n; k++) {
                    CartRefSector* s = &run[k];
                    if (s->state != CARTREF_PENDING) continue;
                    s->attempts++;
                    if (m[k] && !cand[k] && !readerr) { /* matched once, then the plain reads disagreed */
                        s->near_miss++;
                        st.near_misses++;
                        ss->near_misses++;
                        /* close to recovering: keep at it for a while */
                        if (p->near_burst) {
                            uint32_t want = a + 1 + p->near_burst;
                            if (want > CARTREF_BURST_MAX) want = CARTREF_BURST_MAX;
                            if (want > limit) limit = want;
                        }
                    }
                    if (cand[k] && !readerr) {
                        s->state = CARTREF_FIXED;
                        ss->fixed++;
                        continue;
                    }
                    if (!m[k]) { /* a match that did not hold is no evidence of being stuck */
                        uint32_t sum = sector_sum(&data[(size_t) k * CARTREF_SECTOR]);
                        if ((s->attempts > 1) && (sum == s->last_sum)) {
                            s->stuck++;
                        } else {
                            s->stuck = 0;
                            s->last_sum = sum;
                        }
                    }
                    if (p->stuck_limit && (s->stuck >= p->stuck_limit)) {
                        s->state = CARTREF_UNFIXABLE;
                    } else if (at_limit) {
                        if (p->autoskip || (io->poll_skip && io->poll_skip(io->ctx)))
                            s->state = CARTREF_SKIPPED;
                    }
                }
                if (io->msec) ss->msec += (uint32_t) (io->msec(io->ctx) - t0);
            }

            st.current_sector = first;
            st.current_attempts = run[0].attempts;
            CartRef_Count(list, &st);
            report(io, &st);
            i += n;
        }

        CartRef_Count(list, &st);
        if (io->end_pass) io->end_pass(io->ctx, &st);
    }
}

size_t CartRef_SaveState(const CartRefList* l, const CartRefResumeHeader* hdr, void* out, size_t out_size) {
    uint32_t n = 0;
    for (uint32_t i = 0; i < l->count; i++)
        if (l->s[i].state != CARTREF_FIXED) n++;

    size_t size = sizeof(CartRefResumeHeader) + (size_t) n * CARTREF_STATE_ENTRY;
    if (size > out_size) return 0;

    CartRefResumeHeader h = *hdr;
    h.magic = CARTREF_MAGIC;
    h.version = CARTREF_VERSION;
    h.count = n;
    h.reserved = 0;
    memcpy(out, &h, sizeof(h));

    uint8_t* w = (uint8_t*) out + sizeof(h);
    for (uint32_t i = 0; i < l->count; i++) {
        if (l->s[i].state == CARTREF_FIXED) continue;
        memcpy(w, &l->s[i].sector, sizeof(uint32_t));
        memcpy(w + 4, &l->s[i].near_miss, sizeof(uint32_t));
        w += CARTREF_STATE_ENTRY;
    }
    return size;
}

bool CartRef_LoadState(CartRefList* l, const CartRefResumeHeader* expect, const void* data, size_t size) {
    CartRefResumeHeader h;
    if (size < sizeof(h)) return false;
    memcpy(&h, data, sizeof(h));
    if ((h.magic != CARTREF_MAGIC) || ((h.version != 2) && (h.version != CARTREF_VERSION))) return false;
    if ((h.title_id != expect->title_id) || (h.cart_size != expect->cart_size) ||
        (h.ref_size != expect->ref_size)) return false;
    size_t esz = (h.version == 2) ? 4 : CARTREF_STATE_ENTRY;
    if (size != sizeof(h) + (size_t) h.count * esz) return false;

    const uint8_t* rd = (const uint8_t*) data + sizeof(h);
    uint32_t max_sector = (uint32_t) (h.cart_size / CARTREF_SECTOR);
    for (uint32_t i = 0; i < h.count; i++) {
        uint32_t sector, near = 0;
        memcpy(&sector, rd + (size_t) i * esz, sizeof(uint32_t));
        if (esz > 4) memcpy(&near, rd + (size_t) i * esz + 4, sizeof(uint32_t));
        if (sector >= max_sector) return false;
        if (!CartRef_ListAdd(l, sector)) break;
        l->s[l->count - 1].near_miss = near;
    }
    return true;
}
