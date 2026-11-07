#include "cartref.h"

#include <string.h>
#include <math.h>
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
    p->strategies = CARTREF_STRATS_ALL;
}

bool CartRef_ListInit(CartRefList* l, uint32_t capacity) {
    l->count = 0;
    l->overflow = false;
    memset(l->strat, 0, sizeof(l->strat));
    memset(l->dec, 0, sizeof(l->dec));
    l->dec_ready = false;
    l->dec_clock = 0;
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

/* ---- read strategies and how they are chosen ----
 *
 * Every attempt reads the bad sectors in one of several ways (CARTREF_STRAT_*).
 * Which way works best differs between cartridges, so the time is shared out
 * adaptively: each strategy's yield is measured as sectors fixed plus near
 * misses converted to "fix equivalents" (at the rate near misses turn into
 * fixes overall), per second spent. Each strategy's figure is blended with the
 * overall average, weighted like 1/16 of an average strategy's time (at
 * least CARTREF_STRAT_PRIOR_MIN), so little data means "about average". The best strategies get most of the time (share grows with
 * the square of the yield), but every strategy keeps at least
 * CARTREF_STRAT_FLOOR of it, so a change (e.g. the easy sectors are done) is
 * noticed. The figures fade with a half-life of an hour of repair time, so the
 * choice follows what works on the sectors that are left now. The caller may preload the statistics learned on earlier cartridges
 * (CartRefList.strat) as a starting point. */

#define CARTREF_STRAT_FLOOR   0.03   /* minimum time share per strategy */
#define CARTREF_STRAT_PRIOR_MIN 10.0 /* minimum weight of the "average" assumption (s, or k-attempts) */
#define CARTREF_STREAM_READS  8u     /* plain reads after one refresh ("stream") */
#define CARTREF_STRAT_HALFLIFE 3600.0 /* older results fade: half weight after an hour (s, or k-attempts) */

static bool strat_usable(const CartRefIo* io, uint32_t s) {
    switch (s) {
        case CARTREF_STRAT_WAIT:   return io->refresh_only && io->wait_ms;
        case CARTREF_STRAT_DOUBLE:
        case CARTREF_STRAT_STREAM: return io->refresh_only != NULL;
        default:                   return true;
    }
}

/* cost unit: seconds with a clock, else thousands of attempts */
static double strat_cost(const CartRefIo* io, const CartRefStratStats* s) {
    return io->msec ? (double) s->msec / 1000.0 : (double) s->attempts / 1000.0;
}

void CartRef_DecayStrategies(CartRefList* l, double cost_units) {
    if (cost_units <= 0) return;
    double k = pow(0.5, cost_units / CARTREF_STRAT_HALFLIFE);
    for (uint32_t i = 0; i < CARTREF_STRATS; i++) {
        l->dec[i].fixed *= k;
        l->dec[i].near *= k;
        l->dec[i].cost *= k;
    }
}

/* the recent (decayed) figures start from whatever totals are there (e.g. learned ones) */
static void strat_dec_init(const CartRefIo* io, CartRefList* l) {
    if (l->dec_ready) return;
    for (uint32_t i = 0; i < CARTREF_STRATS; i++) {
        l->dec[i].fixed = l->strat[i].fixed;
        l->dec[i].near = (double) l->strat[i].near_misses;
        l->dec[i].cost = strat_cost(io, &l->strat[i]);
    }
    l->dec_ready = true;
}

void CartRef_StrategyShares(const CartRefIo* io, const CartRefParams* p, const CartRefList* l, double* share) {
    double F = 0, N = 0, T = 0;
    uint32_t usable = 0;
    for (uint32_t i = 0; i < CARTREF_STRATS; i++) {
        share[i] = 0;
        if (!strat_usable(io, i) || !(p->strategies & (1u << i))) continue;
        usable++;
        F += l->dec[i].fixed;
        N += l->dec[i].near;
        T += l->dec[i].cost;
    }
    if (!usable) return;
    /* Yield in fix equivalents: near misses count at this strategy's own rate of
     * turning them into fixes, (F_i + 1) / (N_i + N/(F+1)), i.e. the overall rate
     * until the strategy has fixes of its own. A strategy whose near misses rarely
     * lead anywhere (e.g. "stream" on the red console: 489 near misses per 10k
     * attempts but fewer fixes per hour than "wide") is valued by its fixes. */
    double prior_near = (N + 1.0) / (F + 1.0);
    double E[CARTREF_STRATS], Esum = 0;
    for (uint32_t i = 0; i < CARTREF_STRATS; i++) {
        E[i] = 0;
        if (!strat_usable(io, i) || !(p->strategies & (1u << i))) continue;
        const CartRefStratDecay* d = &l->dec[i];
        E[i] = d->near * (d->fixed + 1.0) / (d->near + prior_near);
        if (E[i] < d->fixed) E[i] = d->fixed; /* fixes always count in full */
        Esum += E[i];
    }
    double mean = (T > 0) ? Esum / T : 0;
    double t0 = T / (16.0 * usable);
    if (t0 < CARTREF_STRAT_PRIOR_MIN) t0 = CARTREF_STRAT_PRIOR_MIN;
    double score[CARTREF_STRATS], sum = 0;
    for (uint32_t i = 0; i < CARTREF_STRATS; i++) {
        score[i] = 0;
        if (!strat_usable(io, i) || !(p->strategies & (1u << i))) continue;
        score[i] = (E[i] + mean * t0) / (l->dec[i].cost + t0);
        score[i] *= score[i];
        sum += score[i];
    }
    double floor = (usable > 1) ? CARTREF_STRAT_FLOOR : 1.0;
    double rest = 1.0 - floor * usable;
    for (uint32_t i = 0; i < CARTREF_STRATS; i++) {
        if (!strat_usable(io, i) || !(p->strategies & (1u << i))) continue;
        share[i] = floor + ((sum > 0) ? rest * score[i] / sum : rest / usable);
    }
}

/* the strategy furthest behind its share of the recent time */
static uint32_t strat_pick(const CartRefList* l, const double* share) {
    double total = 0;
    for (uint32_t i = 0; i < CARTREF_STRATS; i++)
        if (share[i] > 0) total += l->dec[i].cost;
    uint32_t best = CARTREF_STRAT_NORMAL;
    double best_def = -1e300;
    for (uint32_t i = 0; i < CARTREF_STRATS; i++) {
        if (share[i] <= 0) continue;
        double def = share[i] * total - l->dec[i].cost;
        if (def > best_def) { best_def = def; best = i; }
    }
    return best;
}

/* One refreshed read of the run list->s[first..first+n) the way strategy `sidx`
 * does it. On success `data` points at the n sectors read (only those with
 * cover[k] set are meaningful) and match[] tells which of them are correct.
 * Returns 0, 1 on a read error, or -1 on a reference error. `sidx` may be
 * changed to the strategy actually used (wide falls back to normal). */
static int strat_attempt(const CartRefIo* io, uint32_t* sidx, CartRefSector* run, uint32_t n,
                         uint8_t* cbuf, uint8_t* wbuf, const uint8_t** data, uint8_t* match, uint8_t* cover) {
    static uint32_t single_next = 0;
    uint32_t first = run[0].sector;
    for (uint32_t k = 0; k < n; k++) { cover[k] = (run[k].state == CARTREF_PENDING); match[k] = 0; }
    *data = cbuf;
    int rr = 1;

    if (*sidx == CARTREF_STRAT_WIDE) {
        uint32_t ws = first & ~(CARTREF_WIDE - 1);
        uint32_t we = ((first + n - 1) | (CARTREF_WIDE - 1)) + 1;
        rr = io->cart_read(io->ctx, ws, we - ws, wbuf, true);
        if (!rr) *data = wbuf + (size_t) (first - ws) * CARTREF_SECTOR;
        else *sidx = CARTREF_STRAT_NORMAL; /* e.g. past the end of the card */
    }
    switch (*sidx) {
        case CARTREF_STRAT_WIDE:
            break;
        case CARTREF_STRAT_WAIT:
            rr = io->refresh_only(io->ctx);
            if (!rr) {
                io->wait_ms(io->ctx, CARTREF_WAIT_MS);
                rr = io->cart_read(io->ctx, first, n, cbuf, false);
            }
            break;
        case CARTREF_STRAT_DOUBLE:
            rr = io->refresh_only(io->ctx);
            if (!rr) rr = io->cart_read(io->ctx, first, n, cbuf, true);
            break;
        case CARTREF_STRAT_SINGLE: {
            /* one pending sector of the run on its own */
            uint32_t k = single_next++ % n, t;
            for (t = 0; (t < n) && (run[k].state != CARTREF_PENDING); t++) k = (k + 1) % n;
            for (uint32_t j = 0; j < n; j++) cover[j] = (j == k);
            rr = io->cart_read(io->ctx, first + k, 1, cbuf + (size_t) k * CARTREF_SECTOR, true);
            if (rr) return rr;
            if (io->ref_compare(io->ctx, first + k, 1, cbuf + (size_t) k * CARTREF_SECTOR, &match[k]) != 0) return -1;
            return 0;
        }
        case CARTREF_STRAT_STREAM: {
            /* one refresh, then plain reads: any correct one is a candidate */
            rr = io->refresh_only(io->ctx);
            if (rr) return rr;
            uint8_t mm[CARTREF_RUN_MAX];
            for (uint32_t r = 0; r < CARTREF_STREAM_READS; r++) {
                rr = io->cart_read(io->ctx, first, n, cbuf, false);
                if (rr) return rr;
                if (io->ref_compare(io->ctx, first, n, cbuf, mm) != 0) return -1;
                bool any = false;
                for (uint32_t k = 0; k < n; k++) { match[k] |= mm[k]; any |= (mm[k] && cover[k]); }
                if (any) break; /* keep this read: confirmation follows */
            }
            return 0;
        }
        default:
            rr = io->cart_read(io->ctx, first, n, cbuf, true);
            break;
    }
    if (rr) return rr;
    if (io->ref_compare(io->ctx, first, n, *data, match) != 0) return -1;
    return 0;
}

/* Bad sectors are worked on in runs of up to CARTREF_RUN_MAX neighbours: the
 * card's error correction works on blocks of about 1 KB, and real logs show bad
 * sectors in adjacent pairs that recover together. */
CartRefResult CartRef_Repair(const CartRefIo* io, const CartRefParams* p, CartRefList* list) {
    static uint8_t cbuf[CARTREF_RUN_MAX * CARTREF_SECTOR]; /* the refreshed read (static: keep it off the small ARM9 stack) */
    static uint8_t vbuf[CARTREF_RUN_MAX * CARTREF_SECTOR]; /* confirmation reads */
    static uint8_t wbuf[2 * CARTREF_WIDE * CARTREF_SECTOR]; /* "wide" reads (a run may cross a block) */
    uint8_t m[CARTREF_RUN_MAX], cand[CARTREF_RUN_MAX], cover[CARTREF_RUN_MAX];
    double share[CARTREF_STRATS];
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
        /* time shares of the read strategies, updated once per pass */
        if (p->strategies & ~1u) {
            strat_dec_init(io, list);
            double now = 0;
            for (uint32_t j = 0; j < CARTREF_STRATS; j++) now += strat_cost(io, &list->strat[j]);
            if (list->dec_clock > 0) CartRef_DecayStrategies(list, now - list->dec_clock);
            list->dec_clock = now;
            CartRef_StrategyShares(io, p, list, share);
        }

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
            int sticky = -1; /* after a near miss, the burst keeps the strategy that got it */
            for (uint32_t a = 0; a < limit; a++) {
                uint32_t open = 0;
                for (uint32_t k = 0; k < n; k++)
                    open += (run[k].state == CARTREF_PENDING);
                if (!open) break;

                uint32_t sidx = CARTREF_STRAT_NORMAL;
                if (sticky >= 0) sidx = (uint32_t) sticky;
                else if (p->strategies & ~1u) sidx = strat_pick(list, share);
                uint32_t t0 = io->msec ? io->msec(io->ctx) : 0;
                const uint8_t* data;
                int rr = strat_attempt(io, &sidx, run, n, cbuf, wbuf, &data, m, cover);
                if (rr < 0) return CARTREF_REF_ERROR;
                if (rr) {
                    if (++read_fails >= CARTREF_MAX_READ_FAILS) return CARTREF_CART_ERROR;
                    continue;
                }
                read_fails = 0;
                st.total_attempts++;
                CartRefStratStats* ss = &list->strat[sidx];
                CartRefStratDecay* sd = &list->dec[sidx];
                ss->attempts++;

                /* matches get confirmed with plain reads (no refresh) */
                uint32_t ncand = 0;
                for (uint32_t k = 0; k < n; k++) {
                    cand[k] = cover[k] && (run[k].state == CARTREF_PENDING) && m[k];
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
                    if ((s->state != CARTREF_PENDING) || !cover[k]) continue;
                    s->attempts++;
                    if (cand[k] && !readerr) {
                        s->state = CARTREF_FIXED;
                        ss->fixed++;
                        sd->fixed += 1.0;
                        continue;
                    }
                    if (m[k] && !readerr) { /* matched once, then the plain reads disagreed */
                        s->near_miss++;
                        st.near_misses++;
                        ss->near_misses++;
                        sd->near += 1.0;
                        /* close to recovering: keep at it for a while, the same way */
                        if (p->near_burst) {
                            uint32_t want = a + 1 + p->near_burst;
                            if (want > CARTREF_BURST_MAX) want = CARTREF_BURST_MAX;
                            if (want > limit) limit = want;
                            sticky = (int) sidx;
                        }
                    } else if (!m[k]) { /* a match that did not hold is no evidence of being stuck */
                        uint32_t sum = sector_sum(&data[(size_t) k * CARTREF_SECTOR]);
                        if ((s->attempts > 1) && (sum == s->last_sum)) {
                            s->stuck++;
                        } else {
                            s->stuck = 0;
                            s->last_sum = sum;
                        }
                    }
                    if (p->stuck_limit && (s->stuck >= p->stuck_limit))
                        s->state = CARTREF_UNFIXABLE;
                }
                if (io->msec) {
                    uint32_t dt = (uint32_t) (io->msec(io->ctx) - t0);
                    ss->msec += dt;
                    sd->cost += dt / 1000.0;
                } else {
                    sd->cost += 1.0 / 1000.0;
                }
            }
            /* retry limit (counted in passes): skip what is left of the run */
            if (at_limit) {
                for (uint32_t k = 0; k < n; k++) {
                    if (run[k].state != CARTREF_PENDING) continue;
                    if (p->autoskip || (io->poll_skip && io->poll_skip(io->ctx)))
                        run[k].state = CARTREF_SKIPPED;
                }
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
