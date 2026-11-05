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
}

bool CartRef_ListInit(CartRefList* l, uint32_t capacity) {
    l->count = 0;
    l->overflow = false;
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
    st->pending = st->fixed = st->unfixable = st->skipped = 0;
    for (uint32_t i = 0; i < l->count; i++) {
        switch (l->s[i].state) {
            case CARTREF_PENDING:   st->pending++; break;
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
        uint32_t sector = e->sector;
        memset(e, 0, sizeof(*e));
        e->sector = sector;
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

            if (io->poll_abort && io->poll_abort(io->ctx))
                return CARTREF_ABORTED;

            for (uint32_t a = 0; a < apv; a++) {
                uint32_t open = 0;
                for (uint32_t k = 0; k < n; k++)
                    open += (run[k].state == CARTREF_PENDING);
                if (!open) break;

                if (io->cart_read(io->ctx, first, n, cbuf, true) != 0) {
                    if (++read_fails >= CARTREF_MAX_READ_FAILS) return CARTREF_CART_ERROR;
                    continue;
                }
                read_fails = 0;
                st.total_attempts++;
                if (io->ref_compare(io->ctx, first, n, cbuf, m) != 0)
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
                    if (cand[k] && !readerr) {
                        s->state = CARTREF_FIXED;
                        continue;
                    }
                    if (!m[k]) { /* a match that did not hold is no evidence of being stuck */
                        uint32_t sum = sector_sum(&cbuf[(size_t) k * CARTREF_SECTOR]);
                        if ((s->attempts > 1) && (sum == s->last_sum)) {
                            s->stuck++;
                        } else {
                            s->stuck = 0;
                            s->last_sum = sum;
                        }
                    }
                    if (p->stuck_limit && (s->stuck >= p->stuck_limit)) {
                        s->state = CARTREF_UNFIXABLE;
                    } else if (s->attempts >= p->retries_before_skip) {
                        if (p->autoskip || (io->poll_skip && io->poll_skip(io->ctx)))
                            s->state = CARTREF_SKIPPED;
                    }
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

    size_t size = sizeof(CartRefResumeHeader) + (size_t) n * sizeof(uint32_t);
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
        w += sizeof(uint32_t);
    }
    return size;
}

bool CartRef_LoadState(CartRefList* l, const CartRefResumeHeader* expect, const void* data, size_t size) {
    CartRefResumeHeader h;
    if (size < sizeof(h)) return false;
    memcpy(&h, data, sizeof(h));
    if ((h.magic != CARTREF_MAGIC) || (h.version != CARTREF_VERSION)) return false;
    if ((h.title_id != expect->title_id) || (h.cart_size != expect->cart_size) ||
        (h.ref_size != expect->ref_size)) return false;
    if (size != sizeof(h) + (size_t) h.count * sizeof(uint32_t)) return false;

    const uint8_t* rd = (const uint8_t*) data + sizeof(h);
    uint32_t max_sector = (uint32_t) (h.cart_size / CARTREF_SECTOR);
    for (uint32_t i = 0; i < h.count; i++) {
        uint32_t sector;
        memcpy(&sector, rd + (size_t) i * sizeof(uint32_t), sizeof(uint32_t));
        if (sector >= max_sector) return false;
        if (!CartRef_ListAdd(l, sector)) break;
    }
    return true;
}
