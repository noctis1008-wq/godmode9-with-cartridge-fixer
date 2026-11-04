// Checks that tools/make_crh.py output matches the layout GodMode9 reads
// (cartrefhash.h) and that every stored hash is SHA-256(sector)[0:8].
#include "cartrefhash.h"
#include <openssl/sha.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char** argv) {
    if (argc != 3) return 2;
    FILE* fi = fopen(argv[1], "rb");
    FILE* fc = fopen(argv[2], "rb");
    if (!fi || !fc) { puts("open failed"); return 1; }
    fseek(fi, 0, SEEK_END); long isize = ftell(fi); rewind(fi);
    fseek(fc, 0, SEEK_END); long csize = ftell(fc); rewind(fc);
    unsigned char* img = malloc(isize);
    unsigned char* crh = malloc(csize);
    if (fread(img, 1, isize, fi) != (size_t) isize || fread(crh, 1, csize, fc) != (size_t) csize) return 1;
    CrhHeader h;
    memcpy(&h, crh, sizeof(h));
    int bad = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL: %s\n", #c); bad++; } } while (0)
    CHECK(sizeof(CrhHeader) == 0x24);
    CHECK(memcmp(h.magic, CRH_MAGIC, 4) == 0);
    CHECK(h.version == CRH_VERSION && h.hash_size == CRH_HASH_SIZE && h.table_offset == CRH_TABLE_OFFSET);
    CHECK(h.image_size == (uint64_t) isize && h.sector_count == isize / 0x200);
    CHECK((uint64_t) csize == h.table_offset + (uint64_t) h.sector_count * h.hash_size);
    uint64_t tid; memcpy(&tid, img + 0x108, 8);
    CHECK(h.title_id == tid);
    CHECK(memcmp(crh + CRH_NCSD_OFFSET, img, 0x200) == 0);
    for (int i = 0; i < 8; i++) {
        uint32_t off, sz; memcpy(&off, img + 0x120 + 8 * i, 4); memcpy(&sz, img + 0x124 + 8 * i, 4);
        if (!sz) continue;
        const unsigned char* n = img + (size_t) off * 0x200;
        CHECK(memcmp(crh + CRH_NCCH_OFFSET + i * 0x200, n, 0x200) == 0);
        uint32_t eoff; memcpy(&eoff, n + 0x1A0, 4);
        static const unsigned char zero[0x200];
        if (n[0x18F] & 4) CHECK(memcmp(crh + CRH_EXEFS_OFFSET + i * 0x200, n + (size_t) eoff * 0x200, 0x200) == 0);
        else CHECK(memcmp(crh + CRH_EXEFS_OFFSET + i * 0x200, zero, 0x200) == 0);
    }
    for (uint32_t s = 0; s < h.sector_count; s++) {
        unsigned char d[32];
        SHA256(img + (size_t) s * 0x200, 0x200, d);
        if (memcmp(d, crh + h.table_offset + (size_t) s * 8, 8) != 0) { printf("hash mismatch at %u\n", s); bad++; break; }
    }
    free(img); free(crh); fclose(fi); fclose(fc);
    if (bad) return 1;
    puts("crh format ok");
    return 0;
}
