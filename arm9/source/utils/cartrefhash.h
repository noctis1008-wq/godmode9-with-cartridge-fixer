#pragma once

// Reference hash file (.crh): a compact stand-in for a full .3ds reference.
// Created on a PC from a verified .3ds with tools/make_crh.py.
// All values little-endian.
//
//   0x0000  char[4]  magic "CRH1"
//   0x0004  u32      version (1)
//   0x0008  u32      hash size in bytes (8)
//   0x000C  u32      offset of the hash table (0x1400)
//   0x0010  u64      title ID (NCSD media ID)
//   0x0018  u64      size of the source .3ds in bytes
//   0x0020  u32      number of sectors (size / 0x200)
//   0x0024  ...      zero up to 0x200
//   0x0200  0x200    copy of the NCSD header (image offset 0)
//   0x0400  8*0x200  copy of each partition's NCCH header (zeros if absent)
//   0x1400  ...      per sector: first `hash size` bytes of SHA-256(sector)
//
// The file is about 1/64 of the image size.

#include <stdint.h>

#define CRH_MAGIC        "CRH1"
#define CRH_VERSION      1u
#define CRH_HASH_SIZE    8u
#define CRH_TABLE_OFFSET 0x1400u
#define CRH_NCSD_OFFSET  0x200u
#define CRH_NCCH_OFFSET  0x400u

typedef struct {
    char     magic[4];
    uint32_t version;
    uint32_t hash_size;
    uint32_t table_offset;
    uint64_t title_id;
    uint64_t image_size;
    uint32_t sector_count;
} __attribute__((packed)) CrhHeader;
