#!/usr/bin/env python3
"""Create a .crh reference hash file from a verified 3DS cartridge dump (.3ds).

The .crh file is about 1/64 of the .3ds size and can be used instead of the
full image with "Fix with reference image" in GodMode9 Cartridge Fixer.

Usage:
    python3 make_crh.py game.3ds [game.crh]

Use an untrimmed or trimmed .3ds of the SAME game and version as the
cartridge. Encrypted (normal GodMode9 dump) and decrypted images both work. Run GodMode9's Verify on it first: the hashes are only as good as
the image they come from.

Format: see arm9/source/utils/cartrefhash.h
"""

import hashlib
import os
import struct
import sys

SECTOR = 0x200
HASH_SIZE = 8
TABLE_OFFSET = 0x2400
NCSD_OFFSET = 0x200
NCCH_OFFSET = 0x400
EXEFS_OFFSET = 0x1400
MEDIA_UNIT = 0x200


def read_ncsd(f, size):
    f.seek(0)
    hdr = f.read(0x200)
    if len(hdr) != 0x200 or hdr[0x100:0x104] != b"NCSD":
        raise ValueError("not a .3ds (NCSD) file")
    title_id = struct.unpack_from("<Q", hdr, 0x108)[0]
    parts = [struct.unpack_from("<II", hdr, 0x120 + 8 * i) for i in range(8)]
    ncch = bytearray(8 * 0x200)
    exefs = bytearray(8 * 0x200)
    for i, (off, sz) in enumerate(parts):
        if not sz:
            continue
        pos = off * MEDIA_UNIT
        if pos + 0x200 > size:
            continue
        f.seek(pos)
        h = f.read(0x200)
        if h[0x100:0x104] != b"NCCH":
            raise ValueError("partition %d has no NCCH header (bad dump?)" % i)
        ncch[i * 0x200:(i + 1) * 0x200] = h
        exefs_off, exefs_size = struct.unpack_from("<II", h, 0x1A0)
        if (h[0x18F] & 0x04) and exefs_size:  # NoCrypto: ExeFS header is plain
            f.seek(pos + exefs_off * MEDIA_UNIT)
            e = f.read(0x200)
            if len(e) != 0x200:
                raise ValueError("partition %d: ExeFS header out of range (bad dump?)" % i)
            exefs[i * 0x200:(i + 1) * 0x200] = e
    return hdr, title_id, bytes(ncch), bytes(exefs)


def make_crh(src, dst):
    size = os.path.getsize(src)
    if size % SECTOR:
        raise ValueError("file size is not a multiple of 0x200")
    sectors = size // SECTOR
    with open(src, "rb") as f, open(dst, "wb") as out:
        ncsd, title_id, ncch, exefs = read_ncsd(f, size)
        head = bytearray(TABLE_OFFSET)
        struct.pack_into("<4sIII QQ I", head, 0, b"CRH1", 2, HASH_SIZE, TABLE_OFFSET,
                         title_id, size, sectors)
        head[NCSD_OFFSET:NCSD_OFFSET + 0x200] = ncsd
        head[NCCH_OFFSET:NCCH_OFFSET + 8 * 0x200] = ncch
        head[EXEFS_OFFSET:EXEFS_OFFSET + 8 * 0x200] = exefs
        out.write(head)

        f.seek(0)
        chunk = SECTOR * 2048
        done = 0
        last_pct = -1
        while True:
            data = f.read(chunk)
            if not data:
                break
            table = bytearray()
            for o in range(0, len(data), SECTOR):
                table += hashlib.sha256(data[o:o + SECTOR]).digest()[:HASH_SIZE]
            out.write(table)
            done += len(data)
            pct = done * 100 // size
            if pct != last_pct:
                print("\r%3d%%" % pct, end="", flush=True)
                last_pct = pct
        print()
    return title_id, sectors


def main():
    if len(sys.argv) not in (2, 3):
        print(__doc__)
        return 1
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) == 3 else os.path.splitext(src)[0] + ".crh"
    try:
        title_id, sectors = make_crh(src, dst)
    except (OSError, ValueError) as e:
        print("error:", e)
        return 1
    print("title %016X, %d sectors -> %s (%d bytes)" % (title_id, sectors, dst, os.path.getsize(dst)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
