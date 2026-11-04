# Build a small fake NCSD image for test_crh (2 partitions, random content).
import os, struct, sys
size = 0x200 * 9000
img = bytearray(os.urandom(size))
img[0x100:0x104] = b"NCSD"
struct.pack_into("<Q", img, 0x108, 0x0004000000ABCD00)
for i in range(8): struct.pack_into("<II", img, 0x120 + 8 * i, 0, 0)
struct.pack_into("<II", img, 0x120, 0x20, 6000)       # content0
struct.pack_into("<II", img, 0x128, 6100, 2900)       # content1
for off, plain in ((0x20, True), (6100, False)):
    base = off * 0x200
    img[base + 0x100: base + 0x104] = b"NCCH"
    img[base + 0x18F] = 0x04 if plain else 0x00          # NoCrypto flag
    struct.pack_into("<II", img, base + 0x1A0, 0x10, 0x40)  # ExeFS at +0x2000, 0x8000 bytes
open(sys.argv[1], "wb").write(img)
