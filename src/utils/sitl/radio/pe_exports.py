#!/usr/bin/env python3
"""Read a PE/PE32+ image's export table and section map, without pefile.

    python3 src/utils/sitl/radio/pe_exports.py <file.dll> [name-substring ...]

The Windows EdgeTX simulator library does not export its channel arrays by name, so
`edgetx_channels.py` needs which symbols it does export and where its sections sit, to
turn a runtime address into an RVA and back. `objdump -p` prints both tables but keeps
them in separate listings whose indices only *look* parallel; this pairs name -> RVA
through the ordinal table the way the loader does.

Read-only. Nothing here writes.
"""

import struct
import sys


class Image:
    def __init__(self, path):
        self.path = path
        self.data = open(path, "rb").read()
        d = self.data
        pe = struct.unpack_from("<I", d, 0x3C)[0]
        if d[pe:pe + 4] != b"PE\0\0":
            raise ValueError(f"{path}: not a PE image")
        n_sections = struct.unpack_from("<H", d, pe + 6)[0]
        opt_size = struct.unpack_from("<H", d, pe + 20)[0]
        opt = pe + 24
        magic = struct.unpack_from("<H", d, opt)[0]
        if magic != 0x20B:
            raise ValueError(f"{path}: not PE32+ (magic {magic:#x})")
        self.image_base = struct.unpack_from("<Q", d, opt + 24)[0]
        self.size_of_image = struct.unpack_from("<I", d, opt + 56)[0]
        data_dirs = opt + 112
        self.export_rva, self.export_size = struct.unpack_from("<II", d, data_dirs)
        self.sections = []
        base = opt + opt_size
        for i in range(n_sections):
            b = base + i * 40
            name = d[b:b + 8].rstrip(b"\0").decode("ascii", "replace")
            vsize, vaddr, rsize, raddr = struct.unpack_from("<IIII", d, b + 8)
            self.sections.append((name, vaddr, vsize, raddr, rsize))

    def section_of(self, rva):
        for name, vaddr, vsize, raddr, rsize in self.sections:
            if vaddr <= rva < vaddr + max(vsize, rsize):
                return name, vaddr, vsize, raddr, rsize
        return None

    def offset(self, rva):
        """File offset of an RVA, or None if it is in uninitialised data."""
        hit = self.section_of(rva)
        if hit is None:
            return None
        _, vaddr, _, raddr, rsize = hit
        delta = rva - vaddr
        return raddr + delta if delta < rsize else None

    def read(self, rva, length):
        off = self.offset(rva)
        if off is None:
            raise ValueError(f"rva {rva:#x} has no file bytes (.bss-like)")
        return self.data[off:off + length]

    def exports(self):
        """{name: rva}, paired through the ordinal table."""
        d = self.data
        e = self.offset(self.export_rva)
        n_names = struct.unpack_from("<I", d, e + 24)[0]
        a_func, a_name, a_ord = struct.unpack_from("<III", d, e + 28)
        of, on, oo = self.offset(a_func), self.offset(a_name), self.offset(a_ord)
        out = {}
        for i in range(n_names):
            name_off = self.offset(struct.unpack_from("<I", d, on + 4 * i)[0])
            name = d[name_off:d.index(b"\0", name_off)].decode("ascii", "replace")
            ordinal = struct.unpack_from("<H", d, oo + 2 * i)[0]
            out[name] = struct.unpack_from("<I", d, of + 4 * ordinal)[0]
        return out


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    img = Image(sys.argv[1])
    print(f"ImageBase {img.image_base:#x}  SizeOfImage {img.size_of_image:#x}")
    for name, vaddr, vsize, raddr, rsize in img.sections:
        print(f"  {name:<8} rva {vaddr:#010x} vsize {vsize:#010x} "
              f"raw {raddr:#010x} rsize {rsize:#010x}")
    exports = img.exports()
    patterns = [p.lower() for p in sys.argv[2:]]
    print(f"\n{len(exports)} exports"
          + (f", matching {sys.argv[2:]}" if patterns else "") + ":")
    for name in sorted(exports):
        if not patterns or any(p in name.lower() for p in patterns):
            print(f"  {exports[name]:#010x}  {name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
