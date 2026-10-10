#!/usr/bin/env python3
"""Build-time OEM database extraction for the M2/SuperB firmware (Phase 9).

Reads the M2 OEM vehicle database's generated JSON and emits a compact,
header-only C++ table for the firmware. The SAM3X8E has 512 KB flash —
the full database (vehicles.json alone is 6.5 MB) is NEVER embedded.

What is emitted (src/m2/oem_gen.h):
  - oem::DIAG_ACCESS[] : per-make diagnostic addressing (physical TX/RX
    base IDs, ECU stride, functional ID, 11/29-bit). Parsed from
    diagnostics.json `services` prose with regexes; every row cites its
    source service entry. A generic row (0x7E0/0x7E8, 0x7DF) is always
    first and is the firmware default.
  - oem::MAKE_DIDS[]   : make-specific UDS DID subset (--make only),
    capped at 128 entries. DID number + name + flags (ASCII hint,
    security-gated hint from the `security` field).

Usage:
  python3 scripts/oem_extract.py [--db PATH] [--make MAKE] [--out PATH]

  --db   : OEM database root (default: ../m2-oem-database relative to
           this script, else ~/workspace/m2-oem-database)
  --make : bake a make-specific DID pack (e.g. gm, ford, toyota, vw).
           Without --make, only the generic addressing row is emitted.
  --out  : output header (default: ../src/m2/oem_gen.h relative to
           this script)

Subset policy (documented for Phase 9; the DBC/custom-PID engine in
Phase 12 will extend this):
  - Addressing: one row per make with a parseable physical-addressing
    service entry. Makes without a parseable entry are skipped (the
    generic row covers them).
  - DIDs: make-specific 0x22 DIDs only, sorted by DID, cap 128. Generic
    SAE DIDs are NOT duplicated here — they are hand-maintained in
    src/m2/udsreg.cpp (the default registry).
  - DTCs, DBCs, module maps, wiki text: NOT extracted (Phase 12).

A "full" make pack (future) would add: the make's DTC subset (text is
large — likely flash-resident compressed or SuperB-side), DBC signal
tables for the dashboard (Phase 12), and gateway/seed-key notes as
comments only (no key algorithms, ever).
"""

import argparse
import json
import os
import re
import sys

RE_RANGE = re.compile(
    r"\$([0-9A-Fa-f]+)-\$([0-9A-Fa-f]+)\s*req\s*/\s*\$([0-9A-Fa-f]+)-\$([0-9A-Fa-f]+)\s*resp")
RE_PAIR = re.compile(r"0[xX]([0-9A-Fa-f]+)\s*->\s*0[xX]([0-9A-Fa-f]+)")
RE_FUNC = re.compile(r"(?:\$|0[xX])7DF\b")

ASCII_DID_LO = 0xF190
ASCII_DID_HI = 0xF1A0
MAX_DIDS = 128


def parse_addressing(services):
    """Return (txBase, rxBase, nEcus, is29, sourceName) or None."""
    func = 0x7DF
    for s in services:
        sid = s.get("service_id", "")
        if RE_FUNC.search(sid):
            func = 0x7DF
    for s in services:
        sid = s.get("service_id", "")
        m = RE_RANGE.search(sid)
        if m:
            tx0, tx1, rx0, rx1 = (int(x, 16) for x in m.groups())
            n = tx1 - tx0 + 1
            is29 = tx0 > 0x7FF
            return tx0, rx0, n, is29, s.get("name", ""), func
        m = RE_PAIR.search(sid)
        if m:
            tx0, rx0 = (int(x, 16) for x in m.groups())
            is29 = tx0 > 0x7FF
            return tx0, rx0, 1, is29, s.get("name", ""), func
    return None


def sec_flag(sec_text):
    s = (sec_text or "").lower()
    if "none required" in s:
        return 0
    if "gate" in s or "security access" in s or "security" in s:
        return 2  # DID_SEC
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--db", default=None)
    ap.add_argument("--make", default=None)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    db = args.db or os.environ.get("OEM_DB")
    if not db:
        cand = os.path.join(script_dir, "..", "m2-oem-database")
        db = cand if os.path.isdir(cand) else os.path.expanduser(
            "~/workspace/m2-oem-database")
    out = args.out or os.path.join(script_dir, "..", "src", "m2", "oem_gen.h")

    diag_p = os.path.join(db, "generated", "json", "diagnostics.json")
    dids_p = os.path.join(db, "generated", "json", "dids.json")
    for p in (diag_p, dids_p):
        if not os.path.isfile(p):
            sys.exit("missing OEM database file: %s" % p)

    diag = json.load(open(diag_p))
    dids = json.load(open(dids_p))["entries"]

    rows = []  # (make, txBase, rxBase, nEcus, is29, funcId, srcName)
    for m in diag["makes"]:
        make = m.get("make", "?")
        parsed = parse_addressing(m.get("services", []))
        if parsed:
            tx0, rx0, n, is29, src, func = parsed
            rows.append((make, tx0, rx0, n, is29, func, src))

    make_dids = []
    if args.make:
        for e in dids:
            if e.get("make") != args.make:
                continue
            if e.get("service") != "0x22":
                continue
            try:
                did = int(str(e.get("did")), 16)
            except (ValueError, TypeError):
                continue
            flags = sec_flag(e.get("security", ""))
            if ASCII_DID_LO <= did <= ASCII_DID_HI:
                flags |= 1  # DID_ASCII
            name = (e.get("name") or "unknown").replace('"', "'")[:64]
            make_dids.append((did, name, flags, e.get("grade", "?")))
        make_dids.sort()
        make_dids = make_dids[:MAX_DIDS]

    with open(out, "w") as f:
        f.write("// GENERATED by scripts/oem_extract.py -- do not hand-edit.\n")
        f.write("// Source: m2-oem-database generated/json (diagnostics.json,\n")
        f.write("// dids.json). Regenerate with:\n")
        f.write("//   python3 scripts/oem_extract.py%s\n" %
                (" --make %s" % args.make if args.make else ""))
        f.write("#pragma once\n#include <Arduino.h>\n#include <stdint.h>\n\n")
        f.write("namespace oem {\n\n")
        f.write("// makeId: index into DIAG_ACCESS (0 = generic default).\n")
        f.write("struct DiagAccess {\n")
        f.write("    const char* make;\n")
        f.write("    uint32_t txBase;   // physical request base ID\n")
        f.write("    uint32_t rxBase;   // physical response base ID\n")
        f.write("    uint8_t nEcus;     // ECU stride count\n")
        f.write("    uint8_t is29;      // 29-bit addressing\n")
        f.write("    uint32_t funcId;   // functional (broadcast) request ID\n")
        f.write("};\n\n")
        f.write("static const DiagAccess DIAG_ACCESS[] = {\n")
        f.write('    { "generic", 0x7E0, 0x7E8, 8, 0, 0x7DF },  // SAE default\n')
        for make, tx0, rx0, n, is29, func, src in rows:
            src_c = src.replace('"', "'")[:60]
            f.write('    { "%s", 0x%X, 0x%X, %d, %d, 0x%X },  // %s\n' %
                    (make, tx0, rx0, min(n, 255), 1 if is29 else 0,
                     func, src_c))
        f.write("};\n")
        f.write("static const uint8_t DIAG_ACCESS_N = "
                "sizeof(DIAG_ACCESS)/sizeof(DIAG_ACCESS[0]);\n\n")
        f.write("// flags: bit0 = ASCII-decodable, bit1 = may need 0x27\n")
        f.write("struct DidEntry { uint16_t did; const char* name;"
                " uint8_t flags; };\n")
        f.write("static const DidEntry MAKE_DIDS[] = {\n")
        for did, name, flags, grade in make_dids:
            f.write('    { 0x%04X, "%s", %d },  // grade %s\n' %
                    (did, name, flags, grade))
        f.write("};\n")
        f.write("static const uint16_t MAKE_DIDS_N = "
                "sizeof(MAKE_DIDS)/sizeof(MAKE_DIDS[0]);\n")
        f.write('static const char MAKE_NAME[] = "%s";\n' %
                (args.make or "generic"))
        f.write("\n} // namespace oem\n")

    print("wrote %s: %d diag-access rows, %d make DIDs (%s)" %
          (out, len(rows) + 1, len(make_dids), args.make or "generic"))


if __name__ == "__main__":
    main()
