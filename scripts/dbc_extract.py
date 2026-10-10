#!/usr/bin/env python3
"""Build-time DBC extraction for the M2/SuperB firmware (Phase 12).

Reads curated DBC files from the M2 OEM vehicle database
(~/workspace/m2-oem-database/source/vehicles.yaml `dbc:` names ->
work/opendbc/opendbc/dbc/<name>.dbc) and emits a compact, header-only C++
signal table for the firmware (src/m2/dbc_gen.h).

NO DBC text parsing happens in firmware (too heavy for the SAM3X8E) —
all parsing is done here, at build time.

What is emitted:
  - dbc::MSG[] : one row per CAN message {id, sigBase, nSig, dlc, ext, dbc}
  - dbc::SIG[] : one row per signal {name[16], startBit, len, flags,
                 factor1000 (int32), offset1000 (int32), msgIdx,
                 muxVal}. Physical value = raw * factor1000/1000 +
                 offset1000/1000 (firmware uses integer math only).
  - dbc::ENUM[]: VAL_ value descriptions {sigIdx, val, desc[16]}
  - dbc::DBC_NAME[] : which DBC files were baked (provenance)

Signal flags bitfield:
  bit0 = 1: Motorola (big-endian) byte order, 0: Intel (little-endian)
  bit1 = 1: signed
  bit2 = 1: this signal IS the multiplexer
  bit3 = 1: this signal is multiplexed (muxVal selects it)

Multiplexing: one level only (multiplexer + its multiplexed signals).
Deeper nesting is flattened with a warning (rare in the curated set).

Usage:
  python3 scripts/dbc_extract.py [--db PATH] [--dbc NAME]... [--out PATH]
                                 [--max-signals N]

  --db          : OEM database root (default: ../m2-oem-database relative
                  to this script, else ~/workspace/m2-oem-database)
  --dbc         : bake this DBC (repeatable). Default: a curated starter
                  set of moderate-size DBCs (see DEFAULT_DBCS). Use
                  --dbc all to bake every curated DBC that has a file
                  (large — check the size report before flashing).
  --max-signals : hard cap on emitted signals (default 6000 ≈ 200 KB).
  --out         : output header (default: ../src/m2/dbc_gen.h)

Subset policy: the full 45-DBC set is ~23k signals (~700 KB) — far too
big. The default starter set is a few representative DBCs; grow it
deliberately with --dbc and watch the size report.
"""

import argparse
import json
import os
import re
import sys

RE_DBCNAME = re.compile(r"^\s*dbc:\s*(\S+)")
RE_BO = re.compile(r"^BO_\s+(\d+)\s+(\S+)\s*:\s*(\d+)\s+(\S+)")
RE_SG = re.compile(
    r"^\s+SG_\s+(\S+)\s+(M\s+)?m(\d+)\s*:\s*"
    r"(\d+)\|(\d+)@([01])([+-])\s*"
    r"\(\s*([^,]+)\s*,\s*([^\)]+)\s*\)\s*"
    r"\[\s*([^|]+)\s*\|\s*([^\]]+)\s*\]\s*\"([^\"]*)\""
)
RE_SG_PLAIN = re.compile(
    r"^\s+SG_\s+(\S+)\s*:\s*"
    r"(\d+)\|(\d+)@([01])([+-])\s*"
    r"\(\s*([^,]+)\s*,\s*([^\)]+)\s*\)\s*"
    r"\[\s*([^|]+)\s*\|\s*([^\]]+)\s*\]\s*\"([^\"]*)\""
)
RE_SG_MUXOR = re.compile(r"^\s+SG_\s+(\S+)\s+M\s*:\s*")
RE_VAL = re.compile(r'^VAL_\s+(\d+)\s+(\S+)\s+(.*);')

# Lean starter set (~66 KB flash): one Ford + one GM + one Mazda + one
# Toyota powertrain DBC. Grow deliberately with --dbc (repeatable) and
# watch the size report; the full 45-DBC set is ~700 KB and will NOT fit.
DEFAULT_DBCS = [
    "ford_fusion_2018_pt",
    "gm_global_a_powertrain_expansion",
    "mazda_3_2019",
    "toyota_2017_ref_pt",
]


def parse_dbc(path):
    """Return (messages, enums). messages: list of dicts."""
    txt = open(path, errors="ignore").read()
    messages = []
    cur = None
    for line in txt.splitlines():
        m = RE_BO.match(line)
        if m:
            can_id = int(m.group(1))
            name = m.group(2)
            dlc = int(m.group(3))
            ext = 1 if can_id > 0x7FF else 0
            # DBC extended IDs have bit 31 set in the raw value
            if can_id & 0x80000000:
                can_id &= 0x1FFFFFFF
                ext = 1
            cur = {"id": can_id, "ext": ext, "dlc": dlc,
                   "name": name, "sigs": []}
            messages.append(cur)
            continue
        if cur is None:
            continue
        m = RE_SG.match(line)
        muxor = False
        muxed = False
        mux_val = 0
        if m:
            muxed = True
            mux_val = int(m.group(3))
            name, start, ln = m.group(1), int(m.group(4)), int(m.group(5))
            bo, sgn = m.group(6), m.group(7)
            factor, offset = m.group(8), m.group(9)
            smin, smax, unit = m.group(10), m.group(11), m.group(12)
        else:
            m2 = RE_SG_MUXOR.match(line)
            if m2:
                # multiplexer: parse the rest with the plain pattern
                muxor = True
                rest = line.replace(" M :", " :", 1)
                m = RE_SG_PLAIN.match(rest)
            else:
                m = RE_SG_PLAIN.match(line)
            if not m:
                continue
            name, start, ln = m.group(1), int(m.group(2)), int(m.group(3))
            bo, sgn = m.group(4), m.group(5)
            factor, offset = m.group(6), m.group(7)
            smin, smax, unit = m.group(8), m.group(9), m.group(10)
        try:
            factor1000 = int(round(float(factor) * 1000))
            offset1000 = int(round(float(offset) * 1000))
        except ValueError:
            factor1000, offset1000 = 1000, 0
        flags = 0
        # DBC byte-order convention (Vector / cantools 44.x verified):
        #   @0 = Motorola (big-endian), @1 = Intel (little-endian).
        # (An earlier revision had this backwards; fixed 2026-10-09.)
        if bo == "0":
            flags |= 1  # Motorola
        if sgn == "-":
            flags |= 2  # signed
        if muxor:
            flags |= 4
        if muxed:
            flags |= 8
        cur["sigs"].append({
            "name": name[:15], "start": start, "len": ln, "flags": flags,
            "factor1000": factor1000, "offset1000": offset1000,
            "unit": unit[:7], "mux_val": mux_val,
        })
    enums = []  # (msg_id, sig_name, val, desc)
    for line in txt.splitlines():
        m = RE_VAL.match(line)
        if not m:
            continue
        msg_id = int(m.group(1))
        if msg_id & 0x80000000:
            msg_id &= 0x1FFFFFFF
        sig = m.group(2)
        rest = m.group(3)
        pairs = re.findall(r"(-?\d+)\s+\"([^\"]*)\"", rest)
        for v, d in pairs:
            enums.append((msg_id, sig[:15], int(v), d[:15]))
    return messages, enums


def cstr(s):
    return '"%s"' % s.replace('"', "'")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--db", default=None)
    ap.add_argument("--dbc", action="append", default=None)
    ap.add_argument("--max-signals", type=int, default=6000)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    db = args.db or os.environ.get("OEM_DB")
    if not db:
        cand = os.path.join(script_dir, "..", "m2-oem-database")
        db = cand if os.path.isdir(cand) else os.path.expanduser(
            "~/workspace/m2-oem-database")
    out = args.out or os.path.join(script_dir, "..", "src", "m2", "dbc_gen.h")

    names = []
    for line in open(os.path.join(db, "source", "vehicles.yaml")):
        m = RE_DBCNAME.match(line)
        if m:
            names.append(m.group(1))
    dbc_dir = os.path.join(db, "work", "opendbc", "opendbc", "dbc")

    want = args.dbc or list(DEFAULT_DBCS)
    want_all = (want == ["all"])
    if want_all:
        want = names
    avail = {}
    for n in names:
        p = os.path.join(dbc_dir, n + ".dbc")
        if os.path.isfile(p):
            avail[n] = p
    missing = [n for n in want if n not in avail]
    if missing:
        # --dbc all is documented best-effort ("every curated DBC that has
        # a file"); explicit --dbc names and the default set are fatal —
        # a silent skip once nearly shipped an empty generated table.
        if want_all:
            print("warning: DBC files not found (skipped): %s" % ", ".join(missing),
                  file=sys.stderr)
        else:
            print("error: requested DBC files not found: %s" % ", ".join(missing),
                  file=sys.stderr)
            sys.exit(1)

    all_msgs = []
    all_enums = []
    baked = []
    for n in want:
        if n not in avail:
            continue
        msgs, enums = parse_dbc(avail[n])
        for mm in msgs:
            mm["dbc"] = n
        all_msgs.extend(msgs)
        all_enums.extend(enums)
        baked.append((n, len(msgs), sum(len(m["sigs"]) for m in msgs)))

    # Cap signals (largest DBCs first would blow the budget — keep message
    # order stable and stop at the cap).
    total_sigs = sum(len(m["sigs"]) for m in all_msgs)
    if total_sigs > args.max_signals:
        print("warning: %d signals exceed --max-signals %d; truncating "
              "trailing messages" % (total_sigs, args.max_signals),
              file=sys.stderr)
        kept = []
        n = 0
        for mm in all_msgs:
            if n + len(mm["sigs"]) > args.max_signals:
                break
            kept.append(mm)
            n += len(mm["sigs"])
        all_msgs = kept

    # Flatten signals with message index.
    sig_rows = []
    for mi, mm in enumerate(all_msgs):
        for s in mm["sigs"]:
            sig_rows.append((mi,) + (s["name"], s["start"], s["len"],
                                    s["flags"], s["factor1000"],
                                    s["offset1000"], s["unit"],
                                    s["mux_val"]))

    # Enum table: resolve (msg_id, sig_name) -> sig index.
    sig_index = {}
    for si, row in enumerate(sig_rows):
        mi = row[0]
        sig_index[(all_msgs[mi]["id"], row[1])] = si
    enum_rows = []
    for msg_id, sig_name, val, desc in all_enums:
        si = sig_index.get((msg_id, sig_name))
        if si is None:
            continue
        enum_rows.append((si, val, desc))
    enum_rows = enum_rows[:2048]

    if not all_msgs or not sig_rows:
        print("error: refusing to write an empty table "
              "(%d DBCs, %d msgs, %d sigs)" %
              (len(baked), len(all_msgs), len(sig_rows)), file=sys.stderr)
        sys.exit(1)

    # Atomic write: build to a temp file in the same directory, then rename,
    # so a crash can never leave a half-written header behind.
    tmp = out + ".tmp"
    with open(tmp, "w") as f:
        f.write("// GENERATED by scripts/dbc_extract.py -- do not hand-edit.\n")
        f.write("// Source: m2-oem-database curated DBCs (%s).\n"
                % ", ".join(n for n, _, _ in baked))
        f.write("// Regenerate: python3 scripts/dbc_extract.py%s\n" %
                ("".join(" --dbc " + n for n in (args.dbc or []))))
        f.write("#pragma once\n#include <Arduino.h>\n#include <stdint.h>\n\n")
        f.write("namespace dbc {\n\n")
        f.write("// flags bit0: 1=Motorola 0=Intel | bit1: signed |\n")
        f.write("//        bit2: is-multiplexer | bit3: is-multiplexed\n")
        f.write("struct Sig {\n")
        f.write("    char name[16];\n")
        f.write("    uint16_t msgIdx;\n")
        f.write("    uint8_t startBit;\n")
        f.write("    uint8_t bitLen;\n")
        f.write("    uint8_t flags;\n")
        f.write("    int8_t muxVal;   // valid when (flags & 8)\n")
        f.write("    int32_t factor1000;\n")
        f.write("    int32_t offset1000;\n")
        f.write("    char unit[8];\n")
        f.write("};\n\n")
        f.write("struct Msg {\n")
        f.write("    uint32_t id;\n")
        f.write("    uint16_t sigBase;  // index into SIG[]\n")
        f.write("    uint16_t nSig;\n")
        f.write("    uint8_t dlc;\n")
        f.write("    uint8_t ext;\n")
        f.write("    uint8_t dbc;   // F-03: index into DBC_NAMES[];\n")
        f.write("                   // disambiguates duplicate CAN IDs\n")
        f.write("};\n\n")
        f.write("struct EnumVal {\n")
        f.write("    uint16_t sigIdx;\n")
        f.write("    int32_t val;\n")
        f.write("    char desc[16];\n")
        f.write("};\n\n")
        f.write("static const Msg MSG[] = {\n")
        base = 0
        # F-03: numeric DBC index per message (name -> position in the
        # baked DBC order, matching DBC_NAMES[] below).
        dbc_index = {}
        for _di, (_dn, _dm, _ds) in enumerate(baked):
            dbc_index[_dn] = _di
        for mi, mm in enumerate(all_msgs):
            f.write("    { 0x%X, %d, %d, %d, %d, %d },  // %s:%s\n" %
                    (mm["id"], base, len(mm["sigs"]), mm["dlc"], mm["ext"],
                     dbc_index[mm["dbc"]], mm["dbc"], mm["name"]))
            base += len(mm["sigs"])
        f.write("};\n")
        f.write("static const uint16_t MSG_N = %d;\n\n" % len(all_msgs))
        f.write("static const Sig SIG[] = {\n")
        for row in sig_rows:
            mi, name, start, ln, flags, f1000, o1000, unit, muxv = row
            f.write('    { "%s", %d, %d, %d, 0x%02X, %d, %ld, %ld, "%s" },\n' %
                    (name, mi, start, ln, flags, muxv, f1000, o1000, unit))
        f.write("};\n")
        f.write("static const uint16_t SIG_N = %d;\n\n" % len(sig_rows))
        f.write("static const EnumVal ENUMVAL[] = {\n")
        for si, val, desc in enum_rows:
            f.write('    { %d, %ld, "%s" },\n' % (si, val, desc))
        f.write("};\n")
        f.write("static const uint16_t ENUMVAL_N = %d;\n\n" % len(enum_rows))
        f.write("static const char* const DBC_NAMES[] = {\n")
        for n, _, _ in baked:
            f.write("    %s,\n" % cstr(n))
        f.write("};\n")
        f.write("static const uint8_t DBC_N = %d;\n\n" % len(baked))
        f.write("} // namespace dbc\n")
    os.replace(tmp, out)

    est = (len(all_msgs) * 12 + len(sig_rows) * 40 + len(enum_rows) * 24)
    print("wrote %s: %d DBCs, %d msgs, %d sigs, %d enums (~%d KB flash)" %
          (out, len(baked), len(all_msgs), len(sig_rows), len(enum_rows),
           est // 1024))


if __name__ == "__main__":
    main()
