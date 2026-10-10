#!/usr/bin/env python3
"""Sign a SuperB firmware image for OTA update (Phase 11).

Package format (uploaded to the SuperB's /ota_upload endpoint):
    header  (16B): magic "M2SBOTA1" (8B) | version u32 LE | image_len u32 LE
    sig     (64B): ECDSA P-256 raw r||s over SHA256(header || image)
    image (image_len bytes): the raw app .bin (pio run -e superb -> .bin)

The private key is read from ~/.ota-keys/superb_ota_priv.pem (created by
scripts/ota_keygen.py) — it NEVER lives in the firmware tree.

Usage:
    python3 scripts/sign_ota.py --bin .pio/build/superb/firmware.bin \\
        --version 2 --key ~/.ota-keys/superb_ota_priv.pem \\
        --out superb-ota-v2.ota

    python3 scripts/sign_ota.py --verify --bin package.ota \\
        --key ~/.ota-keys/superb_ota_priv.pem   # self-test the signature

Version is monotonic: the SuperB rejects version <= its stored version
(anti-rollback). First OTA image should be version 1 (fresh NVS = 0).

Requires: cryptography (pip install cryptography)
"""
import argparse
import hashlib
import struct
import sys

try:
    from cryptography.hazmat.primitives.asymmetric import ec, utils
    from cryptography.hazmat.primitives import hashes, serialization
except ImportError:
    sys.exit("need 'cryptography': pip install cryptography")

MAGIC = b"M2SBOTA1"
HDR_FMT = "<8sII"  # magic, version, image_len
HDR_LEN = struct.calcsize(HDR_FMT)
SIG_LEN = 64


def load_priv(path):
    with open(path, "rb") as f:
        return serialization.load_pem_private_key(f.read(), password=None)


def der_to_raw(sig_der):
    r, s = utils.decode_dss_signature(sig_der)
    return r.to_bytes(32, "big") + s.to_bytes(32, "big")


def raw_to_der(sig_raw):
    r = int.from_bytes(sig_raw[:32], "big")
    s = int.from_bytes(sig_raw[32:], "big")
    return utils.encode_dss_signature(r, s)


def build_package(image, version, priv):
    header = struct.pack(HDR_FMT, MAGIC, version, len(image))
    digest = hashlib.sha256(header + image).digest()
    sig_der = priv.sign(digest,
                        ec.ECDSA(utils.Prehashed(hashes.SHA256())))
    return header + der_to_raw(sig_der) + image


def verify_package(pkg, priv):
    if len(pkg) < HDR_LEN + SIG_LEN:
        return False, "too short"
    magic, version, img_len = struct.unpack(HDR_FMT, pkg[:HDR_LEN])
    if magic != MAGIC:
        return False, "bad magic"
    sig_raw, image = pkg[HDR_LEN:HDR_LEN + SIG_LEN], pkg[HDR_LEN + SIG_LEN:]
    if len(image) != img_len:
        return False, f"image_len {img_len} != actual {len(image)}"
    digest = hashlib.sha256(pkg[:HDR_LEN] + image).digest()
    pub = priv.public_key()
    try:
        pub.verify(raw_to_der(sig_raw), digest,
                   ec.ECDSA(utils.Prehashed(hashes.SHA256())))
    except Exception as e:  # noqa: BLE001 - report any verify failure
        return False, f"bad signature: {e}"
    return True, f"OK: version {version}, {img_len} bytes"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", required=True, help=".bin image (or .ota to verify)")
    ap.add_argument("--version", type=int, default=1)
    ap.add_argument("--key", required=True, help="private key PEM")
    ap.add_argument("--out", help="output .ota path")
    ap.add_argument("--verify", action="store_true", help="verify only")
    args = ap.parse_args()

    priv = load_priv(args.key)
    if not isinstance(priv, ec.EllipticCurvePrivateKey) or \
            not isinstance(priv.curve, ec.SECP256R1):
        sys.exit("key is not P-256")

    with open(args.bin, "rb") as f:
        data = f.read()

    if args.verify:
        ok, msg = verify_package(data, priv)
        print(msg)
        sys.exit(0 if ok else 1)

    if not args.out:
        sys.exit("--out required to sign")
    if args.version < 1 or args.version > 0xFFFFFFFF:
        sys.exit("version must be 1..2^32-1")
    pkg = build_package(data, args.version, priv)
    with open(args.out, "wb") as f:
        f.write(pkg)
    ok, msg = verify_package(pkg, priv)
    print(f"wrote {args.out} ({len(pkg)} bytes); self-verify: {msg}")
    assert ok


if __name__ == "__main__":
    main()
