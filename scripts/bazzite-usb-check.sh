#!/usr/bin/env bash
# bazzite-usb-check.sh — diagnose M2/SuperB USB connectivity on Bazzite.
#
# Checks every known blocker between the Macchina M2 and PlatformIO on an
# immutable/atomic Fedora workstation:
#   1. user in `dialout` group
#   2. udev rule for the M2 (VID 0x2341/PID 0x003e sketch mode,
#      VID 0x03eb/PID 0x6124 SAM-BA bootloader during upload)
#   3. ModemManager not grabbing the M2's port
#   4. /dev/ttyACM* visible AND writable by you
#   5. M2 enumerates on lsusb
#   6. VS Code Flatpak (if installed) has device access
#
# Usage: ./scripts/bazzite-usb-check.sh
# Exit 0 = all checks pass. Exit 1 = at least one blocker found.

PASS=0; FAIL=0

ok()   { echo "  [PASS] $1"; PASS=$((PASS+1)); }
bad()  { echo "  [FAIL] $1"; echo "         -> $2"; FAIL=$((FAIL+1)); }
info() { echo "  [info] $1"; }

echo "=== Macchina M2 USB check (Bazzite) ==="
echo

# --- 0. are we even on an atomic Fedora? -------------------------------------
if grep -qiE "bazzite|kinoite|silverblue|universal-blue" /etc/os-release 2>/dev/null; then
    info "Atomic/immutable Fedora variant detected: $(grep ^PRETTY_NAME= /etc/os-release | cut -d= -f2 | tr -d '\"')"
else
    info "Not an atomic Fedora variant ($(grep ^PRETTY_NAME= /etc/os-release 2>/dev/null | cut -d= -f2 | tr -d '\"')). Checks still apply to regular Fedora."
fi
echo

# --- 1. dialout group ---------------------------------------------------------
echo "--- 1. dialout group membership ---"
if id -nG "$USER" | tr ' ' '\n' | grep -qx dialout; then
    ok "user '$USER' is in the dialout group"
else
    bad "user '$USER' is NOT in the dialout group" \
        "run: sudo usermod -aG dialout \$USER   then LOG OUT and back in"
fi
echo

# --- 2. udev rule --------------------------------------------------------------
echo "--- 2. udev rule for the M2 ---"
RULE_OK=0
for f in /etc/udev/rules.d/*.rules /usr/lib/udev/rules.d/*.rules; do
    [ -f "$f" ] || continue
    if grep -q "2341" "$f" 2>/dev/null && grep -q "003e" "$f" 2>/dev/null; then
        info "found M2 rule in $f"
        RULE_OK=1
    fi
done
if [ "$RULE_OK" -eq 1 ]; then
    ok "udev rule covering VID 2341 / PID 003e exists"
else
    bad "no udev rule found for the M2 (2341:003e)" \
        "create /etc/udev/rules.d/99-macchina-m2.rules per SETUP.md §1b, then: sudo udevadm control --reload-rules && sudo udevadm trigger"
fi
echo

# --- 3. ModemManager ------------------------------------------------------------
echo "--- 3. ModemManager ---"
if systemctl is-active --quiet ModemManager 2>/dev/null; then
    bad "ModemManager is RUNNING — it probes CDC-ACM devices and steals the M2's port" \
        "run: sudo systemctl disable --now ModemManager   (or add ID_MM_DEVICE_IGNORE to the udev rule)"
else
    ok "ModemManager is not running"
fi
echo

# --- 4. /dev/ttyACM* visibility + writability -----------------------------------
echo "--- 4. serial device nodes ---"
ACM_DEVS=(/dev/ttyACM*)
if [ -e "${ACM_DEVS[0]}" ]; then
    for dev in "${ACM_DEVS[@]}"; do
        if [ -r "$dev" ] && [ -w "$dev" ]; then
            ok "$dev is readable+writable"
        else
            bad "$dev exists but is NOT readable/writable by you" \
                "check dialout membership (log out/in after usermod) and the udev rule MODE/GROUP"
        fi
    done
else
    bad "no /dev/ttyACM* device found" \
        "is the M2 plugged into its own micro-USB port? If yes and lsusb (§5) sees it, the Flatpak/container device access is missing (SETUP.md §1)"
fi
echo

# --- 5. lsusb enumeration --------------------------------------------------------
echo "--- 5. lsusb enumeration ---"
if command -v lsusb >/dev/null 2>&1; then
    M2USB=$(lsusb | grep -iE "2341:003e|03eb:6124" || true)
    if [ -n "$M2USB" ]; then
        ok "M2 visible on USB:"
        echo "$M2USB" | sed 's/^/         /'
        echo "$M2USB" | grep -qi "03eb:6124" && \
            info "03eb:6124 = SAM-BA bootloader mode (normal during bossac upload)"
    else
        bad "M2 NOT found on USB (looked for 2341:003e and 03eb:6124)" \
            "try another cable (charge-only cables are common), another port, and check dmesg | tail after plugging in"
    fi
else
    info "lsusb not installed; skipping (sudo dnf install -y usbutils on host, or inside distrobox)"
fi
echo

# --- 6. VS Code Flatpak device access ---------------------------------------------
echo "--- 6. VS Code Flatpak sandbox (if installed) ---"
if flatpak info com.visualstudio.code >/dev/null 2>&1; then
    PERMS=$(flatpak info --show-permissions com.visualstudio.code 2>/dev/null)
    if echo "$PERMS" | grep -q "devices=all"; then
        ok "VS Code Flatpak has --device=all"
    else
        bad "VS Code Flatpak does NOT have device access" \
            "run: flatpak override --device=all com.visualstudio.code   then FULLY quit and reopen VS Code"
    fi
else
    info "VS Code Flatpak not installed — skipping (Path B / distrobox users ignore this)"
fi
echo

# --- summary ----------------------------------------------------------------------
echo "=== result: $PASS passed, $FAIL failed ==="
if [ "$FAIL" -eq 0 ]; then
    echo "All clear. Plug in the M2 and run:  pio device list"
    echo "(then flash per SETUP.md §3; SuperB rides the same USB port per §4)"
    exit 0
else
    echo "Fix the FAIL items above (each has its remedy), then re-run this script."
    exit 1
fi
