# M2 board variant

Copied verbatim from Macchina's official `arduino-boards-sam` repo — the same
source the hardware audit verified. Wired into the build via
`build.variants_dir` + `build.variant` in `boards/macchina_m2.json`.

`libsam_sam3x8e_gcc_rel.a` is the prebuilt Atmel SAM software-framework
archive from that same upstream source. It is intentionally vendored here:
the `[env:m2]` linker looks for it in this directory, so a fresh clone
builds without fetching anything extra.
