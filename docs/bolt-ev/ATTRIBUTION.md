# Attribution — Bolt EV PID sources

- `bolt17.json`, `bolt19.json`: from `iternio/ev-obd-pids` (Apache License 2.0),
  a community consolidation of openly-published EV OBD PID lists. Provided
  as-is; formulas are community reverse-engineering, NOT GM documentation.
- Diagnostic addressing and CAN matrix cross-checked against
  `openvehicles/Open-Vehicle-Monitoring-System-3`,
  `vehicle/OVMS.V3/components/vehicle_boltev` (MIT license), an open-source
  vehicle module validated by its community on real Bolt EVs.
- Bus architecture: GM Upfitter 2023 Bolt EUV documentation (gmupfitter.com,
  official GM) and ACDelco TechConnect GMLAN reference.

Evidence grades in BOLT_EV_TEST.md follow the OEM-database convention:
A = OEM/official document, B = open-source community implementation with
real-vehicle use, C = internet PID-list consolidation. INFERRED = my synthesis.
