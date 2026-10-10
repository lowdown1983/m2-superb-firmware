// sim_shim/sim_main.cpp — host entry point for [env:sim_m2] / [env:sim_superb].
//
// The REAL firmwares' setup()/loop() drive the sims; this file only provides
// int main() and the Serial/SerialUSB globals. Compiled ONLY in sim envs
// (src_filter in platformio.ini); it never leaks into [env:m2]/[env:superb].

#include <Arduino.h>

HardwareSerial Serial(HardwareSerial::TCP_ROLE);    // Bee UART -> broker
HardwareSerial SerialUSB(HardwareSerial::STDIO_ROLE); // human console -> stdio

int main() {
    sim_detail::initTransports();
    setup();
    for (;;) {
        loop();
        delay(1);  // sim-only scheduler yield; keeps host CPU sane, timing INFERRED
    }
    return 0;
}
