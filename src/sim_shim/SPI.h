// SPI.h — sim shim (native Linux build).
//
// The M2 firmware's only direct SPI use is SPI.begin() in main.cpp (the
// MCP2515 SPI traffic is compiled out under SIM_M2). This stub exists so
// sim_m2 keeps building after the 2026-10-08 SPI.begin() boot-hang fix.
#pragma once
#include <stdint.h>

class SPIClass {
public:
    void begin() {}
    uint8_t transfer(uint8_t d) { return d; }
    void end() {}
};

extern SPIClass SPI;
