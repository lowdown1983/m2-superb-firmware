#pragma once
// sim_shim_esp/Update.h — host-native stub of ESP32 Arduino Update for
// [env:sim_superb].
//
// API-compatible subset of what src/superb/main.cpp uses (begin, write,
// end, abort, hasError, getError). All no-ops returning success: OTA is
// not exercised in sim (see docs/SIM.md Phase 11 note).

#include <Arduino.h>

#define UPDATE_ERROR_OK 0

class UpdateClass {
public:
    bool begin(size_t size = 0, int = 0) {
        (void)size;
        written_ = 0;
        return true;
    }
    size_t write(uint8_t* data, size_t len) {
        (void)data;
        written_ += len;
        return len;
    }
    bool end(bool = false) { return true; }
    void abort() { written_ = 0; }
    bool hasError() { return false; }
    uint8_t getError() { return UPDATE_ERROR_OK; }
    size_t progress() { return written_; }

private:
    size_t written_ = 0;
};

extern UpdateClass Update;
