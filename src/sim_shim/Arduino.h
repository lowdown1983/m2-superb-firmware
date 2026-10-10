#pragma once
// sim_shim/Arduino.h — host-native Arduino API shim for the M2/SuperB sim envs.
//
// ADDITIVE-ONLY simulation support: this header is on the include path ONLY
// for [env:sim_m2] / [env:sim_superb] (platformio.ini build_flags). The real
// firmwares (src/m2/*, src/superb/*) compile UNMODIFIED against it.
//
// Evidence: VERIFIED = compiles/runs on host Linux. INFERRED = everything
// this shim stands in for (SAM3X/ESP32 silicon, real UARTs, real CAN,
// real WiFi/BT radios, real NVS flash).
//
// Provided: String, Print, HardwareSerial (Serial = Bee TCP client to the
// broker; SerialUSB = stdio console), GPIO stubs, millis/micros/delay,
// interrupt stubs, EEPROM (file-backed), map/min/max, EspClass (restart).

#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <strings.h>
#include <string>
#include <vector>
#include <deque>
#include <map>
#include <algorithm>
#include <chrono>
#include <thread>

typedef uint8_t byte;
typedef uint16_t word;
typedef bool boolean;

#define HIGH 0x1
#define LOW 0x0
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#define CHANGE 1
#define FALLING 2
#define RISING 3

#define HEX 16
#define DEC 10
#define OCT 8
#define BIN 2

class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper*>(s))

// ---------------------------------------------------------------- String ---
class String {
public:
    String() {}
    String(const char* s) : s_(s ? s : "") {}
    String(const String& o) : s_(o.s_) {}
    String(String&& o) : s_(std::move(o.s_)) {}
    String(char c) { s_ += c; }
    String(int v) : s_(std::to_string(v)) {}
    String(unsigned int v) : s_(std::to_string(v)) {}
    String(long v) : s_(std::to_string(v)) {}
    String(unsigned long v) : s_(std::to_string(v)) {}
    String(int v, int base) : s_(toBase((long long)v, base)) {}
    String(unsigned int v, int base) : s_(toBase((unsigned long long)v, base)) {}
    String(long v, int base) : s_(toBase((long long)v, base)) {}
    String(unsigned long v, int base) : s_(toBase((unsigned long long)v, base)) {}
    String(const __FlashStringHelper* s)
        : s_(reinterpret_cast<const char*>(s)) {}

    String& operator=(const String& o) { s_ = o.s_; return *this; }
    String& operator=(const char* s) { s_ = s ? s : ""; return *this; }

    unsigned int length() const { return (unsigned int)s_.size(); }
    const char* c_str() const { return s_.c_str(); }

    char operator[](unsigned int i) const { return s_[i]; }
    char& operator[](unsigned int i) { return s_[i]; }

    String& operator+=(const String& o) { s_ += o.s_; return *this; }
    String& operator+=(const char* s) { if (s) s_ += s; return *this; }
    String& operator+=(char c) { s_ += c; return *this; }

    bool operator==(const String& o) const { return s_ == o.s_; }
    bool operator==(const char* s) const { return s_ == (s ? s : ""); }
    bool operator!=(const String& o) const { return !(*this == o); }

    String substring(unsigned int begin, unsigned int end) const {
        if (begin > s_.size()) begin = (unsigned int)s_.size();
        if (end > s_.size()) end = (unsigned int)s_.size();
        if (end < begin) end = begin;
        String r; r.s_ = s_.substr(begin, end - begin); return r;
    }
    String substring(unsigned int begin) const { return substring(begin, length()); }

    int indexOf(char c, unsigned int from = 0) const {
        size_t p = s_.find(c, from);
        return p == std::string::npos ? -1 : (int)p;
    }
    int indexOf(const String& s, unsigned int from = 0) const {
        size_t p = s_.find(s.s_, from);
        return p == std::string::npos ? -1 : (int)p;
    }

    void trim() {
        size_t a = s_.find_first_not_of(" \t\r\n");
        if (a == std::string::npos) { s_.clear(); return; }
        size_t b = s_.find_last_not_of(" \t\r\n");
        s_ = s_.substr(a, b - a + 1);
    }

    long toInt() const {
        if (s_.empty()) return 0;
        return strtol(s_.c_str(), nullptr, 10);
    }

private:
    std::string s_;
    static std::string toBase(unsigned long long v, int base) {
        if (base == DEC) return std::to_string(v);
        static const char* D = "0123456789ABCDEF";
        if (v == 0) return "0";
        std::string r;
        while (v) { r = D[v % base] + r; v /= base; }
        return r;
    }
    static std::string toBase(long long v, int base) {
        if (v < 0 && base == DEC) return std::to_string(v);
        return toBase((unsigned long long)v, base);
    }
    friend String operator+(const String&, const String&);
};

inline String operator+(const String& a, const String& b) { String r(a); r += b; return r; }
inline String operator+(const String& a, const char* b) { String r(a); r += b; return r; }
inline String operator+(const char* a, const String& b) { String r(a); r += b; return r; }
inline String operator+(const String& a, char b) { String r(a); r += b; return r; }

// ----------------------------------------------------------------- Print ---
class Print {
public:
    virtual ~Print() {}
    virtual size_t write(uint8_t b) = 0;
    virtual size_t write(const uint8_t* buf, size_t size) {
        size_t n = 0;
        for (size_t i = 0; i < size; i++) n += write(buf[i]);
        return n;
    }

    size_t print(const char* s) { return s ? write((const uint8_t*)s, strlen(s)) : 0; }
    size_t print(const String& s) { return print(s.c_str()); }
    size_t print(const __FlashStringHelper* s) { return print((const char*)s); }
    size_t print(char c) { return write((uint8_t)c); }
    size_t print(int v) { return print(String(v)); }
    size_t print(unsigned int v) { return print(String(v)); }
    size_t print(long v) { return print(String(v)); }
    size_t print(unsigned long v) { return print(String(v)); }
    size_t print(int v, int base) { return print(String(v, base)); }
    size_t print(unsigned int v, int base) { return print(String(v, base)); }
    size_t print(long v, int base) { return print(String(v, base)); }
    size_t print(unsigned long v, int base) { return print(String(v, base)); }

    size_t println() { return write((uint8_t)'\n'); }
    size_t println(const char* s) { size_t n = print(s); n += println(); return n; }
    size_t println(const String& s) { size_t n = print(s); n += println(); return n; }
    size_t println(const __FlashStringHelper* s) { size_t n = print(s); n += println(); return n; }
    size_t println(char c) { size_t n = print(c); n += println(); return n; }
    size_t println(int v) { size_t n = print(v); n += println(); return n; }
    size_t println(unsigned int v) { size_t n = print(v); n += println(); return n; }
    size_t println(long v) { size_t n = print(v); n += println(); return n; }
    size_t println(unsigned long v) { size_t n = print(v); n += println(); return n; }
    size_t println(int v, int base) { size_t n = print(v, base); n += println(); return n; }
    size_t println(unsigned int v, int base) { size_t n = print(v, base); n += println(); return n; }
    size_t println(unsigned long v, int base) { size_t n = print(v, base); n += println(); return n; }
};

// ---------------------------------------------------------- HardwareSerial ---
// Serial  = Bee UART -> TCP client to sim/broker.py (byte pipe, INFERRED).
// SerialUSB = human console -> stdio (the interactive console in sim/run.sh).
class HardwareSerial : public Print {
public:
    enum Role { TCP_ROLE, STDIO_ROLE };
    explicit HardwareSerial(Role r) : role_(r), fd_(-1), eof_(false),
                                        lineLen_(0) {}

    void begin(unsigned long baud);
    void end();
    int available();
    int read();                       // -1 when empty (Arduino semantics)
    size_t write(uint8_t b) override;
    size_t write(const uint8_t* buf, size_t size) override;
    void setRxBufferSize(size_t) {}   // accepted, ignored (INFERRED)
    void flush() {}

private:
    Role role_;
    int fd_;                          // TCP_ROLE: socket
    bool eof_;
    std::deque<uint8_t> rx_;
    // STDIO_ROLE line assembly so sim can intercept `vehicle ...` without
    // feeding those lines into the firmware console parser (src/m2 untouched).
    char lineBuf_[160];
    size_t lineLen_;
    void tcpPump();                   // non-blocking recv into rx_
    void stdioPump();                 // non-blocking stdin into rx_
    void stdioFlushLine();            // handle or forward one assembled line
    static int beePort();             // 14001 (M2) / 14002 (SuperB)
};

extern HardwareSerial Serial;
extern HardwareSerial SerialUSB;

// ------------------------------------------------------------------ GPIO ---
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void analogReadResolution(int bits);  // sim stub: records, no HW effect
void analogWrite(uint8_t pin, int val);
int digitalPinToInterrupt(int pin);
void attachInterrupt(int pin, void (*fn)(), int mode);
void detachInterrupt(int pin);

// ---------------------------------------------------------------- timing ---
unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
void interrupts();
void noInterrupts();

// ------------------------------------------------------------------ misc ---
long map(long x, long in_min, long in_max, long out_min, long out_max);
template <typename T> inline T sim_min(T a, T b) { return a < b ? a : b; }
template <typename T> inline T sim_max(T a, T b) { return a > b ? a : b; }
// NOTE (Phase 11): the min()/max() function-like macros were removed —
// they rewrote std::min/std::max inside <functional>/<unordered_map> and
// broke any TU including those headers. Nothing in the sim or firmware
// uses bare min()/max(); use sim_min/sim_max explicitly.

// M2 pin-name constants. On hardware these come from variants/m2/variant.h;
// the sim include path doesn't carry the variant, so the values are repeated
// here VERBATIM from that file (they are opaque pin IDs to the sim GPIO
// stubs — INFERRED behavior, correct numbering).
#define PS_J1850_9141 49
#define J1850_PWM_VPW 50
#define J1850_PWM_RX 51
#define J1850_VPW_RX 52
#define J1850P_TX 53
#define J1850N_TX 54
#define SPI0_CS3 63
#define SWC_INT 67
#define SWC_M0 64
#define SWC_M1 65

// ---------------------------------------------------------------- EEPROM ---
// File-backed stand-in (M2 firmware doesn't actually touch EEPROM in
// Phase 1-5; provided for API completeness). INFERRED.
class EEPROMClass {
public:
    uint8_t read(int addr);
    void write(int addr, uint8_t v);
    void update(int addr, uint8_t v);
    int length();
};
extern EEPROMClass EEPROM;

// ------------------------------------------------------------------- ESP ---
// On real ESP32 Arduino, ESP comes from the core. Declared here so
// src/superb/main.cpp's ESP.restart() compiles in [env:sim_superb].
// INFERRED: restart() logs and exits(42); the launcher may relaunch.
class EspClass {
public:
    void restart();
    uint32_t getFreeHeap();
    // Phase 11: device identity for the SuperB appliance. INFERRED: fixed
    // fake MAC in sim; real silicon reads the eFuse on hardware.
    uint64_t getEfuseMac() { return 0xAABBCCDDEEFF11ULL; }
};
extern EspClass ESP;

// Phase 11: hardware RNG. INFERRED: deterministic LCG in sim; real
// esp_random() on hardware (declared in esp_system.h there).
inline uint32_t esp_random()
{
    static uint32_t s = 0x12345678;
    s = s * 1664525u + 1013904223u;
    return s;
}

// ------------------------------------------------------- sim entry points ---
void setup();
void loop();

// Internal: called once from sim_main.cpp's main().
namespace sim_detail {
void initTransports();
int beePortDefault();
const char* beeHost();
int beePort();
} // namespace sim_detail
