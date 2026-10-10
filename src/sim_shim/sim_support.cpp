// sim_shim/sim_support.cpp — host implementations for the Arduino API shim.
// Linux-only (INFERRED stand-ins for SAM3X/ESP32 hardware).

#include <Arduino.h>
#include <SPI.h>  // sim shim: SPIClass SPI instance below

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <time.h>

namespace sim_detail {

const char* beeHost() {
    const char* h = getenv("SIM_BEE_HOST");
    return h && *h ? h : "127.0.0.1";
}
int beePortDefault() {
#ifdef SIM_M2
    return 14001;
#else
    return 14002;
#endif
}
int beePort() {
    const char* p = getenv("SIM_BEE_PORT");
    if (p && *p) {
        long v = strtol(p, nullptr, 10);
        if (v > 0 && v < 65536) return (int)v;
    }
    return beePortDefault();
}

void initTransports() {
    // Unbuffered stdout so the SerialUSB console is interactive.
    setvbuf(stdout, nullptr, _IONBF, 0);
}

} // namespace sim_detail

// ---------------------------------------------------------- HardwareSerial ---
int HardwareSerial::beePort() { return sim_detail::beePort(); }

void HardwareSerial::begin(unsigned long /*baud*/) {
    if (role_ == STDIO_ROLE) {
        int fl = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, fl | O_NONBLOCK);
        return;
    }
    // TCP_ROLE: connect to the broker, retrying until it answers. Baud is
    // meaningless in sim (byte pipe); 115200 is documented, not enforced.
    const char* host = sim_detail::beeHost();
    int port = beePort();
    bool announced = false;
    for (;;) {
        int fd = socket(AF_INET, SOCK_STREAM, 0);
        if (fd >= 0) {
            sockaddr_in a{};
            a.sin_family = AF_INET;
            a.sin_port = htons((uint16_t)port);
            inet_pton(AF_INET, host, &a.sin_addr);
            if (connect(fd, (sockaddr*)&a, sizeof(a)) == 0) {
                int fl = fcntl(fd, F_GETFL, 0);
                fcntl(fd, F_SETFL, fl | O_NONBLOCK);
                fd_ = fd;
                if (announced) fprintf(stderr, "[sim] Bee TCP connected (%s:%d)\n", host, port);
                return;
            }
            close(fd);
        }
        if (!announced) {
            fprintf(stderr, "[sim] waiting for Bee broker at %s:%d ...\n", host, port);
            announced = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

void HardwareSerial::end() {
    if (fd_ >= 0) { close(fd_); fd_ = -1; }
}

void HardwareSerial::tcpPump() {
    if (fd_ < 0) return;
    uint8_t tmp[1024];
    for (;;) {
        ssize_t n = recv(fd_, tmp, sizeof(tmp), 0);
        if (n > 0) {
            for (ssize_t i = 0; i < n; i++) rx_.push_back(tmp[i]);
        } else {
            if (n == 0) { close(fd_); fd_ = -1; }  // broker went away
            break;
        }
    }
}

void HardwareSerial::stdioPump() {
    if (eof_) return;
    uint8_t tmp[256];
    for (;;) {
        ssize_t n = ::read(STDIN_FILENO, tmp, sizeof(tmp));
        if (n > 0) {
            for (ssize_t i = 0; i < n; i++) rx_.push_back(tmp[i]);
        } else if (n == 0) {
            eof_ = true;  // stdin closed
            break;
        } else {
            break;  // EAGAIN
        }
    }
}

int HardwareSerial::available() {
    if (role_ == TCP_ROLE) tcpPump();
    else stdioPump();
    return (int)rx_.size();
}

int HardwareSerial::read() {
    if (rx_.empty()) {
        if (role_ == TCP_ROLE) tcpPump();
        else stdioPump();
    }
    if (rx_.empty()) return -1;
    uint8_t b = rx_.front();
    rx_.pop_front();
    return b;
}

size_t HardwareSerial::write(uint8_t b) {
    return write(&b, 1);
}

size_t HardwareSerial::write(const uint8_t* buf, size_t size) {
    if (role_ == STDIO_ROLE) {
        size_t n = fwrite(buf, 1, size, stdout);
        fflush(stdout);
        return n;
    }
    if (fd_ < 0) return 0;
    size_t sent = 0;
    while (sent < size) {
        ssize_t n = send(fd_, buf + sent, size - sent, MSG_NOSIGNAL);
        if (n <= 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            close(fd_); fd_ = -1;
            break;
        }
        sent += (size_t)n;
    }
    return sent;
}

// ------------------------------------------------------------------ GPIO ---
// Pin state is recorded; nothing is driven (INFERRED — no silicon).
static uint8_t pinMode_[256];
static uint8_t pinState_[256];

void pinMode(uint8_t pin, uint8_t mode) { pinMode_[pin] = mode; }
void digitalWrite(uint8_t pin, uint8_t val) { pinState_[pin] = val ? HIGH : LOW; }
int digitalRead(uint8_t pin) { return pinState_[pin]; }
int analogRead(uint8_t /*pin*/) { return 0; }
void analogReadResolution(int /*bits*/) {}
void analogWrite(uint8_t /*pin*/, int /*val*/) {}
int digitalPinToInterrupt(int pin) { return pin; }

static void (*isrTable_[256])() = {nullptr};
void attachInterrupt(int pin, void (*fn)(), int /*mode*/) {
    if (pin >= 0 && pin < 256) isrTable_[pin] = fn;  // stored, never fires (INFERRED)
}
void detachInterrupt(int pin) {
    if (pin >= 0 && pin < 256) isrTable_[pin] = nullptr;
}

// ---------------------------------------------------------------- timing ---
static uint64_t monoNs() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}
unsigned long millis() { return (unsigned long)(monoNs() / 1000000ull); }
unsigned long micros() { return (unsigned long)(monoNs() / 1000ull); }
void delay(unsigned long ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
void delayMicroseconds(unsigned int us) {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}
void interrupts() {}
void noInterrupts() {}

// ------------------------------------------------------------------ misc ---
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ---------------------------------------------------------------- EEPROM ---
static const char* eepromPath() {
#ifdef SIM_M2
    return "sim/eeprom_sim_m2.bin";
#else
    return "sim/eeprom_sim_superb.bin";
#endif
}
static uint8_t eepromImg_[4096];
static bool eepromLoaded_ = false;
static void eepromLoad() {
    if (eepromLoaded_) return;
    eepromLoaded_ = true;
    memset(eepromImg_, 0xFF, sizeof(eepromImg_));
    FILE* f = fopen(eepromPath(), "rb");
    if (f) {
        fread(eepromImg_, 1, sizeof(eepromImg_), f);
        fclose(f);
    }
}
static void eepromSave() {
    FILE* f = fopen(eepromPath(), "wb");
    if (f) {
        fwrite(eepromImg_, 1, sizeof(eepromImg_), f);
        fclose(f);
    }
}
uint8_t EEPROMClass::read(int addr) {
    eepromLoad();
    if (addr < 0 || addr >= (int)sizeof(eepromImg_)) return 0;
    return eepromImg_[addr];
}
void EEPROMClass::write(int addr, uint8_t v) {
    eepromLoad();
    if (addr < 0 || addr >= (int)sizeof(eepromImg_)) return;
    eepromImg_[addr] = v;
    eepromSave();
}
void EEPROMClass::update(int addr, uint8_t v) {
    if (read(addr) != v) write(addr, v);
}
int EEPROMClass::length() { return (int)sizeof(eepromImg_); }
EEPROMClass EEPROM;

// ------------------------------------------------------------------- ESP ---
void EspClass::restart() {
    fprintf(stderr, "[sim] ESP.restart() requested — exiting(42)\n");
    fflush(stderr);
    _exit(42);
}
uint32_t EspClass::getFreeHeap() { return 1u << 20; }  // INFERRED placeholder
EspClass ESP;

// ------------------------------------------------------------------- SPI ---
// Sim instance for the SPI.h shim (main.cpp calls SPI.begin()).
SPIClass SPI;
