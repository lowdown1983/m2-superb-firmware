#pragma once
// sim_shim_esp: host-native SPIFFS stand-in for sim_superb.
// Backed by real files under /tmp/m2sb_spiffs/ so the Phase 12 log sink
// can be exercised on the host. API surface is the subset the SuperB
// firmware uses (begin/open/exists/remove + File read/write/seek/size).
// On real ESP32 hardware this header is shadowed by the Arduino core's
// SPIFFS.h (the sim -I flag is what makes this one win on the host).

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <sys/stat.h>

class File {
public:
    File() : f(nullptr) {}
    explicit File(FILE* f_) : f(f_) {}
    operator bool() const { return f != nullptr; }
    size_t write(const uint8_t* b, size_t n)
    {
        return f ? fwrite(b, 1, n, f) : 0;
    }
    size_t write(uint8_t b)
    {
        return (f && fputc(b, f) != EOF) ? 1 : 0;
    }
    int read() { return f ? fgetc(f) : -1; }
    size_t read(uint8_t* b, size_t n)
    {
        return f ? fread(b, 1, n, f) : 0;
    }
    void close()
    {
        if (f) {
            fclose(f);
            f = nullptr;
        }
    }
    size_t size()
    {
        if (!f) return 0;
        long p = ftell(f);
        fseek(f, 0, SEEK_END);
        long s = ftell(f);
        fseek(f, p, SEEK_SET);
        return (size_t)s;
    }
    size_t position() { return f ? (size_t)ftell(f) : 0; }
    bool seek(size_t p) { return f && fseek(f, (long)p, SEEK_SET) == 0; }
    void flush() { if (f) fflush(f); }

private:
    FILE* f;
};

class SPIFFSClass {
public:
    bool begin(bool formatOnFail = false)
    {
        (void)formatOnFail;
        ::mkdir("/tmp/m2sb_spiffs", 0755);
        return true;
    }
    File open(const char* path, const char* mode = "r")
    {
        std::string p = "/tmp/m2sb_spiffs";
        p += path ? path : "";
        // Ensure the parent directory exists.
        size_t slash = p.rfind('/');
        if (slash != std::string::npos) {
            std::string dir = p.substr(0, slash);
            std::string cmd = "mkdir -p ";
            cmd += dir;
            ::system(cmd.c_str());
        }
        FILE* f = ::fopen(p.c_str(), mode);
        return File(f);
    }
    bool exists(const char* path)
    {
        std::string p = "/tmp/m2sb_spiffs";
        p += path ? path : "";
        struct stat st;
        return ::stat(p.c_str(), &st) == 0;
    }
    bool remove(const char* path)
    {
        std::string p = "/tmp/m2sb_spiffs";
        p += path ? path : "";
        return ::remove(p.c_str()) == 0;
    }

private:
    static void mkdir(const char* d, int m) { ::mkdir(d, (mode_t)m); }
};

inline SPIFFSClass SPIFFS;
