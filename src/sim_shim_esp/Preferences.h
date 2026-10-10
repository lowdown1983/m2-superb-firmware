#pragma once
// sim_shim_esp/Preferences.h — file-backed NVS stand-in for [env:sim_superb].
//
// INFERRED: no flash in sim. Namespaces persist as sim/nvs/<ns>.dat
// (relative to the project root, which is sim/run.sh's working directory).

#include <Arduino.h>

class Preferences {
public:
    Preferences() : open_(false) {}

    bool begin(const char* ns, bool /*readOnly*/ = false) {
        ns_ = ns ? ns : "";
        open_ = true;
        load();
        return true;
    }

    String getString(const char* key, const String& defaultValue) {
        auto it = kv_.find(key ? key : "");
        return it == kv_.end() ? defaultValue : String(it->second.c_str());
    }

    size_t putString(const char* key, const String& value) {
        if (!key) return 0;
        kv_[key] = value.c_str();
        save();
        return value.length();
    }

    void end() { open_ = false; kv_.clear(); }

    // ---- Phase 11: numeric/bytes API (string-backed in sim) ----
    uint16_t getUShort(const char* key, uint16_t dflt) {
        String v = getString(key, "");
        if (v.length() == 0) return dflt;
        char* e; long x = strtol(v.c_str(), &e, 10);
        return (*e == '\0' && x >= 0 && x <= 0xFFFF) ? (uint16_t)x : dflt;
    }
    size_t putUShort(const char* key, uint16_t v) {
        return putString(key, String((unsigned)v));
    }
    uint32_t getUInt(const char* key, uint32_t dflt) {
        String v = getString(key, "");
        if (v.length() == 0) return dflt;
        char* e; unsigned long x = strtoul(v.c_str(), &e, 10);
        return (*e == '\0') ? (uint32_t)x : dflt;
    }
    size_t putUInt(const char* key, uint32_t v) {
        return putString(key, String((unsigned long)v));
    }
    bool getBool(const char* key, bool dflt) {
        String v = getString(key, "");
        if (v.length() == 0) return dflt;
        return v == "1";
    }
    size_t putBool(const char* key, bool v) {
        return putString(key, v ? "1" : "0");
    }
    // Binary as hex (sim only; hardware uses real NVS blobs).
    size_t putBytes(const char* key, const void* data, size_t len) {
        static const char* H = "0123456789ABCDEF";
        const uint8_t* p = (const uint8_t*)data;
        String s;
        for (size_t i = 0; i < len; i++) {
            s += H[(p[i] >> 4) & 0xF];
            s += H[p[i] & 0xF];
        }
        return putString(key, s);
    }
    size_t getBytesLength(const char* key) {
        return getString(key, "").length() / 2;
    }
    size_t getBytes(const char* key, void* buf, size_t maxLen) {
        String s = getString(key, "");
        size_t n = s.length() / 2;
        if (n > maxLen) n = maxLen;
        uint8_t* p = (uint8_t*)buf;
        for (size_t i = 0; i < n; i++) {
            char tmp[3] = { s[(int)(2 * i)], s[(int)(2 * i + 1)], 0 };
            p[i] = (uint8_t)strtoul(tmp, nullptr, 16);
        }
        return n;
    }
    bool isKey(const char* key) {
        return kv_.find(key ? key : "") != kv_.end();
    }

private:
    bool open_;
    std::string ns_;
    std::map<std::string, std::string> kv_;

    std::string path() const { return "sim/nvs/" + ns_ + ".dat"; }

    static std::string esc(const std::string& s) {
        std::string r;
        for (char c : s) {
            if (c == '\\') r += "\\\\";
            else if (c == '\n') r += "\\n";
            else if (c == '\t') r += "\\t";
            else r += c;
        }
        return r;
    }
    static std::string unesc(const std::string& s) {
        std::string r;
        for (size_t i = 0; i < s.size(); i++) {
            if (s[i] == '\\' && i + 1 < s.size()) {
                char n = s[++i];
                r += (n == 'n') ? '\n' : (n == 't') ? '\t' : n;
            } else r += s[i];
        }
        return r;
    }

    void load() {
        kv_.clear();
        FILE* f = fopen(path().c_str(), "r");
        if (!f) return;
        char* line = nullptr;
        size_t cap = 0;
        while (getline(&line, &cap, f) > 0) {
            std::string l(line);
            if (!l.empty() && l.back() == '\n') l.pop_back();
            size_t t = l.find('\t');
            if (t == std::string::npos) continue;
            kv_[unesc(l.substr(0, t))] = unesc(l.substr(t + 1));
        }
        free(line);
        fclose(f);
    }

    void save() {
        // sim/nvs is created by sim/run.sh; be tolerant if missing.
        FILE* f = fopen(path().c_str(), "w");
        if (!f) return;
        for (auto& kv : kv_)
            fprintf(f, "%s\t%s\n", esc(kv.first).c_str(), esc(kv.second).c_str());
        fclose(f);
    }
};
