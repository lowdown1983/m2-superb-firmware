#pragma once
// sim_shim_esp/WiFi.h — host-native WiFi stand-in for [env:sim_superb].
//
// INFERRED: there are no radios in sim. WiFi.softAP()/STA are recorded
// no-ops (status() reports WL_CONNECTED once begin() was called, like the
// firmware's display code expects). WiFiServer/WiFiClient are REAL TCP
// sockets on 127.0.0.1 so SavvyCAN-style TCP apps can connect.
// Port map (docs/SIM.md): firmware TCP 23 -> sim 14023 (port 23 needs root).

#include <Arduino.h>

#include <memory>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>

enum {
    WL_IDLE_STATUS = 0,
    WL_NO_SSID_AVAIL,
    WL_SCAN_COMPLETED,
    WL_CONNECTED,
    WL_CONNECT_FAILED,
    WL_CONNECTION_LOST,
    WL_DISCONNECTED
};

#define WIFI_AP 1
#define WIFI_STA 2
#define WIFI_AP_STA 3
#define WIFI_OFF 0  // Phase 11: radio shutdown (sim no-op)

// Firmware TCP 23 needs root on Linux; the sim remaps it (docs/SIM.md).
inline uint16_t simNetPort(uint16_t p) { return p == 23 ? 14023 : p; }

class IPAddress {
public:
    IPAddress() { b_[0] = b_[1] = b_[2] = b_[3] = 0; }
    IPAddress(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
        b_[0] = a; b_[1] = b; b_[2] = c; b_[3] = d;
    }
    String toString() const {
        char t[16];
        snprintf(t, sizeof(t), "%u.%u.%u.%u", b_[0], b_[1], b_[2], b_[3]);
        return String(t);
    }
private:
    uint8_t b_[4];
};

namespace simnet {
inline int setNonblock(int fd) {
    int fl = fcntl(fd, F_GETFL, 0);
    return fcntl(fd, F_SETFL, fl | O_NONBLOCK);
}
// Shared socket handle: copies of a client share one fd; stop() poisons it.
struct Sock {
    int fd;
    explicit Sock(int f = -1) : fd(f) {}
    ~Sock() { if (fd >= 0) ::close(fd); }
};
} // namespace simnet

class WiFiClient {
public:
    WiFiClient() : s_(new simnet::Sock(-1)) {}
    explicit WiFiClient(int fd) : s_(new simnet::Sock(fd)) {}

    operator bool() const { return s_ && s_->fd >= 0; }

    bool connected() {
        if (!s_ || s_->fd < 0) return false;
        char c;
        ssize_t n = recv(s_->fd, &c, 1, MSG_PEEK | MSG_DONTWAIT);
        if (n == 0) { stop(); return false; }  // orderly shutdown
        return true;
    }

    int available() {
        if (!connected()) return 0;
        int n = 0;
        if (ioctl(s_->fd, FIONREAD, &n) < 0) return 0;
        return n;
    }

    int read() {
        if (!connected()) return -1;
        uint8_t b;
        ssize_t n = recv(s_->fd, &b, 1, 0);
        if (n <= 0) return -1;
        return b;
    }

    size_t write(uint8_t b) { return write(&b, 1); }
    size_t write(const uint8_t* buf, size_t size) {
        if (!connected()) return 0;
        size_t sent = 0;
        while (sent < size) {
            ssize_t n = send(s_->fd, buf + sent, size - sent, MSG_NOSIGNAL);
            if (n <= 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    delay(1);
                    continue;
                }
                stop();
                break;
            }
            sent += (size_t)n;
        }
        return sent;
    }

    void stop() {
        if (s_ && s_->fd >= 0) { ::close(s_->fd); s_->fd = -1; }
    }

private:
    std::shared_ptr<simnet::Sock> s_;
};

class WiFiServer {
public:
    explicit WiFiServer(uint16_t port) : port_(simNetPort(port)), listenFd_(-1) {}

    void begin() {
        listenFd_ = socket(AF_INET, SOCK_STREAM, 0);
        int one = 1;
        setsockopt(listenFd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_port = htons(port_);
        inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
        if (bind(listenFd_, (sockaddr*)&a, sizeof(a)) < 0) {
            fprintf(stderr, "[sim] WiFiServer bind :%u failed: %s\n",
                    port_, strerror(errno));
            close(listenFd_);
            listenFd_ = -1;
            return;
        }
        listen(listenFd_, 4);
        simnet::setNonblock(listenFd_);
        fprintf(stderr, "[sim] TCP bridge listening on 127.0.0.1:%u\n", port_);
    }

    WiFiClient available() {
        if (listenFd_ < 0) return WiFiClient();
        int fd = accept(listenFd_, nullptr, nullptr);
        if (fd < 0) return WiFiClient();
        simnet::setNonblock(fd);
        return WiFiClient(fd);
    }

private:
    uint16_t port_;
    int listenFd_;
};

class WiFiClass {
public:
    void mode(int /*m*/) {}
    bool softAP(const char* /*ssid*/, const char* /*pass*/) {
        apUp_ = true;  // INFERRED: no real AP; recorded only
        return true;
    }
    IPAddress softAPIP() { return IPAddress(192, 168, 4, 1); }  // INFERRED
    int begin(const char* /*ssid*/, const char* /*pass*/) {
        staUp_ = true;  // INFERRED: no real STA join
        return WL_CONNECTED;
    }
    int status() { return staUp_ ? WL_CONNECTED : WL_DISCONNECTED; }
    IPAddress localIP() { return IPAddress(192, 168, 1, 100); }  // INFERRED
    // Phase 11: radio power management (no-ops in sim).
    bool disconnect(bool wifioff = false) {
        staUp_ = false;
        if (wifioff) apUp_ = false;
        return true;
    }
    bool setSleep(bool /*enable*/) { return true; }

private:
    bool apUp_ = false;
    bool staUp_ = false;
};

extern WiFiClass WiFi;
