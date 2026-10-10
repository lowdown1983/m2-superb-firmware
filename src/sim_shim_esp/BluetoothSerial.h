#pragma once
// sim_shim_esp/BluetoothSerial.h — host-native BT SPP stand-in.
//
// INFERRED: no Bluetooth radio in sim. The SPP byte pipe is backed by a
// REAL TCP server on 127.0.0.1:14024 (docs/SIM.md "BT SPP is TCP in sim").
// A phone/app connects with any TCP client instead of a BT terminal.

#include <Arduino.h>

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>

class BluetoothSerial {
public:
    BluetoothSerial() : listenFd_(-1) {}

    bool begin(const char* /*name*/) {
        listenFd_ = socket(AF_INET, SOCK_STREAM, 0);
        int one = 1;
        setsockopt(listenFd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_port = htons(14024);
        inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
        if (bind(listenFd_, (sockaddr*)&a, sizeof(a)) < 0) {
            fprintf(stderr, "[sim] BluetoothSerial bind :14024 failed: %s\n",
                    strerror(errno));
            close(listenFd_);
            listenFd_ = -1;
            return false;
        }
        listen(listenFd_, 1);
        int fl = fcntl(listenFd_, F_GETFL, 0);
        fcntl(listenFd_, F_SETFL, fl | O_NONBLOCK);
        fprintf(stderr, "[sim] BT SPP shim on 127.0.0.1:14024 (TCP)\n");
        return true;
    }

    void end() {
        if (client_ >= 0) { close(client_); client_ = -1; }
        if (listenFd_ >= 0) { close(listenFd_); listenFd_ = -1; }
    }

    bool hasClient() {
        pumpAccept();
        if (client_ < 0) return false;
        char c;
        ssize_t n = recv(client_, &c, 1, MSG_PEEK | MSG_DONTWAIT);
        if (n == 0) { close(client_); client_ = -1; return false; }
        return true;
    }

    int available() {
        if (!hasClient()) return 0;
        int n = 0;
        if (ioctl(client_, FIONREAD, &n) < 0) return 0;
        return n;
    }

    int read() {
        if (!hasClient()) return -1;
        uint8_t b;
        ssize_t n = recv(client_, &b, 1, 0);
        return n <= 0 ? -1 : b;
    }

    size_t write(uint8_t b) { return write(&b, 1); }
    size_t write(const uint8_t* buf, size_t size) {
        if (!hasClient()) return 0;
        size_t sent = 0;
        while (sent < size) {
            ssize_t n = send(client_, buf + sent, size - sent, MSG_NOSIGNAL);
            if (n <= 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) { delay(1); continue; }
                close(client_); client_ = -1;
                break;
            }
            sent += (size_t)n;
        }
        return sent;
    }

private:
    int listenFd_;
    int client_ = -1;

    void pumpAccept() {
        if (listenFd_ < 0) return;
        if (client_ >= 0) return;  // single client (like the firmware's use)
        int fd = accept(listenFd_, nullptr, nullptr);
        if (fd >= 0) {
            int fl = fcntl(fd, F_GETFL, 0);
            fcntl(fd, F_SETFL, fl | O_NONBLOCK);
            client_ = fd;
            fprintf(stderr, "[sim] BT shim client connected\n");
        }
    }
};
