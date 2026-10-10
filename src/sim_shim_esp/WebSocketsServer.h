#pragma once
// sim_shim_esp/WebSocketsServer.h — host-native stub of links2004/WebSockets
// WebSocketsServer for [env:sim_superb].
//
// API-compatible subset of what src/superb/main.cpp uses (begin, loop,
// onEvent, sendTXT/sendBIN, broadcastTXT/broadcastBIN, disconnect). All
// no-ops: the sim has no WS clients. INFERRED: WS transport is untested
// in sim (see docs/SIM.md Phase 11 note).

#include <Arduino.h>
#include <functional>

enum WStype_t : uint8_t {
    WStype_ERROR,
    WStype_DISCONNECTED,
    WStype_CONNECTED,
    WStype_TEXT,
    WStype_BIN,
    WStype_FRAGMENT_TEXT_START,
    WStype_FRAGMENT_BIN_START,
    WStype_FRAGMENT,
    WStype_FRAGMENT_FIN,
    WStype_PING,
    WStype_PONG
};

typedef std::function<void(uint8_t num, WStype_t type, uint8_t* payload,
                           size_t length)>
    WebSocketServerEvent;

class WebSocketsServer {
public:
    explicit WebSocketsServer(uint16_t port) : port_(port) {}
    void begin() {
        fprintf(stderr, "[sim] WebSocketsServer stub on :%u (no-op)\n", port_);
    }
    void onEvent(WebSocketServerEvent cb) { cb_ = cb; }
    void loop() {}
    bool sendTXT(uint8_t, uint8_t*, size_t = 0, bool = false) { return false; }
    bool sendTXT(uint8_t, const uint8_t*, size_t = 0) { return false; }
    bool sendTXT(uint8_t, const char*, size_t = 0) { return false; }
    bool sendTXT(uint8_t, String&) { return false; }  // matches real lib (non-const)
    bool broadcastTXT(uint8_t*, size_t = 0, bool = false) { return false; }
    bool broadcastTXT(const uint8_t*, size_t = 0) { return false; }
    bool broadcastTXT(const char*, size_t = 0) { return false; }
    bool broadcastTXT(String&) { return false; }
    bool sendBIN(uint8_t, uint8_t*, size_t, bool = false) { return false; }
    bool sendBIN(uint8_t, const uint8_t*, size_t) { return false; }
    bool broadcastBIN(uint8_t*, size_t, bool = false) { return false; }
    bool broadcastBIN(const uint8_t*, size_t) { return false; }
    void disconnect() {}
    void disconnect(uint8_t) {}

private:
    uint16_t port_;
    WebSocketServerEvent cb_;
};
