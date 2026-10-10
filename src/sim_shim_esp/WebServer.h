#pragma once
// sim_shim_esp/WebServer.h — host-native WebServer for [env:sim_superb].
//
// A minimal REAL HTTP/1.1 server on 127.0.0.1 (firmware WEB_PORT 80 ->
// sim 18080; port 80 needs root). Single-threaded, one request per
// handleClient() call, connection closed after each response — matching
// how the firmware drives it (handleClient() every loop()).
//
// Supported call pattern (exactly what src/superb/main.cpp uses):
//   on(path, handler) / on(path, HTTP_POST, handler) /
//   on(path, HTTP_POST, handler, uploader)
//   setContentLength(CONTENT_LENGTH_UNKNOWN) + send(200, type, "") +
//       sendContent(...)...            -> streamed with chunked encoding
//   send(200, type, body)              -> Content-Length response
//   sendHeader() (before send), arg()/hasArg() (GET query + urlencoded
//   POST body), upload() -> HTTPUpload& (multipart/form-data file parts
//   drive UPLOAD_FILE_START/WRITE/END).
// INFERRED throughout: no TLS, no keep-alive, no chunked requests.

#include <Arduino.h>

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define HTTP_GET 1
#define HTTP_POST 2
#define HTTP_ANY 0

static const size_t CONTENT_LENGTH_UNKNOWN = (size_t)-1;

enum {
    UPLOAD_FILE_START = 0,
    UPLOAD_FILE_WRITE = 1,
    UPLOAD_FILE_END = 2,
    UPLOAD_FILE_ABORTED = 3  // Phase 11: matches real Arduino WebServer
};

struct HTTPUpload {
    int status = -1;
    const char* filename = "";
    const char* name = "";
    const char* type = "";
    uint8_t* buf = nullptr;
    size_t currentSize = 0;
    size_t totalSize = 0;
};

typedef void (*THandlerFunction)();

class WebServer {
public:
    explicit WebServer(int port)
        : port_(port == 80 ? 18080 : port), listenFd_(-1),
          chunked_(false), headersSent_(false) {}

    void begin() {
        listenFd_ = socket(AF_INET, SOCK_STREAM, 0);
        int one = 1;
        setsockopt(listenFd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_port = htons(port_);
        inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
        if (bind(listenFd_, (sockaddr*)&a, sizeof(a)) < 0) {
            fprintf(stderr, "[sim] WebServer bind :%u failed: %s\n",
                    port_, strerror(errno));
            close(listenFd_);
            listenFd_ = -1;
            return;
        }
        listen(listenFd_, 4);
        int fl = fcntl(listenFd_, F_GETFL, 0);
        fcntl(listenFd_, F_SETFL, fl | O_NONBLOCK);
        fprintf(stderr, "[sim] Web GUI on http://127.0.0.1:%u\n", port_);
    }

    void on(const char* uri, THandlerFunction handler) {
        on(uri, HTTP_GET, handler, nullptr);
    }
    void on(const char* uri, int method, THandlerFunction handler) {
        on(uri, method, handler, nullptr);
    }
    void on(const char* uri, int method, THandlerFunction handler,
            THandlerFunction uploader) {
        routes_.push_back({uri ? uri : "", method, handler, uploader});
    }

    void handleClient() {
        if (listenFd_ < 0) return;
        int cfd = accept(listenFd_, nullptr, nullptr);
        if (cfd < 0) return;  // non-blocking: no pending client
        // Bound the time one request can stall the loop (INFERRED).
        timeval tv{3, 0};
        setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        serve(cfd);
        close(cfd);
    }

    // ---- response API ----
    void setContentLength(size_t len) { chunked_ = (len == CONTENT_LENGTH_UNKNOWN); }

    void sendHeader(const String& name, const String& value) {
        if (!headersSent_) respHeaders_.push_back({name.c_str(), value.c_str()});
        // (Called after send() by the firmware's pageHead(); the real
        // library also has no effect there — headers already on the wire.)
    }

    void send(int code, const char* contentType, const String& content) {
        std::string h = "HTTP/1.1 ";
        h += std::to_string(code);
        h += " ";
        h += reason(code);
        h += "\r\nContent-Type: ";
        h += contentType ? contentType : "text/plain";
        h += "\r\n";
        for (auto& kv : respHeaders_)
            h += kv.first + ": " + kv.second + "\r\n";
        if (chunked_) {
            h += "Transfer-Encoding: chunked\r\nConnection: close\r\n\r\n";
        } else {
            h += "Content-Length: " + std::to_string(content.length()) + "\r\n";
            h += "Connection: close\r\n\r\n";
        }
        sendAll(h.data(), h.size());
        headersSent_ = true;
        if (chunked_) {
            if (content.length() > 0) writeChunk(content.c_str(), content.length());
        } else if (content.length() > 0) {
            sendAll(content.c_str(), content.length());
        }
    }

    void sendContent(const String& s) {
        if (s.length() == 0) return;  // empty chunk would terminate the stream
        if (chunked_ && headersSent_) writeChunk(s.c_str(), s.length());
    }
    void sendContent(const char* s) {
        if (s && *s) sendContent(String(s));
    }

    // ---- request API ----
    String arg(const char* name) const {
        auto it = args_.find(name ? name : "");
        return it == args_.end() ? String("") : String(it->second.c_str());
    }
    bool hasArg(const char* name) const {
        return args_.find(name ? name : "") != args_.end();
    }
    HTTPUpload& upload() { return upload_; }

    // ---- Phase 11: request headers (for the session Cookie) ----
    // Real Arduino WebServer requires collectHeaders() before header()
    // works; the sim keeps every parsed request header, so collectHeaders
    // is a no-op here.
    void collectHeaders(const char* const*, size_t) {}
    String header(const char* name) const {
        if (!name) return String("");
        std::string k = name;
        std::transform(k.begin(), k.end(), k.begin(), ::tolower);
        auto it = reqHeaders_.find(k);
        return it == reqHeaders_.end() ? String("") : String(it->second.c_str());
    }

private:
    struct Route {
        std::string uri;
        int method;
        THandlerFunction handler;
        THandlerFunction uploader;
    };

    uint16_t port_;
    int listenFd_;
    std::vector<Route> routes_;

    // per-request state
    int cfd_ = -1;
    std::map<std::string, std::string> args_;
    std::map<std::string, std::string> reqHeaders_;
    std::vector<std::pair<std::string, std::string>> respHeaders_;
    bool chunked_;
    bool headersSent_;
    HTTPUpload upload_;
    std::vector<uint8_t> upChunk_;

    static const char* reason(int code) {
        switch (code) {
            case 200: return "OK";
            case 400: return "Bad Request";
            case 404: return "Not Found";
            case 500: return "Internal Server Error";
            default: return "OK";
        }
    }

    void sendAll(const char* data, size_t n) {
        size_t sent = 0;
        while (sent < n) {
            ssize_t r = ::send(cfd_, data + sent, n - sent, MSG_NOSIGNAL);
            if (r <= 0) break;
            sent += (size_t)r;
        }
    }

    void writeChunk(const char* data, size_t n) {
        char h[32];
        int hn = snprintf(h, sizeof(h), "%zx\r\n", n);
        sendAll(h, (size_t)hn);
        sendAll(data, n);
        sendAll("\r\n", 2);
    }

    static std::string urlDecode(const std::string& s) {
        std::string r;
        for (size_t i = 0; i < s.size(); i++) {
            if (s[i] == '%' && i + 2 < s.size()) {
                char t[3] = {s[i + 1], s[i + 2], 0};
                r += (char)strtol(t, nullptr, 16);
                i += 2;
            } else if (s[i] == '+') {
                r += ' ';
            } else {
                r += s[i];
            }
        }
        return r;
    }

    void parsePairs(const std::string& q) {
        size_t i = 0;
        while (i < q.size()) {
            size_t amp = q.find('&', i);
            std::string kv = q.substr(i, amp == std::string::npos ? amp : amp - i);
            size_t eq = kv.find('=');
            std::string k = urlDecode(eq == std::string::npos ? kv : kv.substr(0, eq));
            std::string v = eq == std::string::npos ? "" : urlDecode(kv.substr(eq + 1));
            args_[k] = v;
            if (amp == std::string::npos) break;
            i = amp + 1;
        }
    }

    bool readN(std::string& out, size_t n) {
        out.clear();
        out.reserve(n);
        char tmp[4096];
        while (out.size() < n) {
            size_t want = n - out.size() < sizeof(tmp) ? n - out.size() : sizeof(tmp);
            ssize_t r = recv(cfd_, tmp, want, 0);
            if (r <= 0) return false;
            out.append(tmp, (size_t)r);
        }
        return true;
    }

    // Minimal multipart/form-data parser: drives the uploader for file
    // parts, folds text parts into args_. INFERRED simplification.
    void handleMultipart(const std::string& ctype, const std::string& body,
                         THandlerFunction uploader) {
        size_t bp = ctype.find("boundary=");
        if (bp == std::string::npos || !uploader) return;
        std::string bnd = "--" + ctype.substr(bp + 9);
        // strip possible quotes/trailing params
        size_t q = bnd.find('"');
        if (q != std::string::npos) bnd.erase(q, 1);
        size_t sc = bnd.find(';');
        if (sc != std::string::npos) bnd.erase(sc);

        size_t pos = 0;
        while ((pos = body.find(bnd, pos)) != std::string::npos) {
            pos += bnd.size();
            if (body.compare(pos, 2, "--") == 0) break;  // final boundary
            if (body.compare(pos, 2, "\r\n") == 0) pos += 2;
            size_t hend = body.find("\r\n\r\n", pos);
            if (hend == std::string::npos) break;
            std::string ph = body.substr(pos, hend - pos);
            size_t dstart = hend + 4;
            size_t dend = body.find(bnd, dstart);
            if (dend == std::string::npos) break;
            size_t dlen = dend >= 2 ? dend - 2 - dstart : 0;  // strip trailing CRLF

            // Content-Disposition: form-data; name="x"; filename="y"
            std::string name, filename;
            size_t np = ph.find("name=\"");
            if (np != std::string::npos) {
                np += 6;
                name = ph.substr(np, ph.find('"', np) - np);
            }
            size_t fp = ph.find("filename=\"");
            if (fp != std::string::npos) {
                fp += 10;
                filename = ph.substr(fp, ph.find('"', fp) - fp);
            }
            if (!filename.empty()) {
                upload_.status = UPLOAD_FILE_START;
                upload_.filename = "";  // points into locals; handler uses immediately
                upload_.name = "";
                upload_.type = "";
                upload_.buf = nullptr;
                upload_.currentSize = 0;
                upload_.totalSize = body.size();
                // stash names in stable storage for the call sequence
                upName_ = name; upFile_ = filename;
                upload_.name = upName_.c_str();
                upload_.filename = upFile_.c_str();
                uploader();
                size_t off = 0;
                upChunk_.resize(8192);
                while (off < dlen) {
                    size_t cn = dlen - off < upChunk_.size() ? dlen - off : upChunk_.size();
                    memcpy(upChunk_.data(), body.data() + dstart + off, cn);
                    upload_.status = UPLOAD_FILE_WRITE;
                    upload_.buf = upChunk_.data();
                    upload_.currentSize = cn;
                    uploader();
                    off += cn;
                }
                upload_.status = UPLOAD_FILE_END;
                upload_.buf = nullptr;
                upload_.currentSize = 0;
                uploader();
            } else if (!name.empty()) {
                args_[name] = body.substr(dstart, dlen);
            }
            pos = dend;
        }
    }
    std::string upName_, upFile_;

    void serve(int cfd) {
        cfd_ = cfd;
        args_.clear();
        respHeaders_.clear();
        chunked_ = false;
        headersSent_ = false;
        upload_.status = -1;

        // --- read headers ---
        std::string head;
        char tmp[4096];
        for (int tries = 0; tries < 50; tries++) {
            ssize_t r = recv(cfd_, tmp, sizeof(tmp) - 1, 0);
            if (r <= 0) return;
            head.append(tmp, (size_t)r);
            if (head.find("\r\n\r\n") != std::string::npos) break;
            if (head.size() > 65536) return;
        }
        size_t hend = head.find("\r\n\r\n");
        if (hend == std::string::npos) return;
        std::string body = head.substr(hend + 4);
        head.erase(hend);

        // --- request line ---
        size_t eol = head.find("\r\n");
        std::string rl = head.substr(0, eol);
        size_t sp1 = rl.find(' ');
        size_t sp2 = rl.find(' ', sp1 + 1);
        if (sp1 == std::string::npos) return;
        std::string method = rl.substr(0, sp1);
        std::string target = rl.substr(sp1 + 1, sp2 == std::string::npos
                                                   ? sp2 : sp2 - sp1 - 1);
        int mcode = (method == "POST") ? HTTP_POST : HTTP_GET;

        // --- headers ---
        std::map<std::string, std::string> headers;
        size_t p = eol == std::string::npos ? head.size() : eol + 2;
        while (p < head.size()) {
            size_t e = head.find("\r\n", p);
            std::string line = head.substr(p, e == std::string::npos ? e : e - p);
            size_t c = line.find(':');
            if (c != std::string::npos) {
                std::string k = line.substr(0, c), v = line.substr(c + 1);
                // trim + lowercase key
                k.erase(0, k.find_first_not_of(" \t"));
                std::transform(k.begin(), k.end(), k.begin(), ::tolower);
                v.erase(0, v.find_first_not_of(" \t"));
                headers[k] = v;
            }
            if (e == std::string::npos) break;
            p = e + 2;
        }

        // --- path / query ---
        reqHeaders_ = headers;  // Phase 11: expose to header()
        std::string path = target, query;
        size_t qm = target.find('?');
        if (qm != std::string::npos) {
            path = target.substr(0, qm);
            query = target.substr(qm + 1);
        }
        if (!query.empty()) parsePairs(query);

        // --- body ---
        size_t clen = 0;
        auto ci = headers.find("content-length");
        if (ci != headers.end()) clen = (size_t)strtoul(ci->second.c_str(), nullptr, 10);
        if (clen > 0) {
            if (clen > 32 * 1024 * 1024) {  // sanity cap (INFERRED)
                send(400, "text/plain", String("body too large"));
                return;
            }
            std::string rest;
            if (!readN(rest, clen - body.size())) return;
            body += rest;
        }
        std::string ctype;
        auto ti = headers.find("content-type");
        if (ti != headers.end()) ctype = ti->second;

        // --- route ---
        const Route* rt = nullptr;
        for (auto& r : routes_) {
            if (r.uri == path && (r.method == HTTP_ANY || r.method == mcode)) {
                rt = &r;
                break;
            }
        }
        if (!rt) {
            send(404, "text/plain", String("not found"));
            return;
        }

        if (mcode == HTTP_POST && ctype.find("multipart/form-data") == 0) {
            handleMultipart(ctype, body, rt->uploader);
        } else if (mcode == HTTP_POST &&
                   ctype.find("application/x-www-form-urlencoded") == 0) {
            parsePairs(body);
        }

        rt->handler();

        if (chunked_ && headersSent_) sendAll("0\r\n\r\n", 5);  // end chunks
    }
};
