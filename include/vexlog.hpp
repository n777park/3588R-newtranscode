// vexlog.hpp - timestamped diagnostic logging for the VEX visualizer.
//
// Header-only. Drop it into a PROS or VEXcode project's include/ folder.
// Every sample is written to the SD card at full loop rate and, optionally,
// decimated to a live sink (stdout, or the VEXlink to the worker brain) that
// the visualizer server reads. Both get the same line format, so an SD log
// imports into the visualizer exactly like a live recording:
//
//     $S,<stream>,<field>,...          schema
//     $D,<stream>,<brain_ms>,<v>,...   sample
//     $E,<brain_ms>,<text>             event marker
//
// Typical use (PROS):
//
//     #include "vexlog.hpp"
//     auto& L = vexlog::get();
//     L.openSD();                                   // /usd/log_000.txt, ...
//     L.setLiveSink([](const char* s, size_t n) { fwrite(s, 1, n, stdout); }, 5);
//     auto& drive = L.stream("drive", {"l_ref", "l_vel", "l_ff", "l_fb", "l_cmd"});
//     L.startFlushTask();
//
//     while (true) {
//         L.loopStart();
//         ... control code ...
//         drive.log({lRef, lVel, lFF, lFB, lCmd});
//         L.loopEnd();                              // logs the "loop" stream
//         pros::delay(10);
//     }
//
// Logging from the control loop only formats text into a RAM buffer; the SD
// write and the live send happen in the flush task, so a slow card never
// stretches the loop. If the buffer backs up past MAX_BUFFER the newest
// lines are dropped and counted, never blocking.
//
// See docs/log-format.md for the stream and field names the visualizer's
// diagnostic panels look for.

#pragma once

#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <initializer_list>
#include <memory>
#include <string>
#include <vector>

// ---- platform shim: PROS, VEXcode, or a host build for testing ----
#if !defined(VEXLOG_PROS) && !defined(VEXLOG_VEXCODE) && !defined(VEXLOG_HOST)
#  if defined(__has_include)
#    if __has_include("api.h")
#      define VEXLOG_PROS
#    elif __has_include("vex.h")
#      define VEXLOG_VEXCODE
#    else
#      define VEXLOG_HOST
#    endif
#  else
#    define VEXLOG_HOST
#  endif
#endif

#if defined(VEXLOG_PROS)
#  include "api.h"
#elif defined(VEXLOG_VEXCODE)
#  include "vex.h"
#else
#  include <chrono>
#  include <mutex>
#  include <thread>
#endif

namespace vexlog {

namespace platform {
#if defined(VEXLOG_PROS)
inline double nowMs() { return pros::micros() / 1000.0; }
inline void sleepMs(int ms) { pros::delay(ms); }
using Mutex = pros::Mutex;
inline void spawn(void (*fn)(void*), void* arg) {
    // Lowest user priority: logging must never preempt control.
    new pros::Task(fn, arg, TASK_PRIORITY_MIN + 1, TASK_STACK_DEPTH_DEFAULT, "vexlog");
}
#elif defined(VEXLOG_VEXCODE)
inline double nowMs() { return vex::timer::systemHighResolution() / 1000.0; }
inline void sleepMs(int ms) { vex::this_thread::sleep_for(ms); }
using Mutex = vex::mutex;
inline void spawn(void (*fn)(void*), void* arg) {
    vex::thread t(fn, arg);
    t.setPriority(vex::thread::threadPrioritylow);
}
#else
inline double nowMs() {
    using namespace std::chrono;
    static const auto t0 = steady_clock::now();
    return duration<double, std::milli>(steady_clock::now() - t0).count();
}
inline void sleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
using Mutex = std::mutex;
inline void spawn(void (*fn)(void*), void* arg) { std::thread(fn, arg).detach(); }
#endif
}  // namespace platform

using Sink = std::function<void(const char* data, size_t len)>;

constexpr size_t MAX_BUFFER = 64 * 1024;     // per sink, bytes
constexpr double SCHEMA_RESEND_MS = 2000.0;  // live viewers that join late
constexpr int FLUSH_PERIOD_MS = 50;
constexpr double SD_REOPEN_MS = 1000.0;     // see Logger::flush

class Logger;

class Stream {
public:
    // One sample, values in the order the fields were declared. A size
    // mismatch is a programming error; the sample is dropped rather than
    // written misaligned. NaN (e.g. an unplugged sensor) is written as "nan"
    // and shows as a gap in the plots.
    void log(std::initializer_list<double> values);
    void log(const std::vector<double>& values) { logRange(values.data(), values.size()); }

    const std::string& name() const { return name_; }

private:
    friend class Logger;
    Stream(Logger* owner, std::string name, std::vector<std::string> fields, int liveEvery)
        : owner_(owner), name_(std::move(name)), fields_(std::move(fields)),
          liveEvery_(liveEvery < 1 ? 1 : liveEvery) {}
    void logRange(const double* v, size_t n);
    std::string schemaLine() const;

    Logger* owner_;
    std::string name_;
    std::vector<std::string> fields_;
    int liveEvery_;
    unsigned count_ = 0;
};

class Logger {
public:
    // Opens the next free <dir>/<prefix>_NNN.txt. Returns false without an
    // SD card; logging then goes to the live sink only.
    bool openSD(const char* dir = defaultDir(), const char* prefix = "log") {
#if defined(VEXLOG_PROS)
        if (!pros::usd::is_installed()) return false;
#endif
        char path[96];
        for (int i = 0; i < 1000; i++) {
            // Empty dir (VEXcode) = card root, written as a bare name.
            if (*dir) snprintf(path, sizeof path, "%s/%s_%03d.txt", dir, prefix, i);
            else snprintf(path, sizeof path, "%s_%03d.txt", prefix, i);
            FILE* probe = std::fopen(path, "r");
            if (probe) { std::fclose(probe); continue; }
            sd_ = std::fopen(path, "w");
            break;
        }
        if (!sd_) return false;
        sdPath_ = path;
        lock();
        for (auto& s : streams_) sdBuf_ += s->schemaLine();
        unlock();
        return true;
    }

    // Live output (USB serial, VEXlink...). Each stream's liveEvery further
    // decimates on top of `everyN`, so the radio carries what it can afford.
    void setLiveSink(Sink sink, int everyN = 1) {
        live_ = std::move(sink);
        liveEvery_ = everyN < 1 ? 1 : everyN;
        lastSchemaMs_ = -1e9;
    }

    // Declare a stream. Keep the returned reference; declare everything
    // before the control loop starts. Names: letters, digits, underscore.
    Stream& stream(const char* name, std::initializer_list<const char*> fields, int liveEvery = 1) {
        std::vector<std::string> f(fields.begin(), fields.end());
        lock();  // the flush task iterates streams_
        streams_.emplace_back(new Stream(this, name, std::move(f), liveEvery));
        Stream& s = *streams_.back();
        if (sd_) append(sdBuf_, s.schemaLine());
        lastSchemaMs_ = -1e9;
        unlock();
        return s;
    }

    // Event marker ("auton start", "segment 3", "shot"...). Shown as a flag
    // on every plot; analyzers split runs on them.
    void event(const char* fmt, ...) {
        char text[120];
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(text, sizeof text, fmt, ap);
        va_end(ap);
        for (char* c = text; *c; c++)
            if (*c == '\n' || *c == '\r') *c = ' ';
        char line[160];
        int n = snprintf(line, sizeof line, "$E,%.1f,%s\n", platform::nowMs(), text);
        if (n <= 0) return;
        std::string s(line, n < (int)sizeof line ? n : (int)sizeof line - 1);
        lock();
        if (sd_) append(sdBuf_, s);
        if (live_) append(liveBuf_, s);
        unlock();
    }

    // Loop timing: call loopStart() first thing in the control loop and
    // loopEnd() just before its delay. Logs stream "loop" with dt (start to
    // start, ms) and exec (time spent computing, ms).
    void loopStart() {
        double now = platform::nowMs();
        loopDt_ = lastLoopStart_ > 0 ? now - lastLoopStart_ : NAN;
        lastLoopStart_ = now;
    }
    void loopEnd() {
        if (!loop_) loop_ = &stream("loop", {"dt", "exec"});
        loop_->log({loopDt_, platform::nowMs() - lastLoopStart_});
    }

    // Writes buffered data out. startFlushTask() calls this every 50 ms from
    // a low-priority task; call it yourself if you would rather not spawn one.
    void flush() {
        std::string sd, live;
        lock();
        double now = platform::nowMs();
        if (live_ && now - lastSchemaMs_ >= SCHEMA_RESEND_MS) {
            std::string schema;
            for (auto& s : streams_) schema += s->schemaLine();
            liveBuf_.insert(0, schema);
            lastSchemaMs_ = now;
        }
        sd.swap(sdBuf_);
        live.swap(liveBuf_);
        unlock();

        if (sd_ && !sd.empty()) {
            std::fwrite(sd.data(), 1, sd.size(), sd_);
            std::fflush(sd_);
        }
        // The V5 only commits SD writes to the card when the file is closed,
        // so a program stopped mid-run would otherwise leave an empty log.
        // Reopening in append mode every second bounds the loss to ~1 s.
        if (sd_ && now - lastReopenMs_ >= SD_REOPEN_MS) {
            std::fclose(sd_);
            sd_ = std::fopen(sdPath_.c_str(), "a");
            lastReopenMs_ = now;
        }
        if (live_ && !live.empty()) live_(live.data(), live.size());
    }

    void startFlushTask() {
        if (flushing_) return;
        flushing_ = true;
        platform::spawn(&Logger::flushTask, this);
    }

    void close() {
        flush();
        if (sd_) std::fclose(sd_);
        sd_ = nullptr;
    }

    unsigned dropped() const { return dropped_; }
    const std::string& sdPath() const { return sdPath_; }

    static const char* defaultDir() {
#if defined(VEXLOG_PROS)
        return "/usd";
#elif defined(VEXLOG_VEXCODE)
        return "";  // VEXcode maps the card's root to the bare path
#else
        return ".";
#endif
    }

private:
    friend class Stream;

    static void flushTask(void* self) {
        auto* l = static_cast<Logger*>(self);
        while (true) {
            l->flush();
            platform::sleepMs(FLUSH_PERIOD_MS);
        }
    }

    void lock() { mutex_.lock(); }
    void unlock() { mutex_.unlock(); }

    void append(std::string& buf, const std::string& s) {
        if (buf.size() + s.size() > MAX_BUFFER) { dropped_++; return; }
        buf += s;
    }

    void write(const Stream& s, const char* line, size_t n) {
        bool toLive = live_ && (s.count_ % (unsigned)(s.liveEvery_ * liveEvery_)) == 0;
        lock();
        if (sd_) {
            if (sdBuf_.size() + n > MAX_BUFFER) dropped_++;
            else sdBuf_.append(line, n);
        }
        if (toLive) {
            if (liveBuf_.size() + n > MAX_BUFFER) dropped_++;
            else liveBuf_.append(line, n);
        }
        unlock();
    }

    platform::Mutex mutex_;
    std::vector<std::unique_ptr<Stream>> streams_;
    std::string sdBuf_, liveBuf_;
    FILE* sd_ = nullptr;
    std::string sdPath_;
    Sink live_;
    int liveEvery_ = 1;
    double lastSchemaMs_ = -1e9;
    double lastReopenMs_ = 0;
    double lastLoopStart_ = 0;
    double loopDt_ = NAN;
    Stream* loop_ = nullptr;
    unsigned dropped_ = 0;
    bool flushing_ = false;
};

inline Logger& get() {
    static Logger logger;
    return logger;
}

inline void Stream::log(std::initializer_list<double> values) {
    logRange(values.begin(), values.size());
}

inline void Stream::logRange(const double* v, size_t n) {
    if (n != fields_.size()) return;
    char line[512];
    int len = snprintf(line, sizeof line, "$D,%s,%.1f", name_.c_str(), platform::nowMs());
    for (size_t i = 0; i < n && len > 0 && len < (int)sizeof line - 16; i++) {
        len += std::isfinite(v[i])
                   ? snprintf(line + len, sizeof line - len, ",%.6g", v[i])
                   : snprintf(line + len, sizeof line - len, ",nan");
    }
    if (len <= 0 || len >= (int)sizeof line - 2) return;  // too many fields for one line
    line[len++] = '\n';
    owner_->write(*this, line, len);
    count_++;
}

inline std::string Stream::schemaLine() const {
    std::string s = "$S," + name_;
    for (auto& f : fields_) s += "," + f;
    return s + "\n";
}

// Staleness of a sensor that has no timestamp API: the time since its value
// last changed. Distance sensors refresh every ~33 ms and GPS every ~20 ms,
// while the loop runs every 10 ms, so the same reading is reused several
// times; logging this exposes how old the data really is when it is used.
class Freshness {
public:
    double update(double value) {
        double now = platform::nowMs();
        if (!seen_ || value != last_) {
            last_ = value;
            changedAt_ = now;
            seen_ = true;
        }
        return now - changedAt_;
    }

private:
    double last_ = 0, changedAt_ = 0;
    bool seen_ = false;
};

}  // namespace vexlog
