// rover_protocol.h — Pi<->STM32 serial protocol, receive side.
// Header-only, no OS or ROS dependencies. Must match PiLink.cpp on the STM32.
//
// Frame: [0xA5][type][len][payload (len bytes)][crc_lo][crc_hi]
// CRC-16/CCITT-FALSE (init 0xFFFF, poly 0x1021) over type + len + payload.
#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace rover {

constexpr uint8_t  kSync          = 0xA5;
constexpr uint8_t  kMsgImu        = 0x01;
constexpr size_t   kImuPayloadLen = 33;
constexpr size_t   kMaxPayloadLen = 43;   // largest length we accept (kMaxFrameLen 48 - 5)

inline uint16_t crc16CcittFalse(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

struct ImuMsg {
    uint8_t  seq;
    uint32_t timestamp_ms;   // STM32 clock
    float    yaw;            // rad, wrapped to +-pi
    float    gyro[3];        // rad/s
    float    accel[3];       // m/s^2
};

// Assumes a little-endian host (true for the Pi and the STM32).
inline ImuMsg decodeImu(const uint8_t* p) {
    ImuMsg m;
    m.seq = p[0];
    std::memcpy(&m.timestamp_ms, p + 1, 4);
    std::memcpy(&m.yaw,          p + 5, 4);
    std::memcpy(m.gyro,          p + 9, 12);
    std::memcpy(m.accel,         p + 21, 12);
    return m;
}

struct ParserStats {
    uint32_t framesOk       = 0;
    uint32_t crcFailures    = 0;
    uint32_t badLengths     = 0;
    uint32_t bytesDiscarded = 0;   // bytes skipped while hunting for a sync byte
};

// Feed raw bytes in; complete, CRC-valid frames come out through the callback
// as onFrame(type, payload, length). The payload pointer is only valid during the call.
class FrameParser {
public:
    template <typename OnFrame>
    void feed(const uint8_t* data, size_t n, OnFrame&& onFrame) {
        buf_.insert(buf_.end(), data, data + n);
        size_t pos = 0;                                   // start of unconsumed data
        for (;;) {
            // hunt for sync
            while (pos < buf_.size() && buf_[pos] != kSync) { pos++; stats_.bytesDiscarded++; }
            if (buf_.size() - pos < 3) break;             // need sync + type + len
            const uint8_t type = buf_[pos + 1];
            const uint8_t len  = buf_[pos + 2];
            if (len > kMaxPayloadLen) {                   // false sync: skip just this byte
                stats_.badLengths++; pos++; continue;
            }
            const size_t frameLen = 3 + len + 2;
            if (buf_.size() - pos < frameLen) break;      // wait for more bytes
            const uint8_t* f = &buf_[pos];
            const uint16_t rx = f[3 + len] | (static_cast<uint16_t>(f[4 + len]) << 8);
            if (rx != crc16CcittFalse(f + 1, 2 + len)) {  // bad frame: drop only the sync byte
                stats_.crcFailures++; pos++; continue;
            }
            stats_.framesOk++;
            onFrame(type, f + 3, static_cast<size_t>(len));
            pos += frameLen;
        }
        buf_.erase(buf_.begin(), buf_.begin() + pos);
    }
    const ParserStats& stats() const { return stats_; }

private:
    std::vector<uint8_t> buf_;
    ParserStats stats_;
};

}  // namespace rover