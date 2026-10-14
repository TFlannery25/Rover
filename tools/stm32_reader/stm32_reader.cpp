// stm32_reader.cpp — Linux (Raspberry Pi) console reader for the STM32 IMU stream.
// Build: g++ -std=c++17 -O2 -Wall stm32_reader.cpp -o stm32_reader
// Run:   ./stm32_reader [/dev/stm32] [baud]
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include "rover_protocol.h"

static speed_t baudConstant(int baud) {
    switch (baud) {
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return 0;
    }
}

static int openSerial(const char* path, int baud) {
    speed_t speed = baudConstant(baud);
    if (!speed) { fprintf(stderr, "Unsupported baud %d\n", baud); return -1; }

    int fd = open(path, O_RDWR | O_NOCTTY);
    if (fd < 0) { fprintf(stderr, "open %s: %s\n", path, strerror(errno)); return -1; }

    termios tio{};
    if (tcgetattr(fd, &tio) != 0) { perror("tcgetattr"); close(fd); return -1; }
    cfmakeraw(&tio);                       // 8 data bits, no parity, no processing
    tio.c_cflag |= (CLOCAL | CREAD);       // ignore modem lines, enable receiver
    tio.c_cflag &= ~CSTOPB;                // 1 stop bit
    tio.c_cflag &= ~CRTSCTS;               // no hardware flow control
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    tio.c_cc[VMIN]  = 0;                   // read() returns after up to 100 ms
    tio.c_cc[VTIME] = 1;                   // even with no data
    if (tcsetattr(fd, TCSANOW, &tio) != 0) { perror("tcsetattr"); close(fd); return -1; }
    tcflush(fd, TCIOFLUSH);                // drop stale bytes from before we opened
    return fd;
}

int main(int argc, char** argv) {
    const char* port = argc > 1 ? argv[1] : "/dev/stm32";
    int baud         = argc > 2 ? atoi(argv[2]) : 115200;

    int fd = openSerial(port, baud);
    if (fd < 0) return 1;
    fprintf(stderr, "Reading %s at %d baud (Ctrl+C to stop)\n", port, baud);

    setvbuf(stdout, nullptr, _IOLBF, 0);   // line-buffered even when piped to a file
    rover::FrameParser parser;
    int lastSeq = -1;
    uint32_t imuCount = 0, lostTotal = 0;
    auto t0 = std::chrono::steady_clock::now();
    uint32_t countAtT0 = 0;

    uint8_t chunk[256];
    for (;;) {
        ssize_t n = read(fd, chunk, sizeof(chunk));
        if (n < 0) {
            if (errno == EINTR) continue;
            fprintf(stderr, "read: %s\n", strerror(errno));
            break;
        }
        if (n > 0) parser.feed(chunk, static_cast<size_t>(n),
            [&](uint8_t type, const uint8_t* payload, size_t len) {
                if (type != rover::kMsgImu || len != rover::kImuPayloadLen) return;
                rover::ImuMsg m = rover::decodeImu(payload);

                char gap[32] = "";
                if (lastSeq >= 0) {
                    int diff = (m.seq - lastSeq) & 0xFF;
                    if (diff == 0)      snprintf(gap, sizeof gap, "  [duplicate]");
                    else if (diff != 1) { snprintf(gap, sizeof gap, "  [lost %d]", diff - 1); lostTotal += diff - 1; }
                }
                lastSeq = m.seq;
                imuCount++;

                printf("seq=%3u t=%8u ms yaw=%+.3f gyro=(%+.3f,%+.3f,%+.3f) accel=(%+.2f,%+.2f,%+.2f)%s\n",
                       m.seq, m.timestamp_ms, m.yaw, m.gyro[0], m.gyro[1], m.gyro[2],
                       m.accel[0], m.accel[1], m.accel[2], gap);
            });

        // once per second: measured rate and link health, on stderr
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - t0).count();
        if (dt >= 1.0) {
            const auto& s = parser.stats();
            fprintf(stderr, "[stats] %.1f Hz  ok=%u crc_fail=%u bad_len=%u lost=%u\n",
                    (imuCount - countAtT0) / dt, s.framesOk, s.crcFailures, s.badLengths, lostTotal);
            t0 = now; countAtT0 = imuCount;
        }
    }
    close(fd);
    return 0;
}