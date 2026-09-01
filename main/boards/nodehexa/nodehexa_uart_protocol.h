#ifndef NODEHEXA_UART_PROTOCOL_H_
#define NODEHEXA_UART_PROTOCOL_H_

#include <stddef.h>
#include <stdint.h>

#include <vector>

namespace nodehexa_uart {

constexpr size_t kMaxPayloadLength = 512;
constexpr uint32_t kFrameTimeoutMs = 250;

enum class Format : uint8_t { Legacy, V2 };
enum class MessageType : uint8_t { Hello = 1, Request = 2, Response = 3, Event = 4, Heartbeat = 5 };

struct Frame {
    Format format;
    MessageType message_type;
    uint8_t flags;
    uint16_t sequence;
    uint16_t payload_length;
    uint8_t payload[kMaxPayloadLength + 1];
};

struct Stats {
    uint32_t rx_frames;
    uint32_t crc_errors;
    uint32_t length_errors;
    uint32_t timeouts;
    uint32_t overflow_errors;
    uint32_t legacy_frames;
};

uint16_t Crc16Ccitt(const uint8_t* data, size_t length, uint16_t initial = 0xffff);
bool EncodeV2(MessageType type, uint8_t flags, uint16_t sequence, const uint8_t* payload,
              uint16_t payload_length, std::vector<uint8_t>& output);

class Parser {
public:
    Parser();
    bool Feed(uint8_t byte, uint32_t now_ms, Frame& frame);
    void PollTimeout(uint32_t now_ms);
    const Stats& stats() const { return stats_; }

private:
    enum class State : uint8_t {
        Seek,
        SeekMagic1,
        V2Header,
        V2Payload,
        V2CrcLow,
        V2CrcHigh,
        LegacyPayload
    };
    void ResetFrame();
    void StartV2(uint32_t now_ms);
    void StartLegacy(uint32_t now_ms);
    void CheckTimeout(uint32_t now_ms);

    State state_;
    Frame working_;
    uint8_t header_[7];
    uint8_t header_index_;
    uint16_t payload_index_;
    uint16_t received_crc_;
    uint32_t last_byte_ms_;
    bool active_;
    Stats stats_;
};

}  // namespace nodehexa_uart

#endif  // NODEHEXA_UART_PROTOCOL_H_
