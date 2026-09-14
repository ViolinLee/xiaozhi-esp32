#include "nodehexa_uart_protocol.h"

#include <cstring>

namespace nodehexa_uart {
namespace {

constexpr uint8_t kMagic0 = 0xa5;
constexpr uint8_t kMagic1 = 0x4e;
constexpr uint8_t kVersion = 0x02;

}  // namespace

uint16_t Crc16Ccitt(const uint8_t* data, size_t length, uint16_t initial) {
    uint16_t crc = initial;
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

bool EncodeV2(MessageType type, uint8_t flags, uint16_t sequence, const uint8_t* payload,
              uint16_t payload_length, std::vector<uint8_t>& output) {
    if (payload_length > kMaxPayloadLength || (payload_length > 0 && payload == nullptr)) {
        return false;
    }
    output.resize(11 + payload_length);
    output[0] = kMagic0;
    output[1] = kMagic1;
    output[2] = kVersion;
    output[3] = static_cast<uint8_t>(type);
    output[4] = flags;
    output[5] = static_cast<uint8_t>(sequence & 0xff);
    output[6] = static_cast<uint8_t>(sequence >> 8);
    output[7] = static_cast<uint8_t>(payload_length & 0xff);
    output[8] = static_cast<uint8_t>(payload_length >> 8);
    if (payload_length > 0)
        std::memcpy(output.data() + 9, payload, payload_length);
    const uint16_t crc = Crc16Ccitt(output.data() + 2, 7 + payload_length);
    output[9 + payload_length] = static_cast<uint8_t>(crc & 0xff);
    output[10 + payload_length] = static_cast<uint8_t>(crc >> 8);
    return true;
}

Parser::Parser() {
    std::memset(&stats_, 0, sizeof(stats_));
    ResetFrame();
}

void Parser::ResetFrame() {
    state_ = State::Seek;
    std::memset(&working_, 0, sizeof(working_));
    header_index_ = 0;
    payload_index_ = 0;
    received_crc_ = 0;
    last_byte_ms_ = 0;
    active_ = false;
}

void Parser::StartV2(uint32_t now_ms) {
    ResetFrame();
    state_ = State::V2Header;
    working_.format = Format::V2;
    last_byte_ms_ = now_ms;
    active_ = true;
}

void Parser::StartLegacy(uint32_t now_ms) {
    ResetFrame();
    state_ = State::LegacyPayload;
    working_.format = Format::Legacy;
    working_.message_type = MessageType::Response;
    last_byte_ms_ = now_ms;
    active_ = true;
}

void Parser::CheckTimeout(uint32_t now_ms) {
    if (active_ && now_ms - last_byte_ms_ > kFrameTimeoutMs) {
        ++stats_.timeouts;
        ResetFrame();
    }
}

void Parser::PollTimeout(uint32_t now_ms) { CheckTimeout(now_ms); }

bool Parser::Feed(uint8_t byte, uint32_t now_ms, Frame& frame) {
    CheckTimeout(now_ms);
    if ((state_ == State::Seek || state_ == State::SeekMagic1) && byte == '$') {
        StartLegacy(now_ms);
        return false;
    }
    last_byte_ms_ = now_ms;

    switch (state_) {
        case State::Seek:
            if (byte == kMagic0) {
                state_ = State::SeekMagic1;
                active_ = true;
            }
            return false;
        case State::SeekMagic1:
            if (byte == kMagic1) {
                StartV2(now_ms);
            } else if (byte == kMagic0) {
                state_ = State::SeekMagic1;
            } else {
                ResetFrame();
            }
            return false;
        case State::LegacyPayload:
            if (byte == '$') {
                StartLegacy(now_ms);
                return false;
            }
            if (byte == '\r' || byte == '\n') {
                if (payload_index_ == 0) {
                    ResetFrame();
                    return false;
                }
                working_.payload_length = payload_index_;
                working_.payload[payload_index_] = 0;
                frame = working_;
                ++stats_.rx_frames;
                ++stats_.legacy_frames;
                ResetFrame();
                return true;
            }
            if (payload_index_ >= kMaxPayloadLength) {
                ++stats_.overflow_errors;
                ResetFrame();
                return false;
            }
            working_.payload[payload_index_++] = byte;
            return false;
        case State::V2Header:
            header_[header_index_++] = byte;
            if (header_index_ < sizeof(header_))
                return false;
            if (header_[0] != kVersion) {
                ResetFrame();
                return false;
            }
            working_.message_type = static_cast<MessageType>(header_[1]);
            working_.flags = header_[2];
            working_.sequence =
                static_cast<uint16_t>(header_[3]) | (static_cast<uint16_t>(header_[4]) << 8);
            working_.payload_length =
                static_cast<uint16_t>(header_[5]) | (static_cast<uint16_t>(header_[6]) << 8);
            if (working_.payload_length > kMaxPayloadLength) {
                ++stats_.length_errors;
                ResetFrame();
                return false;
            }
            state_ = working_.payload_length == 0 ? State::V2CrcLow : State::V2Payload;
            return false;
        case State::V2Payload:
            working_.payload[payload_index_++] = byte;
            if (payload_index_ == working_.payload_length)
                state_ = State::V2CrcLow;
            return false;
        case State::V2CrcLow:
            received_crc_ = byte;
            state_ = State::V2CrcHigh;
            return false;
        case State::V2CrcHigh: {
            received_crc_ |= static_cast<uint16_t>(byte) << 8;
            uint16_t expected = Crc16Ccitt(header_, sizeof(header_));
            expected = Crc16Ccitt(working_.payload, working_.payload_length, expected);
            if (received_crc_ != expected) {
                ++stats_.crc_errors;
                ResetFrame();
                return false;
            }
            working_.payload[working_.payload_length] = 0;
            frame = working_;
            ++stats_.rx_frames;
            ResetFrame();
            return true;
        }
    }
    ResetFrame();
    return false;
}

}  // namespace nodehexa_uart
