// libFuzzer target: register replies and feature-buffer coverage (AlpsHIDEventDriver::exchangeFeature,
// t4_read_write_register, AlpsFeatureBuffer).
//
// Input layout: byte 0 picks the report ID (bit 0: T4) and bytes 1-4 the register address, then
// write records (offset, length) drive coverageMask() the way a transport would call writeBytes(), and
// the remaining bytes are the device reply. The reply is copied into an exact-size heap buffer, so
// ASan reports any read beyond the bytes the driver checked were present.

#include "AlpsProtocol.hpp"

#include <cstdlib>
#include <cstring>

#define REQUIRE(cond) do { if (!(cond)) std::abort(); } while (0)

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
    if (size < 6)
        return 0;
    const uint8_t id = (data[0] & 1) ? T4_FEATURE_REPORT_ID : U1_FEATURE_REPORT_ID;
    const uint32_t address = alps::le32(data + 1);
    const unsigned writes = data[5] % 4;
    size_t pos = 6;

    // Simulated transport: a few writeBytes() calls, then the descriptor length it reports
    uint32_t bits = 0;
    for (unsigned w = 0; w < writes && pos + 2 <= size; ++w, pos += 2) {
        const uint32_t mask = alps::coverageMask(data[pos], data[pos + 1]);
        REQUIRE(mask & alps::CoverageWritten);
        REQUIRE((mask & ~alps::CoverageWritten) < (1u << alps::CoverageBytes));
        bits |= mask;
    }
    if (pos >= size)
        return 0;
    const uint64_t descriptorLength = data[pos++];
    const uint8_t* reply = data + pos;
    const size_t replySize = size - pos;

    const unsigned needed = alps::replyLength(id);
    REQUIRE(needed <= alps::CoverageBytes);
    const bool covered = alps::covers(bits, descriptorLength, needed);
    if (covered)
        REQUIRE(descriptorLength >= needed);

    // exchangeFeature() copies `needed` bytes only after the coverage check, so model that order
    if (!covered || replySize < needed || reply[0] != id)
        return 0;
    uint8_t* exact = static_cast<uint8_t*>(std::malloc(needed));
    std::memcpy(exact, reply, needed);

    if (id == T4_FEATURE_REPORT_ID) {
        uint8_t value = 0xA5;
        if (alps::parseT4Reply(exact, address, value)) {
            // An accepted reply really names the register, size 1 and a matching checksum
            REQUIRE(alps::le32(exact + 6) == address);
            REQUIRE(alps::le16(exact + 10) == 1);
            REQUIRE(alps::le16(exact + 13) == alps::t4Checksum(exact + 6, 7));
            REQUIRE(value == exact[12]);
        } else {
            REQUIRE(value == 0xA5);    // untouched on rejection
        }
    }
    std::free(exact);

    // Requests built for the same register are well-formed and self-consistent
    uint8_t t4[T4_FEATURE_REPORT_LEN];
    alps::buildT4Request(address, data[0] & 2, data[0], t4);
    REQUIRE(t4[0] == T4_FEATURE_REPORT_ID);
    REQUIRE(alps::le32(t4 + 2) == address);
    REQUIRE(alps::le16(t4 + 9) == alps::t4Checksum(t4 + 1, 8));
    uint8_t u1[U1_FEATURE_REPORT_LEN];
    alps::buildU1Request(address, data[0] & 2, data[0], u1);
    uint8_t sum = U1_FEATURE_REPORT_LEN_ALL;
    for (unsigned i = 0; i < U1_FEATURE_REPORT_LEN - 1; ++i)
        sum = static_cast<uint8_t>(sum + u1[i]);
    REQUIRE(u1[7] == sum);
    return 0;
}
