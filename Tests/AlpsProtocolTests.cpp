// Unit tests for AlpsHID/AlpsProtocol.hpp. Build and run with `make test` in this directory.
//
// Where the kext logic was moved into AlpsProtocol.hpp, the original implementation (AlpsHID 36d1fce) is
// kept below as a reference and the new code is compared against it, exhaustively where the input space
// is small and with a fixed pseudo-random sequence otherwise.

#include "AlpsProtocol.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

static int failures = 0;
static int checks = 0;

#define CHECK(cond) do { \
    ++checks; \
    if (!(cond)) { \
        ++failures; \
        std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define CHECK_EQ(a, b) do { \
    ++checks; \
    const auto va = (a); const auto vb = (b); \
    if (!(va == vb)) { \
        ++failures; \
        std::fprintf(stderr, "%s:%d: CHECK_EQ failed: %s == %s (%lld vs %lld)\n", __FILE__, __LINE__, #a, #b, \
                     static_cast<long long>(va), static_cast<long long>(vb)); \
    } \
} while (0)

// Deterministic generator so failures reproduce
static uint32_t rng_state = 0x12345678u;
static uint32_t rnd() {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

namespace reference {

// AlpsHIDEventDriver::t4_calc_check_sum at 36d1fce
static uint16_t t4_calc_check_sum(const uint8_t* buffer, unsigned long offset, unsigned long length) {
    uint16_t sum1 = 0xFF, sum2 = 0xFF;
    unsigned long i = 0;
    if (offset + length >= 50)
        return 0;
    while (length > 0) {
        unsigned long tlen = length > 20 ? 20 : length;
        length -= tlen;
        do {
            sum1 += buffer[offset + i];
            sum2 += sum1;
            i++;
        } while (--tlen > 0);
        sum1 = (sum1 & 0xFF) + (sum1 >> 8);
        sum2 = (sum2 & 0xFF) + (sum2 >> 8);
    }
    sum1 = (sum1 & 0xFF) + (sum1 >> 8);
    sum2 = (sum2 & 0xFF) + (sum2 >> 8);
    return (sum2 << 8 | sum1);
}

// t4_raw_event contact test and Y inversion at 36d1fce
static bool t4_contact(const t4_contact_data& c, uint32_t y_min, uint32_t y_max, uint32_t& x, uint32_t& y) {
    x = c.x_hi << 8 | c.x_lo;
    y = c.y_hi << 8 | c.y_lo;
    bool valid = c.palm < 0x80 && c.palm > 0 && x != 0xffff;
    if (!valid)
        return false;
    if (y < y_min)
        y = y_min;
    else if (y > y_max)
        y = y_max;
    y = y_max + y_min - y;
    return true;
}

// u1_read_write_register request at 36d1fce
static void u1_request(uint32_t address, bool read_flag, uint8_t write_val, uint8_t (&input)[U1_FEATURE_REPORT_LEN]) {
    std::memset(input, 0, sizeof(input));
    input[0] = U1_FEATURE_REPORT_ID;
    if (read_flag) {
        input[1] = U1_CMD_REGISTER_READ;
        input[6] = 0x00;
    } else {
        input[1] = U1_CMD_REGISTER_WRITE;
        input[6] = write_val;
    }
    input[2] = address; input[3] = address >> 8; input[4] = address >> 16; input[5] = address >> 24;
    uint8_t check_sum = U1_FEATURE_REPORT_LEN_ALL;
    for (int i = 0; i < U1_FEATURE_REPORT_LEN - 1; ++i)
        check_sum += input[i];
    input[7] = check_sum;
}

// t4_read_write_register request at 36d1fce
static void t4_request(uint32_t address, bool read_flag, uint8_t write_val, uint8_t (&input)[T4_FEATURE_REPORT_LEN]) {
    std::memset(input, 0, sizeof(input));
    input[0] = T4_FEATURE_REPORT_ID;
    input[1] = read_flag ? T4_CMD_REGISTER_READ : T4_CMD_REGISTER_WRITE;
    input[8] = read_flag ? 0x00 : write_val;
    input[2] = address; input[3] = address >> 8; input[4] = address >> 16; input[5] = address >> 24;
    input[6] = 1;
    input[7] = 0;
    uint16_t check_sum = t4_calc_check_sum(input, 1, 8);
    input[9] = (uint8_t)check_sum;
    input[10] = (uint8_t)(check_sum >> 8);
    input[11] = 0;
}

// t4_read_write_register reply checks at 36d1fce
static bool t4_reply(const uint8_t* readbuf, uint32_t address, uint8_t& value) {
    uint32_t responseAddress = (uint32_t)readbuf[6] | ((uint32_t)readbuf[7] << 8) |
        ((uint32_t)readbuf[8] << 16) | ((uint32_t)readbuf[9] << 24);
    if (responseAddress != address)
        return false;
    if ((uint16_t)(readbuf[10] | readbuf[11] << 8) != 1)
        return false;
    if ((uint16_t)(readbuf[13] | readbuf[14] << 8) != t4_calc_check_sum(readbuf, 6, 7))
        return false;
    value = readbuf[12];
    return true;
}

} // namespace reference

static void test_checksum_matches_reference() {
    uint8_t buf[50];
    for (int round = 0; round < 20000; ++round) {
        for (auto& b : buf)
            b = static_cast<uint8_t>(rnd());
        const unsigned len = rnd() % 21;      // 0..20: one fold block in the original
        const unsigned off = rnd() % (50 - len);
        if (off + len >= 50)
            continue;
        CHECK_EQ(alps::t4Checksum(buf + off, len), reference::t4_calc_check_sum(buf, off, len));
    }
    // Worst case for the accumulators: all 0xff
    uint8_t ff[20];
    std::memset(ff, 0xff, sizeof(ff));
    for (unsigned len = 0; len <= 20; ++len)
        CHECK_EQ(alps::t4Checksum(ff, len), reference::t4_calc_check_sum(ff, 0, len));
    // Over-long input is rejected instead of overflowing
    uint8_t big[64] = {};
    CHECK_EQ(alps::t4Checksum(big, 21), 0);
}

static void test_t4_contact_matches_reference() {
    const uint32_t ranges[][2] = { {256, 3072}, {256, 2048}, {256, 6144}, {0, 0}, {256, 256} };
    for (auto& r : ranges) {
        for (int round = 0; round < 20000; ++round) {
            t4_contact_data c;
            c.palm = static_cast<uint8_t>(rnd());
            c.x_lo = static_cast<uint8_t>(rnd());
            c.x_hi = static_cast<uint8_t>(rnd());
            c.y_lo = static_cast<uint8_t>(rnd());
            c.y_hi = static_cast<uint8_t>(rnd());
            if (round % 7 == 0)
                c.x_lo = c.x_hi = 0xff;    // empty slot marker
            uint32_t x1 = 0, y1 = 0, x2 = 0, y2 = 0;
            const bool a = alps::t4DecodeContact(c, r[0], r[1], x1, y1);
            const bool b = reference::t4_contact(c, r[0], r[1], x2, y2);
            CHECK_EQ(a, b);
            if (a && b) {
                CHECK_EQ(x1, x2);
                CHECK_EQ(y1, y2);
                CHECK(y1 >= r[0] && y1 <= r[1]);
            }
        }
    }
    // Explicit cases
    t4_contact_data c = {0x10, 0x34, 0x12, 0x00, 0x01};
    uint32_t x, y;
    CHECK(alps::t4DecodeContact(c, 256, 3072, x, y));
    CHECK_EQ(x, 0x1234u);
    CHECK_EQ(y, 3072u + 256u - 256u);     // y = 0x100 is the minimum, inverted to the maximum
    c.palm = 0;
    CHECK(!alps::t4DecodeContact(c, 256, 3072, x, y));
    c.palm = 0x80;
    CHECK(!alps::t4DecodeContact(c, 256, 3072, x, y));
    c = {0x10, 0xff, 0xff, 0x00, 0x05};
    CHECK(!alps::t4DecodeContact(c, 256, 3072, x, y));
}

static void test_u1_contact() {
    for (int z = 0; z < 256; ++z) {
        u1_contact_data c = {0x34, 0x12, 0x78, 0x56, static_cast<uint8_t>(z)};
        uint32_t x = 0, y = 0;
        const bool active = alps::u1DecodeContact(c, x, y);
        CHECK_EQ(active, (z & 0x7f) != 0);
        if (active) {
            CHECK_EQ(x, 0x1234u);
            CHECK_EQ(y, 0x5678u);
        }
    }
}

static void test_u1_buttons() {
    for (int b = 0; b < 256; ++b) {
        const uint8_t buttons = static_cast<uint8_t>(b);
        // 36d1fce: pad_buttons = buttons & (u1_button_count == 1 ? 0x06 : 0x07), click = count == 1 && (buttons & 1)
        CHECK_EQ(alps::u1PadButtons(buttons, true), buttons & 0x06);
        CHECK_EQ(alps::u1PadButtons(buttons, false), buttons & 0x07);
        CHECK_EQ(alps::u1Click(buttons, true), (buttons & 1) != 0);
        CHECK_EQ(alps::u1Click(buttons, false), false);
        // 36d1fce: btn_cnt = (tmp & 0x0F) == ((tmp & 0xF0) >> 4) ? tmp & 0x0F : 1
        const uint32_t expected = (buttons & 0x0F) == ((buttons & 0xF0) >> 4) ? (buttons & 0x0Fu) : 1u;
        CHECK_EQ(alps::u1ButtonCount(buttons), expected);
    }
}

static void test_stick() {
    for (int round = 0; round < 20000; ++round) {
        uint8_t raw[sizeof(u1_sp_input_report)];
        for (auto& b : raw)
            b = static_cast<uint8_t>(rnd());
        u1_sp_input_report r;
        std::memcpy(&r, raw, sizeof(r));
        int32_t dx, dy;
        alps::stickDelta(r, dx, dy);
        CHECK_EQ(dx, static_cast<int16_t>(raw[2] | raw[3] << 8) / 8);
        CHECK_EQ(dy, static_cast<int16_t>(raw[4] | raw[5] << 8) / 8);
        CHECK(dx >= -4096 && dx <= 4095);
        CHECK(dy >= -4096 && dy <= 4095);
        CHECK_EQ(alps::stickButtons(r), raw[1] & 7);
    }
}

static void test_classify() {
    unsigned size = 1234;
    CHECK(alps::classifyInput(true, T4_INPUT_REPORT_ID, size) == alps::ReportKind::T4Touch);
    CHECK_EQ(size, sizeof(t4_input_report));
    CHECK(alps::classifyInput(false, U1_ABSOLUTE_REPORT_ID, size) == alps::ReportKind::U1Touch);
    CHECK_EQ(size, sizeof(u1_input_report));
    CHECK(alps::classifyInput(false, U1_ABSOLUTE_REPORT_ID_SECD, size) == alps::ReportKind::U1Touch);
    CHECK(alps::classifyInput(false, U1_SP_ABSOLUTE_REPORT_ID, size) == alps::ReportKind::U1Stick);
    CHECK_EQ(size, 6u);
    for (uint32_t id = 0; id < 0x200; ++id) {
        for (int t4 = 0; t4 < 2; ++t4) {
            const auto kind = alps::classifyInput(t4, id, size);
            if (kind == alps::ReportKind::None) {
                CHECK_EQ(size, 0u);
            } else {
                CHECK(size > 0 && size <= sizeof(t4_input_report));
                // A report kind only exists for its device family
                CHECK(t4 ? kind == alps::ReportKind::T4Touch : kind != alps::ReportKind::T4Touch);
            }
        }
    }
}

static void test_requests_match_reference() {
    const uint32_t addresses[] = { ADDRESS_U1_DEV_CTRL_1, ADDRESS_U1_PAD_BTN, PRM_SYS_CONFIG_1, T4_PRM_ID_CONFIG_3,
                                   0, 0xffffffffu, 0x12345678u };
    for (uint32_t address : addresses) {
        for (int read = 0; read < 2; ++read) {
            for (int value = 0; value < 256; value += 17) {
                uint8_t a[U1_FEATURE_REPORT_LEN], b[U1_FEATURE_REPORT_LEN];
                alps::buildU1Request(address, read, static_cast<uint8_t>(value), a);
                reference::u1_request(address, read, static_cast<uint8_t>(value), b);
                CHECK(std::memcmp(a, b, sizeof(a)) == 0);

                uint8_t c[T4_FEATURE_REPORT_LEN], d[T4_FEATURE_REPORT_LEN];
                alps::buildT4Request(address, read, static_cast<uint8_t>(value), c);
                reference::t4_request(address, read, static_cast<uint8_t>(value), d);
                CHECK(std::memcmp(c, d, sizeof(c)) == 0);
            }
        }
    }
}

// Build a reply the way the touchpad answers a register read
static void make_t4_reply(uint8_t (&reply)[alps::T4ReplyLength], uint32_t address, uint8_t value) {
    std::memset(reply, 0, sizeof(reply));
    reply[0] = T4_FEATURE_REPORT_ID;
    alps::putLe32(address, reply + 6);
    reply[10] = 1;
    reply[11] = 0;
    reply[12] = value;
    const uint16_t sum = alps::t4Checksum(reply + 6, 7);
    reply[13] = static_cast<uint8_t>(sum);
    reply[14] = static_cast<uint8_t>(sum >> 8);
}

static void test_t4_reply() {
    uint8_t reply[alps::T4ReplyLength];
    for (int value = 0; value < 256; ++value) {
        make_t4_reply(reply, PRM_SYS_CONFIG_1, static_cast<uint8_t>(value));
        uint8_t got = 0;
        CHECK(alps::parseT4Reply(reply, PRM_SYS_CONFIG_1, got));
        CHECK_EQ(got, value);
        CHECK(!alps::parseT4Reply(reply, PRM_SYS_CONFIG_1 + 1, got));   // wrong register
    }
    // Any single corrupted byte of address, size, value or checksum must be rejected
    make_t4_reply(reply, T4_PRM_ID_CONFIG_3, 0x5a);
    for (unsigned i = 6; i < alps::T4ReplyLength; ++i) {
        for (int bit = 0; bit < 8; ++bit) {
            uint8_t bad[alps::T4ReplyLength];
            std::memcpy(bad, reply, sizeof(bad));
            bad[i] ^= static_cast<uint8_t>(1 << bit);
            uint8_t got = 0;
            CHECK(!alps::parseT4Reply(bad, T4_PRM_ID_CONFIG_3, got));
        }
    }
    // Random replies: the new and the original checks agree
    for (int round = 0; round < 50000; ++round) {
        uint8_t r[alps::T4ReplyLength];
        for (auto& b : r)
            b = static_cast<uint8_t>(rnd());
        if (round & 1) {   // make some of them structurally valid
            make_t4_reply(r, PRM_SYS_CONFIG_1, static_cast<uint8_t>(rnd()));
            r[rnd() % alps::T4ReplyLength] ^= static_cast<uint8_t>(rnd() & 1);
        }
        uint8_t a = 0, b = 0;
        const bool ok_a = alps::parseT4Reply(r, PRM_SYS_CONFIG_1, a);
        const bool ok_b = reference::t4_reply(r, PRM_SYS_CONFIG_1, b);
        CHECK_EQ(ok_a, ok_b);
        if (ok_a && ok_b)
            CHECK_EQ(a, b);
    }
}

static void test_coverage() {
    // Mask bits stay within the tracked prefix plus the written marker
    for (uint64_t offset = 0; offset < 40; ++offset) {
        for (uint64_t written = 0; written < 40; ++written) {
            const uint32_t mask = alps::coverageMask(offset, written);
            CHECK(mask & alps::CoverageWritten);
            CHECK((mask & ~alps::CoverageWritten & ~((1u << alps::CoverageBytes) - 1)) == 0);
            for (unsigned byte = 0; byte < alps::CoverageBytes; ++byte) {
                const bool expected = byte >= offset && byte < offset + written;
                CHECK_EQ(((mask >> byte) & 1) != 0, expected);
            }
        }
    }
    CHECK_EQ(alps::coverageMask(~0ull, ~0ull), alps::CoverageWritten);

    // Direct mapping (no writeBytes): only the descriptor length counts
    CHECK(alps::covers(0, 15, 15));
    CHECK(!alps::covers(0, 14, 15));
    // writeBytes() of the whole reply
    CHECK(alps::covers(alps::coverageMask(0, 51), 51, 15));
    // writeBytes() of only the first 8 bytes, descriptor length left at capacity
    CHECK(!alps::covers(alps::coverageMask(0, 8), 51, 15));
    CHECK(alps::covers(alps::coverageMask(0, 8), 51, 8));
    // Two partial writes that together cover the prefix
    CHECK(alps::covers(alps::coverageMask(0, 6) | alps::coverageMask(6, 9), 51, 15));
    // An empty write is not mistaken for direct mapping
    CHECK(!alps::covers(alps::coverageMask(0, 0), 51, 8));
    // Lengths beyond the tracked prefix cannot be covered
    CHECK(!alps::covers(0, 64, 16));
    CHECK(!alps::covers(0xffffffffu, 64, 32));
}

static void test_geometry() {
    // T4 button-less: every CONFIG_3 value, against the 36d1fce arithmetic (UInt8 wrap included)
    for (int v = 0; v < 256; ++v) {
        const uint8_t tmp = static_cast<uint8_t>(v);
        const uint8_t sx = static_cast<uint8_t>(16 + ((tmp & 0x0F) | (tmp & 0x08 ? 0xF0 : 0)));
        const uint8_t sy = static_cast<uint8_t>(12 + (((tmp & 0xF0) >> 4) | (tmp & 0x80 ? 0xF0 : 0)));
        alps::Geometry g;
        alps::t4Geometry(true, tmp, g);
        CHECK_EQ(g.xMax, sx * 256u);
        CHECK_EQ(g.yMax, sy * 256u);
        CHECK_EQ(g.xMin, 256u);
        CHECK_EQ(g.yMin, 256u);
        CHECK(g.yMin <= g.yMax);   // needed by the Y inversion in t4DecodeContact
        CHECK(g.xMin <= g.xMax);
    }
    alps::Geometry g;
    alps::t4Geometry(false, 0xAB, g);
    CHECK_EQ(g.xMax, 20u * 256u);
    CHECK_EQ(g.yMax, 12u * 256u);

    // U1: validity and ranges
    CHECK(!alps::u1Geometry(1, 10, 1, 1, 1, g));
    CHECK(!alps::u1Geometry(10, 1, 1, 1, 1, g));
    CHECK(!alps::u1Geometry(10, 10, 0, 1, 1, g));
    CHECK(!alps::u1Geometry(10, 10, 1, 0, 1, g));
    CHECK(!alps::u1Geometry(10, 10, 1, 1, 0, g));
    CHECK(alps::u1Geometry(21, 13, 52, 51, 25, g));
    CHECK_EQ(g.xActiveLenMm, 52u * 20u);
    CHECK_EQ(g.yActiveLenMm, 51u * 12u);
    CHECK_EQ(g.xMax, (25u << 2) * 20u);
    CHECK_EQ(g.yMax, (25u << 2) * 12u);
    CHECK_EQ(g.xMin, 1u);
    CHECK_EQ(g.yMin, 1u);
    // Largest inputs do not overflow 32 bits
    CHECK(alps::u1Geometry(255, 255, 255, 255, 255, g));
    CHECK_EQ(g.xMax, (255u << 2) * 254u);
}

int main() {
    test_checksum_matches_reference();
    test_t4_contact_matches_reference();
    test_u1_contact();
    test_u1_buttons();
    test_stick();
    test_classify();
    test_requests_match_reference();
    test_t4_reply();
    test_coverage();
    test_geometry();
    if (failures) {
        std::fprintf(stderr, "%d of %d checks failed\n", failures, checks);
        return 1;
    }
    std::printf("All %d checks passed\n", checks);
    return 0;
}
