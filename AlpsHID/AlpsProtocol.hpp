/* ALPS T4/U1 HID protocol: report layouts, register packets and decoding.
 *
 * Everything here is plain data processing with no IOKit dependency, so the same code runs in the kext
 * and in the host unit tests and fuzzers under Tests/. Keep it that way: no kernel calls, no allocation,
 * every input is treated as untrusted device data. Adapted from the Linux hid-alps driver. */

#ifndef AlpsProtocol_hpp
#define AlpsProtocol_hpp

#include <stdint.h>
#include <stddef.h>

#define T4_INPUT_REPORT_LEN sizeof(struct t4_input_report)
#define T4_FEATURE_REPORT_LEN T4_INPUT_REPORT_LEN
#define T4_FEATURE_REPORT_ID 0x07
#define T4_CMD_REGISTER_READ 0x08
#define T4_CMD_REGISTER_WRITE 0x07
#define T4_INPUT_REPORT_ID 0x09

#define T4_ADDRESS_BASE 0xC2C0
#define PRM_SYS_CONFIG_1 (T4_ADDRESS_BASE + 0x0002)
#define T4_PRM_FEED_CONFIG_1 (T4_ADDRESS_BASE + 0x0004)
#define T4_PRM_FEED_CONFIG_4 (T4_ADDRESS_BASE + 0x001A)
#define T4_PRM_ID_CONFIG_3 (T4_ADDRESS_BASE + 0x00B0)
#define T4_GPIO_PINS (T4_ADDRESS_BASE + 0x000C)

#define T4_FEEDCFG4_ADVANCED_ABS_ENABLE 0x01
#define T4_I2C_ABS 0x78
#define T4_COUNT_PER_ELECTRODE 256

/* Number of touches the driver reports to VoodooInput */
#define MAX_TOUCHES 5

/* Number of contact slots in the T4/U1 absolute reports as they come over the wire.
 * This is fixed by the hardware report layout (see the packed structs below) and is deliberately NOT
 * tied to MAX_TOUCHES. Code indexing contact[] must stay below it, e.g. with the max_fingers the device reports. */
#define ALPS_REPORT_MAX_CONTACTS 5

#define U1_MOUSE_REPORT_ID 0x01 /* Mouse data ReportID */
#define U1_ABSOLUTE_REPORT_ID 0x03 /* Absolute data ReportID */
#define U1_ABSOLUTE_REPORT_ID_SECD 0x02 /* FW-PTP Absolute data ReportID */
#define U1_FEATURE_REPORT_ID 0x05 /* Feature ReportID */
#define U1_SP_ABSOLUTE_REPORT_ID 0x06 /* SP (trackstick) absolute data ReportID */

#define U1_FEATURE_REPORT_LEN 0x08 /* Feature Report Length */
#define U1_FEATURE_REPORT_LEN_ALL 0x0A
#define U1_CMD_REGISTER_READ 0xD1
#define U1_CMD_REGISTER_WRITE 0xD2

#define U1_DEVTYPE_SP_SUPPORT 0x10 /* SP Support */
#define U1_DISABLE_DEV 0x01
#define U1_TP_ABS_MODE 0x02
#define U1_SP_ABS_MODE 0x80

#define ADDRESS_U1_DEV_CTRL_1 0x00800040
#define ADDRESS_U1_DEVICE_TYP 0x00800043
#define ADDRESS_U1_NUM_SENS_X 0x00800047
#define ADDRESS_U1_NUM_SENS_Y 0x00800048
#define ADDRESS_U1_PITCH_SENS_X 0x00800049
#define ADDRESS_U1_PITCH_SENS_Y 0x0080004A
#define ADDRESS_U1_RESO_DWN_ABS 0x0080004E
#define ADDRESS_U1_PAD_BTN 0x00800052
#define ADDRESS_U1_SP_BTN 0x0080009F

static_assert(MAX_TOUCHES <= ALPS_REPORT_MAX_CONTACTS, "MAX_TOUCHES cannot exceed the contact slots of the device reports");

struct __attribute__((__packed__)) t4_contact_data {
    uint8_t palm;
    uint8_t x_lo;
    uint8_t x_hi;
    uint8_t y_lo;
    uint8_t y_hi;
};

struct __attribute__((__packed__)) t4_input_report {
    uint8_t reportID;
    uint8_t numContacts;
    struct t4_contact_data contact[ALPS_REPORT_MAX_CONTACTS];
    uint8_t button;
    uint8_t track[ALPS_REPORT_MAX_CONTACTS];
    uint8_t zx[ALPS_REPORT_MAX_CONTACTS], zy[ALPS_REPORT_MAX_CONTACTS];
    uint8_t palmTime[ALPS_REPORT_MAX_CONTACTS];
    uint8_t kilroy;
    uint16_t timeStamp;
};

struct __attribute__((__packed__)) u1_contact_data {
    uint8_t x_lo;
    uint8_t x_hi;
    uint8_t y_lo;
    uint8_t y_hi;
    uint8_t z;
};

struct __attribute__((__packed__)) u1_input_report {
    uint8_t reportID;
    uint8_t buttons;
    uint8_t numContacts;
    struct u1_contact_data contact[ALPS_REPORT_MAX_CONTACTS];
};

struct __attribute__((__packed__)) u1_sp_input_report {
    uint8_t reportID;
    uint8_t buttons;
    int16_t x;
    int16_t y;
    uint16_t z;
};

/* These structs are overlaid on raw device reports, so their size is part of the hardware protocol.
 * A changed packing/field type would silently shift every field after it; fail the build instead. */
static_assert(sizeof(struct t4_contact_data) == 5, "t4_contact_data must be 5 bytes");
static_assert(sizeof(struct t4_input_report) == 51, "t4_input_report must be 51 bytes");
static_assert(sizeof(struct u1_contact_data) == 5, "u1_contact_data must be 5 bytes");
static_assert(sizeof(struct u1_input_report) == 28, "u1_input_report must be 28 bytes");
static_assert(sizeof(struct u1_sp_input_report) == 8, "u1_sp_input_report must be 8 bytes");

namespace alps {

inline uint16_t le16(const void* p) {
    const uint8_t* b = static_cast<const uint8_t*>(p);
    return static_cast<uint16_t>(b[0] | (b[1] << 8));
}

inline uint32_t le32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
        (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

inline void putLe32(uint32_t value, uint8_t* p) {
    p[0] = static_cast<uint8_t>(value);
    p[1] = static_cast<uint8_t>(value >> 8);
    p[2] = static_cast<uint8_t>(value >> 16);
    p[3] = static_cast<uint8_t>(value >> 24);
}

/* T4 register packet checksum (two running sums, as in hid-alps). Register packets cover seven or eight
 * bytes; at eight 0xff bytes the second accumulator stays below 12000, so no intermediate fold is needed.
 * Longer inputs are rejected with 0 like the original, which only accepted offset + length < 50. */
inline uint16_t t4Checksum(const uint8_t* bytes, unsigned length) {
    if (length > 20)
        return 0;
    uint16_t a = 255, b = 255;
    for (unsigned i = 0; i < length; ++i) {
        a = static_cast<uint16_t>(a + bytes[i]);
        b = static_cast<uint16_t>(b + a);
    }
    a = static_cast<uint16_t>((a & 255) + (a >> 8));
    b = static_cast<uint16_t>((b & 255) + (b >> 8));
    a = static_cast<uint16_t>((a & 255) + (a >> 8));
    b = static_cast<uint16_t>((b & 255) + (b >> 8));
    return static_cast<uint16_t>((b << 8) | a);
}

/* Input reports the driver decodes, and how many bytes each needs */
enum class ReportKind { None, T4Touch, U1Touch, U1Stick };

inline ReportKind classifyInput(bool isT4, uint32_t reportId, unsigned& size) {
    if (isT4 && reportId == T4_INPUT_REPORT_ID) {
        size = sizeof(t4_input_report);
        return ReportKind::T4Touch;
    }
    if (!isT4 && (reportId == U1_ABSOLUTE_REPORT_ID || reportId == U1_ABSOLUTE_REPORT_ID_SECD)) {
        size = sizeof(u1_input_report);
        return ReportKind::U1Touch;
    }
    if (!isT4 && reportId == U1_SP_ABSOLUTE_REPORT_ID) {
        // z is not used, a report may end before it
        size = offsetof(u1_sp_input_report, z);
        return ReportKind::U1Stick;
    }
    size = 0;
    return ReportKind::None;
}

/* One T4 contact slot. Returns false for an empty or palm slot. y is clamped to [yMin, yMax] and inverted
 * within that range, which needs yMin <= yMax (the probed geometry guarantees it). */
inline bool t4DecodeContact(const t4_contact_data& slot, uint32_t yMin, uint32_t yMax, uint32_t& x, uint32_t& y) {
    x = static_cast<uint32_t>((slot.x_hi << 8) | slot.x_lo);
    // 0xffff denotes an empty contact in the T4 report format
    if (!slot.palm || slot.palm >= 0x80 || x == 0xffff)
        return false;
    y = static_cast<uint32_t>((slot.y_hi << 8) | slot.y_lo);
    y = y < yMin ? yMin : (y > yMax ? yMax : y);
    y = yMax + yMin - y;
    return true;
}

/* One U1 contact slot. Returns false when the slot has no pressure. */
inline bool u1DecodeContact(const u1_contact_data& slot, uint32_t& x, uint32_t& y) {
    if (!(slot.z & 0x7f))
        return false;
    x = static_cast<uint32_t>((slot.x_hi << 8) | slot.x_lo);
    y = static_cast<uint32_t>((slot.y_hi << 8) | slot.y_lo);
    return true;
}

/* U1 pad buttons sent on the relative-pointer path. A one-button pad clicks through the multitouch path
 * (bit 0), so only the other bits are kept for it. */
inline uint8_t u1PadButtons(uint8_t buttons, bool clickpad) {
    return static_cast<uint8_t>(buttons & (clickpad ? 6 : 7));
}

inline bool u1Click(uint8_t buttons, bool clickpad) {
    return clickpad && (buttons & 1);
}

/* Trackstick report: signed little-endian deltas in eighths, as in hid-alps */
inline void stickDelta(const u1_sp_input_report& report, int32_t& dx, int32_t& dy) {
    dx = static_cast<int16_t>(le16(&report.x)) / 8;
    dy = static_cast<int16_t>(le16(&report.y)) / 8;
}

inline uint8_t stickButtons(const u1_sp_input_report& report) {
    return static_cast<uint8_t>(report.buttons & 7);
}

/* Register request packets */
inline void buildU1Request(uint32_t address, bool read, uint8_t value, uint8_t (&out)[U1_FEATURE_REPORT_LEN]) {
    for (auto& byte : out)
        byte = 0;
    out[0] = U1_FEATURE_REPORT_ID;
    out[1] = read ? U1_CMD_REGISTER_READ : U1_CMD_REGISTER_WRITE;
    putLe32(address, out + 2);
    out[6] = read ? 0 : value;
    uint8_t sum = U1_FEATURE_REPORT_LEN_ALL;
    for (unsigned i = 0; i < U1_FEATURE_REPORT_LEN - 1; ++i)
        sum = static_cast<uint8_t>(sum + out[i]);
    out[7] = sum;
}

inline void buildT4Request(uint32_t address, bool read, uint8_t value, uint8_t (&out)[T4_FEATURE_REPORT_LEN]) {
    for (auto& byte : out)
        byte = 0;
    out[0] = T4_FEATURE_REPORT_ID;
    out[1] = read ? T4_CMD_REGISTER_READ : T4_CMD_REGISTER_WRITE;
    putLe32(address, out + 2);
    out[6] = 1;
    out[7] = 0;
    out[8] = read ? 0 : value;
    const uint16_t sum = t4Checksum(out + 1, 8);
    out[9] = static_cast<uint8_t>(sum);
    out[10] = static_cast<uint8_t>(sum >> 8);
    out[11] = 0;
}

/* Bytes of a feature reply the driver validates: T4 replies carry address, size, value and checksum. */
static constexpr unsigned T4ReplyLength = 15;
static constexpr unsigned U1ReplyLength = U1_FEATURE_REPORT_LEN;

inline unsigned replyLength(uint8_t reportId) {
    return reportId == T4_FEATURE_REPORT_ID ? T4ReplyLength : U1ReplyLength;
}

/* Validate a T4 register read reply of at least T4ReplyLength bytes */
inline bool parseT4Reply(const uint8_t* reply, uint32_t address, uint8_t& value) {
    if (le32(reply + 6) != address || le16(reply + 10) != 1 || le16(reply + 13) != t4Checksum(reply + 6, 7))
        return false;
    value = reply[12];
    return true;
}

/* Reply coverage for transports that fill the descriptor with writeBytes() but leave its length at
 * capacity. Bit 31 marks that writeBytes() ran at all; bits 0..14 mark the protocol prefix bytes written. */
static constexpr uint32_t CoverageWritten = 1U << 31;
static constexpr unsigned CoverageBytes = 15;

inline uint32_t coverageMask(uint64_t offset, uint64_t written) {
    uint32_t mask = CoverageWritten;
    if (offset < CoverageBytes) {
        const uint64_t limit = CoverageBytes - offset;
        const unsigned count = static_cast<unsigned>(written < limit ? written : limit);
        mask |= ((1U << count) - 1) << offset;
    }
    return mask;
}

/* Whether a reply covers its first length bytes. Direct-mapped transports never call writeBytes() (bits
 * stays 0), then only the descriptor length counts. */
inline bool covers(uint32_t bits, uint64_t descriptorLength, unsigned length) {
    if (length > CoverageBytes || descriptorLength < length)
        return false;
    const uint32_t prefix = (1U << length) - 1;
    return !bits || (bits & prefix) == prefix;
}

/* Sensor geometry */
struct Geometry {
    uint32_t xMin, xMax, yMin, yMax;
    uint32_t xActiveLenMm, yActiveLenMm;
};

/* T4: the button-less variant encodes sensor-line offsets as signed nibbles in T4_PRM_ID_CONFIG_3 */
inline void t4Geometry(bool buttonless, uint8_t config3, Geometry& g) {
    uint8_t lines_x = 20, lines_y = 12;
    if (buttonless) {
        lines_x = static_cast<uint8_t>(16 + ((config3 & 0x0F) | (config3 & 0x08 ? 0xF0 : 0)));
        lines_y = static_cast<uint8_t>(12 + (((config3 & 0xF0) >> 4) | (config3 & 0x80 ? 0xF0 : 0)));
    }
    g.xMax = lines_x * T4_COUNT_PER_ELECTRODE;
    g.xMin = T4_COUNT_PER_ELECTRODE;
    g.yMax = lines_y * T4_COUNT_PER_ELECTRODE;
    g.yMin = T4_COUNT_PER_ELECTRODE;
    g.xActiveLenMm = g.yActiveLenMm = 0;
}

/* U1: returns false for geometry the coordinate ranges cannot be derived from */
inline bool u1Geometry(uint8_t lines_x, uint8_t lines_y, uint8_t pitch_x, uint8_t pitch_y, uint8_t resolution, Geometry& g) {
    if (lines_x < 2 || lines_y < 2 || !pitch_x || !pitch_y || !resolution)
        return false;
    g.xActiveLenMm = pitch_x * (lines_x - 1U);
    g.yActiveLenMm = pitch_y * (lines_y - 1U);
    g.xMax = (resolution << 2) * (lines_x - 1U);
    g.xMin = 1;
    g.yMax = (resolution << 2) * (lines_y - 1U);
    g.yMin = 1;
    return true;
}

/* U1 pad button count from ADDRESS_U1_PAD_BTN: both nibbles must agree, else assume a clickpad */
inline uint32_t u1ButtonCount(uint8_t padButtons) {
    return (padButtons & 0x0F) == ((padButtons & 0xF0) >> 4) ? (padButtons & 0x0FU) : 1U;
}

} // namespace alps

#endif /* AlpsProtocol_hpp */
