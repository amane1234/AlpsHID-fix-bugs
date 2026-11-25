// libFuzzer target: the input report path of AlpsHIDEventDriver::handleInterruptReport.
//
// Input layout: byte 0 selects the device family (bit 0: T4) and the T4 sensor configuration (bits 1-7
// are fed to t4Geometry as CONFIG_3), byte 1 is the HID report ID, the rest is the report as it would
// arrive from the transport (any length). The report is copied with the same length rule as the kext
// (copyReport: never more than the frame needs, rejected when shorter) and every contact slot is decoded.

#include "AlpsProtocol.hpp"

#include <cstdlib>
#include <cstring>

#define REQUIRE(cond) do { if (!(cond)) std::abort(); } while (0)

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
    if (size < 2)
        return 0;
    const bool isT4 = data[0] & 1;
    const uint8_t config3 = static_cast<uint8_t>(data[0] >> 1 | (data[0] & 0x80));
    const uint32_t reportId = data[1];
    const uint8_t* report = data + 1;     // the report ID is the first report byte, as on the wire
    const size_t available = size - 1;

    unsigned needed = 0;
    const auto kind = alps::classifyInput(isT4, reportId, needed);
    if (kind == alps::ReportKind::None) {
        REQUIRE(needed == 0);
        return 0;
    }

    union {
        t4_input_report t4;
        u1_input_report pad;
        u1_sp_input_report stick;
    } frame = {};
    REQUIRE(needed <= sizeof(frame));
    const size_t copied = available < needed ? available : needed;
    // The exact-size heap copy lets ASan catch any read past what the transport delivered
    uint8_t* exact = static_cast<uint8_t*>(std::malloc(copied ? copied : 1));
    std::memcpy(exact, report, copied);
    std::memcpy(&frame, exact, copied);
    std::free(exact);
    if (copied != needed)
        return 0;

    switch (kind) {
        case alps::ReportKind::T4Touch: {
            alps::Geometry g;
            alps::t4Geometry(true, config3, g);
            REQUIRE(g.yMin <= g.yMax);
            unsigned active = 0;
            for (unsigned i = 0; i < MAX_TOUCHES; ++i) {
                uint32_t x = 0, y = 0;
                if (alps::t4DecodeContact(frame.t4.contact[i], g.yMin, g.yMax, x, y)) {
                    REQUIRE(x < 0xffff);
                    REQUIRE(y >= g.yMin && y <= g.yMax);
                    ++active;
                }
            }
            REQUIRE(active <= MAX_TOUCHES);
            break;
        }
        case alps::ReportKind::U1Touch: {
            for (int clickpad = 0; clickpad < 2; ++clickpad) {
                const uint8_t pad = alps::u1PadButtons(frame.pad.buttons, clickpad);
                REQUIRE(pad <= 7);
                if (clickpad)
                    REQUIRE(!(pad & 1));
                if (!clickpad)
                    REQUIRE(!alps::u1Click(frame.pad.buttons, clickpad));
            }
            for (unsigned i = 0; i < MAX_TOUCHES; ++i) {
                uint32_t x = 0, y = 0;
                if (alps::u1DecodeContact(frame.pad.contact[i], x, y))
                    REQUIRE(x <= 0xffff && y <= 0xffff);
            }
            break;
        }
        case alps::ReportKind::U1Stick: {
            int32_t dx = 0, dy = 0;
            alps::stickDelta(frame.stick, dx, dy);
            REQUIRE(dx >= -4096 && dx <= 4095);
            REQUIRE(dy >= -4096 && dy <= 4095);
            REQUIRE(alps::stickButtons(frame.stick) <= 7);
            break;
        }
        case alps::ReportKind::None:
            break;
    }
    return 0;
}
