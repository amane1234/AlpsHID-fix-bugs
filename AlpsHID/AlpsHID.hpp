/*This code is derived and adapted from VoodooI2CHID's Multitouch Event Driver and Precision
Touchpad Event Driver (https://github.com/alexandred/VoodooI2C) and the Linux kernel driver
for the alps t4 touchpad (https://github.com/torvalds/linux/blob/master/drivers/hid/hid-alps.c) */

#ifndef AlpsHIDEventDriver_hpp
#define AlpsHIDEventDriver_hpp

// Keeps Apple's IOHIDEventDriver.h out of the build (this driver supplies its own event handling)
#define _IOKIT_HID_IOHIDEVENTDRIVER_H

#include <IOKit/IOLib.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/IOService.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOLocks.h>
#include <IOKit/hid/IOHIDInterface.h>
#include <kern/clock.h>

#include <IOKit/hid/IOHIDEvent.h>
#include <IOKit/hid/IOHIDEventService.h>
#include <IOKit/hid/IOHIDEventTypes.h>
#include <IOKit/hidsystem/IOHIDTypes.h>
#include <IOKit/hid/IOHIDPrivateKeys.h>
#include <IOKit/hid/IOHIDUsageTables.h>
#include <IOKit/hid/IOHIDDevice.h>

#include "../VoodooInput/VoodooInput/VoodooInputMultitouch/VoodooInputTransducer.h"
#include "../VoodooInput/VoodooInput/VoodooInputMultitouch/VoodooInputMessages.h"
#include "../VoodooInput/VoodooInput/Trackpoint/TrackpointDevice.hpp"

#include "helpers.hpp"
#include "AlpsProtocol.hpp"

#define HID_PRODUCT_ID_T3_BTNLESS 0xD0C0
#define HID_PRODUCT_ID_COSMO 0x1202
#define HID_PRODUCT_ID_U1_DUAL 0x120B
#define HID_PRODUCT_ID_T4_BTNLESS 0x120C
#define HID_PRODUCT_ID_G1 0x120D
#define HID_PRODUCT_ID_U1 0x1209
#define HID_PRODUCT_ID_U1_UNICORN_LEGACY 0x121E
#define HID_PRODUCT_ID_U1_OLDMAN 0x1215
#define HID_PRODUCT_ID_T4_USB 0x1216
#define HID_PRODUCT_ID_T4_USB_2 0x1217
#define HID_PRODUCT_ID_T4_USB_3 0x120f
#define HID_PRODUCT_ID_T4_USB_4 0x1218
#define HID_PRODUCT_ID_U1_PTP_1 0x1207
#define HID_PRODUCT_ID_U1_PTP_2 0x120A
#define HID_PRODUCT_ID_U1_DUAL_PTP 0x121F
#define HID_PRODUCT_ID_U1_DUAL_3BTN_PTP 0x1220
#define HID_PRODUCT_ID_U1_1222 0x1222

#define ALPS_VENDOR 0x44e
#define CIRQUE_VENDOR 0x0488

// Message types defined by ApplePS2Keyboard
enum {
    // from keyboard to mouse/touchpad
    kKeyboardSetTouchStatus = iokit_vendor_specific_msg(100),   // set disable/enable touchpad (data is bool*)
    kKeyboardGetTouchStatus = iokit_vendor_specific_msg(101),   // get disable/enable touchpad (data is bool*)
    kKeyboardKeyPressTime = iokit_vendor_specific_msg(110)      // notify of timestamp a non-modifier key was pressed (data is uint64_t*)
};

enum dev_num {
    U1,
    T4,
};

struct alps_dev {
    UInt8 max_fingers;
    UInt8 has_sp;
    UInt8 sp_btn_info;
    UInt32 x_active_len_mm;
    UInt32 y_active_len_mm;
    UInt32 x_max;
    UInt32 y_max;
    UInt32 x_min;
    UInt32 y_min;
    UInt32 btn_cnt;
    UInt32 sp_btn_cnt;
};


class AlpsFeatureBuffer;

class AlpsHIDEventDriver : public IOHIDEventService {
    OSDeclareDefaultStructors(AlpsHIDEventDriver);

public:
    void handleInterruptReport(AbsoluteTime timestamp, IOMemoryDescriptor *report, IOHIDReportType report_type, UInt32 report_id);

    void free() override;
    bool start(IOService* provider) override;
    bool handleStart(IOService* provider) override;
    void handleStop(IOService* provider) override;
    bool handleOpen(IOService *forClient, IOOptionBits options, void *arg) override;
    void handleClose(IOService *forClient, IOOptionBits options) override;
    bool handleIsOpen(const IOService *forClient) const override;
    virtual IOReturn message(UInt32 type, IOService* provider, void* argument) override;
    bool didTerminate(IOService* provider, IOOptionBits options, bool* defer) override;
    IOReturn setPowerState(unsigned long whichState, IOService* whatDevice) override;

    IOReturn publishMultitouchInterface();

protected:
    const char* name = nullptr;
    IOHIDInterface* hid_interface = nullptr;

private:
    void t4_raw_event(AbsoluteTime timestamp, const t4_input_report& report);
    void u1_raw_event(AbsoluteTime timestamp, const u1_input_report& report);
    void stickEventLocked(AbsoluteTime timestamp, const u1_sp_input_report& report);
    template <typename Decode>
    void touchEventLocked(AbsoluteTime timestamp, bool click, Decode decode);
    bool suppressContacts(AbsoluteTime timestamp) const;

    // A callback can re-enter this service. Queue snapshots under input_lock and
    // let one caller deliver them in order after dropping the lock.
    struct Delivery {
        Delivery* next;
        IOService* client;
        bool touch;
        bool pointer;
        VoodooInputEvent contacts;
        RelativePointerEvent relative;
    };
    Delivery first_delivery = {};
    Delivery release_delivery = {}; // Releases can still be queued while a client callback is running.
    Delivery* delivery_head = nullptr;
    Delivery* delivery_tail = nullptr;
    bool delivering = false;
    Delivery* reserveDeliveryLocked(bool release = false);
    void unlockAndDeliver();
    void discardDeliveries();
    void setPointerLocked(Delivery* event, AbsoluteTime timestamp, SInt32 dx, SInt32 dy);
    UInt8 sent_buttons = 0;
    int previous_thumb = -1;
    IONotifier* lid_notifier = nullptr;
    unsigned lid_state = 2; // unknown until registration and initial snapshot
    static IOReturn lidChanged(void* target, void*, UInt32 type, IOService*, void* argument, vm_size_t);

    // Explicit initial values (IOKit zero-fills new objects, this just makes the intended state visible).
    bool ready = false;
    uint64_t max_after_typing = 0;
    uint64_t key_time = 0;

    // Touchpad switched off with the keyboard's touchpad key (kKeyboardSetTouchStatus from VoodooPS2Keyboard).
    // The trackpoint keeps working.
    bool touchpad_disabled = false;
    // Input was already released for the closed lid / the disabled touchpad, so that it is not sent again
    // with every report. Both are protected by input_lock.
    bool lid_released = false;
    bool touchpad_released = false;
    bool dropTouchpadReportLocked(AbsoluteTime timestamp);
    bool isReady() const { return __atomic_load_n(&ready, __ATOMIC_ACQUIRE); }
    void setReady(bool value) { __atomic_store_n(&ready, value, __ATOMIC_RELEASE); }
    dev_num dev_type = U1;

    // Geometry belongs to this HID interface and is re-probed after failed setup.
    UInt32 t4_y_max = 0;
    UInt32 t4_y_min = T4_COUNT_PER_ELECTRODE;

    // Serializes power transitions and register I/O independently of the HID transport loop.
    IOWorkLoop* recovery_loop = nullptr;
    IOTimerEventSource* wake_timer = nullptr;
    unsigned wake_attempt = 0;

    enum class RecoveryAction { InitialStart, PowerOn, PowerOff, Stop };
    RecoveryAction power_state = RecoveryAction::InitialStart;
    void updateRecovery(RecoveryAction action);
    void updateRecoveryGated(RecoveryAction action);
    static void wakeTimerFired(OSObject* owner, IOTimerEventSource* timer);
    void scheduleWakeAttempt();

    VoodooInputEvent inputMessage = {};
    bool geometry_valid = false;
    AlpsFeatureBuffer* feature_buffer = nullptr;
    IOReturn exchangeFeature(UInt8 id, UInt8* bytes, size_t size, bool read);
    UInt32 u1_button_count = 0;
    bool has_stick = false;
    UInt8 pad_buttons = 0;
    UInt8 stick_buttons = 0;
    void releaseInput(AbsoluteTime timestamp);

    IOService* voodooInputInstance = nullptr;

    // Serializes input state, the delivery queue, button caches and voodooInputInstance
    // between the HID interrupt path, the recovery work loop and VoodooInput open/close.
    IOLock* input_lock = nullptr;
    bool releaseInputLocked(AbsoluteTime timestamp, bool releaseStick = true);

    /* Sends a report to the device to instruct it to enter Touchpad mode. They return whether the
     * device is now ready (and set `ready` accordingly). */
    bool t4_device_init();
    bool u1_device_init();

    /* Picks t4_/u1_device_init() by dev_type (and refuses to run once hid_interface is gone) */
    bool initialiseDevice();

    /* initialiseDevice() with back-off: the touchpad needs time to answer after a power loss (S4).
     * first_delay_ms is the settle time before the first attempt. */
    bool initialiseDeviceWithRetry(UInt32 first_delay_ms);

    IOReturn u1_read_write_register(UInt32 address, UInt8 *read_val, UInt8 write_val, bool read_flag);
    IOReturn t4_read_write_register(UInt32 address, UInt8 *read_val, UInt8 write_val, bool read_flag);
};

#endif /* AlpsHIDEventDriver_hpp */
