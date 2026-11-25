/*This code is derived and adapted from VoodooI2CHID's Multitouch Event Driver and Precision
 Touchpad Event Driver (https://github.com/alexandred/VoodooI2C) and the Linux kernel driver
 for the alps t4 touchpad (https://github.com/torvalds/linux/blob/master/drivers/hid/hid-alps.c) */


#include "AlpsHID.hpp"
#include <IOKit/pwr_mgt/RootDomain.h>
#include <kern/task.h>


#define super IOHIDEventService
OSDefineMetaClassAndStructors(AlpsHIDEventDriver, IOHIDEventService);

// I2C transports may call writeBytes() with the actual reply but leave the
// descriptor length at capacity. Track the protocol prefix independently.
class AlpsFeatureBuffer : public IOBufferMemoryDescriptor {
    OSDeclareDefaultStructors(AlpsFeatureBuffer);
public:
    void beginReply() { __atomic_store_n(&coverage, 0U, __ATOMIC_RELAXED); }
    // Direct-mapped transports report length without using writeBytes() (see alps::covers).
    bool covers(unsigned length) const {
        return alps::covers(__atomic_load_n(&coverage, __ATOMIC_ACQUIRE), getLength(), length);
    }
    IOByteCount writeBytes(IOByteCount offset, const void* bytes, IOByteCount length) override {
        auto written = IOBufferMemoryDescriptor::writeBytes(offset, bytes, length);
        __atomic_fetch_or(&coverage, alps::coverageMask(offset, written), __ATOMIC_RELEASE);
        return written;
    }
private:
    unsigned coverage = 0;
};
OSDefineMetaClassAndStructors(AlpsFeatureBuffer, IOBufferMemoryDescriptor);

// Copies at most max_len bytes of a report into buffer, never more than the buffer can hold
// (the report length is device-controlled), and returns the number of bytes copied.
static UInt32 copyReport(IOMemoryDescriptor* report, void* buffer, UInt32 max_len) {
    UInt64 available = report->getLength();
    UInt32 len = available < max_len ? (UInt32)available : max_len;

    return (UInt32)report->readBytes(0, buffer, len);
}

bool AlpsHIDEventDriver::t4_device_init() {

    setReady(false);

    UInt8 tmp = '\0';
    alps_dev pri_data = {};
    IOReturn ret = kIOReturnSuccess;

    // hid_interface is cleared in didTerminate(); this is also called from the wake-up path,
    // so a missing interface must not be dereferenced
    if (!hid_interface) {
        IOLog("%s::%s HID interface not found\n", getName(), name ? name : "unknown");
        return false;
    }
    if (!geometry_valid) {
        const bool buttonless = hid_interface->getProductID() == HID_PRODUCT_ID_T4_BTNLESS;
        if (buttonless) {
            ret = t4_read_write_register(T4_PRM_ID_CONFIG_3, &tmp, 0, true);
            if (ret!=kIOReturnSuccess) {
                IOLog("failed T4_PRM_ID_CONFIG_3 (%d)\n", ret);
                goto exit;
            }
        }

        alps::Geometry g;
        alps::t4Geometry(buttonless, tmp, g);
        pri_data.x_max = g.xMax;
        pri_data.x_min = g.xMin;
        pri_data.y_max = g.yMax;
        pri_data.y_min = g.yMin;
        pri_data.x_active_len_mm = g.xActiveLenMm;
        pri_data.y_active_len_mm = g.yActiveLenMm;
        pri_data.btn_cnt = 1;
    }

    // Preserve unrelated configuration bits on every T4 variant, not only BTNLESS.
    ret = t4_read_write_register(PRM_SYS_CONFIG_1, &tmp, 0, true);
    if (ret != kIOReturnSuccess) {
        IOLog("failed PRM_SYS_CONFIG_1 (%d)\n", ret);
        goto exit;
    }
    tmp |= 0x02;

    ret = t4_read_write_register(PRM_SYS_CONFIG_1, NULL, tmp, false);
    if (ret!=kIOReturnSuccess) {
        IOLog("failed PRM_SYS_CONFIG_1 (%d)\n", ret);
        goto exit;
    }

    ret = t4_read_write_register(T4_PRM_FEED_CONFIG_1, NULL, T4_I2C_ABS, false);
    if (ret!=kIOReturnSuccess) {
        IOLog("failed T4_PRM_FEED_CONFIG_1 (%d)\n", ret);
        goto exit;
    }

    ret = t4_read_write_register(T4_PRM_FEED_CONFIG_4, NULL, T4_FEEDCFG4_ADVANCED_ABS_ENABLE, false);
    if (ret!=kIOReturnSuccess) {
        IOLog("failed T4_PRM_FEED_CONFIG_4 (%d)\n", ret);
        goto exit;
    }
    pri_data.max_fingers = 5;

    if (!geometry_valid) {
        // The Y range differs between T4 variants; use the probed geometry for inversion.
        t4_y_max = pri_data.y_max;
        t4_y_min = pri_data.y_min;

        setProperty(VOODOO_INPUT_LOGICAL_MAX_X_KEY, pri_data.x_max, 32);
        setProperty(VOODOO_INPUT_LOGICAL_MAX_Y_KEY, pri_data.y_max, 32);

        setProperty(VOODOO_INPUT_PHYSICAL_MAX_X_KEY, 10240, 32);
        setProperty(VOODOO_INPUT_PHYSICAL_MAX_Y_KEY, 6140, 32);
    }

    geometry_valid = true;
    setReady(true);
    return true;

exit:
    geometry_valid = false;
    setReady(false);
    return false;
}

IOReturn AlpsHIDEventDriver::lidChanged(void* target, void*, UInt32 type, IOService*, void* argument, vm_size_t) {
    if (type == kIOPMMessageClamshellStateChange) {
        auto* driver = static_cast<AlpsHIDEventDriver*>(target);
        unsigned closed = (reinterpret_cast<uintptr_t>(argument) & kClamshellStateBit) != 0;
        __atomic_store_n(&driver->lid_state, closed, __ATOMIC_RELEASE);
    }
    return kIOReturnSuccess;
}

void AlpsHIDEventDriver::handleInterruptReport(AbsoluteTime timestamp, IOMemoryDescriptor* report, IOHIDReportType report_type, UInt32 report_id) {
    if (!isReady() || !input_lock || !report || report_type != kIOHIDReportTypeInput)
        return;

    union {
        t4_input_report t4;
        u1_input_report pad;
        u1_sp_input_report stick;
    } frame = {};
    unsigned size;
    const auto kind = alps::classifyInput(dev_type == T4, report_id, size);
    if (kind == alps::ReportKind::None)
        return;
    const bool stick = kind == alps::ReportKind::U1Stick;
    if (copyReport(report, &frame, size) != size)
        return;

    unsigned closed = __atomic_load_n(&lid_state, __ATOMIC_ACQUIRE);
    if (closed == 2) {
        auto* root = getPMRootDomain();
        closed = root && root->getProperty("AppleClamshellState") == kOSBooleanTrue;
    }

    IOLockLock(input_lock);
    if (isReady() && voodooInputInstance) {
        if (closed) {
            if (!lid_released)
                lid_released = releaseInputLocked(timestamp);
        } else {
            lid_released = false;
            if (stick) {
                if (has_stick)
                    stickEventLocked(timestamp, frame.stick);
            } else if (!dropTouchpadReportLocked(timestamp)) {
                if (dev_type == T4)
                    t4_raw_event(timestamp, frame.t4);
                else
                    u1_raw_event(timestamp, frame.pad);
            }
        }
    }
    unlockAndDeliver();
}

bool AlpsHIDEventDriver::start(IOService* provider) {
    uint64_t quiet_ns = 500000000;
    OSNumber* quiet = OSDynamicCast(OSNumber, getProperty("QuietTimeAfterTyping"));
    if (quiet)
        quiet_ns = quiet->unsigned64BitValue() * 1000000;
    nanoseconds_to_absolutetime(quiet_ns, &max_after_typing);
    if (!super::start(provider))
        return false;

    setProperty("VoodooI2CServices Supported", kOSBooleanTrue);

    return true;
}

bool AlpsHIDEventDriver::handleStart(IOService* provider) {

    auto* interface = OSDynamicCast(IOHIDInterface, provider);

    if (!interface)
        return false;

    if (interface->getVendorID() != ALPS_VENDOR && interface->getVendorID() != CIRQUE_VENDOR) {
        return false;
    }

    switch (interface->getProductID()) {

        case HID_PRODUCT_ID_T4_USB:
        case HID_PRODUCT_ID_T4_USB_2:
        case HID_PRODUCT_ID_T4_USB_3:
        case HID_PRODUCT_ID_T4_USB_4:
        case HID_PRODUCT_ID_G1:
        case HID_PRODUCT_ID_T4_BTNLESS:
            dev_type = T4;
            break;
        case HID_PRODUCT_ID_U1:
        case HID_PRODUCT_ID_U1_OLDMAN:
        case HID_PRODUCT_ID_U1_UNICORN_LEGACY:
        case HID_PRODUCT_ID_U1_DUAL:
        case HID_PRODUCT_ID_U1_DUAL_PTP:
        case HID_PRODUCT_ID_U1_PTP_1:
        case HID_PRODUCT_ID_U1_PTP_2:
        case HID_PRODUCT_ID_U1_DUAL_3BTN_PTP:
        case HID_PRODUCT_ID_U1_1222:
            dev_type = U1;
            break;

        default:
            return false;
    }

    input_lock = IOLockAlloc();
    if (!input_lock)
        return false;

    for (int i = 0; i < MAX_TOUCHES; ++i) {
        auto& contact = inputMessage.transducers[i];
        contact.type = VoodooInputTransducerType::FINGER;
        contact.secondaryId = i;
        contact.fingerType = static_cast<MT2FingerType>(kMT2FingerTypeIndexFinger + i % 4);
    }
    inputMessage.contact_count = MAX_TOUCHES;
    auto* root = getPMRootDomain();
    if (root) {
        lid_notifier = root->registerInterest(gIOGeneralInterest, lidChanged, this);
        if (lid_notifier) {
            unsigned unknown = 2;
            unsigned closed = root->getProperty("AppleClamshellState") == kOSBooleanTrue;
            // A notification during registration takes precedence over this snapshot.
            __atomic_compare_exchange_n(&lid_state, &unknown, closed, false, __ATOMIC_RELEASE, __ATOMIC_RELAXED);
        }
    }
    recovery_loop = IOWorkLoop::workLoop();
    if (!recovery_loop)
        return false;
    wake_timer = IOTimerEventSource::timerEventSource(this, wakeTimerFired);
    if (!wake_timer || recovery_loop->addEventSource(wake_timer) != kIOReturnSuccess)
        return false;

    if (!interface->open(this, 0, OSMemberFunctionCast(IOHIDInterface::InterruptReportAction, this, &AlpsHIDEventDriver::handleInterruptReport), NULL)) {
        return false;
    }

    hid_interface = interface;

    // getProduct() returns an OSString*. It used to be cast to char* and then printed with %s,
    // i.e. the OSString object itself was read as if it were the text.
    OSString* product = hid_interface->getProduct();
    name = product ? product->getCStringNoCopy() : "unknown";

    // Documented order: PMinit(), joinPMtree(), registerPowerDriver()
    PMinit();
    hid_interface->joinPMtree(this);
    registerPowerDriver(this, VoodooI2CIOPMPowerStates, kVoodooI2CIOPMNumberPowerStates);

    updateRecovery(RecoveryAction::InitialStart);

    publishMultitouchInterface();

    return true;
}


IOReturn AlpsHIDEventDriver::setPowerState(
    unsigned long whichState,
    IOService* whatDevice
) {
    // The return value of a setPowerState() callback is "microseconds until the change is complete"
    // (kIOPMAckImplied == 0 means: done). An IOReturn error code here is read as an enormous delay.
    if (whatDevice != this)
        return kIOPMAckImplied;

    updateRecovery(whichState ? RecoveryAction::PowerOn : RecoveryAction::PowerOff);
    return kIOPMAckImplied;
}

// The initial six delays match the previous wake policy. Leave time between attempts
// for the transport to finish resuming; a failed device must not poll indefinitely.
static const UInt32 wakeDelaysMS[] = { 10, 50, 100, 250, 500, 1000, 2000, 4000, 8000, 8000 };
static const unsigned wakeAttemptCount = sizeof(wakeDelaysMS) / sizeof(wakeDelaysMS[0]);

void AlpsHIDEventDriver::updateRecovery(RecoveryAction action) {
    if (recovery_loop) {
        recovery_loop->runAction([](OSObject* owner, void* arg, void*, void*, void*) -> IOReturn {
            auto* driver = static_cast<AlpsHIDEventDriver*>(owner);
            driver->updateRecoveryGated(*static_cast<RecoveryAction*>(arg));
            return kIOReturnSuccess;
        }, this, &action);
    }
}

void AlpsHIDEventDriver::updateRecoveryGated(RecoveryAction action) {
    if (power_state == RecoveryAction::Stop)
        return;

    switch (action) {
        case RecoveryAction::InitialStart:
            if (power_state == RecoveryAction::InitialStart) {
                power_state = RecoveryAction::PowerOn;
                if (!initialiseDeviceWithRetry(0))
                    IOLog("%s::%s Initial device setup failed\n", getName(), name);
            }
            break;

        case RecoveryAction::PowerOn:
            if (power_state == RecoveryAction::PowerOff) {
                power_state = action;
                wake_attempt = 0;
                scheduleWakeAttempt();
            }
            break;

        case RecoveryAction::PowerOff: {
            power_state = action;
            if (wake_timer)
                wake_timer->cancelTimeout();
            wake_attempt = 0;
            setReady(false);
            AbsoluteTime timestamp;
            clock_get_uptime(&timestamp);
            releaseInput(timestamp);
            IOLog("%s::%s Going to sleep\n", getName(), name ? name : "unknown");
            break;
        }

        case RecoveryAction::Stop:
            power_state = action;
            if (wake_timer)
                wake_timer->cancelTimeout();
            setReady(false);
            if (hid_interface) {
                hid_interface->close(this);
                hid_interface = nullptr;
            }
            break;
    }
}

void AlpsHIDEventDriver::scheduleWakeAttempt() {
    IOReturn ret = wake_timer->setTimeoutMS(wakeDelaysMS[wake_attempt]);
    if (ret != kIOReturnSuccess)
        IOLog("%s::%s Cannot schedule wake attempt %u: 0x%x\n",
              getName(), name, wake_attempt + 1, ret);
}

void AlpsHIDEventDriver::wakeTimerFired(OSObject* owner, IOTimerEventSource*) {
    auto* driver = static_cast<AlpsHIDEventDriver*>(owner);
    if (driver->power_state != RecoveryAction::PowerOn || driver->isInactive() ||
        !driver->hid_interface || driver->isReady())
        return;

    unsigned attempt = ++driver->wake_attempt;
    if (driver->initialiseDevice()) {
        IOLog("%s::%s Wake initialization succeeded (attempt %u/%u)\n",
              driver->getName(), driver->name, attempt, wakeAttemptCount);
        return;
    }

    IOLog("%s::%s Wake initialization failed (attempt %u/%u)\n",
          driver->getName(), driver->name, attempt, wakeAttemptCount);
    if (attempt < wakeAttemptCount)
        driver->scheduleWakeAttempt();
}

bool AlpsHIDEventDriver::initialiseDevice() {
    // hid_interface is cleared in didTerminate(), which can race with a wake-up
    if (!hid_interface) {
        setReady(false);
        return false;
    }

    switch (dev_type) {
        case T4:
            return t4_device_init();

        case U1:
            return u1_device_init();
    }

    setReady(false);
    return false;
}

bool AlpsHIDEventDriver::initialiseDeviceWithRetry(UInt32 first_delay_ms) {
    // Delay (ms) before every further attempt: the firmware needs time to come back after a power loss
    static const UInt32 retry_delays_ms[] = { 50, 100, 250, 500, 1000 };
    const int attempts = 1 + (int)(sizeof(retry_delays_ms) / sizeof(retry_delays_ms[0]));

    for (int attempt = 0; attempt < attempts; ++attempt) {
        UInt32 delay_ms = attempt == 0 ? first_delay_ms : retry_delays_ms[attempt - 1];

        if (delay_ms)
            IOSleep(delay_ms);

        // The device may have been terminated while we were waiting
        if (!hid_interface)
            break;

        if (initialiseDevice()) {
            IOLog("%s::%s Initialization succeeded "
                  "(attempt %d/%d)\n",
                  getName(), name ? name : "unknown",
                  attempt + 1, attempts);
            return true;
        }

        IOLog("%s::%s Initialization failed "
              "(attempt %d/%d)\n",
              getName(), name ? name : "unknown",
              attempt + 1, attempts);
    }

    return false;
}

IOReturn AlpsHIDEventDriver::publishMultitouchInterface() {
    setProperty("IOFBTransform", 0ull, 32);
    setProperty("VoodooInputSupported", kOSBooleanTrue);

    return kIOReturnSuccess;
}

void AlpsHIDEventDriver::handleStop(IOService* provider) {
    if (lid_notifier) {
        lid_notifier->remove();
        lid_notifier = nullptr;
    }
    updateRecovery(RecoveryAction::Stop);

    IOService* previous = nullptr;
    if (input_lock) {
        IOLockLock(input_lock);
        previous = voodooInputInstance;
        voodooInputInstance = nullptr;
        IOLockUnlock(input_lock);
        OSSafeReleaseNULL(previous);
    } else {
        OSSafeReleaseNULL(voodooInputInstance);
    }

    PMstop();
    super::handleStop(provider);
}

IOReturn AlpsHIDEventDriver::message(UInt32 type, IOService* provider, void* argument)
{
    switch (type)
    {
        // VoodooPS2Keyboard's touchpad on/off key. These were declared but never handled, so the key did nothing.
        case kKeyboardGetTouchStatus:
        {
            if (!argument)
                return kIOReturnBadArgument;
            *static_cast<bool*>(argument) = !__atomic_load_n(&touchpad_disabled, __ATOMIC_RELAXED);
            return kIOReturnSuccess;
        }

        case kKeyboardSetTouchStatus:
        {
            if (!argument)
                return kIOReturnBadArgument;
            bool enable = *static_cast<bool*>(argument);
            IOLog("%s::%s Touchpad %s by keyboard\n", getName(), name ? name : "unknown", enable ? "enabled" : "disabled");
            if (input_lock) {
                IOLockLock(input_lock);
                __atomic_store_n(&touchpad_disabled, !enable, __ATOMIC_RELAXED);
                // Lift the fingers right away instead of waiting for the next report
                if (!enable && !touchpad_released) {
                    AbsoluteTime timestamp;
                    clock_get_uptime(&timestamp);
                    touchpad_released = releaseInputLocked(timestamp, false);
                }
                if (enable)
                    touchpad_released = false;
                unlockAndDeliver();
            } else {
                __atomic_store_n(&touchpad_disabled, !enable, __ATOMIC_RELAXED);
            }
            return kIOReturnSuccess;
        }

        case kKeyboardKeyPressTime:
        {
            if (!argument)
                return kIOReturnBadArgument;

            //  Remember last time key was pressed
            uint64_t time;
            nanoseconds_to_absolutetime(*static_cast<const uint64_t*>(argument), &time);
            __atomic_store_n(&key_time, time, __ATOMIC_RELEASE);
            return kIOReturnSuccess;
        }
    }

    // Don't swallow messages we don't handle ourselves
    return super::message(type, provider, argument);
}

void AlpsHIDEventDriver::free() {
    if (lid_notifier) {
        lid_notifier->remove();
        lid_notifier = nullptr;
    }
    discardDeliveries();
    OSSafeReleaseNULL(feature_buffer);
    // Keep the loop alive through the final PM/termination callbacks and partial starts.
    if (wake_timer && wake_timer->getWorkLoop())
        recovery_loop->removeEventSource(wake_timer);
    OSSafeReleaseNULL(wake_timer);
    OSSafeReleaseNULL(recovery_loop);
    OSSafeReleaseNULL(voodooInputInstance);
    if (input_lock) {
        IOLockFree(input_lock);
        input_lock = nullptr;
    }
    super::free();
}

bool AlpsHIDEventDriver::didTerminate(IOService* provider, IOOptionBits options, bool* defer) {
    updateRecovery(RecoveryAction::Stop);

    return super::didTerminate(provider, options, defer);
}

// Called with input_lock held for every touchpad report: drops it while the touchpad is switched off,
// releasing the contacts once.
bool AlpsHIDEventDriver::dropTouchpadReportLocked(AbsoluteTime timestamp) {
    if (!__atomic_load_n(&touchpad_disabled, __ATOMIC_RELAXED)) {
        touchpad_released = false;
        return false;
    }
    if (!touchpad_released) {
        touchpad_released = releaseInputLocked(timestamp, false);
    }
    return true;
}

// Release both endpoints before input is disabled. Clear cached source state
// even if the VoodooInput client has already disconnected.
AlpsHIDEventDriver::Delivery* AlpsHIDEventDriver::reserveDeliveryLocked(bool release) {
    if (!voodooInputInstance)
        return nullptr;
    Delivery* event = &first_delivery;
    if (event->client) {
        if (release && !release_delivery.client)
            event = &release_delivery;
        else
            event = static_cast<Delivery*>(IOMalloc(sizeof(Delivery)));
    }
    if (!event)
        return nullptr;
    event->client = voodooInputInstance;
    event->client->retain();
    event->next = nullptr;
    event->touch = event->pointer = false;
    if (delivery_tail)
        delivery_tail->next = event;
    else
        delivery_head = event;
    delivery_tail = event;
    return event;
}

// Called with input_lock held. Reentrant reports append behind the current
// delivery, including releases, without recursing into the client.
void AlpsHIDEventDriver::unlockAndDeliver() {
    if (delivering || !delivery_head) {
        IOLockUnlock(input_lock);
        return;
    }
    delivering = true;
    while (delivery_head) {
        auto* event = delivery_head;
        delivery_head = event->next;
        if (!delivery_head)
            delivery_tail = nullptr;
        IOService* client = event->client;
        bool current = client == voodooInputInstance;
        IOLockUnlock(input_lock);
        if (current && event->touch)
            super::messageClient(kIOMessageVoodooInputMessage, client, &event->contacts, sizeof(event->contacts));
        if (current && event->touch && event->pointer) {
            IOLockLock(input_lock);
            current = client == voodooInputInstance;
            IOLockUnlock(input_lock);
        }
        if (current && event->pointer)
            super::messageClient(kIOMessageVoodooTrackpointRelativePointer, client, &event->relative, sizeof(event->relative));
        client->release();
        bool embedded = event == &first_delivery || event == &release_delivery;
        if (!embedded)
            IOFree(event, sizeof(Delivery));
        IOLockLock(input_lock);
        if (embedded)
            event->client = nullptr;
    }
    delivering = false;
    IOLockUnlock(input_lock);
}

// Nothing can deliver any more: drop queued snapshots and their client references. Every queued
// event is normally delivered by the caller that queued it, so this only covers a partial start.
void AlpsHIDEventDriver::discardDeliveries() {
    while (delivery_head) {
        auto* event = delivery_head;
        delivery_head = event->next;
        OSSafeReleaseNULL(event->client);
        if (event != &first_delivery && event != &release_delivery)
            IOFree(event, sizeof(Delivery));
    }
    delivery_tail = nullptr;
}

void AlpsHIDEventDriver::setPointerLocked(Delivery* event, AbsoluteTime timestamp, SInt32 dx, SInt32 dy) {
    const UInt8 buttons = pad_buttons | stick_buttons;
    if (!dx && !dy && buttons == sent_buttons)
        return;
    sent_buttons = buttons;
    event->pointer = true;
    event->relative = {};
    event->relative.dx = dx;
    event->relative.dy = dy;
    event->relative.buttons = buttons;
    event->relative.timestamp = timestamp;
}

void AlpsHIDEventDriver::releaseInput(AbsoluteTime timestamp) {
    if (!input_lock)
        return;
    IOLockLock(input_lock);
    releaseInputLocked(timestamp);
    unlockAndDeliver();
}

bool AlpsHIDEventDriver::releaseInputLocked(AbsoluteTime timestamp, bool releaseStick) {
    auto* event = reserveDeliveryLocked(true);
    if (voodooInputInstance && !event)
        return false;
    pad_buttons = 0;
    if (releaseStick)
        stick_buttons = 0;
    for (int i = 0; i < MAX_TOUCHES; ++i) {
        auto& contact = inputMessage.transducers[i];
        contact.isValid = contact.isTransducerActive ||
            (contact.isValid && contact.isPhysicalButtonDown);
        contact.isTransducerActive = false;
        contact.isPhysicalButtonDown = false;
        contact.timestamp = timestamp;
    }
    inputMessage.timestamp = timestamp;
    if (event) {
        event->touch = true;
        event->contacts = inputMessage;
        setPointerLocked(event, timestamp, 0, 0);
    }
    return true;
}

bool AlpsHIDEventDriver::suppressContacts(AbsoluteTime timestamp) const {
    const auto key = __atomic_load_n(&key_time, __ATOMIC_ACQUIRE);
    return key && timestamp >= key && timestamp - key < max_after_typing;
}

void AlpsHIDEventDriver::stickEventLocked(AbsoluteTime timestamp, const u1_sp_input_report& report) {
    SInt32 dx, dy;
    alps::stickDelta(report, dx, dy);
    stick_buttons = alps::stickButtons(report);
    if (!dx && !dy && (pad_buttons | stick_buttons) == sent_buttons)
        return;
    if (auto* event = reserveDeliveryLocked())
        setPointerLocked(event, timestamp, dx, dy);
}

// The caller supplies device-specific decoding. Slot history stays at the
// hardware index, including sparse slots and the first inactive lift frame.
template <typename Decode>
void AlpsHIDEventDriver::touchEventLocked(AbsoluteTime timestamp, bool click, Decode decode) {
    auto* event = reserveDeliveryLocked();
    if (!event)
        return;
    if (previous_thumb >= 0)
        inputMessage.transducers[previous_thumb].fingerType =
            static_cast<MT2FingerType>(kMT2FingerTypeIndexFinger + previous_thumb % 4);
    previous_thumb = -1;
    const bool suppress = suppressContacts(timestamp);
    unsigned count = 0;
    UInt32 lowest_y = 0;
    int lowest = -1;
    for (int i = 0; i < MAX_TOUCHES; ++i) {
        auto& slot = inputMessage.transducers[i];
        UInt32 x, y;
        const bool active = !suppress && decode(i, x, y);
        slot.isValid = active || slot.isTransducerActive || (slot.isValid && slot.isPhysicalButtonDown);
        slot.isTransducerActive = active;
        slot.isPhysicalButtonDown = click;
        slot.previousCoordinates = slot.currentCoordinates;
        slot.timestamp = timestamp;
        if (active) {
            slot.currentCoordinates.x = x;
            slot.currentCoordinates.y = y;
            ++count;
            if (y >= lowest_y) {
                lowest_y = y;
                lowest = i;
            }
        }
    }
    if (lowest >= 0 && (count >= 4 || click)) {
        inputMessage.transducers[lowest].fingerType = kMT2FingerTypeThumb;
        previous_thumb = lowest;
    }
    inputMessage.timestamp = timestamp;
    event->touch = true;
    event->contacts = inputMessage;
    setPointerLocked(event, timestamp, 0, 0);
}

void AlpsHIDEventDriver::t4_raw_event(AbsoluteTime timestamp, const t4_input_report& report) {
    touchEventLocked(timestamp, report.button, [&](int i, UInt32& x, UInt32& y) {
        return alps::t4DecodeContact(report.contact[i], t4_y_min, t4_y_max, x, y);
    });
}

void AlpsHIDEventDriver::u1_raw_event(AbsoluteTime timestamp, const u1_input_report& report) {
    const bool clickpad = u1_button_count == 1;
    pad_buttons = alps::u1PadButtons(report.buttons, clickpad);
    touchEventLocked(timestamp, alps::u1Click(report.buttons, clickpad), [&](int i, UInt32& x, UInt32& y) {
        return alps::u1DecodeContact(report.contact[i], x, y);
    });
}

bool AlpsHIDEventDriver::u1_device_init() {
    setReady(false);

    UInt8 tmp = 0;
    UInt8 dev_ctrl = 0;
    UInt8 sen_line_num_x = 0;
    UInt8 sen_line_num_y = 0;
    UInt8 pitch_x = 0;
    UInt8 pitch_y = 0;
    UInt8 resolution = 0;

    alps_dev pri_data = {};
    IOReturn ret = kIOReturnSuccess;

    if (!hid_interface)
        return false;

    // Read device control register.
    ret = u1_read_write_register(
        ADDRESS_U1_DEV_CTRL_1, &dev_ctrl, 0, true);
    if (ret != kIOReturnSuccess)
        goto exit;

    dev_ctrl &= ~U1_DISABLE_DEV;
    dev_ctrl |= U1_TP_ABS_MODE;

    ret = u1_read_write_register(
        ADDRESS_U1_DEV_CTRL_1, NULL, dev_ctrl, false);
    if (ret != kIOReturnSuccess)
        goto exit;

    if (!geometry_valid) {
        // Read sensor geometry.
        ret = u1_read_write_register(
            ADDRESS_U1_NUM_SENS_X, &sen_line_num_x, 0, true);
        if (ret != kIOReturnSuccess)
            goto exit;

        ret = u1_read_write_register(
            ADDRESS_U1_NUM_SENS_Y, &sen_line_num_y, 0, true);
        if (ret != kIOReturnSuccess)
            goto exit;

        ret = u1_read_write_register(
            ADDRESS_U1_PITCH_SENS_X, &pitch_x, 0, true);
        if (ret != kIOReturnSuccess)
            goto exit;

        ret = u1_read_write_register(
            ADDRESS_U1_PITCH_SENS_Y, &pitch_y, 0, true);
        if (ret != kIOReturnSuccess)
            goto exit;

        ret = u1_read_write_register(
            ADDRESS_U1_RESO_DWN_ABS, &resolution, 0, true);
        if (ret != kIOReturnSuccess)
            goto exit;

        // Reject invalid geometry before calculating coordinate ranges.
        alps::Geometry g;
        if (!alps::u1Geometry(sen_line_num_x, sen_line_num_y, pitch_x, pitch_y, resolution, g)) {
            IOLog(
                "%s::%s Invalid U1 geometry: "
                "sensors=%u,%u pitch=%u,%u resolution=%u\n",
                getName(), name,
                sen_line_num_x, sen_line_num_y,
                pitch_x, pitch_y, resolution);

            ret = kIOReturnError;
            goto exit;
        }

        pri_data.x_active_len_mm = g.xActiveLenMm;
        pri_data.y_active_len_mm = g.yActiveLenMm;
        pri_data.x_max = g.xMax;
        pri_data.x_min = g.xMin;
        pri_data.y_max = g.yMax;
        pri_data.y_min = g.yMin;
    }

    // Read button configuration.
    ret = u1_read_write_register(
        ADDRESS_U1_PAD_BTN, &tmp, 0, true);
    if (ret != kIOReturnSuccess)
        goto exit;

    pri_data.btn_cnt = alps::u1ButtonCount(tmp);

    // Detect optional trackpoint support.
    ret = u1_read_write_register(
        ADDRESS_U1_DEVICE_TYP, &tmp, 0, true);
    if (ret != kIOReturnSuccess)
        goto exit;

    if (tmp & U1_DEVTYPE_SP_SUPPORT) {
        dev_ctrl |= U1_SP_ABS_MODE;

        ret = u1_read_write_register(
            ADDRESS_U1_DEV_CTRL_1, NULL, dev_ctrl, false);

        if (ret == kIOReturnSuccess) {
            ret = u1_read_write_register(
                ADDRESS_U1_SP_BTN,
                &pri_data.sp_btn_info, 0, true);
        }

        if (ret == kIOReturnSuccess) {
            pri_data.has_sp = 1;
        } else {
            // Trackpoint initialization is optional.
            IOLog(
                "%s::%s Trackpoint initialization failed: %d\n",
                getName(), name, ret);

            pri_data.has_sp = 0;
        }
    }

    if (!geometry_valid) {
        setProperty(
            VOODOO_INPUT_LOGICAL_MAX_X_KEY,
            pri_data.x_max, 32);

        setProperty(
            VOODOO_INPUT_LOGICAL_MAX_Y_KEY,
            pri_data.y_max, 32);

        setProperty(
            VOODOO_INPUT_PHYSICAL_MAX_X_KEY,
            pri_data.x_active_len_mm * 10, 32);

        setProperty(
            VOODOO_INPUT_PHYSICAL_MAX_Y_KEY,
            pri_data.y_active_len_mm * 10, 32);
    }

    geometry_valid = ret == kIOReturnSuccess;
    u1_button_count = pri_data.btn_cnt;
    has_stick = pri_data.has_sp != 0;
    setReady(true);
    return true;

exit:
    geometry_valid = false;
    IOLog(
        "%s::%s U1 initialization failed: %d\n",
        getName(), name, ret);

    setReady(false);
    return false;
}

// Only the recovery work loop calls register I/O. getReport/setReport are
// synchronous and the descriptor belongs to this device for its lifetime.
IOReturn AlpsHIDEventDriver::exchangeFeature(UInt8 id, UInt8* bytes, size_t size, bool read) {
    if (!hid_interface || !bytes || !size)
        return kIOReturnBadArgument;
    // The buffer is sized by the first request. setLength() beyond the capacity is invalid, so
    // replace it if a larger report ever comes along (T4 and U1 use different lengths).
    if (feature_buffer && feature_buffer->getCapacity() < size)
        OSSafeReleaseNULL(feature_buffer);
    if (!feature_buffer) {
        feature_buffer = new AlpsFeatureBuffer;
        if (!feature_buffer || !feature_buffer->initWithPhysicalMask(kernel_task, kIODirectionInOut, size, 1, 0)) {
            OSSafeReleaseNULL(feature_buffer);
            return kIOReturnNoMemory;
        }
    }

    feature_buffer->setLength(size);
    auto* data = static_cast<UInt8*>(feature_buffer->getBytesNoCopy());
    memcpy(data, bytes, size);
    auto ret = hid_interface->setReport(feature_buffer, kIOHIDReportTypeFeature, id);
    if (ret != kIOReturnSuccess || !read)
        return ret;

    feature_buffer->setLength(size);
    bzero(data, size);
    feature_buffer->beginReply();
    ret = hid_interface->getReport(feature_buffer, kIOHIDReportTypeFeature, id);
    if (ret != kIOReturnSuccess)
        return ret;
    // Some transports leave the descriptor length at capacity. Clearing the
    // request first also catches empty replies and avoids reusing stale data.
    unsigned needed = alps::replyLength(id);
    if (needed > size || !feature_buffer->covers(needed) || data[0] != id)
        return kIOReturnIOError;
    memcpy(bytes, data, needed);
    return kIOReturnSuccess;
}

IOReturn AlpsHIDEventDriver::u1_read_write_register(
    UInt32 address,
    UInt8 *read_val,
    UInt8 write_val,
    bool read_flag
) {
    if (!hid_interface || (read_flag && !read_val))
        return kIOReturnBadArgument;

    UInt8 input[U1_FEATURE_REPORT_LEN];
    alps::buildU1Request(address, read_flag, write_val, input);

    auto ret = exchangeFeature(U1_FEATURE_REPORT_ID, input, sizeof(input), read_flag);
    if (ret == kIOReturnSuccess && read_flag)
        *read_val = input[6];
    return ret;
}

IOReturn AlpsHIDEventDriver::t4_read_write_register(
    UInt32 address,
    UInt8 *read_val,
    UInt8 write_val,
    bool read_flag
) {
    if (!hid_interface || (read_flag && !read_val))
        return kIOReturnBadArgument;

    UInt8 input[T4_FEATURE_REPORT_LEN];
    alps::buildT4Request(address, read_flag, write_val, input);

    IOReturn ret = exchangeFeature(T4_FEATURE_REPORT_ID, input, sizeof(input), read_flag);
    if (ret != kIOReturnSuccess || !read_flag)
        return ret;

    if (!alps::parseT4Reply(input, address, *read_val)) {
        IOLog("%s: invalid T4 reply for register %x\n", getName(), address);
        return kIOReturnIOError;
    }
    return kIOReturnSuccess;
}

bool AlpsHIDEventDriver::handleOpen(IOService *forClient, IOOptionBits options, void *arg) {
    if (forClient && forClient->getProperty(VOODOO_INPUT_IDENTIFIER)) {
        forClient->retain();

        if (input_lock)
            IOLockLock(input_lock);
        // A second open must not leak the first retain
        IOService* previous = voodooInputInstance;
        if (previous != forClient)
            sent_buttons = 0;
        voodooInputInstance = forClient;
        if (input_lock)
            IOLockUnlock(input_lock);

        OSSafeReleaseNULL(previous);
        return true;
    }

    return super::handleOpen(forClient, options, arg);
}

void AlpsHIDEventDriver::handleClose(IOService *forClient, IOOptionBits options) {
    // Any client can close: only drop the VoodooInput reference when it is VoodooInput that closes,
    // otherwise the pointer used by the report handlers would dangle.
    if (forClient && forClient == voodooInputInstance) {
        if (input_lock)
            IOLockLock(input_lock);
        IOService* previous = voodooInputInstance;
        voodooInputInstance = nullptr;
        if (input_lock)
            IOLockUnlock(input_lock);

        OSSafeReleaseNULL(previous);
        // VoodooInput was never registered with IOHIDEventService::handleOpen()
        return;
    }

    super::handleClose(forClient, options);
}

// IOService::close() only calls handleClose() when handleIsOpen() says the client is open.
// VoodooInput is opened outside IOHIDEventService's bookkeeping, so without this override its
// close() was ignored: the retain leaked and the report handlers kept messaging a stopped client.
bool AlpsHIDEventDriver::handleIsOpen(const IOService *forClient) const {
    if (forClient == nullptr) {
        if (voodooInputInstance)
            return true;
    } else if (forClient == voodooInputInstance) {
        return true;
    }

    return super::handleIsOpen(forClient);
}
