// ESP32-S3 / ESP32-C3 main firmware.
// Reads BMW steering-wheel buttons (0x1D6, K-CAN, listen-only) and sends them
// to the phone as BLE HID consumer-control keys (Next / Previous / Play-Pause).
// Audio is handled by a separate Bluetooth-to-AUX receiver: the S3/C3 have no
// Bluetooth Classic and therefore cannot receive A2DP audio.
//
// Pair the phone with "BMW E92 Buttons" from the phone's Bluetooth settings.
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>
#include "../common/can_bus.h"
#include "../common/mfl.h"

#ifndef CAN_BITRATE
#define CAN_BITRATE 100000
#endif
#ifndef BLE_DEVICE_NAME
#define BLE_DEVICE_NAME "BMW E92 Buttons"
#endif

// Consumer-control report: 1 byte, one bit per key.
enum : uint8_t {
    KEY_NEXT = 1 << 0,
    KEY_PREV = 1 << 1,
    KEY_PLAY_PAUSE = 1 << 2,
    KEY_STOP = 1 << 3,
};

static const uint8_t kReportMap[] = {
    0x05, 0x0C,        // Usage Page (Consumer)
    0x09, 0x01,        // Usage (Consumer Control)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x04,        //   Report Count (4)
    0x09, 0xB5,        //   Usage (Scan Next Track)
    0x09, 0xB6,        //   Usage (Scan Previous Track)
    0x09, 0xCD,        //   Usage (Play/Pause)
    0x09, 0xB7,        //   Usage (Stop)
    0x81, 0x02,        //   Input (Data, Var, Abs)
    0x95, 0x04,        //   Report Count (4) padding
    0x81, 0x03,        //   Input (Const)
    0xC0,              // End Collection
};

static NimBLECharacteristic *g_input = nullptr;
static volatile bool g_connected = false;
static MflDecoder g_mfl;

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *) override {
        g_connected = true;
        Serial.println("BLE: phone connected");
    }
    void onDisconnect(NimBLEServer *) override {
        g_connected = false;
        Serial.println("BLE: disconnected, advertising again");
        NimBLEDevice::startAdvertising();
    }
};

static void bleBegin() {
    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEDevice::setSecurityAuth(true, false, true);  // bonding, no MITM, secure connections
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

    NimBLEServer *server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());

    NimBLEHIDDevice *hid = new NimBLEHIDDevice(server);
    g_input = hid->inputReport(1);
    hid->manufacturer()->setValue("DIY");
    hid->pnp(0x02, 0xE502, 0xA111, 0x0210);
    hid->hidInfo(0x00, 0x01);
    hid->reportMap((uint8_t *)kReportMap, sizeof(kReportMap));
    hid->setBatteryLevel(100);
    hid->startServices();

    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->setAppearance(0x03C0);  // generic HID (not "keyboard": keeps iOS on-screen keyboard)
    adv->addServiceUUID(hid->hidService()->getUUID());
    adv->setScanResponse(true);
    adv->start();
    Serial.printf("BLE: advertising as \"%s\"\n", BLE_DEVICE_NAME);
}

static void sendKey(uint8_t key) {
    if (!g_connected || !g_input) return;
    uint8_t report = key;
    g_input->setValue(&report, 1);
    g_input->notify();
    delay(30);
    report = 0;
    g_input->setValue(&report, 1);
    g_input->notify();
}

static void handlePress(MflButton b) {
    uint8_t key = 0;
    switch (b) {
        case MFL_NEXT: key = KEY_NEXT; break;
        case MFL_PREV: key = KEY_PREV; break;
        case MFL_VOICE:
        case MFL_PHONE: key = KEY_PLAY_PAUSE; break;
        default: return;  // volume stays with the factory radio
    }
    Serial.printf("MFL %s -> BLE key 0x%02X%s\n", mflButtonName(b), key,
                  g_connected ? "" : " (no phone connected)");
    sendKey(key);
}

void setup() {
    Serial.begin(115200);
    delay(500);
    bleBegin();
    if (!canBegin(CAN_BITRATE)) {
        Serial.println("!! CAN init failed");
    } else {
        Serial.printf("CAN: %s @ %lu bit/s listen-only\n", canBackendName(),
                      (unsigned long)CAN_BITRATE);
    }
}

void loop() {
    CanFrame f;
    const uint32_t now = millis();
    if (canRead(f, 10)) {
        const uint8_t pressed = g_mfl.update(f, now);
        for (uint8_t i = 0; i < MFL_BUTTON_COUNT; ++i) {
            if (pressed & (1 << i)) handlePress((MflButton)i);
        }
    }
    g_mfl.tick(now);
}
