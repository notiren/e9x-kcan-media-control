// Listen-only CAN logger. Step 1 of the project: prove that 0x1D6 is visible
// at your tap point and verify the steering-wheel bit layout.
//
// Serial commands (115200 baud, send single character):
//   k  -> 100 kbit/s (K-CAN)          d  -> 500 kbit/s (D-CAN / OBD pins 6+14)
//   a  -> print all frames            c  -> print only frames whose payload changed
//   f  -> print only 0x1D6 frames     m  -> print decoded steering-wheel buttons
//   s  -> statistics (frames/s, list of seen IDs)
//   h  -> help
#include <Arduino.h>
#include "../common/can_bus.h"
#include "../common/mfl.h"

#ifndef CAN_BITRATE
#define CAN_BITRATE 100000
#endif

enum class Mode { All, Changed, OnlyMfl, MflDecoded };

static Mode g_mode = Mode::Changed;
static uint32_t g_bitrate = CAN_BITRATE;
static MflDecoder g_mfl;

struct IdEntry {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    uint32_t count;
};
static constexpr size_t kMaxIds = 160;
static IdEntry g_ids[kMaxIds];
static size_t g_idCount = 0;
static uint32_t g_totalFrames = 0;
static uint32_t g_windowFrames = 0;
static uint32_t g_windowStart = 0;
static uint32_t g_lastFrameMs = 0;
static uint32_t g_lastWarnMs = 0;

static IdEntry *lookup(uint32_t id, bool &isNew) {
    for (size_t i = 0; i < g_idCount; ++i) {
        if (g_ids[i].id == id) {
            isNew = false;
            return &g_ids[i];
        }
    }
    isNew = true;
    if (g_idCount >= kMaxIds) return nullptr;
    IdEntry *e = &g_ids[g_idCount++];
    memset(e, 0, sizeof(*e));
    e->id = id;
    return e;
}

static void printFrame(const CanFrame &f) {
    Serial.printf("%10lu  %s%03lX  [%u] ", (unsigned long)millis(), f.extended ? "x" : " ",
                  (unsigned long)f.id, f.dlc);
    for (uint8_t i = 0; i < f.dlc; ++i) Serial.printf(" %02X", f.data[i]);
    Serial.println();
}

static void printHelp() {
    Serial.println(F("\n--- E92 CAN logger (listen-only) ---"));
    Serial.printf("backend=%s  bitrate=%lu\n", canBackendName(), (unsigned long)g_bitrate);
    Serial.println(F("k=100k(K-CAN) d=500k(D-CAN/OBD) a=all c=changed f=0x1D6 m=buttons s=stats h=help"));
}

static void startBus(uint32_t bitrate) {
    g_bitrate = bitrate;
    g_idCount = 0;
    g_totalFrames = 0;
    g_lastFrameMs = millis();
    if (canBegin(bitrate)) {
        Serial.printf("CAN started: %s @ %lu bit/s (listen-only)\n", canBackendName(),
                      (unsigned long)bitrate);
    } else {
        Serial.println(F("!! CAN init failed - check wiring / pins / crystal setting"));
    }
}

static void printStats() {
    Serial.printf("\nframes total=%lu  unique IDs=%u\n", (unsigned long)g_totalFrames,
                  (unsigned)g_idCount);
    for (size_t i = 0; i < g_idCount; ++i) {
        const IdEntry &e = g_ids[i];
        Serial.printf("  %03lX  n=%-7lu last:", (unsigned long)e.id, (unsigned long)e.count);
        for (uint8_t b = 0; b < e.dlc; ++b) Serial.printf(" %02X", e.data[b]);
        Serial.println(e.id == MFL_CAN_ID ? "   <-- MFL steering wheel" : "");
    }
}

static void handleSerial() {
    while (Serial.available()) {
        switch (Serial.read()) {
            case 'k': startBus(100000); break;
            case 'd': startBus(500000); break;
            case 'a': g_mode = Mode::All; Serial.println(F("mode: all")); break;
            case 'c': g_mode = Mode::Changed; Serial.println(F("mode: changed")); break;
            case 'f': g_mode = Mode::OnlyMfl; Serial.println(F("mode: 0x1D6 only")); break;
            case 'm': g_mode = Mode::MflDecoded; Serial.println(F("mode: buttons")); break;
            case 's': printStats(); break;
            case 'h': printHelp(); break;
            default: break;
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1500);
    printHelp();
    startBus(g_bitrate);
    g_windowStart = millis();
}

void loop() {
    handleSerial();

    CanFrame f;
    const uint32_t now = millis();
    if (canRead(f, 10)) {
        g_lastFrameMs = now;
        ++g_totalFrames;
        ++g_windowFrames;

        bool isNew = false;
        IdEntry *e = lookup(f.id, isNew);
        const bool changed = !e || isNew || e->dlc != f.dlc || memcmp(e->data, f.data, f.dlc) != 0;
        if (e) {
            e->dlc = f.dlc;
            memcpy(e->data, f.data, f.dlc);
            ++e->count;
        }

        switch (g_mode) {
            case Mode::All: printFrame(f); break;
            case Mode::Changed: if (changed) printFrame(f); break;
            case Mode::OnlyMfl: if (f.id == MFL_CAN_ID) printFrame(f); break;
            case Mode::MflDecoded: {
                const uint8_t pressed = g_mfl.update(f, now);
                for (uint8_t i = 0; i < MFL_BUTTON_COUNT; ++i) {
                    if (pressed & (1 << i)) {
                        Serial.printf("PRESS %-6s  raw:", mflButtonName((MflButton)i));
                        for (uint8_t b = 0; b < f.dlc; ++b) Serial.printf(" %02X", f.data[b]);
                        Serial.println();
                    }
                }
                break;
            }
        }
    }
    g_mfl.tick(now);

    if (now - g_windowStart >= 5000) {
        if (g_mode == Mode::MflDecoded || g_mode == Mode::OnlyMfl) {
            Serial.printf("[%lu frames/s]\n", (unsigned long)(g_windowFrames / 5));
        }
        g_windowFrames = 0;
        g_windowStart = now;
    }
    if (now - g_lastFrameMs > 5000 && now - g_lastWarnMs > 5000) {
        g_lastWarnMs = now;
        Serial.printf("no frames for %lus @ %lu bit/s - ignition on? right bus/bitrate? "
                      "(OBD D-CAN is silent without a tester)\n",
                      (unsigned long)((now - g_lastFrameMs) / 1000), (unsigned long)g_bitrate);
    }
}
