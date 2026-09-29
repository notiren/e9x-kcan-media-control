#include "can_bus.h"

#if defined(CAN_BACKEND_MCP2515) && CAN_BACKEND_MCP2515

// ---------------------------------------------------------------- MCP2515 ---
#include <SPI.h>
#include <mcp2515.h>

#ifndef MCP2515_CRYSTAL_MHZ
#define MCP2515_CRYSTAL_MHZ 8
#endif
// Slow SPI: plenty for 100/500 kbit/s CAN and tolerant of the resistor
// divider on SO used with 5 V modules.
#ifndef MCP2515_SPI_HZ
#define MCP2515_SPI_HZ 1000000
#endif

static MCP2515 *s_mcp = nullptr;

static bool mapBitrate(uint32_t bitrate, CAN_SPEED &out) {
    switch (bitrate) {
        case 100000: out = CAN_100KBPS; return true;
        case 125000: out = CAN_125KBPS; return true;
        case 250000: out = CAN_250KBPS; return true;
        case 500000: out = CAN_500KBPS; return true;
        default: return false;
    }
}

bool canBegin(uint32_t bitrate) {
    CAN_SPEED speed;
    if (!mapBitrate(bitrate, speed)) return false;
    const CAN_CLOCK clock = (MCP2515_CRYSTAL_MHZ == 16) ? MCP_16MHZ : MCP_8MHZ;

    if (!s_mcp) {
        // Begin SPI with explicit pins first; the library's own SPI.begin()
        // is then a no-op on ESP32.
        SPI.begin(MCP2515_SCK_PIN, MCP2515_MISO_PIN, MCP2515_MOSI_PIN, MCP2515_CS_PIN);
        s_mcp = new MCP2515(MCP2515_CS_PIN, MCP2515_SPI_HZ);
    }
    if (s_mcp->reset() != MCP2515::ERROR_OK) return false;
    if (s_mcp->setBitrate(speed, clock) != MCP2515::ERROR_OK) return false;
    return s_mcp->setListenOnlyMode() == MCP2515::ERROR_OK;
}

void canEnd() {
    if (s_mcp) s_mcp->reset();
}

bool canRead(CanFrame &frame, uint32_t timeoutMs) {
    if (!s_mcp) return false;
    const uint32_t start = millis();
    struct can_frame f;
    do {
        if (s_mcp->readMessage(&f) == MCP2515::ERROR_OK) {
            frame.extended = f.can_id & CAN_EFF_FLAG;
            frame.rtr = f.can_id & CAN_RTR_FLAG;
            frame.id = f.can_id & (frame.extended ? CAN_EFF_MASK : CAN_SFF_MASK);
            frame.dlc = f.can_dlc > 8 ? 8 : f.can_dlc;
            memcpy(frame.data, f.data, frame.dlc);
            return true;
        }
        if (timeoutMs) delay(1);
    } while (millis() - start < timeoutMs);
    return false;
}

const char *canBackendName() { return "MCP2515"; }

#else

// ------------------------------------------------------------------- TWAI ---
#include "driver/twai.h"

static bool s_installed = false;

bool canBegin(uint32_t bitrate) {
    twai_timing_config_t timing;
    switch (bitrate) {
        case 100000: timing = TWAI_TIMING_CONFIG_100KBITS(); break;
        case 125000: timing = TWAI_TIMING_CONFIG_125KBITS(); break;
        case 250000: timing = TWAI_TIMING_CONFIG_250KBITS(); break;
        case 500000: timing = TWAI_TIMING_CONFIG_500KBITS(); break;
        default: return false;
    }
    canEnd();

    twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_LISTEN_ONLY);
    general.rx_queue_len = 64;
    twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&general, &timing, &filter) != ESP_OK) return false;
    s_installed = true;
    return twai_start() == ESP_OK;
}

void canEnd() {
    if (!s_installed) return;
    twai_stop();
    twai_driver_uninstall();
    s_installed = false;
}

bool canRead(CanFrame &frame, uint32_t timeoutMs) {
    if (!s_installed) return false;
    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(timeoutMs)) != ESP_OK) return false;
    frame.id = msg.identifier;
    frame.extended = msg.extd;
    frame.rtr = msg.rtr;
    frame.dlc = msg.data_length_code > 8 ? 8 : msg.data_length_code;
    memcpy(frame.data, msg.data, frame.dlc);
    return true;
}

const char *canBackendName() { return "TWAI"; }

#endif
