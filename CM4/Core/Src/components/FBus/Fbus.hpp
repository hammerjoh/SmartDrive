#pragma once

#include "stm32h7xx_hal.h"
#include <array>
#include <cstdint>

class FBus {
public:
    static constexpr size_t NUM_CHANNELS = 16;
    static constexpr size_t BUFFER_SIZE  = 64;

    struct TelemetryData {
        uint16_t sensor_id;
        uint32_t value;
    };

    explicit FBus(UART_HandleTypeDef* huart);
    ~FBus() = default;

    bool init();
    void update();
    uint16_t getChannel(size_t channelIndex) const;
    void queueTelemetry(uint16_t sensor_id, uint32_t value);
    void handleRxByte(uint8_t byte);

private:
    UART_HandleTypeDef* m_huart;
    std::array<uint16_t, NUM_CHANNELS> m_channels{0};
    uint8_t m_rxBuffer[BUFFER_SIZE]{0};
    size_t m_rxIndex = 0;

    TelemetryData m_nextTelemetry{0, 0};
    bool m_telemetryReady = false;

    void parseFrame(const uint8_t* buffer, size_t length);
    void processChannelFrame(const uint8_t* payload);
    void processSensorPoll(uint8_t physical_id);
    void sendTelemetryResponse(uint8_t physical_id);
    
    uint16_t calculateCRC(const uint8_t* data, size_t length);
};
