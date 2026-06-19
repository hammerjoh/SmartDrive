#include "FBus.hpp"
#include <cstring>

FBus::FBus(UART_HandleTypeDef* huart) : m_huart(huart) {}

bool FBus::init() {
    if (m_huart == nullptr) return false;

    // FBus Parameter überschreiben (falls CubeMX Standardwerte generiert hat)
    m_huart->Init.BaudRate = 460800;
    m_huart->Init.WordLength = UART_WORDLENGTH_8B;
    m_huart->Init.StopBits = UART_STOPBITS_1;
    m_huart->Init.Parity = UART_PARITY_NONE;
    m_huart->Init.Mode = UART_MODE_TX_RX;
    m_huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    m_huart->Init.OverSampling = UART_OVERSAMPLING_16;
    
    // Invertierung direkt im STM32H7-Register aktivieren (Kein externer Inverter nötig!)
    m_huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT | UART_ADVFEATURE_TXINVERT_INIT;
    m_huart->AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
    m_huart->AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;

    // Single-Wire/Half-Duplex Hardware-Schaltung aktivieren
    if (HAL_HalfDuplex_Init(m_huart) != HAL_OK) {
        return false;
    }

    // Interrupt für den Byte-Empfang im CM4 aktivieren
    __HAL_UART_ENABLE_IT(m_huart, UART_IT_RXNE);
    
    return true;
}

void FBus::handleRxByte(uint8_t byte) {
    if (byte == 0x7E) {
        if (m_rxIndex > 3) {
            parseFrame(m_rxBuffer, m_rxIndex);
        }
        m_rxIndex = 0;
    } else {
        if (m_rxIndex < BUFFER_SIZE) {
            m_rxBuffer[m_rxIndex++] = byte;
        } else {
            m_rxIndex = 0;
        }
    }
}

void FBus::parseFrame(const uint8_t* buffer, size_t length) {
    uint8_t frame_len = buffer[0];
    uint8_t frame_type = buffer[1];

    if (frame_len + 1 > length) return;

    if (frame_type == 0x04) { 
        processChannelFrame(&buffer[2]);
    } 
    else if (frame_type == 0x1B) {
        uint8_t physical_id = buffer[2];
        processSensorPoll(physical_id);
    }
}

void FBus::processChannelFrame(const uint8_t* payload) {
    // SBUS 11-Bit De-Packing
    m_channels[0]  = ((payload[0]    | payload[1] << 8) & 0x07FF);
    m_channels[1]  = ((payload[1] >> 3 | payload[2] << 5) & 0x07FF);
    m_channels[2]  = ((payload[2] >> 6 | payload[3] << 2 | payload[4] << 10) & 0x07FF);
    m_channels[3]  = ((payload[4] >> 1 | payload[5] << 7) & 0x07FF);
    m_channels[4]  = ((payload[5] >> 4 | payload[6] << 4) & 0x07FF);
    m_channels[5]  = ((payload[6] >> 7 | payload[7] << 1 | payload[8] << 9) & 0x07FF);
    m_channels[6]  = ((payload[8] >> 2 | payload[9] << 6) & 0x07FF);
    m_channels[7]  = ((payload[9] >> 5 | payload[10] << 3) & 0x07FF);
    
    m_channels[8]  = ((payload[11]   | payload[12] << 8) & 0x07FF);
    m_channels[9]  = ((payload[12] >> 3 | payload[13] << 5) & 0x07FF);
    m_channels[10] = ((payload[13] >> 6 | payload[14] << 2 | payload[15] << 10) & 0x07FF);
    m_channels[11] = ((payload[15] >> 1 | payload[16] << 7) & 0x07FF);
    m_channels[12] = ((payload[16] >> 4 | payload[17] << 4) & 0x07FF);
    m_channels[13] = ((payload[17] >> 7 | payload[18] << 1 | payload[19] << 9) & 0x07FF);
    m_channels[14] = ((payload[19] >> 2 | payload[20] << 6) & 0x07FF);
    m_channels[15] = ((payload[20] >> 5 | payload[21] << 3) & 0x07FF);
}

void FBus::processSensorPoll(uint8_t physical_id) {
    if (m_telemetryReady) {
        sendTelemetryResponse(physical_id);
    }
}

void FBus::sendTelemetryResponse(uint8_t physical_id) {
    uint8_t txBuffer[11];
    txBuffer[0] = 0x7E; 
    txBuffer[1] = 0x08; 
    txBuffer[2] = 0x10; 
    
    txBuffer[3] = m_nextTelemetry.sensor_id & 0xFF;
    txBuffer[4] = (m_nextTelemetry.sensor_id >> 8) & 0xFF;

    std::memcpy(&txBuffer[5], &m_nextTelemetry.value, sizeof(uint32_t));

    uint16_t crc = calculateCRC(&txBuffer[1], 8);
    txBuffer[9] = crc & 0xFF;
    txBuffer[10] = 0x7E; 

    // Blokierendes Senden über HAL (Sicher für zeitkritische Antworten im Telemetrie-Slot)
    HAL_UART_Transmit(m_huart, txBuffer, 11, 10);
    
    m_telemetryReady = false; 
}

uint16_t FBus::getChannel(size_t channelIndex) const {
    if (channelIndex >= NUM_CHANNELS) return 0;
    return m_channels[channelIndex];
}

void FBus::queueTelemetry(uint16_t sensor_id, uint32_t value) {
    if (!m_telemetryReady) {
        m_nextTelemetry.sensor_id = sensor_id;
        m_nextTelemetry.value = value;
        m_telemetryReady = true;
    }
}

uint16_t FBus::calculateCRC(const uint8_t* data, size_t length) {
    uint16_t sum = 0;
    for (size_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return 0xFF - ((sum & 0xFF) + (sum >> 8));
}
