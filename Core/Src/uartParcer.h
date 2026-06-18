#include <cstdint>
#include <map>
#include <functional>
#include <cstring>

class UartProtocolParser {
public:
    // Тип обработчика: (key, value)
    using MessageHandler = std::function<void(uint16_t key, uint32_t value)>;

    UartProtocolParser() = default;

    // Привязать обработчик к конкретному ключу (типу сообщения)
    void registerHandler(uint16_t key, MessageHandler handler) {
        handlers[key] = handler;
    }

    // Удалить обработчик
    void unregisterHandler(uint16_t key) {
        handlers.erase(key);
    }

    // Установить обработчик для неизвестных ключей (опционально)
    void setDefaultHandler(MessageHandler handler) {
        defaultHandler = handler;
    }

    // Основной метод — подача одного байта из UART (рекомендуется из прерывания)
    void feed(uint8_t byte) {
        switch (state) {
            case State::WAIT_AA:
                if (byte == 0xAA) {
                    state = State::COLLECT;
                    index = 0;
                }
                break;

            case State::COLLECT:
                buffer[index++] = byte;
                if (index == 7) {           // собрали 7 байт после 0xAA
                    processFrame();
                    state = State::WAIT_AA;
                }
                break;
        }
    }

    // Можно также скормить сразу буфер (например после DMA)
    void feedBuffer(const uint8_t* data, size_t len) {
        for (size_t i = 0; i < len; ++i) {
            feed(data[i]);
        }
    }

    // Статистика (опционально)
    uint32_t getValidFrames() const { return validFrames; }
    uint32_t getCrcErrors()   const { return crcErrors; }

private:
    enum class State { WAIT_AA, COLLECT };

    State state = State::WAIT_AA;
    uint8_t buffer[7];          // Key(2) + Value(4) + CRC(1)
    uint8_t index = 0;

    std::map<uint16_t, MessageHandler> handlers;
    MessageHandler defaultHandler = nullptr;

    uint32_t validFrames = 0;
    uint32_t crcErrors   = 0;

    // CRC-8 (полином 0x07, init = 0)
    static uint8_t calculateCrc8(const uint8_t* data, uint8_t len) {
        uint8_t crc = 0x00;
        for (uint8_t i = 0; i < len; ++i) {
            crc ^= data[i];
            for (uint8_t j = 0; j < 8; ++j) {
                if (crc & 0x80)
                    crc = (crc << 1) ^ 0x07;
                else
                    crc <<= 1;
            }
        }
        return crc;
    }

    void processFrame() {
        // Восстанавливаем первые 7 байт кадра для расчёта CRC
        uint8_t temp[7];
        temp[0] = 0xAA;
        std::memcpy(&temp[1], buffer, 6);   // Key + Value

        uint8_t calcCrc = calculateCrc8(temp, 7);
        uint8_t recvCrc = buffer[6];

        if (calcCrc == recvCrc) {
            // Разбираем key и value
            uint16_t key = buffer[0] | (static_cast<uint16_t>(buffer[1]) << 8);
            uint32_t value = buffer[2] |
                             (static_cast<uint32_t>(buffer[3]) << 8) |
                             (static_cast<uint32_t>(buffer[4]) << 16) |
                             (static_cast<uint32_t>(buffer[5]) << 24);

            auto it = handlers.find(key);
            if (it != handlers.end()) {
                it->second(key, value);
            } else if (defaultHandler) {
                defaultHandler(key, value);
            }
            ++validFrames;
        } else {
            ++crcErrors;
        }
    }
};
