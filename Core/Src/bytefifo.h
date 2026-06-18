#include <vector>
#include <cstdint>
#include <cstddef>

class ByteFifo {
private:
    std::vector<uint8_t> buffer;
    size_t head = 0;      // позиция чтения
    size_t tail = 0;      // позиция записи
    size_t count = 0;     // текущее количество байт
    size_t capacity;

public:
    explicit ByteFifo(size_t size)
        : buffer(size), capacity(size) {}

    // Добавить байт. Возвращает true при успехе
    bool push(uint8_t byte) {
        if (count == capacity) {
            return false; // буфер полон
        }
        buffer[tail] = byte;
        tail = (tail + 1) % capacity;
        ++count;
        return true;
    }

    // Извлечь байт. Возвращает true при успехе
    bool pop(uint8_t& byte) {
        if (count == 0) {
            return false; // буфер пуст
        }
        byte = buffer[head];
        head = (head + 1) % capacity;
        --count;
        return true;
    }

    bool is_empty() const { return count == 0; }
    bool is_full()  const { return count == capacity; }
    size_t size()   const { return count; }
    size_t get_capacity() const { return capacity; }

    // Очистить буфер
    void clear() {
        head = tail = count = 0;
    }
};
