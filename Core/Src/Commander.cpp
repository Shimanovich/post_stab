#include "Commander.h"
#include <cstdio>

Commander::Commander(UART_HandleTypeDef* huart, char eol, bool echo)
    : huart_(huart), eol_(eol), echo_(echo)
{
    // Очистка массивов (на всякий случай)
    for (uint8_t i = 0; i < MAX_CALLBACKS; i++) {
        callbacks_[i] = nullptr;
        ids_[i] = 0;
        labels_[i] = nullptr;
    }
}

void Commander::add(char id, CommandCallback cb, const char* label)
{
    if (count_ >= MAX_CALLBACKS || cb == nullptr) return;

    ids_[count_]       = id;
    callbacks_[count_] = cb;
    labels_[count_]    = label;
    count_++;
}

void Commander::run()
{
    // Здесь должен быть ring buffer или прерывание UART,
    // которое вызывает processIncomingChar() при получении байта.
    // Для примера оставлен пустой — реализуйте приём сами.
}

void Commander::run(char* user_input)
{
    if (!user_input || user_input[0] == 0) return;

    char id = user_input[0];

    for (uint8_t i = 0; i < count_; i++) {
        if (ids_[i] == id && callbacks_[i] != nullptr) {
            callbacks_[i](&user_input[1]);   // передаём остаток строки
            return;
        }
    }

    // Неизвестная команда — можно добавить обработку ошибки
    // printf("Unknown command: %c\r\n", id);
}

void Commander::processIncomingChar(char ch)
{
    if (rx_index_ >= MAX_COMMAND_LENGTH - 1) {
        rx_index_ = 0; // переполнение — сбрасываем
        return;
    }

    rx_buffer_[rx_index_++] = ch;

    if (isEndOfLine(ch)) {
        rx_buffer_[rx_index_ - 1] = 0;     // заменяем \n на \0
        run(rx_buffer_);                   // запускаем парсер
        rx_index_ = 0;                     // готовимся к новой команде
    }
}

bool Commander::isEndOfLine(char ch) const
{
    return ch == eol_ || ch == '\r';
}
