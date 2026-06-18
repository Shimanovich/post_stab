#ifndef COMMANDER_H
#define COMMANDER_H

#include "stm32f3xx_hal.h"
#include <cstdint>
#include <cstring>

#define MAX_COMMAND_LENGTH   64
#define MAX_CALLBACKS        16

typedef void (*CommandCallback)(char* args);   // args — остаток строки после идентификатора

class Commander {
public:
    Commander(UART_HandleTypeDef* huart, char eol = '\n', bool echo = false);

    // Регистрация команды
    // id     — один символ-идентификатор (например 'T', 'P', 'C')
    // cb     — ваша функция, которая получит указатель на аргументы
    // label  — необязательная строка для справки (можно nullptr)
    void add(char id, CommandCallback cb, const char* label = nullptr);

    // Главный метод — вызывать в основном цикле (неблокирующий)
    void run();

    // Прямой вызов парсера (если данные уже есть в буфере)
    void run(char* user_input);

public:
    UART_HandleTypeDef* huart_;
    char eol_;
    bool echo_;

    CommandCallback callbacks_[MAX_CALLBACKS];
    char            ids_[MAX_CALLBACKS];
    const char*     labels_[MAX_CALLBACKS];
    uint8_t         count_ = 0;

    char    rx_buffer_[MAX_COMMAND_LENGTH];
    uint8_t rx_index_ = 0;

    void processIncomingChar(char ch);
    bool isEndOfLine(char ch) const;
};

#endif // COMMANDER_H
