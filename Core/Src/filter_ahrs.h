/* components/utilities/filter_ahrs/filter_ahrs.h */
#pragma once


#include <stdint.h>
#include <stdbool.h>

typedef struct {
  char        group_name[5]; /* Имя группы для config_service */
  //cfg_group_t group;         /* Экземпляр группы */
  //cfg_entry_t entries[11];   /* Записи для Config Service */

  uint8_t mode;              /* 0=None, 1=Madgwick, 2=Mahony, 3=Complementary, 4=Classic, 5=EKF */
  bool    use_magnetometer;  /* 1 — магнитометр доступен (норма > 1 мкТл) */
  float   q0, q1, q2, q3;    /* Выходной кватернион */

  float roll, pitch, yaw;    /* Углы Эйлера, градусы */
  float roll_rad, pitch_rad, yaw_rad;
  float heading;             /* Истинный курс с учётом склонения, [0, 360) */
  float declination;         /* Магнитное склонение, градусы */

  float alpha;               /* Коэффициент сглаживания (комплементарный/классический фильтр) */
  float beta;                /* Коэффициент усиления Маджвика */
  float Kp, Ki;              /* Пропорциональный и интегральный коэффициенты Махони */
  float gyroMeasError;       /* Ошибка измерения гироскопа, рад/с */

  float att[4];              /* Состояние комплементарного фильтра (кватернион гироскопа) */
  float eInt[3];             /* Интегральная ошибка Махони */
  float ekf_state[2];        /* Состояние EKF: [roll_rad, pitch_rad] */
  float ekf_cov[4];          /* Ковариационная матрица 2x2 */
  float ekf_Q[4];            /* Матрица шума процесса 2x2 */
  float ekf_R[4];            /* Матрица шума измерений 2x2 */

  float    ax, ay, az;       /* Сырые данные датчиков (внутри компонента) */
  float    gx, gy, gz;       /* рад/с */
  float    mx, my, mz;
  uint32_t last_tick;
} filter_ctx_t;

/* Инициализация контекста фильтра, регистрация в config_service */
void Filter_Init(filter_ctx_t* ctx, const char* group_name);

/* Сброс состояния фильтра без изменения настроек */
void Filter_Reset(filter_ctx_t* ctx);

/* Обновление состояния фильтра (данные магнитометра в мкТл, передавать 0.0f если отсутствует) */
void Filter_Update(filter_ctx_t* ctx, float gx_dps, float gy_dps, float gz_dps, float ax_g, float ay_g, float az_g, float mx_ut, float my_ut,
                   float mz_ut);

/* Получение текущих углов Эйлера */
void Filter_GetEuler(filter_ctx_t* ctx, float* roll, float* pitch, float* yaw);


