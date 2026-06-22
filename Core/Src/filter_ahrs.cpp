/* components/utilities/filter_ahrs/filter_ahrs.c */
//#ifdef FILTER_AHRS_ENABLED
#include "filter_ahrs.h"

#include <math.h>
#include <string.h>
#include <stm32f3xx_hal.h>

//#include "common.h"

/* ********************************************************** */
#define M_PI                  3.14159265358979323846
#define DEG_TO_RAD            (M_PI / 180.0f)
#define RAD_TO_DEG            (180.0f / M_PI)

/* Диапазон доверия акселерометру: при отклонении нормы от 1g более чем на
 * ACCEL_TRUST_BAND доверие снижается до ACCEL_TRUST_MIN
 * Защита от ошибок коррекции при маневрировании (2–5g) */
#define AHRS_ACCEL_TRUST_BAND 0.15f /* ±15% от 1g = [0.85g, 1.15g] — полное доверие */
#define AHRS_ACCEL_TRUST_MIN  0.0f  /* 0 = полное отключение при сильном манёвре */

/* Минимальная норма вектора магнитного поля (мкТл)
 * Земное поле ~25–65 мкТл. Порог защищает от искажений вблизи электроники БПЛА */
#define AHRS_MAG_MIN_UT       15.0f

/* Ограничение тангажа для защиты от gimbal lock в режимах None и Classic
 * Madgwick/Mahony/Complementary работают в кватернионах и не имеют этой проблемы */
#define AHRS_PITCH_LIMIT_RAD  (88.0f * DEG_TO_RAD)

/* Ограничение интегральной ошибки Махони (anti-windup)
 * Предотвращает накопление ошибки при длительном полёте с постоянным смещением */
#define AHRS_EINT_LIMIT       0.5f

/* Максимальный допустимый dt. При превышении — сброс интегрального состояния
 * и замещение dt номинальным значением (защита от зависания/перезапуска) */
#define AHRS_DT_MAX           0.5f
#define AHRS_DT_NOMINAL       0.01f /* Номинальный dt при превышении AHRS_DT_MAX */

/* ********************************************************** */
/* Расширенный фильтр Калмана (2D: крен + тангаж, F=I, H=I) */
static void ekf_predict(filter_ctx_t* ctx, float ctrl_roll, float ctrl_pitch, float dt) {
  ctx->ekf_state[0] += ctrl_roll * dt;
  ctx->ekf_state[1] += ctrl_pitch * dt;
  /* P = F*P*F^T + Q = P + Q (F — единичная матрица) */
  ctx->ekf_cov[0] += ctx->ekf_Q[0];
  ctx->ekf_cov[1] += ctx->ekf_Q[1];
  ctx->ekf_cov[2] += ctx->ekf_Q[2];
  ctx->ekf_cov[3] += ctx->ekf_Q[3];
}

static void ekf_update(filter_ctx_t* ctx, float meas_roll, float meas_pitch) {
  float y0  = meas_roll - ctx->ekf_state[0];  /* Невязка крена */
  float y1  = meas_pitch - ctx->ekf_state[1]; /* Невязка тангажа */

  /* S = P + R (H — единичная матрица) */
  float S00 = ctx->ekf_cov[0] + ctx->ekf_R[0];
  float S01 = ctx->ekf_cov[1] + ctx->ekf_R[1];
  float S10 = ctx->ekf_cov[2] + ctx->ekf_R[2];
  float S11 = ctx->ekf_cov[3] + ctx->ekf_R[3];

  float det = S00 * S11 - S01 * S10;
  if(fabsf(det) < 1e-9f) return; /* Защита от вырожденной матрицы */

  float Si00 = S11 / det, Si01 = -S01 / det;
  float Si10 = -S10 / det, Si11 = S00 / det;

  float K00 = ctx->ekf_cov[0] * Si00 + ctx->ekf_cov[1] * Si10;
  float K01 = ctx->ekf_cov[0] * Si01 + ctx->ekf_cov[1] * Si11;
  float K10 = ctx->ekf_cov[2] * Si00 + ctx->ekf_cov[3] * Si10;
  float K11 = ctx->ekf_cov[2] * Si01 + ctx->ekf_cov[3] * Si11;

  ctx->ekf_state[0] += K00 * y0 + K01 * y1;
  ctx->ekf_state[1] += K10 * y0 + K11 * y1;

  /* P = (I - K)*P */
  float IK00 = 1.0f - K00, IK01 = -K01, IK10 = -K10, IK11 = 1.0f - K11;
  float P00       = IK00 * ctx->ekf_cov[0] + IK01 * ctx->ekf_cov[2];
  float P01       = IK00 * ctx->ekf_cov[1] + IK01 * ctx->ekf_cov[3];
  float P10       = IK10 * ctx->ekf_cov[0] + IK11 * ctx->ekf_cov[2];
  float P11       = IK10 * ctx->ekf_cov[1] + IK11 * ctx->ekf_cov[3];
  ctx->ekf_cov[0] = P00;
  ctx->ekf_cov[1] = P01;
  ctx->ekf_cov[2] = P10;
  ctx->ekf_cov[3] = P11;

  /* Симметризация матрицы ковариации — защита от накопления численной ошибки */
  ctx->ekf_cov[1] = ctx->ekf_cov[2] = (ctx->ekf_cov[1] + ctx->ekf_cov[2]) * 0.5f;
}

/* ********************************************************** */
/* Вычисление весового коэффициента доверия акселерометру
 * Возвращает 1.0 при норме ~1g, плавно снижается к AHRS_ACCEL_TRUST_MIN
 * при отклонении > AHRS_ACCEL_TRUST_BAND. Защита от ошибок при манёврах */
static float accel_trust(float ax_raw, float ay_raw, float az_raw) {
  float norm      = sqrtf(ax_raw * ax_raw + ay_raw * ay_raw + az_raw * az_raw);
  float deviation = fabsf(norm - 1.0f); /* Отклонение от 1g */
  if(deviation <= AHRS_ACCEL_TRUST_BAND) return 1.0f;
  float t = (deviation - AHRS_ACCEL_TRUST_BAND) / AHRS_ACCEL_TRUST_BAND;
  t       = fminf(t, 1.0f);
  return AHRS_ACCEL_TRUST_MIN + (1.0f - AHRS_ACCEL_TRUST_MIN) * (1.0f - t);
}

static void quaternion_normalize(float* q0, float* q1, float* q2, float* q3) {
  float norm = sqrtf((*q0) * (*q0) + (*q1) * (*q1) + (*q2) * (*q2) + (*q3) * (*q3));
  if(norm < 1e-10f) {
    *q0 = 1.0f;
    *q1 = 0.0f;
    *q2 = 0.0f;
    *q3 = 0.0f;
    return;
  }
  norm = 1.0f / norm;
  *q0 *= norm;
  *q1 *= norm;
  *q2 *= norm;
  *q3 *= norm;
}

static void quaternionToEuler(filter_ctx_t* ctx) {
  /* Крен (вращение вокруг X) */
  float sinr     = 2.0f * (ctx->q0 * ctx->q1 + ctx->q2 * ctx->q3);
  float cosr     = 1.0f - 2.0f * (ctx->q1 * ctx->q1 + ctx->q2 * ctx->q2);
  ctx->roll_rad  = atan2f(sinr, cosr);
  ctx->roll      = ctx->roll_rad * RAD_TO_DEG;

  /* Тангаж (вращение вокруг Y) — защита от asinf(±1) → ±π/2 */
  float sinp     = 2.0f * (ctx->q0 * ctx->q2 - ctx->q3 * ctx->q1);
  sinp           = fmaxf(-1.0f, fminf(1.0f, sinp)); /* Зажим в [-1, 1] */
  ctx->pitch_rad = asinf(sinp);
  ctx->pitch     = ctx->pitch_rad * RAD_TO_DEG;

  /* Рыскание (вращение вокруг Z), нормализация в [0, 360) */
  float siny     = 2.0f * (ctx->q0 * ctx->q3 + ctx->q1 * ctx->q2);
  float cosy     = 1.0f - 2.0f * (ctx->q2 * ctx->q2 + ctx->q3 * ctx->q3);
  ctx->yaw_rad   = atan2f(siny, cosy);
  ctx->yaw       = fmodf(ctx->yaw_rad * RAD_TO_DEG + 360.0f, 360.0f);
  ctx->yaw_rad   = ctx->yaw * DEG_TO_RAD;

  /* Истинный курс с учётом магнитного склонения */
  ctx->heading   = fmodf(ctx->yaw - ctx->declination + 360.0f, 360.0f);
}

/* Рыскание по компенсированному магнитометру (для Classic, EKF, None)
 * Применяется только при достаточной норме компенсированного вектора */
static void tiltCompensatedYaw(filter_ctx_t* ctx) {
  float mag_x = ctx->mx * cosf(ctx->pitch_rad) + ctx->mz * sinf(ctx->pitch_rad);
  float mag_y =
      ctx->mx * sinf(ctx->roll_rad) * sinf(ctx->pitch_rad) + ctx->my * cosf(ctx->roll_rad) - ctx->mz * sinf(ctx->roll_rad) * cosf(ctx->pitch_rad);

  /* Проверка нормы компенсированного вектора — защита от atan2f(0,0) */
  if(sqrtf(mag_x * mag_x + mag_y * mag_y) < 1e-6f) return;

  ctx->yaw_rad = atan2f(mag_y, mag_x);
  ctx->yaw     = fmodf(ctx->yaw_rad * RAD_TO_DEG + 360.0f, 360.0f);
  ctx->yaw_rad = ctx->yaw * DEG_TO_RAD;
  ctx->heading = fmodf(ctx->yaw - ctx->declination + 360.0f, 360.0f);
}

/* ********************************************************** */
/* Алгоритмы фильтрации */
static void madgwickUpdate(filter_ctx_t* ctx, float dt) {
  float norm;
  float hx, hy, _2bx, _2bz;
  float s0, s1, s2, s3;
  float qDot0, qDot1, qDot2, qDot3;

  float _2q0 = 2.0f * ctx->q0, _2q1 = 2.0f * ctx->q1;
  float _2q2 = 2.0f * ctx->q2, _2q3 = 2.0f * ctx->q3;
  float _2q0q2 = 2.0f * ctx->q0 * ctx->q2, _2q2q3 = 2.0f * ctx->q2 * ctx->q3;
  float q0q0 = ctx->q0 * ctx->q0, q0q1 = ctx->q0 * ctx->q1, q0q2 = ctx->q0 * ctx->q2, q0q3 = ctx->q0 * ctx->q3;
  float q1q1 = ctx->q1 * ctx->q1, q1q2 = ctx->q1 * ctx->q2, q1q3 = ctx->q1 * ctx->q3;
  float q2q2 = ctx->q2 * ctx->q2, q2q3 = ctx->q2 * ctx->q3, q3q3 = ctx->q3 * ctx->q3;

  float ax = ctx->ax, ay = ctx->ay, az = ctx->az;
  float mx = ctx->mx, my = ctx->my, mz = ctx->mz;

  /* Доверие акселерометру: снижается при манёврах (норма ≠ 1g) */
  float trust = accel_trust(ax, ay, az);

  /* Нормализация акселерометра */
  norm        = sqrtf(ax * ax + ay * ay + az * az);
  if(norm < 1e-10f) return;
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  if(ctx->use_magnetometer) {
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if(norm > 1e-10f) {
      norm = 1.0f / norm;
      mx *= norm;
      my *= norm;
      mz *= norm;
    }

    float _2q0mx = 2.0f * ctx->q0 * mx, _2q0my = 2.0f * ctx->q0 * my;
    float _2q0mz = 2.0f * ctx->q0 * mz, _2q1mx = 2.0f * ctx->q1 * mx;
    hx         = mx * q0q0 - _2q0my * ctx->q3 + _2q0mz * ctx->q2 + mx * q1q1 + _2q1 * my * ctx->q2 + _2q1 * mz * ctx->q3 - mx * q2q2 - mx * q3q3;
    hy         = _2q0mx * ctx->q3 + my * q0q0 - _2q0mz * ctx->q1 + _2q1mx * ctx->q2 - my * q1q1 + my * q2q2 + _2q2 * mz * ctx->q3 - my * q3q3;
    _2bx       = sqrtf(hx * hx + hy * hy);
    _2bz       = -_2q0mx * ctx->q2 + _2q0my * ctx->q1 + mz * q0q0 + _2q1mx * ctx->q3 - mz * q1q1 + _2q2 * my * ctx->q3 - mz * q2q2 + mz * q3q3;
    float _4bx = 2.0f * _2bx, _4bz = 2.0f * _2bz;

    /* Шаг градиентного спуска (9-DOF) */
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) -
         _2bz * ctx->q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (-_2bx * ctx->q3 + _2bz * ctx->q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         _2bx * ctx->q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * ctx->q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) +
         _2bz * ctx->q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (_2bx * ctx->q2 + _2bz * ctx->q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         (_2bx * ctx->q3 - _4bz * ctx->q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * ctx->q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) +
         (-_4bx * ctx->q2 - _2bz * ctx->q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (_2bx * ctx->q1 + _2bz * ctx->q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         (_2bx * ctx->q0 - _4bz * ctx->q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) +
         (-_4bx * ctx->q3 + _2bz * ctx->q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (-_2bx * ctx->q0 + _2bz * ctx->q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         _2bx * ctx->q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
  } else {
    /* Шаг градиентного спуска (6-DOF, только акселерометр) */
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * ctx->q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * ctx->q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay);
  }

  /* Нормализация шага */
  norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
  if(norm < 1e-10f) return;
  norm = 1.0f / norm;
  s0 *= norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;

  /* Эффективный beta снижается при ненадёжном акселерометре */
  float beta_eff = ctx->beta * trust;

  qDot0          = 0.5f * (-ctx->q1 * ctx->gx - ctx->q2 * ctx->gy - ctx->q3 * ctx->gz) - beta_eff * s0;
  qDot1          = 0.5f * (ctx->q0 * ctx->gx + ctx->q2 * ctx->gz - ctx->q3 * ctx->gy) - beta_eff * s1;
  qDot2          = 0.5f * (ctx->q0 * ctx->gy - ctx->q1 * ctx->gz + ctx->q3 * ctx->gx) - beta_eff * s2;
  qDot3          = 0.5f * (ctx->q0 * ctx->gz + ctx->q1 * ctx->gy - ctx->q2 * ctx->gx) - beta_eff * s3;

  ctx->q0 += qDot0 * dt;
  ctx->q1 += qDot1 * dt;
  ctx->q2 += qDot2 * dt;
  ctx->q3 += qDot3 * dt;
  quaternion_normalize(&ctx->q0, &ctx->q1, &ctx->q2, &ctx->q3);

  quaternionToEuler(ctx);
}

static void mahonyUpdate(filter_ctx_t* ctx, float dt) {
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  float q0q0 = ctx->q0 * ctx->q0, q0q1 = ctx->q0 * ctx->q1, q0q2 = ctx->q0 * ctx->q2, q0q3 = ctx->q0 * ctx->q3;
  float q1q1 = ctx->q1 * ctx->q1, q1q2 = ctx->q1 * ctx->q2, q1q3 = ctx->q1 * ctx->q3;
  float q2q2 = ctx->q2 * ctx->q2, q2q3 = ctx->q2 * ctx->q3, q3q3 = ctx->q3 * ctx->q3;

  float ax = ctx->ax, ay = ctx->ay, az = ctx->az;
  float mx = ctx->mx, my = ctx->my, mz = ctx->mz;

  /* Доверие акселерометру: снижается при манёврах */
  float trust = accel_trust(ax, ay, az);

  norm        = sqrtf(ax * ax + ay * ay + az * az);
  if(norm < 1e-10f) return;
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  if(ctx->use_magnetometer) {
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if(norm > 1e-10f) {
      norm = 1.0f / norm;
      mx *= norm;
      my *= norm;
      mz *= norm;
    }

    hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3) + 2.0f * mz * (q1q3 + q0q2);
    hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3) + 2.0f * mz * (q2q3 - q0q1);
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
    wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
    wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
    wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);
  } else wx = wy = wz = 0.0f;


  vx = 2.0f * (q1q3 - q0q2);
  vy = 2.0f * (q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;

  /* Ошибка = векторное произведение измеренного и расчётного направлений.
   * Часть от акселерометра масштабируется по trust — защита при манёврах. */
  ex = trust * (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = trust * (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = trust * (ax * vy - ay * vx) + (mx * wy - my * wx);

  /* Интегральная ошибка с ограничением (anti-windup) */
  if(ctx->Ki > 0.0f) {
    ctx->eInt[0] = fmaxf(-AHRS_EINT_LIMIT, fminf(AHRS_EINT_LIMIT, ctx->eInt[0] + ex));
    ctx->eInt[1] = fmaxf(-AHRS_EINT_LIMIT, fminf(AHRS_EINT_LIMIT, ctx->eInt[1] + ey));
    ctx->eInt[2] = fmaxf(-AHRS_EINT_LIMIT, fminf(AHRS_EINT_LIMIT, ctx->eInt[2] + ez));
  } else {
    ctx->eInt[0] = 0.0f;
    ctx->eInt[1] = 0.0f;
    ctx->eInt[2] = 0.0f;
  }

  float gx = ctx->gx + ctx->Kp * ex + ctx->Ki * ctx->eInt[0];
  float gy = ctx->gy + ctx->Kp * ey + ctx->Ki * ctx->eInt[1];
  float gz = ctx->gz + ctx->Kp * ez + ctx->Ki * ctx->eInt[2];

  pa       = ctx->q1;
  pb       = ctx->q2;
  pc       = ctx->q3;
  ctx->q0  = ctx->q0 + (-ctx->q1 * gx - ctx->q2 * gy - ctx->q3 * gz) * (0.5f * dt);
  ctx->q1  = pa + (ctx->q0 * gx + pb * gz - pc * gy) * (0.5f * dt);
  ctx->q2  = pb + (ctx->q0 * gy - pa * gz + pc * gx) * (0.5f * dt);
  ctx->q3  = pc + (ctx->q0 * gz + pa * gy - pb * gx) * (0.5f * dt);
  quaternion_normalize(&ctx->q0, &ctx->q1, &ctx->q2, &ctx->q3);

  quaternionToEuler(ctx);
}

static void complementaryUpdate(filter_ctx_t* ctx, float dt) {
  float rollAcc  = atan2f(ctx->ay, ctx->az);
  float pitchAcc = atan2f(-ctx->ax, sqrtf(ctx->ay * ctx->ay + ctx->az * ctx->az));

  float halfDt   = dt * 0.5f;
  float cosTheta = cosf(rollAcc), sinTheta = sinf(rollAcc);
  float cosPhi = cosf(pitchAcc), sinPhi = sinf(pitchAcc);
  float cosHalfTheta = cosf(rollAcc * 0.5f), sinHalfTheta = sinf(rollAcc * 0.5f);
  float cosHalfPhi = cosf(pitchAcc * 0.5f), sinHalfPhi = sinf(pitchAcc * 0.5f);

  /* Нормализация att перед интегрированием — предотвращает дрейф единичного кватерниона */
  quaternion_normalize(&ctx->att[0], &ctx->att[1], &ctx->att[2], &ctx->att[3]);

  /* Интегрирование кватерниона от гироскопа */
  float qDot0 = -ctx->gx * ctx->att[1] - ctx->gy * ctx->att[2] - ctx->gz * ctx->att[3];
  float qDot1 = ctx->gx * ctx->att[0] - ctx->gy * ctx->att[3] + ctx->gz * ctx->att[2];
  float qDot2 = ctx->gx * ctx->att[3] + ctx->gy * ctx->att[0] - ctx->gz * ctx->att[1];
  float qDot3 = -ctx->gx * ctx->att[2] + ctx->gy * ctx->att[1] + ctx->gz * ctx->att[0];
  ctx->att[0] += qDot0 * halfDt;
  ctx->att[1] += qDot1 * halfDt;
  ctx->att[2] += qDot2 * halfDt;
  ctx->att[3] += qDot3 * halfDt;

  /* Рыскание: от магнитометра или из текущего кватерниона (6-DOF) */
  float yaw;
  if(ctx->use_magnetometer) {
    float bx = ctx->mx * cosTheta + ctx->my * sinTheta * sinPhi + ctx->mz * sinTheta * cosPhi;
    float by = ctx->my * cosPhi - ctx->mz * sinPhi;
    if(sqrtf(bx * bx + by * by) < 1e-6f) {
      /* Магнитный вектор вырожден — берём рыскание из текущего кватерниона */
      float siny = 2.0f * (ctx->q0 * ctx->q3 + ctx->q1 * ctx->q2);
      float cosy = 1.0f - 2.0f * (ctx->q2 * ctx->q2 + ctx->q3 * ctx->q3);
      yaw        = atan2f(siny, cosy);
    } else {
      yaw = atan2f(-by, bx);
    }
  } else {
    float siny = 2.0f * (ctx->q0 * ctx->q3 + ctx->q1 * ctx->q2);
    float cosy = 1.0f - 2.0f * (ctx->q2 * ctx->q2 + ctx->q3 * ctx->q3);
    yaw        = atan2f(siny, cosy);
  }

  float cosHalfYaw = cosf(yaw * 0.5f), sinHalfYaw = sinf(yaw * 0.5f);
  float qam[4];
  qam[0]  = cosHalfPhi * cosHalfTheta * cosHalfYaw + sinHalfPhi * sinHalfTheta * sinHalfYaw;
  qam[1]  = sinHalfPhi * cosHalfTheta * cosHalfYaw - cosHalfPhi * sinHalfTheta * sinHalfYaw;
  qam[2]  = cosHalfPhi * sinHalfTheta * cosHalfYaw + sinHalfPhi * cosHalfTheta * sinHalfYaw;
  qam[3]  = cosHalfPhi * cosHalfTheta * sinHalfYaw - sinHalfYaw * sinHalfTheta * cosHalfYaw;

  ctx->q0 = ctx->alpha * ctx->att[0] + (1.0f - ctx->alpha) * qam[0];
  ctx->q1 = ctx->alpha * ctx->att[1] + (1.0f - ctx->alpha) * qam[1];
  ctx->q2 = ctx->alpha * ctx->att[2] + (1.0f - ctx->alpha) * qam[2];
  ctx->q3 = ctx->alpha * ctx->att[3] + (1.0f - ctx->alpha) * qam[3];
  quaternion_normalize(&ctx->q0, &ctx->q1, &ctx->q2, &ctx->q3);

  quaternionToEuler(ctx);
}

static void classicUpdate(filter_ctx_t* ctx, float dt) {
  float rollDen  = sqrtf(ctx->ax * ctx->ax + ctx->az * ctx->az);
  float pitchDen = sqrtf(ctx->ay * ctx->ay + ctx->az * ctx->az);
  if(rollDen < 1e-6f || pitchDen < 1e-6f) return;

  float accRoll  = atanf(-ctx->ay / rollDen);
  float accPitch = -atanf(-ctx->ax / pitchDen);

  /* Коэффициент alpha зависит от частоты вызова
   * Для изменения постоянной времени при другой частоте пересчитать alpha */
  (void) dt;
  ctx->pitch     = ctx->alpha * ctx->pitch + (1.0f - ctx->alpha) * accPitch * RAD_TO_DEG;
  ctx->roll      = ctx->alpha * ctx->roll + (1.0f - ctx->alpha) * accRoll * RAD_TO_DEG;
  ctx->pitch_rad = ctx->pitch * DEG_TO_RAD;
  ctx->roll_rad  = ctx->roll * DEG_TO_RAD;
}

/* Интеграция угловых скоростей гироскопа в углы Эйлера (для None и Classic).
 * ВНИМАНИЕ: содержит gimbal lock при |pitch| → 90°.
 * Тангаж ограничивается AHRS_PITCH_LIMIT_RAD для предотвращения NaN/Inf. */
static void updateEulerAngles(filter_ctx_t* ctx, float dt) {
  /* Ограничение тангажа для защиты от gimbal lock (tan → ∞ при ±90°) */
  float pitch_safe = fmaxf(-AHRS_PITCH_LIMIT_RAD, fminf(AHRS_PITCH_LIMIT_RAD, ctx->pitch_rad));

  float sinPHI     = sinf(ctx->roll_rad);
  float cosPHI     = cosf(ctx->roll_rad);
  float sinTHETA   = sinf(pitch_safe);
  float cosTHETA   = cosf(pitch_safe); /* Гарантированно ≠ 0 при ограниченном тангаже */
  float tanTHETA   = sinTHETA / cosTHETA;

  float rollRate   = ctx->gy + sinPHI * tanTHETA * ctx->gx + cosPHI * tanTHETA * ctx->gz;
  float pitchRate  = cosPHI * ctx->gx - sinPHI * ctx->gz;
  float yawRate    = (sinPHI * ctx->gx + cosPHI * ctx->gz) / cosTHETA;

  ctx->roll_rad += rollRate * dt;
  ctx->pitch_rad += pitchRate * dt;

  /* Зажим тангажа в безопасный диапазон после интегрирования */
  ctx->pitch_rad = fmaxf(-AHRS_PITCH_LIMIT_RAD, fminf(AHRS_PITCH_LIMIT_RAD, ctx->pitch_rad));

  ctx->yaw_rad += yawRate * dt;
  ctx->roll    = ctx->roll_rad * RAD_TO_DEG;
  ctx->pitch   = ctx->pitch_rad * RAD_TO_DEG;
  ctx->yaw     = fmodf(ctx->yaw_rad * RAD_TO_DEG + 360.0f, 360.0f);
  ctx->heading = fmodf(ctx->yaw - ctx->declination + 360.0f, 360.0f);
}

static void extendedKalmanUpdate(filter_ctx_t* ctx, float dt) {
  float accRoll      = atan2f(ctx->ay, ctx->az);
  float accPitch     = atan2f(-ctx->ax, sqrtf(ctx->ay * ctx->ay + ctx->az * ctx->az));

  /* Масштабирование шума измерений в зависимости от манёвра */
  float trust        = accel_trust(ctx->ax, ctx->ay, ctx->az);
  float r_scale      = (trust < 0.5f) ? (1.0f / (trust + 0.01f)) : 1.0f; /* ↑R при недостоверном accel */
  float ekf_R_saved0 = ctx->ekf_R[0], ekf_R_saved3 = ctx->ekf_R[3];
  ctx->ekf_R[0] *= r_scale;
  ctx->ekf_R[3] *= r_scale;

  ekf_predict(ctx, ctx->gx, ctx->gy, dt);
  ekf_update(ctx, accRoll, accPitch);

  ctx->ekf_R[0] = ekf_R_saved0;
  ctx->ekf_R[3] = ekf_R_saved3;

  /* Оборачивание углов состояния в [-π, π] */
  while(ctx->ekf_state[0] > M_PI) ctx->ekf_state[0] -= 2.0f * M_PI;
  while(ctx->ekf_state[0] < -M_PI) ctx->ekf_state[0] += 2.0f * M_PI;
  while(ctx->ekf_state[1] > M_PI) ctx->ekf_state[1] -= 2.0f * M_PI;
  while(ctx->ekf_state[1] < -M_PI) ctx->ekf_state[1] += 2.0f * M_PI;

  ctx->roll_rad  = ctx->ekf_state[0];
  ctx->pitch_rad = ctx->ekf_state[1];
  ctx->roll      = ctx->roll_rad * RAD_TO_DEG;
  ctx->pitch     = ctx->pitch_rad * RAD_TO_DEG;
}

/* ********************************************************** */
/* Колбэки Config Service */
static void CB_ResetConfig(void* ctx) {
  filter_ctx_t* filter_ctx  = (filter_ctx_t*) ctx;
  filter_ctx->mode          = 1;                                              /* Madgwick по умолчанию для БПЛА */
  filter_ctx->alpha         = 0.98f;
  filter_ctx->gyroMeasError = 5.0f * DEG_TO_RAD;                              /* Типичный bias гироскопа ~1–5°/с */
  filter_ctx->beta          = sqrtf(3.0f / 4.0f) * filter_ctx->gyroMeasError; /* ≈ 0.075 рад/с */
  filter_ctx->Kp            = 2.0f;
  filter_ctx->Ki            = 0.005f;                                         /* Малая интегральная составляющая */
  filter_ctx->declination   = 0.0f;

}

//static bool CB_GetConfig(void* ctx, JSON_Context* json_ctx, JSON* payload) {
//  if(!json_ctx || !payload) {
//    SC_LOG_E("Filter AHRS CB GetConfig", "Invalid Params");
//    return false;
//  }
//  filter_ctx_t* filter_ctx = (filter_ctx_t*) ctx;
//  char          key[CFG_KEY_MAX_LEN];
//  snprintf(key, sizeof(key), "%s.Mode", filter_ctx->group_name);
//  if(!JSON_AddNumberToObject(json_ctx, payload, key, filter_ctx->mode)) return false;
//  snprintf(key, sizeof(key), "%s.KA", filter_ctx->group_name);
//  if(!JSON_AddNumberToObject(json_ctx, payload, key, filter_ctx->alpha)) return false;
//  snprintf(key, sizeof(key), "%s.KB", filter_ctx->group_name);
//  if(!JSON_AddNumberToObject(json_ctx, payload, key, filter_ctx->beta)) return false;
//  snprintf(key, sizeof(key), "%s.KP", filter_ctx->group_name);
//  if(!JSON_AddNumberToObject(json_ctx, payload, key, filter_ctx->Kp)) return false;
//  snprintf(key, sizeof(key), "%s.KI", filter_ctx->group_name);
//  if(!JSON_AddNumberToObject(json_ctx, payload, key, filter_ctx->Ki)) return false;
//  snprintf(key, sizeof(key), "%s.GME", filter_ctx->group_name);
//  if(!JSON_AddNumberToObject(json_ctx, payload, key, filter_ctx->gyroMeasError)) return false;
//  snprintf(key, sizeof(key), "%s.Decl", filter_ctx->group_name);
//  if(!JSON_AddNumberToObject(json_ctx, payload, key, filter_ctx->declination)) return false;
//  return true;
//}

/* ********************************************************** */
void Filter_Reset(filter_ctx_t* ctx) {
  if(!ctx) return;
  ctx->q0           = 1.0f;
  ctx->q1           = 0.0f;
  ctx->q2           = 0.0f;
  ctx->q3           = 0.0f;
  ctx->att[0]       = 1.0f;
  ctx->att[1]       = 0.0f;
  ctx->att[2]       = 0.0f;
  ctx->att[3]       = 0.0f;
  ctx->roll         = 0.0f;
  ctx->pitch        = 0.0f;
  ctx->yaw          = 0.0f;
  ctx->heading      = 0.0f;
  ctx->roll_rad     = 0.0f;
  ctx->pitch_rad    = 0.0f;
  ctx->yaw_rad      = 0.0f;
  ctx->eInt[0]      = 0.0f;
  ctx->eInt[1]      = 0.0f;
  ctx->eInt[2]      = 0.0f;
  ctx->ekf_state[0] = 0.0f;
  ctx->ekf_state[1] = 0.0f;
  ctx->ekf_cov[0]   = 0.01f;
  ctx->ekf_cov[1]   = 0.0f;
  ctx->ekf_cov[2]   = 0.0f;
  ctx->ekf_cov[3]   = 0.01f;
  ctx->last_tick    = HAL_GetTick();
 // SC_LOG_I("Filter AHRS Reset", "Group: '%s'", ctx->group_name);
}

void Filter_Update(filter_ctx_t* ctx, float gx_dps, float gy_dps, float gz_dps, float ax_g, float ay_g, float az_g, float mx_ut, float my_ut,
                   float mz_ut) {
  if(!ctx) return;

  /* Определение наличия магнитометра по норме вектора.
   * Порог 15 мкТл защищает от помех двигателей БПЛА (земное поле ~25–65 мкТл). */
  float oldMagValue     = sqrtf(ctx->mx * ctx->mx + ctx->my * ctx->my + ctx->mz * ctx->mz);
  float newMagValue     = sqrtf(mx_ut * mx_ut + my_ut * my_ut + mz_ut * mz_ut);
  ctx->use_magnetometer = (newMagValue > AHRS_MAG_MIN_UT && newMagValue != oldMagValue) ? true : false;

  /* Расчёт интервала времени */
  uint32_t  now        = HAL_GetTick();
  float      dt         = (float) (now - ctx->last_tick) * 1000.0 / 1000.0f;
  ctx->last_tick        = now;

  if(dt <= 0.0f) return;

  if(dt > AHRS_DT_MAX) {
    /* Большой разрыв — полный сброс состояния, чтобы не интегрировать устаревшие данные */
   // SC_LOG_W("Filter AHRS", "dt=%.2fs > %.2fs, resetting state", (double) dt, (double) AHRS_DT_MAX);
    ctx->q0           = 1.0f;
    ctx->q1           = 0.0f;
    ctx->q2           = 0.0f;
    ctx->q3           = 0.0f;
    ctx->att[0]       = 1.0f;
    ctx->att[1]       = 0.0f;
    ctx->att[2]       = 0.0f;
    ctx->att[3]       = 0.0f;
    ctx->eInt[0]      = 0.0f;
    ctx->eInt[1]      = 0.0f;
    ctx->eInt[2]      = 0.0f;
    ctx->ekf_state[0] = 0.0f;
    ctx->ekf_state[1] = 0.0f;
    ctx->ekf_cov[0]   = 0.01f;
    ctx->ekf_cov[1]   = 0.0f;
    ctx->ekf_cov[2]   = 0.0f;
    ctx->ekf_cov[3]   = 0.01f;
    dt                = AHRS_DT_NOMINAL;
  }

  /* Сохранение данных датчиков.
   * Гироскоп переводится в рад/с. Ось X магнитометра инвертирована под соглашение NED. */
  ctx->ax = ax_g;
  ctx->ay = ay_g;
  ctx->az = az_g;
  ctx->gx = gx_dps * DEG_TO_RAD;
  ctx->gy = gy_dps * DEG_TO_RAD;
  ctx->gz = gz_dps * DEG_TO_RAD;
  ctx->mx = mx_ut;
  ctx->my = my_ut;
  ctx->mz = mz_ut;

  switch(ctx->mode) {
    case 0: /* None — чистая интеграция гироскопа */
      updateEulerAngles(ctx, dt);
      if(ctx->use_magnetometer) tiltCompensatedYaw(ctx);
      break;
    case 1: madgwickUpdate(ctx, dt); break;      /* Madgwick */
    case 2: mahonyUpdate(ctx, dt); break;        /* Mahony */
    case 3: complementaryUpdate(ctx, dt); break; /* Complementary */
    case 4:                                      /* Classic */
      updateEulerAngles(ctx, dt);
      classicUpdate(ctx, dt);
      if(ctx->use_magnetometer) tiltCompensatedYaw(ctx);
      break;
    case 5: /* EKF */
      extendedKalmanUpdate(ctx, dt);
      if(ctx->use_magnetometer) tiltCompensatedYaw(ctx);
      break;
    default: break;
  }
}

void Filter_GetEuler(filter_ctx_t* ctx, float* roll, float* pitch, float* yaw) {
  if(!ctx) return;
  if(roll) *roll = ctx->roll;
  if(pitch) *pitch = ctx->pitch;
  if(yaw) *yaw = ctx->yaw;
}

/* Инициализация фильтра */
void Filter_Init(filter_ctx_t* ctx, const char* group_name) {
  if(!ctx) return;
  memset(ctx, 0, sizeof(filter_ctx_t));
  strncpy(ctx->group_name, group_name, sizeof(ctx->group_name) - 1);

  uint8_t idx = 0;
//  CS_ADD_ENTRY(ctx->group_name, ctx->entries, idx, "Mode", &ctx->mode, CFG_TYPE_U8, sizeof(uint8_t), true, NULL);
//  CS_ADD_ENTRY(ctx->group_name, ctx->entries, idx, "R", &ctx->roll, CFG_TYPE_FLOAT, sizeof(float), false, NULL);
//  CS_ADD_ENTRY(ctx->group_name, ctx->entries, idx, "P", &ctx->pitch, CFG_TYPE_FLOAT, sizeof(float), false, NULL);
//  CS_ADD_ENTRY(ctx->group_name, ctx->entries, idx, "Y", &ctx->yaw, CFG_TYPE_FLOAT, sizeof(float), false, NULL);
//  CS_ADD_ENTRY(ctx->group_name, ctx->entries, idx, "Head", &ctx->heading, CFG_TYPE_FLOAT, sizeof(float), false, NULL);
//  CS_ADD_ENTRY(ctx->group_name, ctx->entries, idx, "KA", &ctx->alpha, CFG_TYPE_FLOAT, sizeof(float), true, NULL);
//  CS_ADD_ENTRY(ctx->group_name, ctx->entries, idx, "KB", &ctx->beta, CFG_TYPE_FLOAT, sizeof(float), true, NULL);
//  CS_ADD_ENTRY(ctx->group_name, ctx->entries, idx, "KP", &ctx->Kp, CFG_TYPE_FLOAT, sizeof(float), true, NULL);
//  CS_ADD_ENTRY(ctx->group_name, ctx->entries, idx, "KI", &ctx->Ki, CFG_TYPE_FLOAT, sizeof(float), true, NULL);
//  CS_ADD_ENTRY(ctx->group_name, ctx->entries, idx, "GME", &ctx->gyroMeasError, CFG_TYPE_FLOAT, sizeof(float), true, NULL);
//  CS_ADD_ENTRY(ctx->group_name, ctx->entries, idx, "Decl", &ctx->declination, CFG_TYPE_FLOAT, sizeof(float), true, NULL);
//  ctx->group.name            = ctx->group_name;
//  ctx->group.entries         = ctx->entries;
//  ctx->group.entries_count   = idx;
//  ctx->group.user_ctx        = ctx;
//  ctx->group.on_default      = CB_ResetConfig;
//  ctx->group.on_get_cfg_json = CB_GetConfig;
//  if(!CS_RegGroup(&ctx->group)) {
//    SC_LOG_E("Filter AHRS Init", "Failed Register Group: '%s'", ctx->group.name);
//    return;
//  }
//  CS_LoadGroup(ctx->group.name);

  ctx->q0         = 1.0f;
  ctx->q1         = 0.0f;
  ctx->q2         = 0.0f;
  ctx->q3         = 0.0f;
  ctx->att[0]     = 1.0f;
  ctx->att[1]     = 0.0f;
  ctx->att[2]     = 0.0f;
  ctx->att[3]     = 0.0f;
  ctx->ekf_cov[0] = 0.01f;
  ctx->ekf_cov[1] = 0.0f;
  ctx->ekf_cov[2] = 0.0f;
  ctx->ekf_cov[3] = 0.01f;
  ctx->ekf_Q[0]   = 0.01f;
  ctx->ekf_Q[1]   = 0.0f;
  ctx->ekf_Q[2]   = 0.0f;
  ctx->ekf_Q[3]   = 0.01f;
  ctx->ekf_R[0]   = 0.1f;
  ctx->ekf_R[1]   = 0.0f;
  ctx->ekf_R[2]   = 0.0f;
  ctx->ekf_R[3]   = 0.1f;

  ctx->last_tick  = HAL_GetTick();
  //SC_LOG_I("Filter AHRS Init", "Group: '%s'", ctx->group.name);
}

//#endif
