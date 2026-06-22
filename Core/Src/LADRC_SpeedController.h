#include <algorithm>
#include <cmath>

class LADRC_SpeedController {
public:
    /**
     * @param Ts   - период дискретизации, сек (например 0.001)
     * @param wc   - полоса пропускания контроллера, рад/с
     * @param wo   - полоса пропускания наблюдателя, рад/с (обычно 3..10 * wc)
     * @param b0   - оценка коэффициента усиления b (подбирается экспериментально!)
     * @param u_min, u_max - ограничения на управляющее воздействие
     */
    LADRC_SpeedController(float Ts, float wc, float wo, float b0,
    		float u_min = -1.0, float u_max = 1.0)
        : Ts_(Ts), wc_(wc), wo_(wo), b0_(b0), u_min_(u_min), u_max_(u_max),
          z1_(0.0), z2_(0.0)
    {
        beta1_ = 2.0 * wo_;
        beta2_ = wo_ * wo_;
        kp_    = wc_;           // P-коэффициент для 1-го порядка
    }

    /**
     * Основной метод. Вызывать каждый период Ts
     * @param speed_meas - измеренная скорость (рад/с или rpm — единицы должны совпадать с b0)
     * @param speed_ref  - задание скорости
     * @return управляющее воздействие u (напряжение / duty cycle / момент)
     */
    float update(float speed_meas, float speed_ref) {
        // 1. Закон управления (компенсация возмущения)
    	float u = (kp_ * (speed_ref - z1_) - z2_) / b0_;
        u = std::max(u_min_, std::min(u_max_, u));

        // 2. Обновление Extended State Observer (Euler)
        float e = speed_meas - z1_;
        z1_ += Ts_ * (z2_ + beta1_ * e + b0_ * u);
        z2_ += Ts_ * (beta2_ * e);

        return u;
    }

    float get_estimated_speed() const { return z1_; }
    float get_estimated_disturbance() const { return z2_; }

    void reset() {
        z1_ = 0.0;
        z2_ = 0.0;
    }

public:
    float Ts_, wc_, wo_, b0_;
    float u_min_, u_max_;
    float z1_, z2_;      // z1 — оценка скорости, z2 — оценка total disturbance
    float beta1_, beta2_, kp_;
};
