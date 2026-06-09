#pragma once

#include "BLDCDriver.h"
#include "tim.h"          // htim2, htim3, htim4 из CubeMX
#include "foc_utils.h"

class BLDCDriver3PWM : public BLDCDriver
{
public:
    BLDCDriver3PWM(TIM_HandleTypeDef* timA, uint32_t channelA,
                   TIM_HandleTypeDef* timB, uint32_t channelB,
                   TIM_HandleTypeDef* timC, uint32_t channelC);

    virtual int init() override;      // ← ИЗМЕНЕНО: теперь int, как в базе
    virtual void enable() override;
    virtual void disable() override;
    virtual void setPwm(float Ua, float Ub, float Uc) override;

private:
    TIM_HandleTypeDef* _timA;
    TIM_HandleTypeDef* _timB;
    TIM_HandleTypeDef* _timC;

    uint32_t _channelA;
    uint32_t _channelB;
    uint32_t _channelC;

    uint32_t _pwmPeriod;
};
