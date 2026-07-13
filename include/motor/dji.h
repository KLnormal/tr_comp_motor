//
// Created by fish on 2025/10/10.
//

#pragma once
#include <cstdint>

#include "bsp/can.h"

namespace motor {
#define DJI_MOTOR_LIMIT 8
    /*
     *  DJI Motor Driver
     *  GM6020:
     *      Feedback id: 0x205 - 0x20B
     *      Control id: 0x1FF/0x2FF (VOLTAGE), 0x1FE/0x2FE (CURRENT)
     *      VOLTAGE Range: -25000 ~ 25000
     *      CURRENT Range: -16384 ~ 16384
     *      Angle Range: 0 ~ 8191
     *      Speed Unit: rpm
     *  M3508 (C620):
     *      Feedback id: 0x201 - 0x208
     *      Control id: 0x200/0x1FF (CURRENT)
     *      CURRENT Range: -16384 ~ 16384 (-20A ~ 20A)
     *      Angle Range: 0 ~ 8191
     *      Speed Unit: rpm
     *  M2006 (C610):
     *      Feedback id: 0x201 - 0x208
     *      Control id: 0x200/0x1FF (CURRENT)
     *      CURRENT Range: -10000 ~ 10000 (-10A ~ 10A)
     *      Angle Range: 0 ~ 8191
     *      Speed Unit: rpm
     */
    class dji {
    public:
        dji() = delete;
        ~dji() = default;

        enum control_mode_e {
            VOLTAGE, CURRENT
        };

        enum model_e {
            GM6020, M3508, M2006
        };

        struct param_t {
            uint8_t id;
            bsp_can_e port;
            control_mode_e mode;
        };

        struct power_param_t {
            float k0, k1, k2, k3, k4, k5;
        };

        struct feedback_t {
            // 电机原始反馈值
            struct {
                uint16_t angle;
                int16_t speed, current;
                uint8_t temp;
            } raw;
            // 编码器轴累计圈数，首帧为 0；原始角度正向过零时加 1
            int32_t round;
            // 编码器轴单圈绝对角度 (rad, [0, 2\pi))
            float angle;
            // 输出轴角速度 (rad/s)
            float speed;
            // 相电流 (A)
            float current;
            // 转矩 (Nm)
            float torque;
            // 功率 (W)
            float power;
            uint32_t timestamp;
        };

        // timeout_ms 默认为 -1, 即不设超时时间; 若不为 -1, 则在电机离线或 update 超时时, output 将被强制设为 0
        dji(const char *name, const model_e &model, const param_t &param);
        dji(const char *name, const model_e &model, const param_t &param, int timeout_ms);
        dji(const char *name, const model_e &model, const param_t &param, int timeout_ms, float ratio);
        dji(const char *name, const model_e &model, const param_t &param, int timeout_ms, float ratio, const power_param_t &power_param);

        static void decoder(bsp_can_e device, uint32_t id, const uint8_t *data, size_t len);

        void init();
        void clear();

        void enable() { this->enabled = true; }
        void disable() { clear(); this->enabled = false; }

        // 考虑到有电压控制 6020 的需求, 这里 update 的值是发给电机的原始值, 不会重新映射
        void update(float val);
        // void update_torque(float val);

        [[nodiscard]] float predict_power(float val) const;
        [[nodiscard]] feedback_t state() const;

        float ratio = 0;
        char name[16] = { };
        bool enabled = false;
        feedback_t feedback = feedback_t();
        uint16_t ctrl_id = 0, feedback_id = 0;
        power_param_t power_param = power_param_t();

        int16_t output = 0;
        int32_t timeout_ms = 0;
        uint32_t lst_update_time = 0;
    private:
        model_e model {};
        param_t param {};
        uint16_t lst_angle = 0;
        bool feedback_received = false;
    };
}
