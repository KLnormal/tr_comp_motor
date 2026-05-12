//
// Created by fish on 2026/1/9.
//

#pragma once

#include <cstdint>

#include "bsp/can.h"

namespace motor {
#define DM_MOTOR_LIMIT 4
    class dm {
    public:
        dm() = delete;
        ~dm() = default;

        enum control_mode_e {
            MIT, POSITION_SPEED, SPEED
        };

        struct param_t {
            uint8_t slave_id, master_id;
            bsp_can_e port;
            control_mode_e mode;
            float p_max, v_max, t_max; // Kp 和 Kd 的最大值固定，分别为 500 和 5
        };

        struct feedback_t {
            // 电机原始反馈值
            struct {
                uint8_t id, err;
                uint16_t pos, vel, torque;
                uint8_t temp_mos, temp_rotor;
            } raw;
            // uint_to_float 后的值
            uint8_t err, temp_mos, temp_rotor;
            float pos, vel, torque;
            uint32_t timestamp;
        };

        dm(const char *name, const param_t &param);

        static void decoder(bsp_can_e device, uint32_t id, const uint8_t *data, size_t len);

        void init();

        // Base Control
        void reset() const;
        void enable() const;
        void disable() const;
        // MIT Control
        void control(float position, float speed, float Kp, float Kd, float torque) const;
        // Position Speed Control
        void control(float position, float speed) const;
        // Speed Control
        void control(float speed) const;

        [[nodiscard]] const param_t *get_param() const { return &param; }

        char name[16] = { };
        uint16_t ctrl_id, feedback_id;
        feedback_t feedback = feedback_t();
    private:
        param_t param;
    };
}
