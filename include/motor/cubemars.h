#pragma once

#include <cstdint>

#include "bsp/can.h"

namespace motor {
#define CUBEMARS_MOTOR_LIMIT 4
    class cubemars {
    public:
        cubemars() = delete;
        ~cubemars() = default;

        enum control_mode_e {
            MIT = 0,
            POSITION_SPEED = 1,
            SPEED = 2
        };

        struct param_t {
            uint8_t slave_id, master_id;
            bsp_can_e port;
            control_mode_e mode;
            float p_max, v_max, t_max;
        };

        struct feedback_t {
            struct {
                uint8_t id, err;
                uint16_t pos, vel, torque;
                int8_t temp_mos, temp_rotor;
            } raw;
            uint8_t err;
            int8_t temp_mos, temp_rotor;
            float pos, vel, torque;
            uint32_t timestamp;
        };

        cubemars(const char *name, const param_t &param);

        static void decoder(bsp_can_e device, uint32_t id, const uint8_t *data, size_t len);

        void init();

        void enable() const;
        void disable() const;
        void set_zero() const;
        void clear_error() const;

        void control(float position, float speed, float kp, float kd, float torque) const;
        void control(float position, float speed) const;
        void control(float speed) const;

        [[nodiscard]] feedback_t state() const;
        [[nodiscard]] const param_t *get_param() const { return &param; }

        char name[16] = {};
        uint16_t ctrl_id = 0, feedback_id = 0;
        feedback_t feedback = feedback_t();
    private:
        param_t param;
    };
}
