#pragma once

#include <cstddef>
#include <cstdint>

#include "bsp/can.h"

namespace motor {
#define ROBSTRIDE_MOTOR_LIMIT 8

    class robstride {
    public:
        robstride() = delete;
        ~robstride() = default;

        enum control_mode_e {
            MOTION_CONTROL,
            SPEED,
            POSITION
        };

        enum motor_state_e : uint8_t {
            RESET = 0,
            CALIBRATION = 1,
            RUNNING = 2,
            UNKNOWN = 3
        };

        enum fault_e : uint8_t {
            NO_FAULT = 0,
            UNDERVOLTAGE = 1U << 0U,
            PHASE_CURRENT = 1U << 1U,
            OVER_TEMPERATURE = 1U << 2U,
            ENCODER = 1U << 3U,
            STALL_OVERLOAD = 1U << 4U,
            UNCALIBRATED = 1U << 5U
        };

        struct param_t {
            uint8_t slave_id, master_id;
            bsp_can_e port;
            control_mode_e mode;
        };

        struct feedback_t {
            struct {
                uint8_t id, fault, state;
                uint16_t pos, vel, torque;
                int16_t temp;
            } raw;
            uint8_t fault;
            motor_state_e state;
            float pos, vel, torque, temp;
            uint32_t timestamp;
        };

        robstride(const char *name, const param_t &param);

        static void decoder(
            bsp_can_e device,
            uint32_t id,
            const uint8_t *data,
            size_t len
        );

        void init();

        void enable() const;
        void disable() const;
        void clear_error() const;
        void set_zero() const;

        void control(
            float position,
            float speed,
            float kp,
            float kd,
            float torque
        ) const;
        // SPEED 模式下单位为 rad/s，POSITION 模式下单位为 rad
        void control(float target) const;

        [[nodiscard]] feedback_t state() const;
        [[nodiscard]] const param_t *get_param() const { return &param; }

        char name[16] = {};
        uint16_t feedback_id = 0;
        feedback_t feedback = feedback_t();

    private:
        void write_float(uint16_t index, float value) const;

        param_t param;
    };
}
