//
// Created by fish on 2026/01/27.
//

#pragma once
#include <cstdint>

#include "bsp/can.h"

namespace motor {
#define GYJ_MOTOR_LIMIT 8
    /*
     *  葛老板画的直流电机板子, CAN 驱动
     */
    class gyj {
    public:
        gyj() = delete;
        ~gyj() = default;

        enum mode_e {
            DISABLE = 0b000,
            OPEN_LOOP = 0b001,
            CURRENT_LOOP = 0b010,
            SPEED_LOOP = 0b011,
            POSITION_LOOP = 0b100,
        };

        struct param_t {
            uint8_t id;
            bsp_can_e port;
            mode_e mode;
            bool have_feedback;
        };

        struct feedback_t {
            // 电机原始反馈值
            struct {
                int16_t angle, speed, current;
                uint8_t temp;
            } raw;
            // 反馈的角度
            float angle;
            // 反馈的速度
            float speed;
            // 反馈的电流
            float current;
            uint32_t timestamp;
        };

        gyj(const char *name, const param_t &param, float ratio = 1);

        static void decoder(bsp_can_e device, uint32_t id, const uint8_t *data, size_t len);

        void init();
        void clear();

        void set_mode(mode_e m, bool have_feedback, bool modified = true);
        void enable() { set_mode(this->param.mode, this->param.have_feedback, false); this->enabled = true; }
        void disable() { set_mode(DISABLE, false, false); clear(); this->enabled = false; }
        void update(float val);
        [[nodiscard]] feedback_t state() const;

        float ratio = 0;
        char name[16] = { };
        bool enabled = false;
        feedback_t feedback = feedback_t();
        uint16_t ctrl_id = 0, feedback_id = 0;

        int16_t output = 0;
    private:
        param_t param {};
    };
}
