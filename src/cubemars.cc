#include "motor/cubemars.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "bsp/sys.h"
#include "bsp/time.h"
#include "device_registry.h"
#include "utils/logger.h"

using namespace motor;

static internal::device_registry<cubemars, CUBEMARS_MOTOR_LIMIT> registry;

static constexpr uint8_t enable_cmd[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc
};
static constexpr uint8_t disable_cmd[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd
};
static constexpr uint8_t set_zero_cmd[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe
};
static constexpr uint8_t clear_error_cmd[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb
};

static float uint_to_float(uint16_t value, float min, float max, uint8_t bits) {
    const float span = max - min;
    return static_cast<float>(value) * span /
           static_cast<float>((1U << bits) - 1U) + min;
}

static uint16_t float_to_uint(float value, float min, float max, uint8_t bits) {
    const float span = max - min;
    return static_cast<uint16_t>(
        (value - min) * static_cast<float>((1U << bits) - 1U) / span
    );
}

cubemars::cubemars(const char *name_, const param_t &param_) : param(param_) {
    BSP_ASSERT(
        std::isfinite(param_.p_max) && std::isfinite(param_.v_max) &&
        std::isfinite(param_.t_max) && param_.p_max > 0.f &&
        param_.v_max > 0.f && param_.t_max > 0.f
    );
    BSP_ASSERT(0 <= param_.port && param_.port < BSP_CAN_DEVICE_COUNT);
    BSP_ASSERT(
        param_.mode == MIT || param_.mode == POSITION_SPEED ||
        param_.mode == SPEED
    );
    std::snprintf(name, sizeof(name), "%s", name_ != nullptr ? name_ : "");

    ctrl_id = static_cast<uint16_t>(
        static_cast<uint16_t>(param_.mode) << 8U | param_.slave_id
    );
    feedback_id = param_.master_id;
    BSP_ASSERT(registry.add(param.port, this));
}

void cubemars::enable() const {
    bsp_can_send(param.port, ctrl_id, enable_cmd, sizeof(enable_cmd));
}

void cubemars::disable() const {
    bsp_can_send(param.port, ctrl_id, disable_cmd, sizeof(disable_cmd));
}

void cubemars::set_zero() const {
    bsp_can_send(param.port, ctrl_id, set_zero_cmd, sizeof(set_zero_cmd));
}

void cubemars::clear_error() const {
    bsp_can_send(param.port, ctrl_id, clear_error_cmd, sizeof(clear_error_cmd));
}

void cubemars::control(
    float position, float speed, float kp, float kd, float torque
) const {
    if (param.mode != MIT) {
        disable();
        return;
    }
    if (!std::isfinite(position) || !std::isfinite(speed) ||
        !std::isfinite(kp) || !std::isfinite(kd) ||
        !std::isfinite(torque)) {
        disable();
        return;
    }

    position = std::clamp(position, -param.p_max, param.p_max);
    speed = std::clamp(speed, -param.v_max, param.v_max);
    kp = std::clamp(kp, 0.f, 500.f);
    kd = std::clamp(kd, 0.f, 5.f);
    torque = std::clamp(torque, -param.t_max, param.t_max);
    if (kp != 0.f && kd == 0.f) {
        disable();
        return;
    }

    const uint16_t desired_position = float_to_uint(
        position, -param.p_max, param.p_max, 16
    );
    const uint16_t desired_speed = float_to_uint(
        speed, -param.v_max, param.v_max, 12
    );
    const uint16_t desired_kp = float_to_uint(kp, 0.f, 500.f, 12);
    const uint16_t desired_kd = float_to_uint(kd, 0.f, 5.f, 12);
    const uint16_t feedforward_torque = float_to_uint(
        torque, -param.t_max, param.t_max, 12
    );

    const uint8_t data[8] = {
        static_cast<uint8_t>(desired_position >> 8U),
        static_cast<uint8_t>(desired_position),
        static_cast<uint8_t>(desired_speed >> 4U),
        static_cast<uint8_t>(
            (desired_speed & 0x0fU) << 4U | desired_kp >> 8U
        ),
        static_cast<uint8_t>(desired_kp),
        static_cast<uint8_t>(desired_kd >> 4U),
        static_cast<uint8_t>(
            (desired_kd & 0x0fU) << 4U | feedforward_torque >> 8U
        ),
        static_cast<uint8_t>(feedforward_torque)
    };
    bsp_can_send(param.port, ctrl_id, data, sizeof(data));
}

void cubemars::control(float position, float speed) const {
    if (param.mode != POSITION_SPEED) {
        disable();
        return;
    }
    if (!std::isfinite(position) || !std::isfinite(speed)) {
        disable();
        return;
    }

    const float data[] = {
        std::clamp(position, -param.p_max, param.p_max),
        std::clamp(speed, -param.v_max, param.v_max)
    };
    static_assert(sizeof(data) == 8);
    bsp_can_send(
        param.port, ctrl_id, reinterpret_cast<const uint8_t *>(data),
        sizeof(data)
    );
}

void cubemars::control(float speed) const {
    if (param.mode != SPEED) {
        disable();
        return;
    }
    if (!std::isfinite(speed)) {
        disable();
        return;
    }

    speed = std::clamp(speed, -param.v_max, param.v_max);
    static_assert(sizeof(speed) == 4);
    bsp_can_send(
        param.port, ctrl_id, reinterpret_cast<const uint8_t *>(&speed),
        sizeof(speed)
    );
}

void cubemars::decoder(
    bsp_can_e device, uint32_t id, const uint8_t *data, size_t len
) {
    if (data == nullptr || len != 8) return;

    cubemars *motor = registry.find_by_feedback_id(
        device, static_cast<uint16_t>(id)
    );
    if (motor == nullptr) return;

    feedback_t next{};
    next.raw.err = data[0] >> 4U;
    next.raw.id = data[0] & 0x0fU;
    next.raw.pos = static_cast<uint16_t>(data[1] << 8U | data[2]);
    next.raw.vel = static_cast<uint16_t>(
        data[3] << 4U | data[4] >> 4U
    );
    next.raw.torque = static_cast<uint16_t>(
        (data[4] & 0x0fU) << 8U | data[5]
    );
    next.raw.temp_mos = static_cast<int8_t>(data[6]);
    next.raw.temp_rotor = static_cast<int8_t>(data[7]);

    const param_t *param = motor->get_param();
    next.err = next.raw.err;
    next.temp_mos = next.raw.temp_mos;
    next.temp_rotor = next.raw.temp_rotor;
    next.pos = uint_to_float(
        next.raw.pos, -param->p_max, param->p_max, 16
    );
    next.vel = uint_to_float(
        next.raw.vel, -param->v_max, param->v_max, 12
    );
    next.torque = uint_to_float(
        next.raw.torque, -param->t_max, param->t_max, 12
    );
    next.timestamp = bsp_time_get_ms();

    const unsigned long state = bsp_sys_enter_critical();
    motor->feedback = next;
    bsp_sys_exit_critical(state);
}

void cubemars::init() {
    logger::info("motor '%s' inited", name);
    bsp_can_set_callback(param.port, feedback_id, decoder);
}

cubemars::feedback_t cubemars::state() const {
    const unsigned long state = bsp_sys_enter_critical();
    const feedback_t copy = feedback;
    bsp_sys_exit_critical(state);
    return copy;
}
