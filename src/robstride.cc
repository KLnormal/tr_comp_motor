#include "motor/robstride.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdio>
#include <limits>

#include "bsp/sys.h"
#include "bsp/time.h"
#include "device_registry.h"
#include "utils/logger.h"

using namespace motor;

static internal::device_registry<robstride, ROBSTRIDE_MOTOR_LIMIT> registry;

static constexpr float position_min = -12.57f;
static constexpr float position_max = 12.57f;
static constexpr float speed_min = -50.f;
static constexpr float speed_max = 50.f;
static constexpr float speed_mode_min = -33.f;
static constexpr float speed_mode_max = 33.f;
static constexpr float kp_min = 0.f;
static constexpr float kp_max = 500.f;
static constexpr float kd_min = 0.f;
static constexpr float kd_max = 5.f;
static constexpr float torque_min = -5.5f;
static constexpr float torque_max = 5.5f;

static constexpr uint8_t control_type = 1;
static constexpr uint8_t feedback_type = 2;
static constexpr uint8_t enable_type = 3;
static constexpr uint8_t stop_type = 4;
static constexpr uint8_t set_zero_type = 6;
static constexpr uint8_t write_parameter_type = 18;

static constexpr uint16_t speed_reference_index = 0x700a;
static constexpr uint16_t position_reference_index = 0x7016;

static constexpr uint32_t feedback_filter_mask = 0x1f0000ff;

static uint32_t make_id(uint8_t type, uint16_t data, uint8_t target_id) {
    return static_cast<uint32_t>(type) << 24U |
           static_cast<uint32_t>(data) << 8U |
           target_id;
}

static uint16_t read_be_u16(const uint8_t *data) {
    return static_cast<uint16_t>(
        static_cast<uint16_t>(data[0]) << 8U | data[1]
    );
}

static void write_be_u16(uint8_t *data, uint16_t value) {
    data[0] = static_cast<uint8_t>(value >> 8U);
    data[1] = static_cast<uint8_t>(value);
}

static uint16_t float_to_uint(
    float value,
    float min,
    float max
) {
    const float span = max - min;
    return static_cast<uint16_t>(
        (value - min) * static_cast<float>(UINT16_MAX) / span
    );
}

static float uint_to_float(
    uint16_t value,
    float min,
    float max
) {
    const float span = max - min;
    return static_cast<float>(value) * span /
           static_cast<float>(UINT16_MAX) + min;
}

robstride::robstride(
    const char *name_,
    const param_t &param_
) : param(param_) {
    BSP_ASSERT(0 <= param.port && param.port < BSP_CAN_DEVICE_COUNT);
    BSP_ASSERT(param.slave_id <= 0x7f);
    BSP_ASSERT(
        param.mode == MOTION_CONTROL ||
        param.mode == SPEED ||
        param.mode == POSITION
    );
    BSP_ASSERT(
        registry.find_by_feedback_id(param.port, param.slave_id) == nullptr
    );
    std::snprintf(name, sizeof(name), "%s", name_ != nullptr ? name_ : "");

    feedback_id = param.slave_id;
    BSP_ASSERT(registry.add(param.port, this));
}

void robstride::enable() const {
    const uint8_t data[8] = {};
    bsp_can_send_ext(
        param.port,
        make_id(enable_type, param.master_id, param.slave_id),
        data,
        sizeof(data)
    );
}

void robstride::disable() const {
    const uint8_t data[8] = {};
    bsp_can_send_ext(
        param.port,
        make_id(stop_type, param.master_id, param.slave_id),
        data,
        sizeof(data)
    );
}

void robstride::clear_error() const {
    const uint8_t data[8] = { 1 };
    bsp_can_send_ext(
        param.port,
        make_id(stop_type, param.master_id, param.slave_id),
        data,
        sizeof(data)
    );
}

void robstride::set_zero() const {
    const uint8_t data[8] = { 1 };
    bsp_can_send_ext(
        param.port,
        make_id(set_zero_type, param.master_id, param.slave_id),
        data,
        sizeof(data)
    );
}

void robstride::control(
    float position,
    float speed,
    float kp,
    float kd,
    float torque
) const {
    if (param.mode != MOTION_CONTROL) {
        disable();
        return;
    }
    if (!std::isfinite(position) || !std::isfinite(speed) ||
        !std::isfinite(kp) || !std::isfinite(kd) ||
        !std::isfinite(torque)) {
        disable();
        return;
    }

    position = std::clamp(position, position_min, position_max);
    speed = std::clamp(speed, speed_min, speed_max);
    kp = std::clamp(kp, kp_min, kp_max);
    kd = std::clamp(kd, kd_min, kd_max);
    torque = std::clamp(torque, torque_min, torque_max);
    if (kp != 0.f && kd == 0.f) {
        disable();
        return;
    }

    const uint16_t desired_position = float_to_uint(
        position,
        position_min,
        position_max
    );
    const uint16_t desired_speed = float_to_uint(
        speed,
        speed_min,
        speed_max
    );
    const uint16_t desired_kp = float_to_uint(kp, kp_min, kp_max);
    const uint16_t desired_kd = float_to_uint(kd, kd_min, kd_max);
    const uint16_t desired_torque = float_to_uint(
        torque,
        torque_min,
        torque_max
    );

    uint8_t data[8];
    write_be_u16(&data[0], desired_position);
    write_be_u16(&data[2], desired_speed);
    write_be_u16(&data[4], desired_kp);
    write_be_u16(&data[6], desired_kd);
    bsp_can_send_ext(
        param.port,
        make_id(control_type, desired_torque, param.slave_id),
        data,
        sizeof(data)
    );
}

void robstride::motion_velocity_control(float velocity, float kd) const {
    control(0.f, velocity, 0.f, kd, 0.f);
}

void robstride::motion_damping_control(float kd) const {
    control(0.f, 0.f, 0.f, kd, 0.f);
}

void robstride::motion_position_control(
    float position,
    float kp,
    float kd
) const {
    control(position, 0.f, kp, kd, 0.f);
}

void robstride::write_float(uint16_t index, float value) const {
    static_assert(sizeof(float) == sizeof(uint32_t));
    static_assert(std::numeric_limits<float>::is_iec559);

    const uint32_t raw = std::bit_cast<uint32_t>(value);
    const uint8_t data[8] = {
        static_cast<uint8_t>(index),
        static_cast<uint8_t>(index >> 8U),
        0,
        0,
        static_cast<uint8_t>(raw),
        static_cast<uint8_t>(raw >> 8U),
        static_cast<uint8_t>(raw >> 16U),
        static_cast<uint8_t>(raw >> 24U)
    };
    bsp_can_send_ext(
        param.port,
        make_id(write_parameter_type, param.master_id, param.slave_id),
        data,
        sizeof(data)
    );
}

void robstride::control(float target) const {
    if (!std::isfinite(target)) {
        disable();
        return;
    }

    switch (param.mode) {
    case SPEED:
        write_float(
            speed_reference_index,
            std::clamp(target, speed_mode_min, speed_mode_max)
        );
        break;
    case POSITION:
        write_float(position_reference_index, target);
        break;
    case MOTION_CONTROL:
    default:
        disable();
        break;
    }
}

void robstride::decoder(
    bsp_can_e device,
    uint32_t id,
    const uint8_t *data,
    size_t len
) {
    if (data == nullptr || len != 8 ||
        (id >> 24U & 0x1fU) != feedback_type) {
        return;
    }

    const uint8_t motor_id = static_cast<uint8_t>(id >> 8U);
    robstride *motor = registry.find_by_feedback_id(device, motor_id);
    if (motor == nullptr ||
        motor->get_param()->master_id !=
        static_cast<uint8_t>(id & 0xffU)) {
        return;
    }

    feedback_t next{};
    next.raw.id = motor_id;
    next.raw.fault = static_cast<uint8_t>(id >> 16U & 0x3fU);
    next.raw.state = static_cast<uint8_t>(id >> 22U & 0x03U);
    next.raw.pos = read_be_u16(&data[0]);
    next.raw.vel = read_be_u16(&data[2]);
    next.raw.torque = read_be_u16(&data[4]);
    next.raw.temp = static_cast<int16_t>(read_be_u16(&data[6]));

    next.fault = next.raw.fault;
    next.state = static_cast<motor_state_e>(next.raw.state);
    next.pos = uint_to_float(
        next.raw.pos,
        position_min,
        position_max
    );
    next.vel = uint_to_float(next.raw.vel, speed_min, speed_max);
    next.torque = uint_to_float(
        next.raw.torque,
        torque_min,
        torque_max
    );
    next.temp = static_cast<float>(next.raw.temp) / 10.f;
    next.timestamp = bsp_time_get_ms();

    const unsigned long state = bsp_sys_enter_critical();
    motor->feedback = next;
    bsp_sys_exit_critical(state);
}

void robstride::init() {
    struct callback_route_t {
        bsp_can_e port;
        uint8_t master_id;
    };
    static callback_route_t routes[
        BSP_CAN_DEVICE_COUNT * ROBSTRIDE_MOTOR_LIMIT
    ];
    static uint8_t route_count = 0;

    bool route_exists = false;
    for (uint8_t i = 0; i < route_count; i++) {
        if (routes[i].port == param.port &&
            routes[i].master_id == param.master_id) {
            route_exists = true;
            break;
        }
    }
    if (!route_exists) {
        BSP_ASSERT(
            route_count <
            BSP_CAN_DEVICE_COUNT * ROBSTRIDE_MOTOR_LIMIT
        );
        BSP_ASSERT(
            bsp_can_set_ext_callback(
                param.port,
                make_id(feedback_type, 0, param.master_id),
                feedback_filter_mask,
                decoder
            ) == BSP_STATUS_OK
        );
        routes[route_count++] = {
            .port = param.port,
            .master_id = param.master_id
        };
    }

    logger::info("motor '%s' inited", name);
}

robstride::feedback_t robstride::state() const {
    const unsigned long state = bsp_sys_enter_critical();
    const feedback_t copy = feedback;
    bsp_sys_exit_critical(state);
    return copy;
}
