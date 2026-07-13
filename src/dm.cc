#include "motor/dm.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "bsp/time.h"
#include "bsp/sys.h"
#include "utils/logger.h"

using namespace motor;

static dm* device_ptr[BSP_CAN_DEVICE_COUNT][DM_MOTOR_LIMIT];
static uint8_t device_cnt[BSP_CAN_DEVICE_COUNT];

static const uint8_t reset_cmd[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb };
static const uint8_t enable_cmd[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc };
static const uint8_t disable_cmd[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd };

dm::dm(const char *name_, const param_t &param_) : param(param_) {
    BSP_ASSERT(
        std::isfinite(param_.p_max) && std::isfinite(param_.v_max) &&
        std::isfinite(param_.t_max) && param_.p_max > 0.f &&
        param_.v_max > 0.f && param_.t_max > 0.f
    );
    BSP_ASSERT(0 <= param_.port and param_.port < BSP_CAN_DEVICE_COUNT);
    BSP_ASSERT(device_cnt[param_.port] < DM_MOTOR_LIMIT);
    std::snprintf(name, sizeof(name), "%s", name_ != nullptr ? name_ : "");

    if (param_.mode == MIT) {
        ctrl_id = param_.slave_id;
    }
    if (param_.mode == POSITION_SPEED) {
        ctrl_id = 0x100 + param_.slave_id;
    }
    if (param_.mode == SPEED) {
        ctrl_id = 0x200 + param_.slave_id;
    }

    feedback_id = param_.master_id;
    device_ptr[param.port][device_cnt[param.port] ++] = this;
}

void dm::reset() const {
    bsp_can_send(param.port, ctrl_id, reset_cmd, sizeof(reset_cmd));
}

void dm::enable() const {
    bsp_can_send(param.port, ctrl_id, enable_cmd, sizeof(enable_cmd));
}

void dm::disable() const {
    bsp_can_send(param.port, ctrl_id, disable_cmd, sizeof(disable_cmd));
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min, offset = x_min;
    return static_cast <float> (x_int) * span / static_cast <float> ((1 << bits) - 1) + offset;
}

static int float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min, offset = x_min;
    return static_cast <int> ((x - offset) * (static_cast <float> ((1 << bits) - 1)) / span);
}

// MIT Control
void dm::control(float position, float speed, float Kp, float Kd, float torque) const {
    if (param.mode != MIT) { disable(); return; }
    if (Kp != 0 && Kd == 0) { disable(); return; }
    if (!std::isfinite(position) || !std::isfinite(speed) || !std::isfinite(Kp) ||
        !std::isfinite(Kd) || !std::isfinite(torque)) {
        disable();
        return;
    }

    position = std::clamp(position, -param.p_max, param.p_max);
    speed = std::clamp(speed, -param.v_max, param.v_max);
    Kp = std::clamp(Kp, 0.f, 500.f);
    Kd = std::clamp(Kd, 0.f, 5.f);
    torque = std::clamp(torque, -param.t_max, param.t_max);

    uint16_t P_des = float_to_uint(position, -param.p_max, param.p_max, 16),
             V_des = float_to_uint(speed, -param.v_max, param.v_max, 12),
             Kp_ = float_to_uint(Kp, 0, 500, 12),
             Kd_ = float_to_uint(Kd, 0, 5, 12),
             T_ff = float_to_uint(torque, -param.t_max, param.t_max, 12);

    uint8_t msg[8] = {};
    msg[0] = P_des >> 8;
    msg[1] = P_des & 0xff;
    msg[2] = V_des >> 4;
    msg[3] = (V_des & 0xf) << 4 | (Kp_ >> 8);
    msg[4] = Kp_ & 0xff;
    msg[5] = Kd_ >> 4;
    msg[6] = (Kd_ & 0xf) << 4 | (T_ff >> 8);
    msg[7] = T_ff & 0xff;
    bsp_can_send(param.port, ctrl_id, msg, sizeof msg);
}

void dm::control(float position, float speed) const {
    if (param.mode != POSITION_SPEED) { disable(); return; }
    if (!std::isfinite(position) || !std::isfinite(speed)) {
        disable();
        return;
    }
    position = std::clamp(position, -param.p_max, param.p_max);
    speed = std::clamp(speed, -param.v_max, param.v_max);
    const float f[] = { position, speed };
    static_assert(sizeof f == 8);
    bsp_can_send(param.port, ctrl_id, reinterpret_cast<const uint8_t *>(f), sizeof f);
}

void dm::control(float speed) const {
    if (param.mode != SPEED) { disable(); return; }
    if (!std::isfinite(speed)) {
        disable();
        return;
    }
    speed = std::clamp(speed, -param.v_max, param.v_max);
    static_assert(sizeof speed == 4);
    bsp_can_send(param.port, ctrl_id, reinterpret_cast<const uint8_t *>(&speed), sizeof speed);
}

void dm::decoder(bsp_can_e device, uint32_t id, const uint8_t* data, size_t len) {
    const int device_index = static_cast<int>(device);
    if (device_index < 0 || device_index >= BSP_CAN_DEVICE_COUNT ||
        !device_cnt[device] || data == nullptr || len != 8) return;

    dm *p = nullptr;
    for (uint8_t i = 0; i < device_cnt[device]; i++) {
        if (device_ptr[device][i]->feedback_id == id) {
            p = device_ptr[device][i];
            break;
        }
    }

    if (p == nullptr) return;

    const auto s = data;
    feedback_t next{};
    next.raw.err = s[0] >> 4;
    next.raw.id = s[0] & 0xf;
    next.raw.pos = s[1] << 8 | s[2];
    next.raw.vel = s[3] << 4 | (s[4] >> 4);
    next.raw.torque = (s[4] & 0xf) << 8 | s[5];
    next.raw.temp_mos = s[6];
    next.raw.temp_rotor = s[7];

    const auto para = p->get_param();
    next.pos = uint_to_float(next.raw.pos, -para->p_max, para->p_max, 16);
    next.vel = uint_to_float(next.raw.vel, -para->v_max, para->v_max, 12);
    next.torque = uint_to_float(next.raw.torque, -para->t_max, para->t_max, 12);
    next.err = next.raw.err;
    next.temp_mos = next.raw.temp_mos;
    next.temp_rotor = next.raw.temp_rotor;
    next.timestamp = bsp_time_get_ms();

    const unsigned long state = bsp_sys_enter_critical();
    p->feedback = next;
    bsp_sys_exit_critical(state);
}

void dm::init() {
    logger::info("motor '%s' inited", name);
    bsp_can_set_callback(param.port, feedback_id, decoder);
}

dm::feedback_t dm::state() const {
    const unsigned long state = bsp_sys_enter_critical();
    const feedback_t copy = feedback;
    bsp_sys_exit_critical(state);
    return copy;
}
