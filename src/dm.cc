#include "motor/dm.h"

#include <algorithm>
#include <cstring>

#include "bsp/time.h"
#include "utils/logger.h"

using namespace motor;

static dm* device_ptr[BSP_CAN_DEVICE_COUNT][DM_MOTOR_LIMIT];
uint8_t device_cnt[BSP_CAN_DEVICE_COUNT];

const uint8_t reset_cmd[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb };
const uint8_t enable_cmd[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc };
const uint8_t disable_cmd[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd };

dm::dm(const char *name_, const param_t &param_) : param(param_) {
    BSP_ASSERT(0 <= param_.port and param_.port < BSP_CAN_DEVICE_COUNT);
    strcpy(name, name_);

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

float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min, offset = x_min;
    return static_cast <float> (x_int) * span / static_cast <float> ((1 << bits) - 1) + offset;
}

int float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min, offset = x_min;
    return static_cast <int> ((x - offset) * (static_cast <float> ((1 << bits) - 1)) / span);
}

// MIT Control
void dm::control(float position, float speed, float Kp, float Kd, float torque) const {
    BSP_ASSERT(param.mode == MIT);
    BSP_ASSERT(Kp == 0 or Kd != 0); // 根据 MIT 模式说明，若 Kp != 0 且 Kd == 0，会引起震荡。

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
    BSP_ASSERT(param.mode == POSITION_SPEED);
    position = std::clamp(position, -param.p_max, param.p_max);
    speed = std::clamp(speed, -param.v_max, param.v_max);
    const float f[] = { position, speed };
    static_assert(sizeof f == 8);
    bsp_can_send(param.port, ctrl_id, reinterpret_cast<const uint8_t *>(f), sizeof f);
}

void dm::control(float speed) const {
    BSP_ASSERT(param.mode == SPEED);
    speed = std::clamp(speed, -param.v_max, param.v_max);
    static_assert(sizeof speed == 4);
    bsp_can_send(param.port, ctrl_id, reinterpret_cast<uint8_t *>(&speed), sizeof speed);
}

void dm::decoder(bsp_can_e device, uint32_t id, const uint8_t* data, size_t len) {
    if (!device_cnt[device]) return;

    dm *p = nullptr;
    for (uint8_t i = 0; i < device_cnt[device]; i++) {
        if (device_ptr[device][i]->feedback_id == id) {
            p = device_ptr[device][i];
            break;
        }
    }

    if (p == nullptr) return;

    const auto s = data;
    auto &raw = p->feedback.raw; auto &fb = p->feedback;
    raw.err = s[0] >> 4;
    raw.id = s[0] & 0xf;
    raw.pos = s[1] << 8 | s[2];
    raw.vel = s[3] << 4 | (s[4] >> 4);
    raw.torque = (s[4] & 0xf) << 8 | s[5];
    raw.temp_mos = s[6];
    raw.temp_rotor = s[7];

    const auto para = p->get_param();
    fb.pos = uint_to_float(raw.pos, -para->p_max, para->p_max, 16);
    fb.vel = uint_to_float(raw.vel, -para->v_max, para->v_max, 12);
    fb.torque = uint_to_float(raw.torque, -para->t_max, para->t_max, 12);
    fb.err = raw.err;
    fb.temp_mos = raw.temp_mos;
    fb.temp_rotor = raw.temp_rotor;

    fb.timestamp = bsp_time_get_ms();
}

void dm::init() {
    logger::info("motor '%s' inited", name);
    bsp_can_set_callback(param.port, feedback_id, decoder);
}
