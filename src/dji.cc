#include "motor/dji.h"

#include "device_registry.h"

#include "bsp/can.h"
#include "bsp/sys.h"
#include "bsp/time.h"
#include "utils/logger.h"

#include "task.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "cmsis_os2.h"

using namespace motor;

#define TASK_STACK_SIZE 512

void dji::update_round(uint16_t raw_angle) {
    if (raw_angle >= encoder_resolution) return;
    if (feedback_received) {
        const int32_t delta = static_cast<int32_t>(raw_angle) - lst_angle;
        if (delta < -static_cast<int32_t>(encoder_resolution / 2))
            ++feedback.round;
        else if (delta > static_cast<int32_t>(encoder_resolution / 2))
            --feedback.round;
    }
    lst_angle = raw_angle;
    feedback_received = true;
}

// 电机默认减速比
#define GM6020_DEFAULT_RATIO 1.f
#define M3508_DEFAULT_RATIO (3591.f/187.f)
#define M2006_DEFAULT_RATIO (36.f/1.f)

// 电机默认转矩常数（24V下）
#define GM6020_TORQUE_CONSTANT 0.741f
#define M3508_TORQUE_CONSTANT 0.3f
#define M2006_TORQUE_CONSTANT 0.18f

// 映射和限幅
#define GM6020_VOLTAGE_LIMIT 25000.f
#define GM6020_CURRENT_LIMIT 16384.f
#define M3508_CURRENT_LIMIT 16384.f
#define M2006_CURRENT_LIMIT 10000.f
#define GM6020_CURRENT_LIMIT_REAL 3.f
#define M3508_CURRENT_LIMIT_REAL 20.f
#define M2006_CURRENT_LIMIT_REAL 10.f

// 电机功率默认拟合系数
// W = K0 + K1 * current + K2 * speed + K3 * current * speed + K4 * current * current + K5 * speed * speed)
constexpr dji::power_param_t GM6020_DEFAULT_POWER_PARAM = {
    .k0 = 0.7507578,
    .k1 = -0.0759636,
    .k2 = -0.00153397,
    .k3 = 0.01225624,
    .k4 = 0.19101805,
    .k5 = 0.0000066450
};

constexpr dji::power_param_t M3508_DEFAULT_POWER_PARAM = {
    .k0 = 0.65213,
    .k1 = -0.15659,
    .k2 = 0.00041660,
    .k3 = 0.00235415,
    .k4 = 0.20022,
    .k5 = 1.08e-7
};

constexpr dji::power_param_t M2006_DEFAULT_POWER_PARAM = {
    .k0 = 0, .k1 = 0, .k2 = 0, .k3 = 0, .k4 = 0, .k5 = 0
};


#define ID_COUNT 5
static constexpr uint16_t ctrl_id_map[] = { 0x2ff, 0x1ff, 0x2fe, 0x1fe, 0x200 };

static uint8_t id_trans(uint16_t x) {
    for (uint8_t i = 0; i < ID_COUNT; i++)
        if (ctrl_id_map[i] == x) return i;
    return 0;
}

static internal::device_registry<dji, DJI_MOTOR_LIMIT> registry;
static bool ctrl_id_used[BSP_CAN_DEVICE_COUNT][ID_COUNT + 1];
static uint8_t can_tx_buf[BSP_CAN_DEVICE_COUNT][ID_COUNT + 1][8];


// FreeRTOS Task
static bool inited = false;
[[noreturn]] static void task(void* args);
static TaskHandle_t task_handle = nullptr;


dji::dji(const char* name, const model_e& model, const param_t& param)
    : dji(name, model, param, -1) {}

dji::dji(const char* name, const model_e& model, const param_t& param, int timeout_ms)
    : dji(name, model, param, timeout_ms,
          model == GM6020 ? GM6020_DEFAULT_RATIO
        : model == M3508  ? M3508_DEFAULT_RATIO
                          : M2006_DEFAULT_RATIO) {}

dji::dji(const char* name, const model_e& model, const param_t& param,
         int timeout_ms, float ratio)
    : dji(name, model, param, timeout_ms, ratio,
          model == GM6020 ? GM6020_DEFAULT_POWER_PARAM
        : model == M3508  ? M3508_DEFAULT_POWER_PARAM
                          : M2006_DEFAULT_POWER_PARAM) {}

dji::dji(const char* name, const model_e& model, const param_t& param,
         int timeout_ms, float ratio, const power_param_t& power_param)
    : ratio(ratio), power_param(power_param), timeout_ms(timeout_ms),
      model(model), param(param)
{
    BSP_ASSERT(std::isfinite(ratio) && ratio > 0.f && timeout_ms >= -1);
    BSP_ASSERT(
        std::isfinite(power_param.k0) && std::isfinite(power_param.k1) &&
        std::isfinite(power_param.k2) && std::isfinite(power_param.k3) &&
        std::isfinite(power_param.k4) && std::isfinite(power_param.k5)
    );
    BSP_ASSERT(0 <= param.port && param.port < BSP_CAN_DEVICE_COUNT);
    std::snprintf(this->name, sizeof(this->name), "%s", name ? name : "");

    switch (model) {
    case GM6020: {
        BSP_ASSERT(1 <= param.id && param.id <= 7);
        BSP_ASSERT(param.mode == VOLTAGE || param.mode == CURRENT);
        if (param.mode == VOLTAGE)
            ctrl_id = param.id < 5 ? 0x1ff : 0x2ff;
        if (param.mode == CURRENT)
            ctrl_id = param.id < 5 ? 0x1fe : 0x2fe;
        feedback_id = 0x204 + param.id;
        break;
    }
    case M3508: {
        BSP_ASSERT(1 <= param.id && param.id <= 8);
        BSP_ASSERT(param.mode == CURRENT);
        ctrl_id = param.id < 5 ? 0x200 : 0x1ff;
        feedback_id = 0x200 + param.id;
        break;
    }
    case M2006: {
        BSP_ASSERT(1 <= param.id && param.id <= 8);
        BSP_ASSERT(param.mode == CURRENT);
        ctrl_id = param.id < 5 ? 0x200 : 0x1ff;
        feedback_id = 0x200 + param.id;
        break;
    }
    }

    registry.add(param.port, this);
    ctrl_id_used[param.port][id_trans(ctrl_id)] = true;
}


void dji::update(float val) {
    if (!enabled) return;
    // 控制链路的运行时异常不能让整机停在断言中，非有限输入按零输出处理。
    if (!std::isfinite(val)) val = 0.0f;
    int16_t next_output = 0;
    switch (model) {
    case GM6020:
        if (param.mode == VOLTAGE)
            next_output = static_cast<int16_t>(
                std::clamp(val, -GM6020_VOLTAGE_LIMIT, GM6020_VOLTAGE_LIMIT));
        else
            next_output = static_cast<int16_t>(
                std::clamp(val, -GM6020_CURRENT_LIMIT, GM6020_CURRENT_LIMIT));
        break;
    case M3508:
        next_output = static_cast<int16_t>(
            std::clamp(val, -M3508_CURRENT_LIMIT, M3508_CURRENT_LIMIT));
        break;
    case M2006:
        next_output = static_cast<int16_t>(
            std::clamp(val, -M2006_CURRENT_LIMIT, M2006_CURRENT_LIMIT));
        break;
    }

    const uint8_t cid = id_trans(ctrl_id);
    const uint8_t mid = param.id < 5 ? param.id : param.id - 4;
    const unsigned long state = bsp_sys_enter_critical();
    output = next_output;
    lst_update_time = bsp_time_get_ms();
    can_tx_buf[param.port][cid][(mid - 1) << 1]       = output >> 8;
    can_tx_buf[param.port][cid][((mid - 1) << 1) | 1] = output & 0xff;
    bsp_sys_exit_critical(state);
}

// void dji::update_torque(float val) {
//     ;
// }

void dji::clear() {
    const uint8_t cid = id_trans(ctrl_id);
    const uint8_t mid = param.id < 5 ? param.id : param.id - 4;
    const unsigned long state = bsp_sys_enter_critical();
    output = 0;
    can_tx_buf[param.port][cid][(mid - 1) << 1]       = 0;
    can_tx_buf[param.port][cid][((mid - 1) << 1) | 1] = 0;
    feedback = feedback_t{};
    lst_angle = 0;
    feedback_received = false;
    bsp_sys_exit_critical(state);
}


// speed 为 raw, 但 current 不是
static float calc_power(float k0, float k1, float k2, float k3,
                        float k4, float k5, float current, float speed) {
    current = std::abs(current);
    speed = std::abs(speed);
    current /= 1000;
    return k0 + k1 * current + k2 * speed
           + k3 * current * speed
           + k4 * current * current
           + k5 * speed * speed;
}

// 不妨考虑速度不突变, 假设控制量为 val, 估计功率
float dji::predict_power(float val) const {
    auto [k0, k1, k2, k3, k4, k5] = power_param;
    return calc_power(k0, k1, k2, k3, k4, k5, val, state().raw.speed);
}

dji::feedback_t dji::state() const {
    const unsigned long state = bsp_sys_enter_critical();
    const feedback_t copy = feedback;
    bsp_sys_exit_critical(state);
    return copy;
}

void dji::decoder(bsp_can_e device, uint32_t id, const uint8_t* data, size_t len) {
    if (!data || len != 8) return;

    dji* p = registry.find_by_feedback_id(device, static_cast<uint16_t>(id));
    if (!p) return;

    const uint16_t raw_angle = static_cast<uint16_t>(data[0] << 8 | data[1]);

    const unsigned long state = bsp_sys_enter_critical();
    auto& fb = p->feedback;

    p->update_round(raw_angle);

    fb.raw.angle   = raw_angle;
    fb.raw.speed   = static_cast<int16_t>(data[2] << 8 | data[3]);
    fb.raw.current = static_cast<int16_t>(data[4] << 8 | data[5]);
    fb.raw.temp    = data[6];

    // angle 始终反映编码器轴单圈绝对值；round 仅记录启动后检测到的过零圈数。
    fb.angle = static_cast<float>(raw_angle) * two_pi
             / static_cast<float>(encoder_resolution);

    // speed - rad/s
    fb.speed = static_cast<float>(fb.raw.speed) / 30.f
             * static_cast<float>(M_PI) / p->ratio;
    // current - A, torque - Nm
    switch (p->model) {
    case GM6020:
        fb.current = static_cast<float>(fb.raw.current) / GM6020_CURRENT_LIMIT
                   * GM6020_CURRENT_LIMIT_REAL;
        fb.torque  = fb.current * p->ratio / GM6020_DEFAULT_RATIO
                   * GM6020_TORQUE_CONSTANT;
        break;
    case M3508:
        fb.current = static_cast<float>(fb.raw.current) / M3508_CURRENT_LIMIT
                   * M3508_CURRENT_LIMIT_REAL;
        fb.torque  = fb.current * p->ratio / M3508_DEFAULT_RATIO
                   * M3508_TORQUE_CONSTANT;
        break;
    case M2006:
        fb.current = static_cast<float>(fb.raw.current) / M2006_CURRENT_LIMIT
                   * M2006_CURRENT_LIMIT_REAL;
        fb.torque  = fb.current * p->ratio / M2006_DEFAULT_RATIO
                   * M2006_TORQUE_CONSTANT;
        break;
    }
    // power - W
    auto [k0, k1, k2, k3, k4, k5] = p->power_param;
    fb.power    = calc_power(k0, k1, k2, k3, k4, k5, fb.raw.current, fb.raw.speed);
    fb.timestamp = bsp_time_get_ms();
    bsp_sys_exit_critical(state);
}

void dji::init() {
    logger::info("motor '%s' inited", name);
    if (!inited) {
        const BaseType_t ok = xTaskCreate(
            task, "motor::dji", TASK_STACK_SIZE, nullptr,
            osPriorityHigh, &task_handle);
        BSP_ASSERT(ok == pdPASS);
        inited = true;
    }
    bsp_can_set_callback(param.port, feedback_id, decoder);
    enable();
}
static void task(void* args) {
    logger::info("module inited");
    uint32_t lst_wkup = bsp_time_get_ms();
    for (;;) {
        for (uint8_t i = 0; i < BSP_CAN_DEVICE_COUNT; i++) {
            if (!registry.cnt[i]) continue;
            for (uint8_t j = 0; j < registry.cnt[i]; j++) {
                const auto p = registry.devices[i][j];
                if (p->timeout_ms == -1) continue;
                const auto timeout_ms = static_cast<uint32_t>(p->timeout_ms);
                const unsigned long state = bsp_sys_enter_critical();
                const int16_t  output        = p->output;
                const uint32_t feedback_time = p->feedback.timestamp;
                const uint32_t update_time   = p->lst_update_time;
                bsp_sys_exit_critical(state);
                if (const auto cur_ms = bsp_time_get_ms();
                    output != 0 &&
                    (cur_ms - feedback_time > timeout_ms ||
                     cur_ms - update_time   > timeout_ms)) {
                    p->update(0);
                }
            }
            for (uint8_t j = 0; j < ID_COUNT; j++) {
                if (ctrl_id_used[i][j])
                    bsp_can_send(static_cast<bsp_can_e>(i), ctrl_id_map[j],
                                 can_tx_buf[i][j], 8);
            }
        }
        vTaskDelayUntil(&lst_wkup, pdMS_TO_TICKS(1));
    }
}
