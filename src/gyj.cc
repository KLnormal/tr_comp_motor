#include "motor/gyj.h"

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

const float fpi = M_PI;


#define ID_COUNT 2
static constexpr uint16_t ctrl_id_map[] = { 0xaf, 0xae };

static uint8_t id_trans(uint16_t x) {
    for (uint8_t i = 0; i < ID_COUNT; i++)
        if (ctrl_id_map[i] == x) return i;
    return 0;
}

static internal::device_registry<gyj, GYJ_MOTOR_LIMIT> registry;
static bool ctrl_id_used[BSP_CAN_DEVICE_COUNT][ID_COUNT + 1];
static uint8_t can_tx_buf[BSP_CAN_DEVICE_COUNT][ID_COUNT + 1][8];

// ── FreeRTOS 任务 ────────────────────────────────────────────────────────────

static bool inited = false;
[[noreturn]] static void task(void* args);
static TaskHandle_t task_handle = nullptr;


gyj::gyj(const char* name, const param_t& param, float ratio)
    : ratio(ratio), param(param)
{
    BSP_ASSERT(ratio > 0.f);
    BSP_ASSERT(0 <= param.port && param.port < BSP_CAN_DEVICE_COUNT);
    BSP_ASSERT(param.id < GYJ_MOTOR_LIMIT);
    std::snprintf(this->name, sizeof(this->name), "%s", name ? name : "");

    ctrl_id     = param.id < 4 ? 0xaf : 0xae;
    feedback_id = 0xf0 + param.id;

    registry.add(param.port, this);
    ctrl_id_used[param.port][id_trans(ctrl_id)] = true;
}


void gyj::update(float val) {
    if (!enabled) return;
    if (!std::isfinite(val)) val = 0.0f;
    const auto next_output = static_cast<int16_t>(
        std::clamp(val, -32768.0f, 32767.0f));
    const uint8_t cid = id_trans(ctrl_id);
    const uint8_t mid = param.id < 4 ? param.id : param.id - 4;
    const unsigned long state = bsp_sys_enter_critical();
    output = next_output;
    can_tx_buf[param.port][cid][mid << 1]       = output >> 8;
    can_tx_buf[param.port][cid][(mid << 1) | 1] = output & 0xff;
    bsp_sys_exit_critical(state);
}

void gyj::clear() {
    const uint8_t cid = id_trans(ctrl_id);
    const uint8_t mid = param.id < 4 ? param.id : param.id - 4;
    const unsigned long state = bsp_sys_enter_critical();
    output = 0;
    can_tx_buf[param.port][cid][mid << 1]       = 0;
    can_tx_buf[param.port][cid][(mid << 1) | 1] = 0;
    feedback = feedback_t{};
    bsp_sys_exit_critical(state);
}


gyj::feedback_t gyj::state() const {
    const unsigned long state = bsp_sys_enter_critical();
    const feedback_t copy = feedback;
    bsp_sys_exit_critical(state);
    return copy;
}

static uint8_t mode_data[BSP_CAN_DEVICE_COUNT][8];

void gyj::set_mode(mode_e m, bool have_feedback, bool modified) {
    if (modified) {
        this->param.mode = m;
        this->param.have_feedback = have_feedback;
    }
    uint8_t data[8];
    const unsigned long state = bsp_sys_enter_critical();
    mode_data[this->param.port][this->param.id] =
        ((have_feedback << 3) & 0x08) | (m & 0x07);
    std::copy_n(mode_data[this->param.port], 8, data);
    bsp_sys_exit_critical(state);
    bsp_can_send(this->param.port, 0x0a, data, 8);
}

void gyj::decoder(bsp_can_e device, uint32_t id, const uint8_t* data, size_t len) {
    if (!data || len != 8) return;

    gyj* p = registry.find_by_feedback_id(device, static_cast<uint16_t>(id));
    if (!p || !p->enabled) return;

    // CAN 数据来自外设，错误 ID 只丢弃该帧，不能按程序不变量处理。
    if (data[0] >> 4 != p->param.id) return;

    feedback_t next{};
    next.raw.current = static_cast<int16_t>(data[1] << 8 | data[2]);
    next.raw.speed   = static_cast<int16_t>(data[3] << 8 | data[4]);
    next.raw.angle   = static_cast<int16_t>(data[5] << 8 | data[6]);
    next.raw.temp    = data[7];

    // 暂时不知道原始数据是什么单位，先直接赋值
    next.angle     = next.raw.angle;
    next.speed     = next.raw.speed;
    next.current   = next.raw.current;
    next.timestamp = bsp_time_get_ms();

    const unsigned long state = bsp_sys_enter_critical();
    p->feedback = next;
    bsp_sys_exit_critical(state);
}


void gyj::init() {
    logger::info("motor '%s' inited", name);
    if (!inited) {
        const BaseType_t ok = xTaskCreate(
            task, "motor::gyj", TASK_STACK_SIZE, nullptr,
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
            for (uint8_t j = 0; j < ID_COUNT; j++) {
                if (ctrl_id_used[i][j])
                    bsp_can_send(static_cast<bsp_can_e>(i), ctrl_id_map[j],
                                 can_tx_buf[i][j], 8);
            }
        }
        vTaskDelayUntil(&lst_wkup, pdMS_TO_TICKS(5));
    }
}
