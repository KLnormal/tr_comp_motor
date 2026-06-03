#include "motor/gyj.h"

#include "bsp/can.h"
#include "bsp/time.h"
#include "utils/logger.h"

#include "task.h"

#include <cstring>
#include <cmath>
#include <cstdio>

#include "cmsis_os2.h"

using namespace motor;

#define TASK_STACK_SIZE 512

const float fpi = M_PI;

// FreeRTOS Task
static bool inited = false;
[[noreturn]] static void task(void *args);
static TaskHandle_t task_handle = nullptr;

// Device Class Ptr & Device Cnt (以 bsp_can_e 分类，保证一条 can 总线上的设备 id 不冲突)
#define ID_COUNT 2
static constexpr uint16_t ctrl_id_map[] = { 0xaf, 0xae };
static uint8_t id_trans(uint16_t x) {
    for(uint8_t i = 0; i < ID_COUNT; i++) if(ctrl_id_map[i] == x) return i;
    BSP_ASSERT(false); return 0;
}
static gyj* device_ptr[BSP_CAN_DEVICE_COUNT][GYJ_MOTOR_LIMIT];
static uint8_t device_cnt[BSP_CAN_DEVICE_COUNT];
static bool ctrl_id_used[BSP_CAN_DEVICE_COUNT][ID_COUNT + 1];
static uint8_t can_tx_buf[BSP_CAN_DEVICE_COUNT][ID_COUNT + 1][8];

gyj::gyj(const char *name, const param_t &param, float ratio) : ratio(ratio), param(param) {
    BSP_ASSERT(ratio > 0.f);
    BSP_ASSERT(0 <= param.port and param.port < BSP_CAN_DEVICE_COUNT);
    BSP_ASSERT(param.id < GYJ_MOTOR_LIMIT);
    BSP_ASSERT(device_cnt[param.port] < GYJ_MOTOR_LIMIT);
    std::snprintf(this->name, sizeof(this->name), "%s", name != nullptr ? name : "");

    ctrl_id = param.id < 4 ? 0xaf : 0xae;
    feedback_id = 0xf0 + param.id;

    device_ptr[param.port][device_cnt[param.port] ++] = this;
    ctrl_id_used[param.port][id_trans(ctrl_id)] = true;
}

void gyj::update(float val) {
    if (!enabled) return;
    output = static_cast <int16_t> (val);
    uint8_t cid = id_trans(ctrl_id), mid = param.id < 4 ? param.id : param.id - 4;
    can_tx_buf[param.port][cid][mid << 1] = output >> 8;
    can_tx_buf[param.port][cid][mid << 1 | 1] = output & 0xff;
}

void gyj::clear() {
    update(0);
    memset(&feedback, 0, sizeof feedback);
}

static uint8_t mode_data[BSP_CAN_DEVICE_COUNT][8];

void gyj::set_mode(mode_e m, bool have_feedback, bool modified) {
    if (modified) {
        this->param.mode = m, this->param.have_feedback = have_feedback;
    }
    mode_data[this->param.port][this->param.id] = ((this->param.have_feedback << 3) & 0x08) | (this->param.mode & 0x07);
    bsp_can_send(this->param.port, 0x0a, mode_data[this->param.port], 8);
}


void gyj::decoder(bsp_can_e device, uint32_t id, const uint8_t *data, size_t len) {
    if (!device_cnt[device] or len != 8) return;

    gyj *p = nullptr;
    for(uint8_t i = 0; i < device_cnt[device]; i++) {
        if(device_ptr[device][i]->feedback_id == id) {
            p = device_ptr[device][i];
            break;
        }
    }
    if(p == nullptr or !p->enabled) return;

    auto &fb = p->feedback;

    if (data[0]>>4 != p->param.id) BSP_ASSERT(false);

    fb.raw.current = static_cast<int16_t>(data[1] << 8 | data[2]);
    fb.raw.speed = static_cast<int16_t>(data[3] << 8 | data[4]);
    fb.raw.angle = static_cast<int16_t>(data[5] << 8 | data[6]);
    fb.raw.temp = data[7];

    // 暂时不知道原始数据是什么单位，先直接赋值
    fb.angle = fb.raw.angle;
    fb.speed = fb.raw.speed;
    fb.current = fb.raw.current;
    fb.timestamp = bsp_time_get_ms();
}

void gyj::init() {
    logger::info("motor '%s' inited", name);
    if (!inited) {
        const BaseType_t ok = xTaskCreate(
            task,
            "motor::gyj",
            TASK_STACK_SIZE,
            nullptr,
            osPriorityHigh,
            &task_handle
        );
        BSP_ASSERT(ok == pdPASS);
        inited = true;
    }
    bsp_can_set_callback(param.port, feedback_id, decoder);
    enable();
}

// 控制包 200Hz
static void task(void *args) {
    logger::info("module inited");
    for (;;) {
        for(uint8_t i = 0; i < BSP_CAN_DEVICE_COUNT; i++) {
            if(!device_cnt[i]) continue;
            for(uint8_t j = 0; j < ID_COUNT; j++) {
                if(ctrl_id_used[i][j]) {
                    bsp_can_send(static_cast<bsp_can_e>(i), ctrl_id_map[j], can_tx_buf[i][j], 8);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
