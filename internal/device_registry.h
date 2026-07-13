#pragma once

#include "bsp/can.h"

#include <cstdint>

namespace motor::internal {

template <typename T, uint8_t Limit>
struct device_registry {
    T*     devices[BSP_CAN_DEVICE_COUNT][Limit] = {};
    uint8_t cnt[BSP_CAN_DEVICE_COUNT]           = {};

    bool add(bsp_can_e port, T* dev) {
        if (cnt[port] >= Limit) return false;
        devices[port][cnt[port]++] = dev;
        return true;
    }

    [[nodiscard]] T* find_by_feedback_id(bsp_can_e port, uint16_t fid) const {
        for (uint8_t i = 0; i < cnt[port]; i++)
            if (devices[port][i]->feedback_id == fid)
                return devices[port][i];
        return nullptr;
    }
};

}
