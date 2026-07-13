#pragma once

#include <cstdint>

namespace motor::internal {
    inline constexpr uint16_t dji_encoder_resolution = 8192;
    inline constexpr int32_t dji_encoder_half_range = dji_encoder_resolution / 2;
    inline constexpr float dji_two_pi = 6.28318530717958647692f;

    [[nodiscard]] constexpr bool dji_encoder_angle_valid(uint16_t raw_angle) {
        return raw_angle < dji_encoder_resolution;
    }

    [[nodiscard]] constexpr float dji_encoder_to_angle(uint16_t raw_angle) {
        return static_cast<float>(raw_angle) * dji_two_pi /
            static_cast<float>(dji_encoder_resolution);
    }

    // 根据相邻编码器值推断过零圈数；两帧间转动超过半圈时无法可靠判断。
    [[nodiscard]] inline bool dji_update_round(
        uint16_t raw_angle,
        uint16_t &last_angle,
        bool &received,
        int32_t &round
    ) {
        if (!dji_encoder_angle_valid(raw_angle)) return false;

        if (received) {
            const int32_t delta = static_cast<int32_t>(raw_angle) - last_angle;
            if (delta < -dji_encoder_half_range) {
                ++round;
            } else if (delta > dji_encoder_half_range) {
                --round;
            }
        }
        last_angle = raw_angle;
        received = true;
        return true;
    }

    inline void dji_clear_round(bool &received, int32_t &round) {
        received = false;
        round = 0;
    }
}