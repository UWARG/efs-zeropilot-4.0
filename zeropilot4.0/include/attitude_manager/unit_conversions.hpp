#pragma once

namespace ZP_UNITS {

    // Constants strictly for conversion math
    static constexpr float PI          = 3.14159265358979323846f;
    static constexpr float DEG_TO_RAD  = PI / 180.0f;
    static constexpr float RAD_TO_DEG  = 180.0f / PI;

    /**
     * @brief Converts degrees to radians
     */
    constexpr float deg2rad(float deg) {
        return deg * DEG_TO_RAD;
    }

    /**
     * @brief Converts radians to degrees
     */
    constexpr float rad2deg(float rad) {
        return rad * RAD_TO_DEG;
    }

} // namespace ZP_UNITS
