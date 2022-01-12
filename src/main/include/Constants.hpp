#pragma once

#include <units/length.h>

namespace Constants {
    constexpr bool BangBang = false;
    constexpr bool PID = false;
    constexpr bool StateSpace = true;
    constexpr double kCountsPerRevolution = 1440.0;
    constexpr double kWheelDiameterInch = 2.75;
    static constexpr units::meter_t kWheelDiameter = 70_mm;
}
