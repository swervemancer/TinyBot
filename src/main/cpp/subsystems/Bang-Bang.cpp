#include "subsystems/BangBang.hpp"

BangBang::BangBang() {
    m_leftEncoder.SetDistancePerPulse(
      wpi::math::pi * kWheelDiameter.to<double>() / kCountsPerRevolution);
    m_rightEncoder.SetDistancePerPulse(
      wpi::math::pi * kWheelDiameter.to<double>() / kCountsPerRevolution);
}

void BangBang::Drive(units::meter_t setpoint) {
    while (GetAverageDistance() < setpoint) {
        m_drive.ArcadeDrive(1.0, 0.0, false);
    }
    m_drive.ArcadeDrive(0.0, 0.0, false);
}

units::meter_t BangBang::GetAverageDistance() {
    return (units::meter_t{m_leftEncoder.GetDistance()} + units::meter_t{m_rightEncoder.GetDistance()}) / 2.0;
}

void BangBang::Reset() {
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
}