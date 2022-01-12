#include "subsystems/PID.hpp"

PID::PID() {
    m_rightMotor.SetInverted(true);

    m_leftEncoder.SetDistancePerPulse(
      wpi::math::pi * kWheelDiameter.to<double>() / kCountsPerRevolution);
    m_rightEncoder.SetDistancePerPulse(
      wpi::math::pi * kWheelDiameter.to<double>() / kCountsPerRevolution);
    
    Reset();
}

void PID::Drive(units::meters_per_second_t xSpeed,
                       units::radians_per_second_t rot) {
    SetSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
}

void PID::DriveAtSetpoint(units::meter_t distance, units::meters_per_second_t speed) {
    while (GetAverageDistance() < distance) {
       SetSpeeds(m_kinematics.ToWheelSpeeds({speed, 0_mps, units::radians_per_second_t{0}}));
    }
    SetSpeeds(m_kinematics.ToWheelSpeeds({0_mps, 0_mps, units::radians_per_second_t{0}}));
}

void PID::SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) {
    const double leftOutput = m_leftPid.Calculate(
      m_leftEncoder.GetRate(), speeds.left.to<double>());
    const double rightOutput = m_rightPid.Calculate(
      m_rightEncoder.GetRate(), speeds.right.to<double>());

    m_leftMotor.SetVoltage(units::volt_t{leftOutput});
    m_rightMotor.SetVoltage(units::volt_t{rightOutput});
}

void PID::Logging() {
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("datatable");
    pEntry = table->GetEntry("Proportional");
    iEntry = table->GetEntry("Integral");
    dEntry = table->GetEntry("Derivitative");
    pEntry.SetDouble(kP);
    iEntry.SetDouble(kI);
    dEntry.SetDouble(kD);
}

void PID::Periodic() {
  frc::SmartDashboard::PutData("Left PID Controller", &m_leftPid);
  frc::SmartDashboard::PutData("Right PID Controller", &m_rightPid);
}

void PID::UpdateOdometry() {
    m_odometry.Update(m_gyro.GetRotation2d(),
                    units::meter_t(m_leftEncoder.GetDistance()),
                    units::meter_t(m_rightEncoder.GetDistance()));
}

units::meter_t PID::GetAverageDistance() { 
    return (units::meter_t{m_leftEncoder.GetDistance()} + units::meter_t{m_rightEncoder.GetDistance()}) / 2.0;
}

void PID::Reset() {
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
}

int PID::GetLeftEncoderCount() {
  return m_leftEncoder.Get();
}

int PID::GetRightEncoderCount() {
  return m_rightEncoder.Get();
}

units::meter_t PID::GetLeftDistance() {
  return units::meter_t(m_leftEncoder.GetDistance());
}

units::meter_t PID::GetRightDistance() {
  return units::meter_t(m_rightEncoder.GetDistance());
}

double PID::GetAccelX() {
  return m_accelerometer.GetX();
}

double PID::GetAccelY() {
  return m_accelerometer.GetY();
}

double PID::GetAccelZ() {
  return m_accelerometer.GetZ();
}

double PID::GetGyroAngleX() {
  return m_gyro.GetAngleX();
}

double PID::GetGyroAngleY() {
  return m_gyro.GetAngleY();
}

double PID::GetGyroAngleZ() {
  return m_gyro.GetAngleZ();
}

void PID::ResetGyro() {
  m_gyro.Reset();
}