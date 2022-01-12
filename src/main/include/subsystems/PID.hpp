#pragma once

#include <frc/smartdashboard/Field2d.h>
#include <frc/BuiltInAccelerometer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Encoder.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/controller/PIDController.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <units/length.h>

#include "gyro/RomiGyro.hpp"
#include "Constants.hpp"

class PID {

    public:

        PID();

        void Drive(units::meters_per_second_t xSpeed,
                       units::radians_per_second_t rot);

        void DriveAtSetpoint(units::meter_t distance, units::meters_per_second_t speed);

        void SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds);

        void Logging();

        void UpdateOdometry();

        void Periodic();

        void Reset();

        void ResetEncoders();

        int GetLeftEncoderCount();

        int GetRightEncoderCount();

        units::meter_t GetLeftDistance();

        units::meter_t GetRightDistance();

        units::meter_t GetAverageDistance();

        double GetAccelX();

        double GetAccelY();

        double GetAccelZ();

        double GetGyroAngleX();

        double GetGyroAngleY();

        double GetGyroAngleZ();

        void ResetGyro();

        nt::NetworkTableEntry leftVoltageEntry;
        nt::NetworkTableEntry rightVoltageEntry;

    private:
        frc::Spark m_leftMotor{0};
        frc::Spark m_rightMotor{1};

        frc::Encoder m_leftEncoder{4, 5};
        frc::Encoder m_rightEncoder{6, 7};

        frc2::PIDController m_leftPid {kP, kI, kD};
        frc2::PIDController m_rightPid {kP, kI, kD};

        frc::DifferentialDrive m_drive{m_leftMotor, m_rightMotor};
        frc::DifferentialDriveKinematics m_kinematics{kTrackWidth};
        frc::DifferentialDriveOdometry m_odometry{m_gyro.GetRotation2d()};

        RomiGyro m_gyro;

        frc::BuiltInAccelerometer m_accelerometer;

        nt::NetworkTableEntry pEntry;
        nt::NetworkTableEntry iEntry;
        nt::NetworkTableEntry dEntry;
};