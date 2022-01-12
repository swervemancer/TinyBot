  #pragma once

#include <iostream>
#include <wpi/math>
#include <frc/BuiltInAccelerometer.h>
#include <frc/Encoder.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/drive/DifferentialDrive.h>

#include "Constants.hpp"

class BangBang {

    public:
        BangBang();

         /**
         * Drives the robot using Bang-Bang control. It uses the following logic:
         * 
         * - If not at setpoint:
         *      - voltage = 100%
         * - If at setpoint:
         *      - voltage = 0%
         */
        void Drive(units::meter_t setpoint);

        units::meter_t GetAverageDistance();

        void Reset();

    private:
        frc::Spark m_leftMotor{0};
        frc::Spark m_rightMotor{1};

        frc::Encoder m_leftEncoder{4, 5};
        frc::Encoder m_rightEncoder{6, 7};

        frc::DifferentialDrive m_drive{m_leftMotor, m_rightMotor};
        
        frc::BuiltInAccelerometer m_accelerometer;
};