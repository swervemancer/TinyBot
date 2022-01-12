
#include "Robot.h"

#include <iostream>

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  if (Constants::BangBang == true) {
    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
    bang.Reset();
  }
}


void Robot::RobotPeriodic() {}


void Robot::AutonomousInit() {
  if (Constants::BangBang == true) {
    m_autoSelected = m_chooser.GetSelected();
    bang.Reset();
    // m_autoSelected = SmartDashboard::GetString("Auto Selector",
    //     kAutoNameDefault);
    std::cout << "Auto selected: " << m_autoSelected << std::endl;

    if (m_autoSelected == kAutoNameCustom) {
      // Custom Auto goes here
    } else {
      // Default Auto goes here
    }
  }
}

void Robot::AutonomousPeriodic() {
  if (Constants::BangBang == true) {
     if (m_autoSelected == kAutoNameCustom) {
      bang.Drive(units::meter_t{10000});
    } else {
      // Default Auto goes here
    }
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {
  if (Constants::BangBang == true) {
    bang.Reset();
  }
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
