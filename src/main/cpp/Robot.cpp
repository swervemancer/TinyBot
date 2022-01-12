
#include "Robot.h"

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/SmallString.h>
#include <iostream>
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  if (BangBangEnable == true) {
    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
    bang.Reset();
  }
  if (PIDEnable == true) {
    pid.Reset();
  }
}


void Robot::RobotPeriodic() {
  if (PIDEnable == true) {
    pid.Logging();
    pid.Periodic();
  }
}


void Robot::AutonomousInit() {
  if (BangBangEnable == true) {
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
  if(PIDEnable == true) {
    pid.Reset();
    pid.DriveAtSetpoint(units::meter_t{2}, 5_mps);
  }
}

void Robot::AutonomousPeriodic() {
  if (BangBangEnable == true) {
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
  if (BangBangEnable == true) {
    bang.Reset();
  }
  if (PIDEnable == true) {
    pid.Reset();
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
