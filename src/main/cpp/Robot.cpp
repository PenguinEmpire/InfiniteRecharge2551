/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <units/units.h>

#include "Robot.h"
#include "PenguinUtil.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {}

void Robot::RobotPeriodic()
{
  m_drivetrain.PutDiagnostics();
  ProcessJoysticks();
  m_drivetrain.Update();

}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

void Robot::ProcessJoysticks() {
  constexpr double driveDeadband = 0.09;
  bool fieldOrient = !m_rightJoystick.GetRawButton(3);

  double forward = -m_rightJoystick.GetRawAxis(1);
  forward = PenguinUtil::deadband(forward, driveDeadband);
  forward = copysign(pow(forward, 2), forward);

  double strafe = m_rightJoystick.GetRawAxis(0);
  strafe = PenguinUtil::deadband(strafe, driveDeadband);
  strafe = copysign(pow(strafe, 2), strafe);

  double rotation = m_leftJoystick.GetRawAxis(2);
  rotation = PenguinUtil::smartDeadband(rotation, -0.25, 0.05, 0.1);
  rotation = copysign(pow(rotation, 2), rotation);

  m_drivetrain.Drive(
    units::meters_per_second_t(forward),
    units::meters_per_second_t(strafe),
    units::radians_per_second_t(rotation),
    true);

  frc::SmartDashboard::PutNumber("joy fwd post-db", forward);
  frc::SmartDashboard::PutNumber("joy str post-db", strafe);
  frc::SmartDashboard::PutNumber("joy rot post-db", rotation);
}

#ifndef RUNNING_FRC_TESTS
int main() {return frc::StartRobot<Robot>();}
#endif
