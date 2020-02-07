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

void Robot::RobotPeriodic() {
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
  using SD = frc::SmartDashboard;

  constexpr double DRIVE_DEADBAND = 0.09;

  bool fieldOrient = !m_rightJoystick.GetRawButton(3);

  double forward = m_rightJoystick.GetRawAxis(1);
  // SD::PutNumber("fwd raw", forward);
  forward = PenguinUtil::smartDeadband(forward, -0.055, 0.079, 0.1);
  // forward = PenguinUtil::deadband(forward, DRIVE_DEADBAND);
  forward = copysign(pow(forward, 2), forward);

  double strafe = m_rightJoystick.GetRawAxis(0);
  // SD::PutNumber("str raw", strafe);
  strafe = PenguinUtil::smartDeadband(strafe, -0.109, 0.126, 0.1);
  // strafe = PenguinUtil::deadband(strafe, DRIVE_DEADBAND);
  strafe = copysign(pow(strafe, 2), strafe);

  double rotation = m_leftJoystick.GetRawAxis(2);
  rotation = PenguinUtil::smartDeadband(rotation, 0.09, 0.34, 0.1);
  rotation = copysign(pow(rotation, 2), rotation);

  m_drivetrain.Drive(forward, strafe, rotation, fieldOrient);

  SD::PutNumber("fwd command", forward);
  SD::PutNumber("str command", strafe);
  SD::PutNumber("rot command", rotation);

  if(m_leftJoystick.GetRawButtonPressed(4)) {
    m_drivetrain.ResetGyroscope();
  }

}

#ifndef RUNNING_FRC_TESTS
int main() {return frc::StartRobot<Robot>();}
#endif
