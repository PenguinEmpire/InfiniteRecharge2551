/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <units/units.h>

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  trajectoryConfig.SetKinematics(m_drivetrain.m_kinematics);
  exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    // Start at the origin facing the +X direction
    frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
    // Pass through these two interior waypoints, making an 's' curve path
    {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)},
    // End 3 meters straight ahead of where we started, facing forward
    frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
    // Pass the config
    trajectoryConfig      
  );

  m_timer.Reset();
  m_timer.Start();

}

void Robot::RobotPeriodic() {
  m_drivetrain.PutDiagnostics();
  m_drivetrain.Update();
}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {

  double currentTime = m_timer.Get();
  frc::Trajectory::State state = exampleTrajectory.Sample(units::second_t(currentTime));
  frc::Trajectory::State nextState = exampleTrajectory.Sample(units::second_t(currentTime + 0.02));
  frc::Pose2d rel = nextState.pose.RelativeTo(state.pose);
  units::meter_t x = rel.Translation().X();
  units::meter_t y = rel.Translation().Y();
  units::radian_t omega = rel.Rotation().Radians();

  units::meters_per_second_t y_ = y / units::second_t(0.02); 
  units::meters_per_second_t x_ = x / units::second_t(0.02);     
  units::radians_per_second_t omega_ = omega / units::second_t(0.02); 

  m_drivetrain.Drive(y_, x_, omega_, false);
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  Drive();
  ProcessJoysticks();
}

void Robot::TestPeriodic() {}

void Robot::Drive() {
  using SD = frc::SmartDashboard;

  bool fieldOrient = !m_rightJoystick.GetRawButton(3);

  double forward = m_rightJoystick.GetRawAxis(1);
  // SD::PutNumber("fwd raw", forward);
  forward = PenguinUtil::smartDeadband(forward, -0.2, 0.16);
  forward *= -1;
  forward = copysign(pow(forward, 2), forward);

  double strafe = m_rightJoystick.GetRawAxis(0);
  // SD::PutNumber("str raw", strafe);
  strafe = PenguinUtil::smartDeadband(strafe, -0.18, 0.15);
  strafe *= -1;
  strafe = copysign(pow(strafe, 2), strafe);

  double rotation = m_leftJoystick.GetRawAxis(2);
  // SD::PutNumber("rot raw", rotation);
  rotation = PenguinUtil::smartDeadband(rotation, -0.31, 0.02, 0.15);
  rotation *= -1;
  rotation = copysign(pow(rotation, 2), rotation);

  SwerveDrive::ModuleLocation centerOfRotation;
  if (m_leftJoystick.GetRawButton(5)) {
    centerOfRotation = SwerveDrive::ModuleLocation::FRONT_LEFT;
  } else if (m_leftJoystick.GetRawButton(6)) {
    centerOfRotation = SwerveDrive::ModuleLocation::FRONT_RIGHT;
  } else if (m_leftJoystick.GetRawButton(3)) {
    centerOfRotation = SwerveDrive::ModuleLocation::BACK_LEFT;
  } else if (m_leftJoystick.GetRawButton(4)) {
    centerOfRotation = SwerveDrive::ModuleLocation::BACK_RIGHT;
  } else {
    centerOfRotation = SwerveDrive::ModuleLocation::NONE;
  }

  m_drivetrain.Drive(forward, strafe, rotation, fieldOrient, centerOfRotation);

  SD::PutNumber("fwd command", forward);
  SD::PutNumber("str command", strafe);
  SD::PutNumber("rot command", rotation);
}

void Robot::ProcessJoysticks() {
  if(m_leftJoystick.GetRawButtonPressed(11)) {
    m_drivetrain.ResetGyroscope();
  }
  // if (m_leftJoystick.GetRawButtonPressed(12)) {m_drivetrain.UpdateModuleEncoderOFfsetAngles();} // TODO?
}

#ifndef RUNNING_FRC_TESTS
int main() {return frc::StartRobot<Robot>();}
#endif