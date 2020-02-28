#include <units/units.h>

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(trajectoryAutoName, trajectoryAutoName);
  m_chooser.AddOption(limelightAutoName, limelightAutoName);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  m_timer.Reset();
  m_timer.Start();
}

 /* Runs every packet. Runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard integrated updating. */
void Robot::RobotPeriodic() {
  m_currentTime = m_timer.Get();

  m_drivetrain.PutDiagnostics();
  m_drivetrain.Update();
}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == trajectoryAutoName) {
    // trajectoryConfig.SetKinematics(m_drivetrain.m_kinematics);
    // exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    //   // Start at the origin facing the +X direction
    //   frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
    //   // Pass through these two interior waypoints, making an 's' curve path
    //   {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)},
    //   // End 3 meters straight ahead of where we started, facing forward
    //   frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
    //   // Pass the config
    //   trajectoryConfig      
    // );
  } else if (m_autoSelected == limelightAutoName) {

  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == trajectoryAutoName) {
    const units::second_t timeStep = units::second_t(0.02);

    if (m_currentTime < exampleTrajectory.TotalTime()) {
      // frc::Trajectory::State state = exampleTrajectory.Sample(m_currentTime);
      // frc::Trajectory::State nextState = exampleTrajectory.Sample(m_currentTime + timeStep);
      // frc::Pose2d rel = nextState.pose.RelativeTo(state.pose);
      // units::meter_t x = rel.Translation().X();
      // units::meter_t y = rel.Translation().Y();
      // units::radian_t omega = rel.Rotation().Radians();

      // units::meters_per_second_t y_ = y / timeStep;
      // units::meters_per_second_t x_ = x / timeStep;
      // units::radians_per_second_t omega_ = omega / timeStep;

      // m_drivetrain.Drive(y_, x_, omega_, false);
    }
  } else if (m_autoSelected == limelightAutoName) {
    if (limelightAuto.GetState() == LimelightAutonomous::AutoState::ALIGNING) {
      const units::degree_t currentAngle = m_drivetrain.GetAngle();
      units::radians_per_second_t desiredRot = limelightAuto.CalculateRot(currentAngle);
      
      m_drivetrain.Drive(0_mps, 0_mps, desiredRot, false);
    } else if (limelightAuto.GetState() == LimelightAutonomous::AutoState::SHOOTING) {
      // TODO: shoot the balls. can't just run belt and shooter at the same time
      // or: we could here, but we need the capability to not.
      shooter.Set(ControlMode::Velocity, 1);
      belt.Set(1);
    } else {}

  } else {
    if (m_currentTime < units::second_t(10)) {
      m_drivetrain.Drive(0.5_mps, 0_mps, units::radians_per_second_t(0), false);
    }
  }
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

  frc::Translation2d centerOfRotation;
  if (m_leftJoystick.GetRawButton(5)) {
    centerOfRotation = m_drivetrain.FRONT_LEFT_CORNER_LOCATION;
  } else if (m_leftJoystick.GetRawButton(6)) {
    centerOfRotation = m_drivetrain.FRONT_RIGHT_CORNER_LOCATION;
  } else if (m_leftJoystick.GetRawButton(3)) {
    centerOfRotation = m_drivetrain.BACK_LEFT_CORNER_LOCATION;
  } else if (m_leftJoystick.GetRawButton(4)) {
    centerOfRotation = m_drivetrain.BACK_RIGHT_CORNER_LOCATION;
  } else {
    centerOfRotation = frc::Translation2d();
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
