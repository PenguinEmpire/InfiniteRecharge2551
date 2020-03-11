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

  m_beltEncoder->Reset();
  m_elevatorEncoder->Reset();

  m_beltEncoder->SetDistancePerPulse(PenguinUtil::PI / 8192);
  m_elevatorEncoder->SetDistancePerPulse(2 * PenguinUtil::PI * 3 * 4 / 8192); // TODO: how far up does the elevator go w one revolution of this motor? I think replace 3 with the gear ratio and 4 with the circumference of the first gear? Or maybe of the last. we want `Get`s to end up in meters, hopefully
  
  ConfigESCs();
}

 /* Runs every packet. Runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard integrated updating. */
void Robot::RobotPeriodic() {
  m_currentTime = m_timer.Get();
  PutDiagnostics();

  m_drivetrain.Update();
  m_shooterSystem.Update();

  elevatorPosition = units::meter_t(m_elevatorEncoder->GetDistance());  
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
    // const units::second_t timeStep = units::second_t(0.02);

    // if (m_currentTime < exampleTrajectory.TotalTime()) {
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
    // }
  } else if (m_autoSelected == limelightAutoName) {
    if (limelightAuto.GetState() == LimelightAutonomous::AutoState::ALIGNING) {
      const units::degree_t currentAngle = m_drivetrain.GetAngle();
      units::radians_per_second_t desiredRot = limelightAuto.CalculateRot(currentAngle);
      
      m_drivetrain.Drive(0_mps, 0_mps, desiredRot, false);
    } else if (limelightAuto.GetState() == LimelightAutonomous::AutoState::SHOOTING) {
      // TODO: shoot the balls. can't just run belt and shooter at the same time
      // or: we could here, but we need the capability to not.
      // m_shooter->velocity, 1); // TODO (look up syntax): velocity closed-loop control
      // m_belt->Set(1);
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
  
  m_elevator->Set(m_controller.Calculate(units::meter_t(m_elevatorEncoder->GetDistance()))); //TODO get rid of get distance call.
 
  
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

/** Joystick gets that aren't to do with driving.
 */
void Robot::ProcessJoysticks() {
  if(m_leftJoystick.GetRawButtonPressed(11)) {
    m_drivetrain.ResetGyroscope();
  }

  // if (m_leftJoystick.GetRawButtonPressed(12)) {m_drivetrain.UpdateModuleEncoderOFfsetAngles();} // TODO?

  m_elevator->Set(ControlMode::PercentOutput, m_gamerJoystick.GetRawAxis(5));
  // m_intake->Set(ControlMode::PercentOutput, m_gamerJoystick.GetRawAxis(1));
  // m_belt->Set(ControlMode::PercentOutput, m_gamerJoystick.GetRawAxis(0));
  // m_aimer->Set(ControlMode::PercentOutput, m_gamerJoystick.GetRawAxis(4));
  // m_shooter->Set(m_utilityJoystick.GetRawAxis(1)); 

  m_shooterSystem.RunMotorIf(m_gamerJoystick.GetRawButton(3));
  m_shooterSystem.RunIntakeIf(m_gamerJoystick.GetRawButton(1));
  m_shooterSystem.RunBeltIf(m_gamerJoystick.GetRawButton(2));

  //set elevator positions
  if(m_leftJoystick.GetRawButtonPressed(2)) {
    m_controller.SetGoal(2_m);
  }
  else if(m_leftJoystick.GetRawButtonPressed(3)) {
    m_controller.SetGoal(0_m);
  }

}

void Robot::ConfigESCs() {
  m_elevator->ConfigFactoryDefault();
  m_elevatorSlave->ConfigFactoryDefault();
  // m_centerer->ConfigFactoryDefault();

  m_elevatorSlave->Set(ControlMode::Follower, PenguinConstants::CAN::ELEVATOR_MASTER);

  // m_centerer->NeutralOutput(); // TODO/temp

  m_elevator->SetInverted(true);
  // m_centerer->SetInverted(false); // TODO

  /** # Things that need to be set on all of the talons (TODO):
   * open loop ramp
   * closed loop ramp
   * continuous and peak current limits
   * neutral modes
   * 
   * everything, but also with the SparkMaxs
   * 
   * also the P, I, and D and I guess F for the aimer
   * 
   * Some of this is being done already, below, for the elevator and the aimer
  */


  //m_elevator->ConfigOpenloopRamp(0.05); //TODO
  m_elevator->ConfigContinuousCurrentLimit(39);
  m_elevator->ConfigPeakCurrentLimit(0);    
  m_elevator->SetNeutralMode(NeutralMode::Brake);
  

  m_shooterSystem.ConfigESCs();

}

void Robot::PutDiagnostics() {
  // m_drivetrain.PutDiagnostics();
  m_shooterSystem.PutDiagnostics();
}

#ifndef RUNNING_FRC_TESTS
int main() {return frc::StartRobot<Robot>();}
#endif