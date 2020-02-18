/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "SwerveDrive.h"

SwerveDrive::SwerveDrive() {
  ResetGyroscope();
  // m_navX->SetInverted(true); // just have to take the opposite of the result every time, I guess

  printf("front left angle offset: %f", FRONT_LEFT_ANGLE_OFFSET.to<double>());
  printf("front left angle offset: %f", FRONT_RIGHT_ANGLE_OFFSET.to<double>());
  printf("front left angle offset: %f", BACK_LEFT_ANGLE_OFFSET.to<double>());
  printf("front left angle offset: %f", BACK_RIGHT_ANGLE_OFFSET.to<double>());
}

void SwerveDrive::Drive(units::meters_per_second_t fwd, units::meters_per_second_t str, units::radians_per_second_t rot, bool fieldOriented, bool b, frc::Translation2d centerOfRotation = frc::Translation2d()) {
  // rot *= 2. / HYPOT;

  auto states = m_kinematics.ToSwerveModuleStates(
    fieldOriented
      ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(fwd, str, rot, frc::Rotation2d(units::degree_t(fmod(-m_navX->GetAngle(), 360))))
      : frc::ChassisSpeeds{fwd, str, rot},
    centerOfRotation
  );

  m_kinematics.NormalizeWheelSpeeds(&states, 1_mps);

  auto [fl, fr, bl, br] = states;

  m_backLeftModule.SetDesiredState(bl, b);
  m_backRightModule.SetDesiredState(br, b);
  m_frontLeftModule.SetDesiredState(fl, b);
  m_frontRightModule.SetDesiredState(fr, b);
}

void SwerveDrive::Drive(double fwd, double str, double rot, bool fieldOriented, SwerveDrive::ModuleLocation COR, bool b) {

  frc::Translation2d centerOfRotation;
  switch (COR) {
    case BACK_LEFT:
      centerOfRotation = BACK_LEFT_CORNER_LOCATION;
      break;
    case BACK_RIGHT:
      centerOfRotation = BACK_RIGHT_CORNER_LOCATION;
      break;
    case FRONT_LEFT:
      centerOfRotation = FRONT_LEFT_CORNER_LOCATION;
      break;
    case FRONT_RIGHT:
      centerOfRotation = FRONT_RIGHT_CORNER_LOCATION;
      break;
    default:
      centerOfRotation = Translation2d();
  }

  Drive(
    fwd * K_MAX_VELOCITY,
    str * K_MAX_VELOCITY,
    rot * K_MAX_ANGULAR_VELOCITY,
    fieldOriented, b,
    centerOfRotation
  );
}

void SwerveDrive::Driveish(double fwd, double str, double rot) {
  const double angle = str * PenguinUtil::TWO_PI;
  const double speed = 0.05;
  m_backLeftModule.SetDirectly(angle, speed);
  m_backRightModule.SetDirectly(angle, speed);
  m_frontLeftModule.SetDirectly(angle, speed);
  m_frontRightModule.SetDirectly(angle, speed);
}

void SwerveDrive::PutDiagnostics() {
  using SD = frc::SmartDashboard;

  m_backLeftModule.PutDiagnostics();
  m_backRightModule.PutDiagnostics();
  m_frontLeftModule.PutDiagnostics();
  m_frontRightModule.PutDiagnostics();

  SD::PutNumber("Gryoscope Angle", m_navX->GetAngle()); // TODO: probably this is the right function?
}

void SwerveDrive::Update() {
  // m_backLeftModule.SDS_UpdateState();
  // m_backRightModule.SDS_UpdateState();
  // m_frontLeftModule.SDS_UpdateState();
  // m_frontRightModule.SDS_UpdateState();

  m_backLeftModule.ReadSensors();
  m_backRightModule.ReadSensors();
  m_frontLeftModule.ReadSensors();
  m_frontRightModule.ReadSensors();
}

void SwerveDrive::ResetGyroscope() {
  // m_navX->SetAngleAdjustment(m_navX->GetAngle());
  m_navX->Reset();
  // m_navX->SetAngleAdjustment(m_navX->GetUnadjustedAngle()); // that's not a real function, but it's how they do it in SDS
}

void SwerveDrive::UpdateModuleEncoderOFfsetAngles() {
  m_backLeftModule.UpdateAnalogOffset();
  m_backRightModule.UpdateAnalogOffset();
  m_frontLeftModule.UpdateAnalogOffset();
  m_frontRightModule.UpdateAnalogOffset();
}