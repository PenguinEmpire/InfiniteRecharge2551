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

void SwerveDrive::Drive(units::meters_per_second_t fwd, units::meters_per_second_t str, units::radians_per_second_t rot, bool fieldOriented) {
  // rot *= 2. / HYPOT;

  auto states = m_kinematics.ToSwerveModuleStates(
      fieldOriented ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(fwd, str, rot, frc::Rotation2d(units::degree_t(fmod(-m_navX->GetAngle(), 360))))
                    : frc::ChassisSpeeds{fwd, str, rot});

  auto [fl, fr, bl, br] = states;

  m_backLeftModule.SetDesiredState(bl);
  m_backRightModule.SetDesiredState(br);
  m_frontLeftModule.SetDesiredState(fl);
  m_frontRightModule.SetDesiredState(fr);
}

void SwerveDrive::Drive(double fwd, double str, double rot, bool fieldOriented) {
  Drive(
    fwd * K_MAX_VELOCITY,
    str * K_MAX_VELOCITY,
    rot * K_MAX_ANGULAR_VELOCITY,
    fieldOriented
  );
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

  m_backLeftModule.UpdateSensors();
  m_backRightModule.UpdateSensors();
  m_frontLeftModule.UpdateSensors();
  m_frontRightModule.UpdateSensors();
}

void SwerveDrive::ResetGyroscope() {
  // m_navX->SetAngleAdjustment(m_navX->GetAngle());
  m_navX->Reset();
}

