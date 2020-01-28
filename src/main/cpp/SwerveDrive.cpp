/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "SwerveDrive.h"

SwerveDrive::SwerveDrive() {
  m_navX->Reset();
  // m_navX->SetInverted(true); // just have to take the opposite of the result every time, I guess
}

void SwerveDrive::Drive(units::meters_per_second_t fwd, units::meters_per_second_t str, units::radians_per_second_t rot, bool fieldOriented) {
  rot *= 2. / HYPOT; // pythagorean theorem

  auto states = m_kinematics.ToSwerveModuleStates(
      fieldOriented ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(fwd, str, rot, frc::Rotation2d(units::degree_t(m_navX->GetAngle())))
                    : frc::ChassisSpeeds{fwd, str, rot});

  auto [fl, fr, bl, br] = states;

  m_backLeftModule.SetDesiredState(bl);
  m_backRightModule.SetDesiredState(br);
  m_frontLeftModule.SetDesiredState(fl);
  m_frontRightModule.SetDesiredState(fr);
}

void SwerveDrive::PutDiagnostics() {
  using SD = frc::SmartDashboard;

  m_backLeftModule.PutDiagnostics();
  m_backRightModule.PutDiagnostics();
  m_frontLeftModule.PutDiagnostics();
  m_frontRightModule.PutDiagnostics();

  SD::PutNumber("Gryoscope Angle", m_navX->GetAngle()); // TODO: probably this is the right function?

  m_backLeftModule.UpdateState();
  m_backRightModule.UpdateState();
  m_frontLeftModule.UpdateState();
  m_frontRightModule.UpdateState();



}

void SwerveDrive::ResetGyroscope() {
  m_navX->SetAngleAdjustment(m_navX->GetAngle());
}

