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

void SwerveDrive::Drive(double fwd, double str, double rot, bool fieldOriented) {}

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

