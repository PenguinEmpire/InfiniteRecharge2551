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

void SwerveDrive::Drive(double fwd, double stf, double rot, bool fieldOriented) {}

void SwerveDrive::ResetGyroscope() {
  m_navX->SetAngleAdjustment(m_navX->GetAngle());
}

