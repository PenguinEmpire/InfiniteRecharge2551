/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "ShooterSystem.h"

ShooterSystem::ShooterSystem(std::shared_ptr<WPI_TalonSRX> shooter, std::shared_ptr<WPI_TalonSRX> belt, std::shared_ptr<WPI_TalonSRX> aimer, std::shared_ptr<WPI_TalonSRX> intake, std::shared_ptr<Lidar> ballDetector, std::shared_ptr<frc::Encoder> shooterEncoder)
    : m_shooter{shooter}, m_belt{belt}, m_aimer{aimer}, m_intake{intake},
    m_ballDetector{ballDetector}, m_shooterEncoder{shooterEncoder} {}

void ShooterSystem::Update() {
  const units::inch_t currentLidarDistance = m_lidar->GetDistance();

  if (!m_ballCurrentlyPassingInFrontOfLidar) {
    if (currentLidarDistance < NORMAL_DISTANCE) {
      m_ballCurrentlyPassingInFrontOfLidar = true;
      m_ballCount += 1;
    }
  } else {
    if (currentLidarDistance >= NORMAL_DISTANCE) {
      m_ballCurrentlyPassingInFrontOfLidar = false;
    }
  }
}

void ShooterSystem::PutDiagnostics() {
  using SD = frc::SmartDashboard;

  SD::PutBoolean("ball in front of lidar", m_ballCurrentlyPassingInFrontOfLidar);
  SD::PutNumber("balls in system", m_ballCount);
}

bool ShooterSystem::ShooterReadyToShoot() {
  const bool flywheelAdjusted = true; // TODO: adjust based off of limelight, probably. maybe also use odometry?
  return 
    flywheelAdjusted && 
    PenguinUtil::withinPercentTolerance(m_shooterEncoder->GetRate(), SHOOTING_SPEED.to<double>(), 10);
}