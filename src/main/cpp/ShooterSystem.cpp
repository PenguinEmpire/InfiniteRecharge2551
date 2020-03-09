/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "ShooterSystem.h"

ShooterSystem::ShooterSystem(std::shared_ptr<WPI_TalonSRX> shooter, std::shared_ptr<WPI_TalonSRX> belt, std::shared_ptr<WPI_TalonSRX> aimer, std::shared_ptr<WPI_TalonSRX> intake, std::shared_ptr<Lidar> ballDetector, std::shared_ptr<frc::Encoder> shooterEncoder)
    : m_shooter{shooter}, m_belt{belt}, m_aimer{aimer}, m_intake{intake},
    m_ballDetector{ballDetector}, m_shooterEncoder{shooterEncoder} {
  m_ballCurrentlyPassingInFrontOfLidar = BallDetectedByLidar();
}

void ShooterSystem::Update() {
  m_currentLidarDistance = m_ballDetector->GetDistance();
  m_ballDetectedByLidar = BallDetectedByLidar();

  if (!m_ballCurrentlyPassingInFrontOfLidar) {
    if (m_ballDetectedByLidar) {
      m_ballCurrentlyPassingInFrontOfLidar = true;
      m_ballCount += 1;
    }
  } else {
    if (!m_ballDetectedByLidar) {
      m_ballCurrentlyPassingInFrontOfLidar = false;
    }
  }
}

void ShooterSystem::PutDiagnostics() {
  using SD = frc::SmartDashboard;

  SD::PutBoolean("ball in front of lidar", m_ballCurrentlyPassingInFrontOfLidar);
  SD::PutNumber("balls in system", m_ballCount);
  SD::PutNumber("lidar currently reporting (in)", units::inch_t(m_currentLidarDistance).to<double>());
}

bool ShooterSystem::ShooterReadyToShoot() {
  const bool flywheelAdjusted = true; // TODO: adjust based off of limelight, probably. maybe also use odometry?
  return 
    flywheelAdjusted && 
    PenguinUtil::withinPercentTolerance(m_shooterEncoder->GetRate(), SHOOTING_SPEED.to<double>(), 10);
}

bool ShooterSystem::BallDetectedByLidar() {
  return m_currentLidarDistance < NORMAL_DISTANCE;
}