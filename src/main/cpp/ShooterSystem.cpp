/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "ShooterSystem.h"

// ShooterSystem::ShooterSystem(int shooterID, std::shared_ptr<WPI_TalonSRX> belt, std::shared_ptr<WPI_TalonSRX> aimer, std::shared_ptr<WPI_TalonSRX> intake, std::shared_ptr<Lidar> ballDetector)
//     : m_shooter{shooterID, rev::CANSparkMax::MotorType::kBrushless},
//     m_belt{belt}, m_aimer{aimer}, m_intake{intake},
//     m_ballDetector{ballDetector},
//     m_shooterEncoder{m_shooter.GetEncoder()}, m_shooterPID{m_shooter.GetPIDController()}
// {
//   m_ballCurrentlyPassingInFrontOfLidar = BallDetectedByLidar();
// }

ShooterSystem::ShooterSystem(int shooterID, int beltID, int aimerID, int intakeID, frc::I2C::Port lidarPort)
  : m_shooter{shooterID, rev::CANSparkMax::MotorType::kBrushless},
  m_belt{beltID},
  m_aimer{aimerID},
  m_intake{intakeID},

  m_ballDetector{lidarPort}
{
  m_ballCurrentlyPassingInFrontOfLidar = BallDetectedByLidar();

  m_shooterPID.SetP(1);
  m_shooterPID.SetI(0);
  m_shooterPID.SetD(0);
}

void ShooterSystem::Update() {
  m_currentLidarDistance = m_ballDetector.GetDistance();
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
    flywheelAdjusted && \
    PenguinUtil::withinPercentTolerance(m_shooterEncoder.GetVelocity(), SHOOTING_SPEED.to<double>(), 10);
}

bool ShooterSystem::BallDetectedByLidar() {
  return m_currentLidarDistance < NORMAL_DISTANCE;
}

void ShooterSystem::Intake() {
  m_intake.Set(-0.8);
  if (m_ballCount <= 2 && m_ballDetectedByLidar) {
    m_belt.Set(-0.15);
  }
}

void ShooterSystem::RunMotorIf(bool run) {
  if (run) {
    m_shooterPID.SetReference(SHOOTING_SPEED.to<double>(), rev::ControlType::kVelocity);
  } else {
    m_shooterPID.SetReference(0, rev::ControlType::kVelocity);
  }
}

void ShooterSystem::RunIntakeIf(bool run) {
  if (run) {
    m_intake.Set(1);
  } else {
    m_intake.Set(0);
  }
}

void ShooterSystem::RunBeltIf(bool run) {
  if (run) {
    m_belt.Set(-1);
  } else {
    m_belt.Set(0);
  }
}

void ShooterSystem::ConfigESCs() {
  m_intake.ConfigFactoryDefault();
  m_belt.ConfigFactoryDefault();
  m_aimer.ConfigFactoryDefault();
  // m_shooter.ConfigFactoryDefault();

  m_intake.SetInverted(false);
  m_belt.SetInverted(false);
  m_aimer.SetInverted(true);
  // m_shooter.SetInverted(false);


  // m_aimer.ConfigForwardSoftLimitThreshold(___); // TODO
  // m_aimer.ConfigReverseSoftLimitThreshold(___); // TODO
  // m_aimer.ConfigForwardSoftLimitEnable(true); // TODO: when we get the above two done
  // m_aimer.ConfigReverseSoftLimitEnable(true); // TODO: when we get the above two done

  m_aimer.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative);
  m_aimer.ConfigFeedbackNotContinuous(false); // TODO: is this true?
  m_aimer.SetSensorPhase(false); // TODO
}