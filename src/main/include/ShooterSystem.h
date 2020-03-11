/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
#include "rev/CANEncoder.h"
#include "rev/CANPIDController.h"

#include "PenguinUtil.h"
#include "Lidar.h"
#include "Limelight.h"

#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/Encoder.h"

#include "units/units.h"

class ShooterSystem {
 public:
  // ShooterSystem(int shooterID, std::shared_ptr<WPI_TalonSRX> belt, std::shared_ptr<WPI_TalonSRX> aimer, std::shared_ptr<WPI_TalonSRX> intake, std::shared_ptr<Lidar> ballDetector);
  ShooterSystem(int shooterID, int beltID, int aimerID, int intakeID, frc::I2C::Port lidarPort);

  bool BallDetectedByLidar();

  void Update();
  void PutDiagnostics();
  void RunMotorIf(bool run);
  void ConfigESCs();
  void RunIntakeIf(bool run);
  void RunBeltIf(bool run);

 private:

  enum Mode {
    COLLECTING, SHOOTING
  } m_mode;
  const std::unordered_map<Mode, std::string> MODE_STRINGS = {
    {Mode::COLLECTING, "Collecting"}, {Mode::SHOOTING, "Shooting"}
  };

  rev::CANSparkMax m_shooter;
  WPI_TalonSRX m_belt;
  WPI_TalonSRX m_aimer;
  WPI_TalonSRX m_intake;

  Lidar m_ballDetector;

  rev::CANEncoder m_shooterEncoder = m_shooter.GetEncoder();
  rev::CANPIDController m_shooterPID = m_shooter.GetPIDController();

  /**  The distance the lidar should (approximately) read if there are balls in front of it.
   * This should be the smallest value it will ever reasonably read with no ball, not the average value.
   * TODO: 15 inches is a placeholder
   */
  static constexpr units::inch_t NORMAL_DISTANCE = 30_in * 0.8;

  void Intake();

  bool m_ballCurrentlyPassingInFrontOfLidar;
  bool m_ballDetectedByLidar;
  units::inch_t m_currentLidarDistance; // temp, probably. Would want to move into update and make `const` eventually, but need global to reference in `PutDiagnostics()`.

  /** How many balls are currently being carried by the robot. Hard-coded as 3 to start. */
  int m_ballCount = 3;

  /** RPM of CIMs operating at maximum efficiency/max power output.
   * From here: https://motors.vex.com/vexpro-motors/cim-motor
   */
  static constexpr units::revolutions_per_minute_t SHOOTING_SPEED = units::revolutions_per_minute_t(2940);
  bool ShooterReadyToShoot();
};
