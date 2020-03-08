/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "ctre/Phoenix.h"

#include "Lidar.h"
#include "Limelight.h"

#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/Encoder.h"

#include "units/units.h"

class ShooterSystem {
 public:
  ShooterSystem(std::shared_ptr<WPI_TalonSRX> shooter, std::shared_ptr<WPI_TalonSRX> belt, std::shared_ptr<WPI_TalonSRX> aimer, std::shared_ptr<WPI_TalonSRX> intake, std::shared_ptr<Lidar> ballDetector, std::shared_ptr<frc::Encoder> shooterEncoder);

  void Update();
  void PutDiagnostics();

 private:
  std::shared_ptr<WPI_TalonSRX> m_shooter;
  std::shared_ptr<WPI_TalonSRX> m_belt;
  std::shared_ptr<WPI_TalonSRX> m_aimer;
  std::shared_ptr<WPI_TalonSRX> m_intake;

  std::shared_ptr<Lidar> m_ballDetector;

  std::shared_ptr<frc::Encoder> m_shooterEncoder;

  /**  The distance the lidar should (approximately) read if there are balls in front of it.
   * This should be the smallest value it will ever reasonably read with no ball, not the average value.
   * TODO: 15 inches is a placeholder
   */
  static constexpr units::inch_t NORMAL_DISTANCE = 15_in * 0.8;

  bool ballCurrentlyPassingInFrontOfLidar;

  /** How many balls are currently being carried by the robot. Hard-coded as 3 to start. */
  int ballCount = 3;



  static constexpr units::revolutions_per_minute_t SHOOTING_SPEED = units::revolutions_per_minute_t(2670);
  bool ShooterReadyToShoot();


};
