/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "ctre/Phoenix.h"
#include "Lidar.h"
#include "frc/Encoder.h"

class ShooterSystem {
 public:
  ShooterSystem(std::shared_ptr<WPI_TalonSRX> shooter, std::shared_ptr<WPI_TalonSRX> belt, std::shared_ptr<WPI_TalonSRX> aimer, std::shared_ptr<WPI_TalonSRX> intake, std::shared_ptr<Lidar> ballDetector, std::shared_ptr<frc::Encoder> shooterEncoder);

 private:
  std::shared_ptr<WPI_TalonSRX> m_shooter;
  std::shared_ptr<WPI_TalonSRX> m_belt;
  std::shared_ptr<WPI_TalonSRX> m_aimer;
  std::shared_ptr<WPI_TalonSRX> m_intake;

  std::shared_ptr<Lidar> m_ballDetector;

  std::shared_ptr<frc::Encoder> m_shooterEncoder;





};
