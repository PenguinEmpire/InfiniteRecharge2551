#pragma once

#include <string>
#include <math.h>

#include <frc/TimedRobot.h>

#include <frc/smartdashboard/SendableChooser.h>

#include "Limelight.h"
#include "Lidar.h"

#include "AHRS.h"
#include "ctre/Phoenix.h"

#include "frc/Joystick.h"
#include "frc/SPI.h"
#include "frc2/Timer.h"
#include "frc/Spark.h"
#include "frc/Encoder.h"
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

// #include "frc/trajectory/TrajectoryGenerator.h"
// #include "frc/trajectory/Trajectory.h"
// #include "frc/trajectory/constraint/SwerveDriveKinematicsConstraint.h"
// #include "frc/trajectory/TrajectoryConfig.h"

#include "LimelightAutonomous.h"
#include "PenguinUtil.h"
#include "SwerveDrive.h"

class Robot : public frc::TimedRobot {
 public:

  // Default functions
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  // Autonomous selection
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string trajectoryAutoName = "trajectoryAutoTest";
  const std::string limelightAutoName = "limelightAutoTest";
  std::string m_autoSelected;

  // Global state
  frc2::Timer m_timer;
  units::second_t m_currentTime = units::second_t(0);
  units::meter_t elevatorPosition;
  bool elevatorAtZero;
  bool elevatorNearZero;
  bool elevatorAboveZero;

  // Subsystems
  Limelight m_limelight;
  SwerveDrive m_drivetrain;
  std::shared_ptr<Lidar> m_ballLidar = std::make_shared<Lidar>(PenguinConstants::I2C::BALL_LIDAR);

  // Subsystem-specific values
    //elevator PID controller
  frc::TrapezoidProfile<units::meters>::Constraints m_elevatorConstraints{PenguinConstants::ElevatorControl::MAX_VEL, PenguinConstants::ElevatorControl::MAX_ACCEL};
  frc::ProfiledPIDController<units::meters> m_controller{
    PenguinConstants::ElevatorControl::P,
    PenguinConstants::ElevatorControl::I,
    PenguinConstants::ElevatorControl::D,
    m_constraints, PenguinConstants::DT
  };

  // Autonomous classes
  LimelightAutonomous limelightAuto{&m_limelight};

  // Other functions
  void ProcessJoysticks();
  void Drive();
  void ConfigESCs();

  // Global devices
    // Joysticks
  frc::Joystick m_leftJoystick{0};
  frc::Joystick m_rightJoystick{1};
  frc::Joystick m_gamerJoystick{2};
  frc::Joystick m_utilityJoystick{3};
  
    // Other motors
  std::shared_ptr<WPI_TalonSRX> m_elevator = std::make_shared<WPI_TalonSRX>(PenguinConstants::CAN::ELEVATOR_MASTER);
  std::shared_ptr<WPI_VictorSPX> m_elevatorSlave = std::make_shared<WPI_VictorSPX>(PenguinConstants::CAN::ELEVATOR_SLAVE);
  std::shared_ptr<WPI_TalonSRX> m_intake = std::make_shared<WPI_TalonSRX>(PenguinConstants::CAN::INTAKE);
  std::shared_ptr<WPI_TalonSRX> m_belt = std::make_shared<WPI_TalonSRX>(PenguinConstants::CAN::BELT);
  std::shared_ptr<WPI_TalonSRX> m_aimer = std::make_shared<WPI_TalonSRX>(PenguinConstants::CAN::AIMER);
  std::shared_ptr<WPI_TalonSRX> m_shooter = std::make_shared<WPI_TalonSRX>(PenguinConstants::CAN::SHOOTER);
  std::shared_ptr<WPI_TalonSRX> m_centerer = std::make_shared<WPI_TalonSRX>(PenguinConstants::CAN::CENTERER);

    // Encoders
  std::shared_ptr<frc::Encoder> m_shooterEncoder = std::make_shared<frc::Encoder>(PenguinConstants::DIO::SHOOTER_ENCODER_A, PenguinConstants::DIO::SHOOTER_ENCODER_B);
  std::shared_ptr<frc::Encoder> m_elevatorEncoder = std::make_shared<frc::Encoder>(PenguinConstants::DIO::ELEVATOR_ENCODER_A, PenguinConstants::DIO::ELEVATOR_ENCODER_B);
};
