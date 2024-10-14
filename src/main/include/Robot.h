// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include "choreo/Choreo.h"
#include "choreo/auto/AutoLoop.h"
#include "frc/smartdashboard/Field2d.h"
#include "subsystems/SwerveDrive.h"
#include <frc2/command/button/CommandXboxController.h>
#include <frc/controller/PIDController.h>
#include "choreo/auto/AutoFactory.h"
#include "choreo/auto/AutoChooser.h"
#include "Autos.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DisabledExit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;
  void TestInit() override;
  void TestPeriodic() override;
  void TestExit() override;
  void SimulationPeriodic() override;

 private:
  frc2::CommandXboxController controller{0};
  SwerveDrive drivetrain{};
  frc::PIDController xController{10, 0, 0};
  frc::PIDController yController{10, 0, 0};
  frc::PIDController rotationController{10, 0, 0};
  choreo::AutoFactory<choreo::SwerveSample> autoFactory{
      [this] { return drivetrain.GetPose(); },
      [this](frc::Pose2d refPose, choreo::SwerveSample currentSample) {
        units::meters_per_second_t xFF = currentSample.vx;
        units::meters_per_second_t yFF = currentSample.vy;
        units::radians_per_second_t rotationFF = currentSample.omega;

        units::meters_per_second_t xFeedback{xController.Calculate(
            refPose.X().value(), currentSample.x.value())};
        units::meters_per_second_t yFeedback{yController.Calculate(
            refPose.Y().value(), currentSample.y.value())};
        units::radians_per_second_t rotationFeedback{
            rotationController.Calculate(refPose.Rotation().Radians().value(),
                                         currentSample.heading.value())};

        auto speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback,
            refPose.Rotation());
        drivetrain.SetChassisSpeeds(speeds, false, false);
      },
      [] {
        auto ally = frc::DriverStation::GetAlliance();
        if (ally.has_value()) {
          if (ally == frc::DriverStation::Alliance::kRed) {
            return true;
          }
        }
        return false;
      },
      {&drivetrain},
      [this](choreo::Trajectory<choreo::SwerveSample> trajectory,
             bool starting) {
        if (!starting) {
          debugField.GetObject("Choreo Trajectory")->SetPoses({});
        } else {
          debugField.GetObject("Choreo Trajectory")
              ->SetPoses(trajectory.GetPoses());
        }
      }};
    
  choreo::AutoLoop<choreo::SwerveSample> loop{autoFactory.NewLoop("Auto Loop")};
  choreo::AutoTrajectory<choreo::SwerveSample> straightTraj;

  std::optional<frc2::CommandPtr> m_autonomousCommand;
  frc::Field2d debugField;
};
