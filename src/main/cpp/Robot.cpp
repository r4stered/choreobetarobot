// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <choreo/Choreo.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"

void Robot::RobotInit() {
  drivetrain.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        auto forward = -1.0 * frc::ApplyDeadband(controller.GetLeftY(), 0.1) *
                       constants::Swerve::kMaxLinearSpeed;
        auto strafe = -1.0 * frc::ApplyDeadband(controller.GetLeftX(), 0.1) *
                      constants::Swerve::kMaxLinearSpeed;
        auto turn = -1.0 * frc::ApplyDeadband(controller.GetRightX(), 0.1) *
                    constants::Swerve::kMaxAngularSpeed;
        drivetrain.Drive(forward, strafe, turn);
      },
      {&drivetrain}));

  rotationController.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
}

void Robot::RobotPeriodic() {
  drivetrain.Log();
  frc2::CommandScheduler::GetInstance().Run();
  frc::SmartDashboard::PutData("FRC Field", &debugField);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {
  drivetrain.Stop();
}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  autoFactory.Bind("test", [] { return frc2::cmd::Print("Hello from marker"); });
  straightTraj = autoFactory.Trajectory("Straight", loop);
  loop.Enabled().OnTrue(frc2::cmd::RunOnce([this] {
                          drivetrain.ResetPose(
                              straightTraj.GetInitialPose().value(), true);
                        })
                            .AndThen(straightTraj.Cmd())
                            .WithName("Straight Traj Name"));

  m_autonomousCommand = loop.Cmd().WithName("Test Auto Loop Cmd");

  if (m_autonomousCommand.has_value()) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand.has_value()) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::SimulationPeriodic() {
  debugField.GetObject("EstimatedRobot")->SetPose(drivetrain.GetPose());
  debugField.GetObject("EstimatedRobotModules")
      ->SetPoses(drivetrain.GetModulePoses());

  units::ampere_t totalCurrent = drivetrain.GetCurrentDraw();
  units::volt_t loadedBattVolts =
      frc::sim::BatterySim::Calculate({totalCurrent});
  frc::sim::RoboRioSim::SetVInVoltage(loadedBattVolts);
}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
