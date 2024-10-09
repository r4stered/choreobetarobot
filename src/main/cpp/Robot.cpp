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

  autoFactory.Bind("test",
                   []() { return frc2::cmd::Print("Hello from marker 1"); });
  autoFactory.Bind("MARKERTWO",
                   []() { return frc2::cmd::Print("Hello from marker 2"); });

  autoTraj1 = choreo::AutoTrajectory<choreo::SwerveSample>(
      autoFactory.Trajectory("Straight", loop));
  chooser.AddAutoRoutine(
      "ONLY STRAIGHT",
      [this](choreo::AutoFactory<choreo::SwerveSample>& factory) {
        return autoTraj1.Cmd().BeforeStarting([this] {
          drivetrain.ResetPose(autoTraj1.GetInitialPose().value(), true);
        });
      });

  chooser.Choose("ONLY STRAIGHT");
}

void Robot::RobotPeriodic() {
  drivetrain.Log();
  frc2::CommandScheduler::GetInstance().Run();
  frc::SmartDashboard::PutData("FRC Field", &debugField);
  chooser.Update();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {
  drivetrain.Stop();
}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  // autoTraj2 =
  // choreo::AutoTrajectory<choreo::SwerveSample>(autoFactory.Trajectory("Straight",
  // loop)); autoTraj3 =
  // choreo::AutoTrajectory<choreo::SwerveSample>(autoFactory.Trajectory("RedTest",
  // loop));

  // test = autoTraj2.AtTime(2.5_s);
  // m_autonomousCommand = loop.Cmd().AlongWith(
  //   frc2::cmd::Sequence(autoTraj2.Cmd())
  // );
  // drivetrain.ResetPose(autoTraj2.GetInitialPose().value(), true);
  // m_autonomousCommand = loop.Cmd().AlongWith(autoTraj3.Cmd());
  // drivetrain.ResetPose(autoTraj3.GetInitialPose().value(), true);

  m_autonomousCommand = chooser.GetSelectedAutoRoutine();

  if (m_autonomousCommand.has_value()) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {
  if (test.Get()) {
    fmt::print("Hello from test marker!\n");
  }
}

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
