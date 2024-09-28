// Copyright (c) Choreo contributors

#pragma once

#include <functional>
#include <string>
#include <string_view>
#include <utility>

#include <fmt/format.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <units/length.h>
#include <units/time.h>

#include "choreo/auto/AutoBindings.h"
#include "choreo/trajectory/Trajectory.h"
#include "choreo/trajectory/TrajectorySample.h"

namespace choreo {

template <TrajectorySample SampleType>
using ChoreoControllerFunction =
    std::function<frc::ChassisSpeeds(frc::Pose2d, SampleType)>;

template <TrajectorySample SampleType>
using TrajectoryLogger = std::function<void(Trajectory<SampleType>, bool)>;

static constexpr units::meter_t DEFAULT_TOLERANCE = 3_in;
static constexpr frc::ChassisSpeeds DEFAULT_CHASSIS_SPEEDS;

template <TrajectorySample SampleType, int Year>
class AutoTrajectory {
 public:
  AutoTrajectory(std::string_view name, Trajectory<SampleType> trajectory,
                 std::function<frc::Pose2d()> poseSupplier,
                 ChoreoControllerFunction<SampleType> controller,
                 std::function<void(frc::ChassisSpeeds)> outputChassisSpeeds,
                 std::function<bool()> mirrorTrajectory,
                 std::optional<TrajectoryLogger<SampleType>> trajectoryLogger,
                 frc2::Requirements drivebaseRequirements, frc::EventLoop* loop,
                 std::shared_ptr<AutoBindings> autoBindings)
      : name{name},
        trajectory{trajectory},
        poseSupplier{std::move(poseSupplier)},
        controller{controller},
        outputChassisSpeeds{std::move(outputChassisSpeeds)},
        mirrorTrajectory{std::move(mirrorTrajectory)},
        trajectoryLogger{std::move(trajectoryLogger)},
        drivebaseRequirements{drivebaseRequirements},
        loop(loop),
        offTrigger(loop, [] { return false; }) {
    for (const auto& [key, cmdFactory] : autoBindings->GetBindings()) {
      (Active() && AtTime(key)).OnTrue(cmdFactory());
    }
  }

  frc2::CommandPtr Cmd() {
    if (trajectory.samples.size() == 0) {
      return frc2::cmd::RunOnce([this] {
               FRC_ReportError(frc::warn::Warning,
                               "Trajectory {} has no samples", name);
             })
          .WithName("Trajectory_" + name);
    }
    return frc2::FunctionalCommand(
               [this] { return CmdInitialize(); },
               [this] { return CmdExecute(); },
               [this](bool interrupted) { return CmdEnd(interrupted); },
               [this] { return CmdIsFinished(); }, drivebaseRequirements)
        .WithName("Trajectory_" + name);
  }

  std::optional<frc::Pose2d> GetInitialPose() const {
    if (trajectory.samples.size() == 0) {
      return {};
    } else {
      return trajectory.GetInitialPose<Year>(mirrorTrajectory());
    }
  }

  std::optional<frc::Pose2d> GetFinalPose() const {
    if (trajectory.samples.size() == 0) {
      return {};
    } else {
      return trajectory.GetFinalPose<Year>(mirrorTrajectory());
    }
  }

  frc2::Trigger Active() {
    return frc2::Trigger(loop, [this] { return isActive; });
  }

  frc2::Trigger Inactive() { return Active().Negate(); }

  frc2::Trigger Done() {
    return frc2::Trigger(loop, [this] {
      if (isActive) {
        wasJustActive = true;
        return false;
      } else if (wasJustActive) {
        wasJustActive = false;
        return true;
      }
      return false;
    });
  }

  frc2::Trigger AtTime(units::second_t timeSinceStart) {
    if (timeSinceStart < 0_s) {
      FRC_ReportError(frc::warn::Warning,
                      "Trigger time cannot be negative for {}", name);
      return offTrigger;
    }

    if (timeSinceStart > TotalTime()) {
      FRC_ReportError(
          frc::warn::Warning,
          "Trigger time cannout be greater than total trajectory time for {}",
          name);
      return offTrigger;
    }

    bool hasTriggered = false;

    return frc2::Trigger(loop, [this, timeSinceStart, hasTriggered]() mutable {
      units::second_t nowTimestamp = timer.Get();
      bool result = !hasTriggered && nowTimestamp >= timeSinceStart;
      if (result) {
        hasTriggered = true;
      }
      return result;
    });
  }

  frc2::Trigger AtTime(std::string_view eventName) {
    bool foundEvent = false;
    frc2::Trigger trig = offTrigger;

    for (const auto& event : trajectory.GetEvents(eventName)) {
      trig = frc2::Trigger{trig || AtTime(event.timestamp)};
      foundEvent = true;
    }

    if (!foundEvent) {
      FRC_ReportError(frc::warn::Warning, "Event \"{}\" not found for {}",
                      eventName, name);
    }

    return trig;
  }

  frc2::Trigger AtPose(std::string_view eventName,
                       units::meter_t tolerance = DEFAULT_TOLERANCE) {
    bool foundEvent = false;
    frc2::Trigger trig = offTrigger;

    for (const auto& event : trajectory.GetEvents(eventName)) {
      frc::Pose2d pose =
          trajectory.SampleAt<Year>(event.timestamp, mirrorTrajectory())
              .GetPose();
      trig = frc2::Trigger(trig || AtPose(pose, tolerance));
      foundEvent = true;
    }

    if (!foundEvent) {
      FRC_ReportError(frc::warn::Warning, "Event \"{}\" not found for {}",
                      eventName, name);
    }

    return trig;
  }

  frc2::Trigger AtTimeAndPlace(std::string_view eventName,
                               units::meter_t tolerance = DEFAULT_TOLERANCE) {
    return frc2::Trigger{AtTime(eventName) && AtPose(eventName, tolerance)};
  }

 private:
  units::second_t TimeIntoTraj() const { return timer.Get() + timeOffset; }

  units::second_t TotalTime() const { return trajectory.GetTotalTime(); }

  void LogTrajectory(bool starting) {
    if (trajectoryLogger.has_value()) {
      trajectoryLogger.value()(
          mirrorTrajectory() ? trajectory.Flipped<Year>() : trajectory,
          starting);
    }
  }

  void CmdInitialize() {
    timer.Restart();
    isActive = true;
    timeOffset = 0.0_s;
    LogTrajectory(true);
  }

  void CmdExecute() {
    auto sampleOpt =
        trajectory.SampleAt<Year>(TimeIntoTraj(), mirrorTrajectory());
    frc::ChassisSpeeds chassisSpeeds = DEFAULT_CHASSIS_SPEEDS;
    if (sampleOpt.has_value()) {
      chassisSpeeds = controller(poseSupplier(), sampleOpt.value());
    }
    outputChassisSpeeds(chassisSpeeds);
  }

  void CmdEnd(bool interrupted) {
    timer.Stop();
    if (interrupted) {
      outputChassisSpeeds(frc::ChassisSpeeds{});
    } else {
      if (trajectory.GetFinalSample().has_value()) {
        outputChassisSpeeds(
            trajectory.GetFinalSample().value().GetChassisSpeeds());
      } else {
        outputChassisSpeeds(frc::ChassisSpeeds{});
      }
    }
    isActive = false;
    LogTrajectory(false);
  }

  bool CmdIsFinished() { return TimeIntoTraj() > TotalTime(); }

  frc2::Trigger AtPose(frc::Pose2d pose, units::meter_t tolerance) {
    frc::Translation2d checkedTrans =
        mirrorTrajectory()
            ? frc::Translation2d{16.5410515_m - pose.Translation().X(),
                                 pose.Translation().Y}
            : pose.Translation();
    return frc2::Trigger{
        loop, [this, checkedTrans, tolerance] {
          frc::Translation2d currentTrans = poseSupplier().Translation();
          return currentTrans.Distance(checkedTrans) < tolerance;
        }};
  }

  std::string name;
  Trajectory<SampleType> trajectory;
  std::function<frc::Pose2d()> poseSupplier;
  ChoreoControllerFunction<SampleType> controller;
  std::function<void(frc::ChassisSpeeds)> outputChassisSpeeds;
  std::function<bool()> mirrorTrajectory;
  std::optional<TrajectoryLogger<SampleType>> trajectoryLogger;
  frc2::Requirements drivebaseRequirements;
  frc::EventLoop* loop;

  frc::Timer timer;
  bool isActive = false;
  bool wasJustActive = false;
  units::second_t timeOffset = 0_s;
  frc2::Trigger offTrigger;
};

}  // namespace choreo
