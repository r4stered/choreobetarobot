// Copyright (c) Choreo contributors

#pragma once

#include <functional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>

#include "choreo/Choreo.h"
#include "choreo/auto/AutoLoop.h"

namespace choreo {

template <choreo::TrajectorySample SampleType, int Year>
class AutoFactory {
 public:
  AutoFactory(std::function<frc::Pose2d()> poseSupplier,
              ChoreoControllerFunction<SampleType> controller,
              std::function<void(frc::ChassisSpeeds)> outputChassisSpeeds,
              std::function<bool()> mirrorTrajectory,
              frc2::Requirements drivebaseRequirements, AutoBindings bindings,
              std::optional<TrajectoryLogger<SampleType>> trajectoryLogger)
      : poseSupplier{std::move(poseSupplier)},
        controller{controller},
        outputChassisSpeeds{std::move(outputChassisSpeeds)},
        mirrorTrajectory{std::move(mirrorTrajectory)},
        drivebaseRequirements{drivebaseRequirements},
        autoBindings{std::move(bindings)},
        trajectoryLogger{std::move(trajectoryLogger)} {}

  AutoLoop<SampleType, Year> NewLoop() const {
    return AutoLoop<SampleType, Year>();
  }

  AutoTrajectory<SampleType, Year> Trajectory(
      std::string_view trajectoryName, AutoLoop<SampleType, Year>& loop) const {
    std::optional<choreo::Trajectory<SampleType>> optTraj =
        Choreo::LoadTrajectory<SampleType>(trajectoryName);
    choreo::Trajectory<SampleType> trajectory;
    if (optTraj.has_value()) {
      trajectory = optTraj.value();
    } else {
      FRC_ReportError(frc::warn::Warning, "Could not load trajectory: {}",
                      trajectoryName);
    }
    AutoTrajectory<SampleType, Year> autoTraj{
        trajectoryName,      trajectory,
        poseSupplier,        controller,
        outputChassisSpeeds, mirrorTrajectory,
        trajectoryLogger,    drivebaseRequirements,
        loop.GetLoop(),      std::move(autoBindings)};
    return autoTraj;
  }

  AutoTrajectory<SampleType, Year> Trajectory(
      std::string_view trajectoryName, int splitIndex,
      AutoLoop<SampleType, Year> loop) const {
    std::optional<choreo::Trajectory<SampleType>> optTraj =
        Choreo::LoadTrajectory<SampleType>(trajectoryName);
    choreo::Trajectory<SampleType> trajectory;
    if (optTraj.has_value()) {
      trajectory = optTraj.value();
    } else {
      FRC_ReportError(frc::warn::Warning, "Could not load trajectory: {}",
                      trajectoryName);
    }
    return Trajectory(trajectory, loop);
  }

  AutoTrajectory<SampleType, Year> Trajectory(
      choreo::Trajectory<SampleType> trajectory,
      AutoLoop<SampleType, Year> loop) const {
    AutoTrajectory<SampleType, Year> autoTraj{
        trajectory.name,     trajectory,
        poseSupplier,        controller,
        outputChassisSpeeds, mirrorTrajectory,
        trajectoryLogger,    drivebaseRequirements,
        loop.GetLoop(),      autoBindings};
    loop.AddTrajectory(autoTraj);
    return autoTraj;
  }

  void Bind(std::string_view name, frc2::CommandPtr cmd) {
    autoBindings = std::move(autoBindings).Bind(name, cmd);
  }

 private:
  std::function<frc::Pose2d()> poseSupplier;
  ChoreoControllerFunction<SampleType> controller;
  std::function<void(frc::ChassisSpeeds)> outputChassisSpeeds;
  std::function<bool()> mirrorTrajectory;
  frc2::Requirements drivebaseRequirements;
  AutoBindings autoBindings{};
  std::optional<TrajectoryLogger<SampleType>> trajectoryLogger;
};

}  // namespace choreo
