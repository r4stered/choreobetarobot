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
              frc2::Requirements drivebaseRequirements,
              std::optional<TrajectoryLogger<SampleType>> trajectoryLogger)
      : poseSupplier{std::move(poseSupplier)},
        controller{controller},
        outputChassisSpeeds{std::move(outputChassisSpeeds)},
        mirrorTrajectory{std::move(mirrorTrajectory)},
        drivebaseRequirements{drivebaseRequirements},
        autoBindings{std::make_shared<AutoBindings>()},
        trajectoryLogger{std::move(trajectoryLogger)} {}

  AutoLoop<SampleType, Year> NewLoop(std::string_view name) const {
    return AutoLoop<SampleType, Year>(name);
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
        loop.GetLoop(),      autoBindings};
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

  void Bind(std::string_view name, std::function<frc2::CommandPtr()> cmdFactory) {
    autoBindings->Bind(name, std::move(cmdFactory));
  }

 private:
  std::function<frc::Pose2d()> poseSupplier;
  ChoreoControllerFunction<SampleType> controller;
  std::function<void(frc::ChassisSpeeds)> outputChassisSpeeds;
  std::function<bool()> mirrorTrajectory;
  frc2::Requirements drivebaseRequirements;
  std::shared_ptr<AutoBindings> autoBindings;
  std::optional<TrajectoryLogger<SampleType>> trajectoryLogger;
};

}  // namespace choreo
