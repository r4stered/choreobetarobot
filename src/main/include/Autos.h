#pragma once

#include "choreo/auto/AutoFactory.h"
#include "choreo/auto/AutoLoop.h"
#include "choreo/auto/AutoTrajectory.h"
#include "choreo/trajectory/SwerveSample.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "subsystems/SwerveDrive.h"

class AutoRoutines {
 public:
  explicit AutoRoutines(SwerveDrive& swerveSub,
                        choreo::AutoFactory<choreo::SwerveSample>& factory)
      : swerveSub{swerveSub},
        factory{factory},
        loop{factory.NewLoop("Auto Routine Loops")} {}

  frc2::CommandPtr TestAuto() {

  }

 private:
  SwerveDrive& swerveSub;
  choreo::AutoFactory<choreo::SwerveSample>& factory;
  choreo::AutoLoop<choreo::SwerveSample> loop;
  choreo::AutoTrajectory<choreo::SwerveSample> straightTraj;
};