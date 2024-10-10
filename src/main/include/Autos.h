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
    factory.Bind("test", [this] {
      fmt::print("Hello from bind lambda!\n");
      return frc2::cmd::RunOnce([] { double test = 1 + 1; }); 
    });
    straightTraj = factory.Trajectory("Straight", loop);
    loop.Enabled().OnTrue(frc2::cmd::RunOnce([this] {
                            swerveSub.ResetPose(
                                straightTraj.GetInitialPose().value(), true);
                          }).AndThen(straightTraj.Cmd()));

    return loop.Cmd().WithName("Test Auto Loop Cmd");
  }

 private:
  SwerveDrive& swerveSub;
  choreo::AutoFactory<choreo::SwerveSample>& factory;
  choreo::AutoLoop<choreo::SwerveSample> loop;
  choreo::AutoTrajectory<choreo::SwerveSample> straightTraj;
};